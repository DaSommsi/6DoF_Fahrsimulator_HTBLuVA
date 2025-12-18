#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include "Adafruit_MCP23X17.h"

Adafruit_MCP23X17 mcp;

// #define DEBUG_MODE 1 // Hier aktivieren Sie den Debug-Modus

#ifdef DEBUG_MODE
  // Wenn der Debug-Modus AKTIV ist, wird DEBUG_PRINT zu Serial.print()
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  // Wenn der Debug-Modus INAKTIV ist, wird DEBUG_PRINT zu einem leeren Ausdruck.
  // Der Compiler entfernt dies komplett und spart Speicher & Zeit.
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Wichtige-INFOS
/*
Aktuell vohrer ESP einstecken ohne Laptop dann Powern und dann erst Serielle Schnitstelle aktivieren. 

*/

// SPI Pins
#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCK  14
#define PIN_CS   15

// ######## Globale Variablen ########

float rawAxisDataArray[6];
float normalizedAxisDataArray[6];

float calculatedRotationMatrix[3][3];

// #### Motoren ####

// 6 Motoren
bool tryingToReachHome = false;

const int motorCount = 6;

const int pwmPins[motorCount]       = {0, 1, 2, 3, 4, 5};   // GPA0–GPA5
const int directionPins[motorCount] = {6, 7, 8, 9, 10, 11}; // GPA6–GPB3
const int inductionSensorPins[motorCount] = {33, 32, 35, 34, 39, 36};

const int oddMotor[motorCount] = {1, 0, 1, 0, 1, 0};
bool motorActive[motorCount] = {true, true, true, true, true, true};

const unsigned long motorInterval = 6;                                        // Overall Motor Speed 15ms
int motorDirection[motorCount];                             // 0 = rückwärts, 1 = vorwärts
bool pwmState[motorCount] = {false, false, false, false, false, false};
unsigned long lastToggle[motorCount] = {0, 0, 0, 0, 0, 0};

// in Rad
float motorCurrentPosition[motorCount];
float motorTargetPosition[motorCount];

// ######## Globale Konstanten (in cm) ########

constexpr float PLATFORM_JOINT_COORDINATES[6][3] = {{15.0, -55.5, 62.5}, // Rechts unten Point 1
                                                    {54.5, 14.5, 62.5}, // Rechts mitte  Point 2
                                                    {39.5, 39.5, 62.5}, // Rechts oben Point 3
                                                    {-39.5, 39.5, 62.5}, // Links oben Point 4
                                                    {-54.5, 14.5, 62.5}, // Links mitte  Point 5
                                                    {-15.0, -55.5, 62.5}}; // Links unten Point 6

constexpr float PLATFORM_TO_BASE_DISPLACMENT[3] = {0.0, 0.0, 62.5};

constexpr float BASE_SERVO_COORDINATES[6][3] = {{23.0, -38.7, 0}, // Rechts unten Servo 1
                                                {45.0, 0, 0}, // Rechts mitte Servo 2
                                                {23.0, 38.7, 0}, // Rechts oben Servo 3
                                                {-23.0, 38.7, 0}, // Links oben Servo 4
                                                {-45.0, 0, 0}, // Links mitte Servo 5
                                                {-23.0, -38.7, 0}}; // Links unten Servo 6

const float SERVOARM_ARM_LENGHT = 72.0f;
const float SERVOARM_LENGHT = 37.5f;

const float SERVO_BETA[6] = {30, 90, 150, 210, 270, 330};

const float MOTOR_MAX_STEP_RESULTION = 500000.0f;
const float MOTOR_STEPS_PER_SIGNAL = 1000.0f;
const float RAD_PER_SIGNAL = (MOTOR_STEPS_PER_SIGNAL/MOTOR_MAX_STEP_RESULTION) * 2 * PI;
const float MOTOR_RAD_OFFSET_TOLERANCE = 1 * PI / 180;

const float RAD_BEFORE_IND_SENSOR = 5 * PI / 180;

// Anderes

enum SystemState {
  STATE_GOTO_HOME,
  STATE_WAIT_FOR_HOME,
  STATE_RUNNING
};

SystemState systemState = STATE_GOTO_HOME;

// Funktionen

bool ProcessIncomingDataFromSimTools(float rawDataArray[], float normalizedDataArray[]);
float MapFloat(double inputValue, double inputMin, double inputMax, double outputMin, double outputMax);
void ConvertIncomingDataStringToIntArray(float axisData[], const String& inputData);
void CalculateServoAlpha(float normalizedDataArray[], float rotationMatrix[3][3]);
void CalculateRotationMatrix(float normalizedDataArray[], float rotationMatrix[3][3]);
float CalculateSegmentLength(float rotationMatrix[3][3], int index);

void GoToHomePosition();
bool CheckIfAtHome();
void CalculateMotorDirectionAndPosition();
bool CheckIfMotorIsAtPosition(int index);
void UpdatePWM();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  if (!mcp.begin_SPI(PIN_CS)) {
    DEBUG_PRINTLN("Fehler: MCP23S17 nicht gefunden!");
    while (1);
  }

  DEBUG_PRINTLN("MCP23S17 verbunden!");

  for (int i = 0; i < motorCount; i++) {
    mcp.pinMode(pwmPins[i], OUTPUT);
    mcp.pinMode(directionPins[i], OUTPUT);
    mcp.digitalWrite(pwmPins[i], LOW);
    mcp.digitalWrite(directionPins[i], LOW);
  }

  for (int i = 0; i < motorCount; i++) {
    pinMode(inductionSensorPins[i], INPUT);
  }
  
}

void loop() {

  switch(systemState) {

    case STATE_GOTO_HOME:
      GoToHomePosition();       // nur einmal gesendet
      systemState = STATE_WAIT_FOR_HOME;
      break;

    case STATE_WAIT_FOR_HOME:
      if(CheckIfAtHome()) {     // wartet bis true 
        systemState = STATE_RUNNING;
        Serial.read();
        DEBUG_PRINTLN("##############  Home normal State is rechead!  ####################");
      }
      break;

    case STATE_RUNNING:
      // Serial.println("Working!!!!!!!!!!!!!!");
      if(ProcessIncomingDataFromSimTools(rawAxisDataArray, normalizedAxisDataArray)) {
        CalculateServoAlpha(normalizedAxisDataArray, calculatedRotationMatrix);
        CalculateMotorDirectionAndPosition();
        for (int i = 0; i < motorCount; i++){
          motorActive[i] = !CheckIfMotorIsAtPosition(i);
          DEBUG_PRINTLN("Motor state fuer" + String(i) + String(motorActive[i]));
        }
      }
      break;
  }

  UpdatePWM();   // PWM muss natürlich immer weiterlaufen
}

// Funktionen

// <1500>,<2500>,<50>,<60>,<842>,<4000>X 2048,2048,2048,2048,2048,2048X

// Funktion wird dauerhaft aufgerufen und nimmt die Daten entgegen und verarbeitet sie, gibt True zurück wenn etwas empfangen wurde
bool ProcessIncomingDataFromSimTools(float rawDataArray[], float normalizedDataArray[]){
  // Überprüft ob Daten im Buffer sind
  if (Serial.available() > 0){
    String incomingData = Serial.readStringUntil('X');     // Daten aus Buffer holen

    // Daten von String zu den einzelnen Werten umwandeln
    ConvertIncomingDataStringToIntArray(rawDataArray, incomingData);

    // Daten zu ihren bestimmten Zahlenbereichen normalisieren
    for(int i = 0; i<6; i++){
      if(i == 0 || i == 1){
        normalizedDataArray[i] = MapFloat(rawDataArray[i], 0, 4096, -8, 8);               // Surge und Sway
      }else if(i == 2){
        normalizedDataArray[i] = MapFloat(rawDataArray[i], 0, 4096, -7, 7);               // Heave
      }else{
        normalizedDataArray[i] = MapFloat(rawDataArray[i], 0, 4096, -30, 30) * PI/180.0;  // Roll, Pitch und Yaw
      }
    }
    return true;
  }
  return false;
}


// Die Funktion extrahiert die Zahlenwerte die in dem Daten String sind und verpackt sie uns in einen Array den wir dann nutzen können
void ConvertIncomingDataStringToIntArray(float axisData[], const String& inputData){
  int axisIndex = 0;
  int strLength = inputData.length();
  String currentValue = ""; 

  for(int i = 0; i < strLength; i++){
    char c = inputData.charAt(i);

    // Wenn das Zeichen eine Zahl, ein Punkt oder ein Minus ist, sammeln
    if(isDigit(c) || c == '.' || c == '-'){
      currentValue += c;
    } 
    // Wenn ein Komma kommt oder wir am Ende des Strings ('X' oder String-Ende) sind
    if (c == ',' || i == strLength - 1) {
      if(currentValue.length() > 0 && axisIndex < 6){
        axisData[axisIndex] = currentValue.toFloat();
        axisIndex++;
        currentValue = ""; // String leeren für die nächste Zahl
      }
    }
  }

  // Wichtig: Falls SimTools weniger als 6 Achsen schickt, 
  // füllen wir den Rest mit dem Mittelwert (2047) auf, damit nichts crasht.
  while(axisIndex < 6){
    axisData[axisIndex] = 2047.0; 
    axisIndex++;
  }
}

// Diese Funktion skaliert einen Wert von einem Eingabebereich in einen Zielbereich.
float MapFloat(double inputValue, double inputMin, double inputMax, double outputMin, double outputMax) {
    
    // Schritt 1: Berechne die Position des Eingabewerts im Verhältnis zum Eingabebereich (zwischen 0 und 1)
    double normalized = (inputValue - inputMin) / (inputMax - inputMin);

    // Schritt 2: Skaliere diesen Wert auf die Größe des Ausgabebereichs
    double scaled = normalized * (outputMax - outputMin);

    // Schritt 3: Verschiebe den Wert in den gewünschten Ausgabebereich
    double mappedValue = scaled + outputMin;

    // Ergebnis zurückgeben als float
    return (float)mappedValue;
}

void CalculateServoAlpha(float normalizedDataArray[], float rotationMatrix[3][3]){
  CalculateRotationMatrix(normalizedAxisDataArray, calculatedRotationMatrix);

  float servoAlpha = 0.0f;

  for (int i = 0; i < 6; i++){
    
    // ######### Öfter verwendete in Berechnungen #########

    float segmentLength = CalculateSegmentLength(calculatedRotationMatrix, i);
    float cosBeta = cos(SERVO_BETA[i] * PI / 180);
    float sinBeta = sin(SERVO_BETA[i] * PI / 180);
    float _2a = (2 * SERVOARM_LENGHT);

    float dx = PLATFORM_JOINT_COORDINATES[i][0] - BASE_SERVO_COORDINATES[i][0];
    float dy = PLATFORM_JOINT_COORDINATES[i][1] - BASE_SERVO_COORDINATES[i][1];
    float dz = PLATFORM_JOINT_COORDINATES[i][2] - BASE_SERVO_COORDINATES[i][2];

    float ax = _2a * dz;
    float ay = _2a * (cosBeta * dx + sinBeta * dy);  // oft benutzter Term

    float r2 = ax*ax + ay*ay;      // ersetzt pow() + sqrt später
    float r = sqrt(r2);

    // ######### Berechnung #########

    float num = segmentLength*segmentLength 
            - (SERVOARM_ARM_LENGHT*SERVOARM_ARM_LENGHT 
            - SERVOARM_LENGHT*SERVOARM_LENGHT);

    float x = num / r;
    x = constrain(x, -1.0, 1.0);
    float asinTerm = asin(x);

    // atan-Term
    float atanTerm = atan2(ay, ax);

    servoAlpha = asinTerm - atanTerm;

    motorTargetPosition[i] = servoAlpha;

    // DEBUG_PRINT(String(servoAlpha) + ", ");

    // servoAlpha = asin((pow(segmentLength, 2) - (pow(SERVOARM_ARM_LENGHT, 2) - pow(SERVOARM_LENGHT, 2))) / (sqrt(pow(_2a * (PLATFORM_JOINT_COORDINATES[i][2] - BASE_SERVO_COORDINATES[i][2]), 2) + pow((_2a * (cosBeta * (PLATFORM_JOINT_COORDINATES[i][0] - BASE_SERVO_COORDINATES[i][0])) + sinBeta * (PLATFORM_JOINT_COORDINATES[i][1] - BASE_SERVO_COORDINATES[i][1])), 2)))) - atan((_2a * (PLATFORM_JOINT_COORDINATES[i][2] - BASE_SERVO_COORDINATES[i][2])) / (_2a * ((cosBeta * (PLATFORM_JOINT_COORDINATES[i][0] - BASE_SERVO_COORDINATES[i][0])) + (sinBeta * (PLATFORM_JOINT_COORDINATES[i][1] - BASE_SERVO_COORDINATES[i][1])))));
  }
}

// Berechnet die Rotationmatrix aus den gegebenen Werten
void CalculateRotationMatrix(float normalizedDataArray[], float rotationMatrix[3][3]) {
  float psi = normalizedDataArray[3];     // Roll
  float theta = normalizedDataArray[4];   // Pitch
  float phi = normalizedDataArray[5];     // Yaw
  
  
  // Berechnung der Rotationmatrix
/*  rotationMatrix = {
    {cos(psi)*cos(theta), (-sin(psi)*cos(phi)) + (cos(psi)*sin(theta)*sin(phi)), (sin(psi)*sin(phi)) + (cos(psi)*sin(theta)*cos(phi))},
    {sin(psi)*cos(theta), (cos(psi)*cos(phi)) + (sin(psi)*sin(theta)*sin(phi)), (-cos(psi)*sin(phi)) + (sin(psi)*sin(theta)*cos(phi))},
    {-sin(theta),         cos(theta)*sin(phi),                                  cos(theta)*cos(phi)}
    };*/

  rotationMatrix[0][0] = cos(psi) * cos(theta);
  rotationMatrix[0][1] = (-sin(psi) * cos(phi)) + (cos(psi) * sin(theta) * sin(phi));
  rotationMatrix[0][2] = (sin(psi) * sin(phi)) + (cos(psi) * sin(theta) * cos(phi));

  rotationMatrix[1][0] = sin(psi) * cos(theta);
  rotationMatrix[1][1] = (cos(psi) * cos(phi)) + (sin(psi) * sin(theta) * sin(phi));
  rotationMatrix[1][2] = (-cos(psi) * sin(phi)) + (sin(psi) * sin(theta) * cos(phi));

  rotationMatrix[2][0] = -sin(theta);
  rotationMatrix[2][1] = cos(theta) * sin(phi);
  rotationMatrix[2][2] = cos(theta) * cos(phi);
}

// Berechnet die Segment Länge für den i-ten Servo
float CalculateSegmentLength(float rotationMatrix[3][3], int index){
  
  // Erstellt platformJoint Vectoren nach den realen Abmessungen, z auf Null damit das von der Mitte der Platform ausgeht nicht von der Base
  float platformJoints[3] = {
    PLATFORM_JOINT_COORDINATES[index][0],
    PLATFORM_JOINT_COORDINATES[index][1],
    0
  };
  
  // Multipliziert die Rotationsmatrix mit dem Verbingungs Punkt an der Platform
  float tempPoint[3] = {0,0,0};

  for(int i = 0; i<3; i++){
    for(int j = 0; j<3; j++){
      tempPoint[i] += rotationMatrix[i][j] * platformJoints[j];
    }
  }

  // Berechnung von dem Vector segmentLenght
  float segmentLength[3] = {0,0,0};

  // Berechnet den T Vector der das Displacment von der der Mitte der Base zur der Mitte der Platform wieder gibt.
  float translationVector[3] = {
    PLATFORM_TO_BASE_DISPLACMENT[0] + normalizedAxisDataArray[0], // Surge
    PLATFORM_TO_BASE_DISPLACMENT[1] + normalizedAxisDataArray[1], // Sway
    PLATFORM_TO_BASE_DISPLACMENT[2] + normalizedAxisDataArray[2]  // Heave
  };

  for (int i = 0; i < 3; i++) {
    segmentLength[i] = translationVector[i] + tempPoint[i] - BASE_SERVO_COORDINATES[index][i];
  }

  // Berechnet Betrag von dem Vector
  return sqrt(
    pow(segmentLength[0], 2) +
    pow(segmentLength[1], 2) +
    pow(segmentLength[2], 2)
  );
}

void UpdatePWM() {
  unsigned long now = millis();

  for (int i = 0; i < motorCount; i++) {
    if (!motorActive[i]) {
      // Motor deaktiviert → ausschalten
      mcp.digitalWrite(pwmPins[i], LOW);
      continue;
    }

    if (now - lastToggle[i] >= motorInterval) {
      lastToggle[i] = now;
      pwmState[i] = !pwmState[i];
      mcp.digitalWrite(pwmPins[i], pwmState[i]);

      // Richtung setzen
      mcp.digitalWrite(directionPins[i], motorDirection[i]);

      // Rad hinzufügen zu aktueller Position
      if(systemState == STATE_RUNNING){
        // DEBUG_PRINTLN("Before Motor" + String(i) + " Direction:" + String(motorDirection[i]) + " Target:" + String(motorTargetPosition[i]) + " Current" + String(motorCurrentPosition[i]));
        motorCurrentPosition[i] += motorDirection[i] ? +RAD_PER_SIGNAL : -RAD_PER_SIGNAL;
        DEBUG_PRINTLN("After Motor" + String(i) + " Direction:" + String(motorDirection[i]) + " Target:" + String(motorTargetPosition[i]) + " Current" + String(motorCurrentPosition[i]));
        motorActive[i] = !CheckIfMotorIsAtPosition(i);
        // DEBUG_PRINTLN("Set Motor Line 375: " + String(motorActive[i]) + String(i));
      }
    } 
  }
}

void GoToHomePosition(){
  tryingToReachHome = true;

  for (int i = 0; i < motorCount; i++){
    motorActive[i] = true;
    
    if(oddMotor[i]){
      motorDirection[i] = 1;
      motorCurrentPosition[i] = PI / 2 + RAD_BEFORE_IND_SENSOR;
    } else {
      motorDirection[i] = 0;
      motorCurrentPosition[i] = PI / 2 - RAD_BEFORE_IND_SENSOR;
    }
  }
}

bool CheckIfAtHome() {
  bool allAtHome = true;

  for(int i = 0; i < motorCount; i++) {
    if(!motorActive[i]) continue;

    // Beispiel: Wenn Induktivsensor ausgelöst hat → HOME erreicht
    if(digitalRead(inductionSensorPins[i])) {
      allAtHome = false;      // Noch nicht am Home-Punkt
    } else {
      motorActive[i] = false; // Motor stoppen
    }
  }

  return allAtHome;
}

void CalculateMotorDirectionAndPosition() {
  for (int i = 0; i < motorCount; i++) {

    float target = motorTargetPosition[i];

    // Nicht-odd Motoren drehen von der negativen X-Achse aus
    if (!oddMotor[i]) {
      target = PI - target;
      motorTargetPosition[i] = target;
    }
    // DEBUG_PRINTLN("Motor" + String(i) + " Target:" + String(target) + " Current" + String(motorCurrentPosition[i]));
    // Delta berechnen — je nach Motor-Typ dreht er „umgekehrt“
    float delta;
    if(oddMotor[i]){
      delta = motorCurrentPosition[i] - target; 
      
      if(delta<0){ 
        motorDirection[i] = 1; 
      }else{ 
        motorDirection[i] = 0; } 
    }else{ 
      delta = target - motorCurrentPosition[i]; 
        
      if(delta<0){ 
        motorDirection[i] = 0; 
      }else{ 
        motorDirection[i] = 1; 
      }
    }
  }
}

bool CheckIfMotorIsAtPosition(int index) {
    return fabs(motorCurrentPosition[index] - motorTargetPosition[index]) <= MOTOR_RAD_OFFSET_TOLERANCE;
}