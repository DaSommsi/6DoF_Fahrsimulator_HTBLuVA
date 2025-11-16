#include <Arduino.h>
#include <math.h>

// Globale Variablen

float rawAxisDataArray[6];
float normalizedAxisDataArray[6];

float calculatedRotationMatrix[3][3];

// Globale Konstanten (in cm)

constexpr float PLATFORM_JOINT_COORDINATES[6][3] = {{15.0, -55.5, 0}, // Rechts unten Point 1
                                                {54.5, 14.5, 0}, // Rechts mitte  Point 2
                                                {39.5, 39.5, 0}, // Rechts oben Point 3
                                                {-39.5, 39.5, 0}, // Links oben Point 4
                                                {-54.5, 14.5, 0}, // Links mitte  Point 5
                                                {-15.0, -55.5, 0}}; // Links unten Point 6

constexpr float PLATFORM_TO_BASE_DISPLACMENT[3][1] = {{0.0},
                                                      {0.0},
                                                      {62.5}};

constexpr float BASE_SERVO_COORDINATES[6][3] = {{23.0, -38.7, 0}, // Rechts unten Servo 1
                                                {45.0, 0, 0}, // Rechts mitte Servo 2
                                                {23.0, 38.7, 0}, // Rechts oben Servo 3
                                                {-23.0, 38.7, 0}, // Links oben Servo 4
                                                {-45.0, 0, 0}, // Links mitte Servo 5
                                                {-23.0, -38.7, 0}}; // Links unten Servo 6

// Funktionen

bool ProcessIncomingDataFromSimTools(float rawDataArray[], float normalizedDataArray[]);
float MapFloat(double inputValue, double inputMin, double inputMax, double outputMin, double outputMax);
void ConvertIncomingDataStringToIntArray(float axisData[], const String& inputData);
void CalculateServoAlpha(float normalizedDataArray[], float rotationMatrix[3][3]);
void CalculateRotationMatrix(float normalizedDataArray[], float rotationMatrix[3][3]);
float CalculateSegmentLength(float rotationMatrix[3][3], int index);

void setup() {
  Serial.begin(115200);
}

void loop() {
  if(ProcessIncomingDataFromSimTools(rawAxisDataArray, normalizedAxisDataArray)){
    CalculateServoAlpha(normalizedAxisDataArray, calculatedRotationMatrix);
  }
}

// Funktionen

// <1500>,<2500>,<50>,<60>,<842>,<4000>X <2048>,<2048>,<2048>,<2048>,<2048>,<4096>X

// Funktion wird dauerhaft aufgerufen und nimmt die Daten entgegen und verarbeitet sie, gibt True zurück wenn etwas empfangen wurde
bool ProcessIncomingDataFromSimTools(float rawDataArray[], float normalizedDataArray[]){
  // Überprüft ob Daten im Buffer sind
  if (Serial.available() > 0){
    String incomingData = Serial.readStringUntil('\n');     // Daten aus Buffer holen
    Serial.println(incomingData);                           // Daten ausgeben um zum Testen

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
  int axisIndex = 0;                // Index für das Array
  bool insideBracket = false;       // Flag, ob wir gerade zwischen < > sind
  String currentValue = "";         // Temporäre Speicherung des aktuellen Werts

  
  for(int i = 0; i<inputData.length(); i++){
    char currentChar = inputData.charAt(i);

    if(currentChar == '<'){
      insideBracket = true;
      currentValue = ""; // Neues Zahlenfragment anfangen
    }
    else if(currentChar == '>'){
      if(insideBracket && axisIndex < 6){
        axisData[axisIndex++] = currentValue.toFloat(); // Umwandlung und Speichern
      }
      insideBracket = false;
    }
    else if(insideBracket){
      currentValue += currentChar; // Zeichen anhängen
    }
  }

  // Falls weniger als 6 Werte empfangen wurden → Rest auffüllen mit 0
  while(axisIndex < 6){
    axisData[axisIndex++] = 0;
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

  for(int i = 0; i<6; i++){
    Serial.print(String(CalculateSegmentLength(calculatedRotationMatrix, i)) + ", ");
  }
}

// Berechnet die Rotationmatrix aus den gegebenen Werten
void CalculateRotationMatrix(float normalizedDataArray[], float rotationMatrix[3][3]) {
  float psi = normalizedDataArray[5];     // Roll
  float theta = normalizedDataArray[4];   // Pitch
  float phi = normalizedDataArray[3];     // Yaw
  
  
  // Berechnung der Rotationmatrix
/*  rotationMatrix = {
    {cos(psi)*cos(theta), (-sin(psi)*cos(phi)) + (cos(psi)*sin(theta)*sin(phi)), (sin(psi)*sin(phi)) + (cos(psi)*sin(theta)*cos(phi))},
    {sin(psi)*cos(theta), (cos(psi)*cos(phi)) + (sin(psi)*sin(theta)*sin(phi)), (-cos(psi)*sin(phi)) + (sin(psi)*sin(theta)*cos(phi))},
    {-sin(theta),         cos(theta)*sin(phi),                                  cos(theta)*cos(phi)}
    };*/

  rotationMatrix[0][0] = cos(psi) * cos(theta);
  rotationMatrix[0][1] = -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
  rotationMatrix[0][2] = sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi);

  rotationMatrix[1][0] = sin(psi) * cos(theta);
  rotationMatrix[1][1] = cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
  rotationMatrix[1][2] = -cos(psi) * sin(phi) + sin(psi) * sin(theta) * cos(phi);

  rotationMatrix[2][0] = -sin(theta);
  rotationMatrix[2][1] = cos(theta) * sin(phi);
  rotationMatrix[2][2] = cos(theta) * cos(phi);
}

// Berechnet die Segment Länge für den i-ten Servo
float CalculateSegmentLength(float rotationMatrix[3][3], int index){
  
  // Erstellt platformJoint Vectoren nach den realen Abmessungen
  float platformJoints[3] = {
    PLATFORM_JOINT_COORDINATES[index][0],
    PLATFORM_JOINT_COORDINATES[index][1],
    PLATFORM_JOINT_COORDINATES[index][2]
  };
  
  // Multipliziert die Rotationsmatrix mit dem Verbingungs Punkt an der Platform
  float tempPoint[3][1] = {{0},{0},{0}};

  for(int i = 0; i<3; i++){
    for(int j = 0; j<3; j++){
      tempPoint[i][0] += rotationMatrix[i][j] * platformJoints[j];
    }
  }

  // Berechnung von dem Vector segmentLenght
  float segmentLength[3][1] = {{0},{0},{0}};

  // Berechnet den T Vector der das Displacment von der der Mitte der Base zur der Mitte der Platform wieder gibt.
  float translationVector[3] = {
    PLATFORM_TO_BASE_DISPLACMENT[0][0] + normalizedAxisDataArray[0], // Surge
    PLATFORM_TO_BASE_DISPLACMENT[1][0] + normalizedAxisDataArray[1], // Sway
    PLATFORM_TO_BASE_DISPLACMENT[0][0] + normalizedAxisDataArray[2]  // Heave
  };

  for (int i = 0; i < 3; i++) {
    segmentLength[i][0] = translationVector[i] + tempPoint[i][0] - BASE_SERVO_COORDINATES[index][i];
  }

  // Berechnet Betrag von dem Vector
  return sqrt(
    pow(segmentLength[0][0], 2) +
    pow(segmentLength[1][0], 2) +
    pow(segmentLength[2][0], 2)
  );
}