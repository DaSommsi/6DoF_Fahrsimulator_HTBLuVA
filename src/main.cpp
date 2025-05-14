#include <Arduino.h>
#include <math.h>

// Globale Variablen

float rawAxisDataArray[6];
float normalizedAxisDataArray[6];

float calculatedRotationMatrix[3][3];

// Globale Konstanten (in cm)

const float PLATFORM_JOINT_COORDINATES[6][3] = {{-15.0, -55.5, 0}, // Links unten
                                                {-54.5, 14.5, 0}, // Links mitte
                                                {-39.5, 39.5, 0}, // Links oben
                                                {39.5, 39.5, 0}, // Rechts oben
                                                {54.5, 14.5, 0}, // Rechts mitte
                                                {15, -55.5, 0}}; // Rechts unten

const float PLATFORM_TO_BASE_DISPLACMENT[3][1] = {{0.0},
                                                  {0.0},
                                                  {62.5}};

const float BASE_SERVO_COORDINATES[6][3] = {{-23, -38.7, 0}, // Links unten
                                            {-45, 0, 0}, // Links mitte
                                            {-23, 38.7, 0}, // Links oben
                                            {23, 38.7, 0}, // Rechts oben
                                            {45, 0, 0}, // Rechts mitte
                                            {23, 38.7, 0}}; // Rechts unten

// Funktionen

void ProcessIncomingDataFromSimTools(float rawDataArray[], float normalizedDataArray[]);
float MapFloat(double inputValue, double inputMin, double inputMax, double outputMin, double outputMax);
void ConvertIncomingDataStringToIntArray(float axisData[], const String& inputData);
void CalculateServoAlpha(float normalizedDataArray[], float rotationMatrix[3][3]);
void CalculateRotationMatrix(float normalizedDataArray[], float rotationMatrix[3][3]);

void setup() {
  Serial.begin(115200);
}

void loop() {
  ProcessIncomingDataFromSimTools(rawAxisDataArray, normalizedAxisDataArray);
}

// Funktionen

// Funktion wird dauerhaft aufgerufen und nimmt die Daten entgegen und verarbeitet sie
void ProcessIncomingDataFromSimTools(float rawDataArray[], float normalizedDataArray[]){
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

    for(int i = 0; i<6; i++){
      Serial.println(normalizedDataArray[i]);
    }
  }
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
    Serial.print(CalculateSegmentLength(calculatedRotationMatrix, i));
    Serial.print(", "); 
  }
  Serial.print("\n");
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
  // Multipliziert die Rotationsmatrix mit dem Verbingungs Punkt an der Platform
  float tempPoint[3][1];

  for(int i = 0; i<3; i++){
    for(int j = 0; j<3; j++){
      tempPoint[i][1] = rotationMatrix[i][j] * PLATFORM_JOINT_COORDINATES[index][j];
    }
  }

  // Berechnung von dem Vector segmentLenght
  float segmentLength[3][1];

  segmentLength[1][1] = PLATFORM_TO_BASE_DISPLACMENT[1][1] + tempPoint[1][1] - BASE_SERVO_COORDINATES[index][1];
  segmentLength[2][1] = PLATFORM_TO_BASE_DISPLACMENT[2][1] + tempPoint[2][1] - BASE_SERVO_COORDINATES[index][2];
  segmentLength[1][1] = PLATFORM_TO_BASE_DISPLACMENT[3][1] + tempPoint[3][1] - BASE_SERVO_COORDINATES[index][3];

  // Berechnet Betrag von dem Vector
  return sqrt(pow(segmentLength[1][1], 2) + pow(segmentLength[2][1], 2) + pow(segmentLength[3][1], 2));
}