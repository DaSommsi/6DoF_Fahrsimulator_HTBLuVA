#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  ProcessIncomingDataFromSimTools();
}

// Funktionen

// Funktion wird dauerhaft aufgerufen und nimmt die Daten entgegen und verarbeitet sie
void ProcessIncomingDataFromSimTools(){
  // Überprüft ob Daten im Buffer sind
  if (Serial.available() > 0){
    // Daten aus Buffer holen
    String incomingData = Serial.readStringUntil('\n');
    // Daten ausgeben um zum Testen
    Serial.println(incomingData);


  }
}

