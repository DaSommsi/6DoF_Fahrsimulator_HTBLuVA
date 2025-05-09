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
  
  int axisArray[6];
  
  // Überprüft ob Daten im Buffer sind
  if (Serial.available() > 0){
    String incomingData = Serial.readStringUntil('\n');     // Daten aus Buffer holen
    Serial.println(incomingData);                           // Daten ausgeben um zum Testen

    ConvertIncomingDataStringToIntArray(axisArray, incomingData);

    for(int i = 0; i<6; i++){
      Serial.print(axisArray[i]);
    }
  }
}

// Die Funktion extrahiert die Zahlenwerte die in dem Daten String sind und verpackt sie uns in einen Array den wir dann nutzen können
void ConvertIncomingDataStringToIntArray(int axisData[], const String& inputData){
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
        axisData[axisIndex++] = currentValue.toInt(); // Umwandlung und Speichern
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