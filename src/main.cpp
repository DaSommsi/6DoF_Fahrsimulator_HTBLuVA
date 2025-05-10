#include <Arduino.h>

float rawAxisDataArray[6];
float normalizedAxisDataArray[6];

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
        normalizedDataArray[i] = mapFloat(rawDataArray[i], 0, 4096, -8, 8);               // Surge und Sway
      }else if(i == 2){
        normalizedDataArray[i] = mapFloat(rawDataArray[i], 0, 4096, -7, 7);               // Heave
      }else{
        normalizedDataArray[i] = mapFloat(rawDataArray[i], 0, 4096, -30, 30) * PI/180.0;  // Roll, Pitch und Yaw
      }
    }

    for(int i = 0; i<6; i++){
      Serial.print(normalizedDataArray[i]);
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
float mapFloat(double inputValue, double inputMin, double inputMax, double outputMin, double outputMax) {
    
    // Schritt 1: Berechne die Position des Eingabewerts im Verhältnis zum Eingabebereich (zwischen 0 und 1)
    double normalized = (inputValue - inputMin) / (inputMax - inputMin);

    // Schritt 2: Skaliere diesen Wert auf die Größe des Ausgabebereichs
    double scaled = normalized * (outputMax - outputMin);

    // Schritt 3: Verschiebe den Wert in den gewünschten Ausgabebereich
    double mappedValue = scaled + outputMin;

    // Ergebnis zurückgeben als float
    return (float)mappedValue;
}
