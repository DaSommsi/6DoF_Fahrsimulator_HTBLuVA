# Entwicklungslogbuch – 6DoF Fahrsimulator (HTBLuVA Salzburg)

## Projektstart
**Datum:** 09.05.2025<br>
**Ziel:** Wiederherstellung und Neuentwicklung der Software für einen 6DoF-Fahrsimulator zur realistischen Bewegungsdarstellung basierend auf Eingabedaten (Geschwindigkeit, Neigung etc.)

---

## 📅 Datum: 09.05.2025
### 📝 Zusammenfassung
- GitHub-Repository erstellt
- PlatformIO eingerichtet
- `README.md` erstellt
- `development.md` erstellt
- Informationen aus der Diplomarbeit gesammelt die untern Angeführt sind
- Projekt für PlatformIO eingerichtet, um den ESP32-Code kompatibel zu machen
- Kopieren und Dokumentieren des aktuell vorhandenem Code

### 🔍 Detaillierte Beschreibung

#### GitHub Repository

Heute wurde das GitHub-Repository für das 6DoF-Fahrsimulator-Projekt erstellt und als Basis für die weitere Entwicklung eingerichtet.

In Visual Studio Code wurde die grundlegende Projektstruktur aufgesetzt – inklusive `src/`-Ordner, `platformio.ini` und der Startdatei `main.cpp`.

#### Basis Informationen

Als erstes wurde die Datei `development.md` erstellt, um die Dokumentation und den Fortschritt des Projekts zentral an einem Ort festzuhalten.

Aus der ursprünglichen Diplomarbeit ging hervor, dass das Tool **SimTools** verwendet wurde, um **Telemetriedaten** direkt vom Spiel auszulesen. Diese Daten werden lokal am Simulator-PC verarbeitet und anschließend über eine USB-Verbindung an den integrierten **ESP32** übertragen. Auf diesem Mikrocontroller läuft der Steuerungscode (zukünftig in der Datei `main.c`), der für die präzise Ansteuerung der **sechs Motoren** verantwortlich ist. Dadurch wird die Bewegung des Simulators in Echtzeit an das Spielgeschehen angepasst und ein realistisches Fahrerlebnis ermöglicht.

#### PlatformIO

Das Projekt wurde so aufgesetzt, dass der Code für den ESP32 mit PlatformIO entwickelt und kompiliert werden kann. Dazu wurde die benötigte Projektstruktur erstellt, die passende platformio.ini-Konfigurationsdatei angelegt und der Code so vorbereitet, dass er mit der ESP32-Toolchain kompatibel ist. PlatformIO erleichtert das Build-System, das Flashen auf das Board und die Verwaltung von Abhängigkeiten.

#### Datenübertragung von SimTools zu ESP32

Um die Bewegungsdaten der sechs Achsen an den ESP32 zu übertragen, wird in SimTools ein vordefiniertes Kommunikationsprotokoll verwendet(Hinweis: Die Einstellungen dieses Protokolls werden zu einem späteren Zeitpunkt noch folgen). Die Übertragung erfolgt über eine serielle Schnittstelle. Dabei sendet SimTools die Werte für die sechs Achsen in Form der Variablen `Axis1a`, `Axis2a`, `Axis3a`, `Axis4a`, `Axis5a` und `Axis6a`.

Jede dieser Achsenvariablen steht dabei für eine Bewegungsrichtung oder -achse des Simulators. Diese Rohdaten werden anschließend im Code des ESP32 verarbeitet, um die entsprechenden Motoren gezielt anzusteuern und die Bewegung des Simulators an das Spielgeschehen anzupassen.

| **Bezeichnung** | **Achse / Bewegung** | **Art der Bewegung**                    |
| --------------- | -------------------- | --------------------------------------- |
| **Axis1a**      | Surge                | Vorwärts / Rückwärts (Translation)      |
| **Axis2a**      | Sway                 | Links / Rechts (Translation)            |
| **Axis3a**      | Heave                | Auf / Ab (Translation)                  |
| **Axis4a**      | Roll                 | Seitliches Kippen (Rotation um X)       |
| **Axis5a**      | Pitch                | Vor / Zurück kippen (Rotation um Y)     |
| **Axis6a**      | Yaw                  | Drehen um die Hochachse (Rotation um Z) |

Die Bewegungsdaten werden im folgenden Format an den ESP32 über die serielle Schnittstelle gesendet:

`<Axis1a>,<Axis2a>,<Axis3a>,<Axis4a>,<Axis5a>,<Axis6a>X`

Dabei dienen die Zeichen `>,<` als Trenner zwischen den einzelnen Achsdaten. Das `X` am Ende des Strings fungiert als Stop-Zeichen, um das Ende der Datenübertragung zu kennzeichnen.
(Hinweis: Dieses Format kann in Zukunft noch angepasst werden.)

Hier ist der aktuelle Code, der die Datenübertragung und -verarbeitung auf dem ESP32 umsetzt:

```c
void processIncomingByte (const byte inByte) {
    static char input_line [MAX_SERIAL_INPUT];
    static unsigned int input_pos = 0;

    switch (inByte)
    {
        case ’X’: // end of text
        input_line [ input_pos ] = 0; // terminating null byte
        process_data ( input_line );
        input_pos = 0;
        break;
    case ’\r’: // discard carriage return
        break;

    default :
        // buffer data 
        if ( input_pos < ( MAX_SERIAL_INPUT - 1) )
            input_line [ input_pos ++] = inByte ;
        break;
    }
}
```

Das ist der Code der für die Entgegen nahme des Datenpakets das von SimTools gesendet wird. Hier ist noch die `process_data()` Funktion.

```c
// parses the data packet from the pc = > x,y,z,Ry ,Rx ,RZ
void process_data ( char * data ){

    int i = 0;
    char * tok = strtok ( data , ",") ;

    float arrRaw []={0 ,0 ,0 ,0 ,0 ,0};
    float arrRateLimited []={0 ,0 ,0 ,0 ,0 ,0};

    while ( tok != NULL ) {
        double value = ( float ) atof ( tok ) ;
        float temp = 0.0;

        if( i == 2)
            temp = mapfloat ( value , 0 , 4094 , -7 , 7) ;// hieve
        else if( i > 2) // rotations , pitch ,roll ,yaw
            temp = mapfloat ( value , 0 , 4094 , -30 , 30) *( pi /180.0) ; // these are tuned to this specific platform ,         
        else // sway , surge
            temp = mapfloat ( value , 0 , 4094 , -8 , 8) ;

            arrRaw [ i ++] = temp ;

            tok = strtok ( NULL , ",") ;
    }
}

float mapfloat ( double x , double in_min , double in_max , double out_min ,double out_max ){

return ( float ) ( x - in_min ) * ( out_max - out_min ) / ( float ) ( in_max -in_min ) + out_min ; 
}
```

Das sieht alles sehr komplex aus meiner Meinung nach. Deswegen werde ich das hier einfacher coden und unter dem hier Dokumentieren:

```c
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
    String incomingData = Serial.readStringUntil('X');
    // Daten ausgeben um zum Testen
    Serial.println(incomingData);
  }
}
```

Hier haben wir einen effizienteren Code um die Daten zu verarbeiten. Wir haben die Funktion `ProcessIncomingDataFromSimTools()` erstellt, die dauerhaft aufgerufen wird. Diese Funktion überprüft, ob Daten im Buffer sind. Wenn ja, werden die Daten aus dem Buffer geholt und ausgegeben. Dieser Code ist einfacher und effizienter als der ursprüngliche Code. 

Als Beispiel wenn dieser String `<120>,<25>,<255>,<20>,<16>,<189>X` von SimTools über die Serielle Schnitstelle geschickt wird, dann ist der Buffer des ESP32 nicht mehr leer und mit `Serial.readStringUntil('X')` bekommen wir dann `<120>,<25>,<255>,<20>,<16>,<189>`. Jetzt müssen wir diesen String nur noch in einzelne Werte aufteilen und diese dann in die entsprechenden Variablen speichern.

```c
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
```

Die Funktion `ConvertIncomingDataStringToIntArray()` dient dazu, einen Datenstring im SimTools-Format wie `<120>,<25>,<255>,<20>,<16>,<189>` auszulesen und die darin enthaltenen Zahlenwerte in ein Integer-Array (`axisDataArray[]`) zu übertragen.


---
