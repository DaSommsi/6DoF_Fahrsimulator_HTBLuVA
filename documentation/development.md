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

## 📅 Datum: 010.05.2025
### 📝 Zusammenfassung
- Arbeit an Datenübergabe
- Verstehen des AC Servo Drivers

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

Um die Bewegungsdaten der sechs Achsen an den ESP32 zu übertragen, wird in SimTools ein vordefiniertes Kommunikationsprotokoll verwendet(Hinweis: Die Einstellungen dieses Protokolls werden zu einem späteren Zeitpunkt noch folgen). Die Übertragung erfolgt über eine serielle Schnittstelle. Dabei sendet SimTools die Werte für die sechs Achsen in Form der Variablen `Axis1a`, `Axis2a`, `Axis3a`, `Axis4a`, `Axis5a` und `Axis6a`. Jede Axis ist 12 Bit groß und eine Decimalzahl.

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

| **Array Position** | **Achse / Bewegung** |
| --------------- | -------------------- |
| **1**      | Surge                |
| **2**      | Sway                 |
| **3**      | Heave                |
| **4**      | Roll                 |
| **5**      | Pitch                |
| **6**      | Yaw                  |

Hier ist nochmal visualisiert welcher Wert in welchem Array-Position landet.

Jetzt fehlt nur noch was wir noch nicht im Code umgestetzt haben und das ist dieser Teil:

```c
float mapfloat ( double x , double in_min , double in_max , double out_min ,double out_max ){
  return ( float ) ( x - in_min ) * ( out_max - out_min ) / ( float ) ( in_max -in_min ) + out_min ; 
}
```

Und die Umsaklierung der Werte in ihre entsprechenden Größen zubekommen werde ich die Funktion `mapFloat()` umschreiben damit sie besser lesbar ist:

```c
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
```

Das ist die fertige `mapFloat()` Funktion. Um ein bessers Verständnis zuerlangen werden wir es kurz durchrechnen.

Beispiel: `inputValue = 100`, `inputMin = 0`, `inputMax = 200`, `outputMin = 0`, `outputMax = 100`

Wir werden mit dem ersten Teil anfangen:
```c
double normalized = (inputValue - inputMin) / (inputMax - inputMin);
```
Das sieht dann mit unseren Werten so aus:

$$
\frac{100-0}{200-0} = \frac{100}{200} = 0.5
$$

Die `0.5` kann man auch als `50%` sehen. Das bedeutet, dass der Eingabewert 100 im Eingabebereich 0-200 50% des Bereichs ausmacht. Jetzt skalieren wir den Wert auf den Ausgabebereich:

```c
double scaled = normalized * (outputMax - outputMin);
```

Rechnung mit Werte:

$$
0.5*(100-0) = 0.5*(100) = 50
$$

Jetzt wird nur noch der Wert um den Ausgabeminimum verschoben. Das Ergebnis des Umskalieren ist dann `50`. Das ist der Wert, den wir mit der Funktion `mapFloat()` erhalten.


Im Original Code wird die Funktion `mapFloat()` Funktion schon verwendet und sie haben gleich die Wert in ihre neuen Größen umgeskaliert. Das werden wir jetzt noch für unsere Werte auch machen. Das hier sind die Skalierungen die verwendet wurden:

| **Array Position** | **Achse / Bewegung** | **Eingabebereich** | **Ausgabebereich** |
| ------------------ | -------------------- | ------------------ | ------------------ |
| **1**      | Surge                | 0-4096                  | -8 - 8                  |
| **2**      | Sway                 | 0-4096                  | -8 - 8                  |
| **3**      | Heave                | 0-4096                  | -7 - 7                  |
| **4**      | Roll                 | 0-4096                  | -30 - 30 *(pi/180.0)    |
| **5**      | Pitch                | 0-4096                  | -30 - 30 *(pi/180.0)    |
| **6**      | Yaw                  | 0-4096                  | -30 - 30 *(pi/180.0)    |

Wir man auch erkennen kann sind bei Roll, Pitch und Yaw die Ausgabewerte in Radiant umgerechnet. Das ist ist wichtig für die Berechnungne später aber jetzt werden wir die Umrechnung in den Code implemntieren:

```c
for(int i = 0; i<6; i++){
      if(i == 0 || i == 1){
        normalizedDataArray[i] = mapFloat(rawDataArray[i], 0, 4096, -8, 8);               // Surge und Sway
      }else if(i == 2){
        normalizedDataArray[i] = mapFloat(rawDataArray[i], 0, 4096, -7, 7);               // Heave
      }else{
        normalizedDataArray[i] = mapFloat(rawDataArray[i], 0, 4096, -30, 30) * PI/180.0;  // Roll, Pitch und Yaw
      }
    }
```

Das ist die for-Schleife die die rohen Daten in ihren neuen Größen umskalieren.

#### Berechnung der Servorotationen zur Plattformbewegung

Beim ersten Durchsehen des Codes für das Diplomprojekt wurde mir schnell klar, dass die Berechnung der Servorotationen zur gezielten Bewegung der Plattform äußerst komplex ist. Hier ist der Code zu Referenz:

```c
float getAlpha(int i, volatile float arr[]) {
    // For Platform Coord algorithm
    float platformPDx[6] = {0};
    float platformPDy[6] = {0};
    float platformAngle[6] = {0};
    float platformCoordsx[6] = {0};
    float platformCoordsy[6] = {0};
    float basePDx[6] = {0};
    float basePDy[6] = {0};
    float baseAngle[6] = {0};
    float baseCoordsx[6] = {0};
    float baseCoordsy[6] = {0};
    float DxMultiplier[6] = {1, 1, 1, -1, -1, -1};
    float AngleMultiplier[6] = {1, -1, 1, 1, -1, 1};
    float OffsetAngle[6] = {pi / 6, pi / 6, -pi / 2, -pi / 2, pi / 6, pi / 6};

    float platformPivotx[6] = {0};
    float platformPivoty[6] = {0};
    float platformPivotz[6] = {0};

    float deltaLx[6] = {0};
    float deltaLy[6] = {0};
    float deltaLz[6] = {0};
    float deltaL2Virtual[6] = {0};

    float l[6] = {0};
    float m[6] = {0};
    float n[6] = {0};
    float alpha[6] = {0};

    platformPDx[i] = DxMultiplier[i] * RD;
    platformPDy[i] = RD;
    platformAngle[i] = OffsetAngle[i] + AngleMultiplier[i] * radians(theta_r);

    platformCoordsx[i] = platformPDx[i] * cos(platformAngle[i]);
    platformCoordsy[i] = platformPDy[i] * sin(platformAngle[i]);

    basePDx[i] = DxMultiplier[i] * PD;
    basePDy[i] = PD;
    baseAngle[i] = OffsetAngle[i] + AngleMultiplier[i] * radians(theta_p);

    baseCoordsx[i] = basePDx[i] * cos(baseAngle[i]);
    baseCoordsy[i] = basePDy[i] * sin(baseAngle[i]);

    // Platform pivots
    platformPivotx[i] = platformCoordsx[i] * cos(arr[3]) * cos(arr[5]) +
                        platformCoordsy[i] * (sin(arr[4]) * sin(arr[3]) * cos(arr[3]) - cos(arr[4]) * sin(arr[5])) + arr[0];
    platformPivoty[i] = platformCoordsx[i] * cos(arr[4]) * sin(arr[5]) +
                        platformCoordsy[i] * (cos(arr[3]) * cos(arr[5]) + sin(arr[3]) * sin(arr[4]) * sin(arr[5])) + arr[1];
    platformPivotz[i] = -platformCoordsx[i] * sin(arr[3]) +
                        platformCoordsy[i] * sin(arr[4]) * cos(arr[3]) + platformHeight + arr[2];

    deltaLx[i] = baseCoordsx[i] - platformPivotx[i];
    deltaLy[i] = baseCoordsy[i] - platformPivoty[i];
    deltaLz[i] = -platformPivotz[i];

    deltaL2Virtual[i] = sqrt(pow(deltaLx[i], 2.0) + pow(deltaLy[i], 2.0) + pow(deltaLz[i], 2.0));

    l[i] = pow(deltaL2Virtual[i], 2.0) - (pow(ConnectingArmLengthL2, 2.0) - pow(ServoArmLengthL1, 2.0));
    m[i] = 2 * ServoArmLengthL1 * platformPivotz[i];
    n[i] = 2 * ServoArmLengthL1 * (cos(theta_s[i] * pi / 180) * (platformPivotx[i] - baseCoordsx[i]) +
                                   sin(theta_s[i] * pi / 180) * (platformPivoty[i] - baseCoordsy[i]));

    return asin(l[i] / sqrt(pow(m[i], 2.0) + pow(n[i], 2.0))) - atan(n[i] / m[i]);
}
```

Das ist der Code für die Berechnung. Leider sind keine Kommentare vorhanden, um die Funktionsweise zu erklären. Auch in der Diplomarbeit steht nicht wie man zu dieser Lösung gekommen ist deswegen habe ich mich hingesetzt und versucht herauszufinden wie das umgesetzt wurde. Nach einer Weile habe ich herausgefunden das das eine Stewart-Platform ist und das wahrscheinlich der Code anhand dem gemacht wurde. Der kommende Text wurde aus diesem [PDF](https://cdn.instructables.com/ORIG/FFI/8ZXW/I55MMY14/FFI8ZXWI55MMY14.pdf) verfasst.

Um das Konzept einer Stewart-Platform zu verstehen, ist es wichtig zu wissen, dass sie aus zwei Plattformen besteht, die durch sechs Gelenke miteinander verbunden sind. Die Basis stellt dabei die feste Referenz dar und hat die Koordinaten $x, y, z$, während die bewegliche Plattform durch die Koordinaten $x', y', z'$ beschrieben wird. Diese Plattform kann sich in 6 Freiheitsgraden gegenüber der Basis bewegen: drei translatorische und drei rotatorische Bewegungen.

Die Position der Plattform kann durch einen Displacement-Vektor relativ zur Basis beschrieben werden. Zusätzlich wird die Ausrichtung der Plattform in Bezug auf die Basis durch drei Winkelverschiebungen definiert. Diese werden mit einem Satz von Eulerwinkeln beschrieben, der die folgende Reihenfolge von Drehungen umfasst:

1. Drehung um den Winkel ψ (Yaw) um die z-Achse
2. Drehung um den Winkel θ (Pitch) um die y-Achse
3. Drehung um den Winkel φ (Roll) um die x-Achse

Wir werden als erstes die Rotations Matrix für $R_z (\psi)$ erstellen:
$$
R_z (\psi) = \begin{bmatrix}cos(\psi) & -sin(\psi) & 0 \\ sin(\psi) & cos(\psi) & 0 \\ 0 & 0 & 1\end{bmatrix}
$$

Als nächstes werden wir die Rotations Matrix für $R_y (\theta)$ erstellen:
$$
R_z (\theta) = \begin{bmatrix}cos(\theta) & 0 & sin(\theta) \\ 0 & 1 & 0 \\ -sin(\theta) & 0 & cos(\theta)\end{bmatrix}
$$

Und dann noch die Rotations Matrix für $R_x (\phi)$:
$$
R_z (\phi) = \begin{bmatrix}1 & 0 & 0 \\ 0 & cos(\phi) & -sin(\phi) \\ 0 & sin(\phi) & cos(\phi)\end{bmatrix}
$$

Jetzt müssen wir nur mehr die ganze Rotations Matrix für die Platform berechnen:
$$
R^P_B = R_x (\phi) \cdot R_y (\theta) \cdot R_z (\psi)
\\\
\\
R^P_B = \begin{bmatrix}cos(\psi) & -sin(\psi) & 0 \\ sin(\psi) & cos(\psi) & 0 \\ 0 & 0 & 1\end{bmatrix} \cdot \begin{bmatrix}cos(\theta) & 0 & sin(\theta) \\ 0 & 1 & 0 \\ -sin(\theta) & 0 & cos(\theta)\end{bmatrix} \cdot \begin{bmatrix}1 & 0 & 0 \\ 0 & cos(\phi) & -sin(\phi) \\ 0 & sin(\phi) & cos(\phi)\end{bmatrix}
\\\
\\
R^P_B = \begin{bmatrix}cos(\psi)cos(\theta) & -sin(\psi) & cos(\psi)sin(\theta) \\ sin(\psi)cos(\theta) & cos(\psi) & sin(\psi)sin(\theta) \\ -sin(\theta) & 0 & cos(\theta)\end{bmatrix} \cdot \begin{bmatrix}1 & 0 & 0 \\ 0 & cos(\phi) & -sin(\phi) \\ 0 & sin(\phi) & cos(\phi)\end{bmatrix}
\\\
\\
R^P_B = \begin{bmatrix}cos(\psi)cos(\theta) & -sin(\psi)cos(\phi)+cos(\psi)sin(\theta)sin(\phi) & sin(\psi)sin(\phi)+cos(\psi)sin(\theta)cos(\psi) \\ sin(\psi)cos(\theta) & cos(\psi)cos(\phi)+sin(\psi)sin(\theta)sin(\psi) & -cos(\psi)sin(\phi)+sin(\psi)sin(\theta)cos(\psi) \\ -sin(\theta) & cos(\theta)sin(\phi) & cos(\theta)cos(\phi)\end{bmatrix}
$$
---

#### Verstehen des AC Servo Drivers

Im Datenblatt des Drivers sind mehrer Parameter beschrieben, die wir nutzen können um die Bewegungen zu berechnen:

| **Parameter**   | **Name / Bedeutung**                            | **Empfohlene Einstellung**                                      | **Hinweis**                                  |
| --------------- | ----------------------------------------------- | --------------------------------------------------------------- | -------------------------------------------- |
| `Pn002`         | Control Mode                                    | `2` = **Position Control Mode**                                 | Muss gesetzt sein                            |
| `Pn096`         | Command Pulse Input Mode                        | `0` = Pulse+Direction (häufig)                                  | Alternativen: `1` = CW/CCW, `2` = Quadratur  |
| `Pn097`         | Logic der Pulseingabe (Dir Invertierung)        | `0` = normal                                                    | `1` = invertierte Richtung                   |
| `Pn098`         | Elektronisches Getriebe – Zähler (Numerator)    | z. B. `1000`                                                    | Stellt Schrittauflösung ein                  |
| `Pn102`         | Elektronisches Getriebe – Nenner (Denominator)  | z. B. `1`                                                       | → `1000:1` bedeutet 1000 Pulse pro Umdrehung |
| `Pn117`         | Position command source selection               | `0` = Externe Pulsquelle                                        | Sonst: `1` = interne Positionstabelle        |
| `Pn132`         | Umschaltlogik Speed ↔ Position                  | `0` = Nur bei Stillstand möglich                                | Sicherheit                                   |
| `Pn052`–`Pn055` | SigIn 1–4 Funktion (z. B. Servo ON, Cmode etc.) | Typischerweise: `1=Servo ON`, `19=Cmode`, ...                   | Siehe unten                                  |
| `Pn070`         | Input Logic – Signal Pegel invertieren          | Default: `32691` (Binary: invertierte Logik auf manchen Inputs) | Kann relevant sein                           |

In unserer Diplomarbeit (siehe Seite 130) haben wir die grundlegenden Einstellungen des AC Servo Drivers beschrieben. Um zu verstehen, wie dieser funktioniert, müssen wir den Position Control Mode genauer betrachten. In diesem Modus sendet unser ESP32 Mikrocontroller zwei wichtige Signale an den Servo Driver: DIR (Direction) und PULS.

DIR (Direction): Gibt die Drehrichtung des Motors an und ist entweder auf HIGH oder LOW gesetzt.

PULS: Steuert die Anzahl der Schritte, die der Motor bei jedem Impuls dreht. Die Auflösung dieses Impulses hängt von zwei Parametern des Drivers ab: Pn098 (Molekül) und Pn102 (Nenner).

Berechnung der Auflösung
Die Auflösung des Motors wird durch das Verhältnis von 360° (Vollumdrehung) zur Anzahl der Pulses pro Umdrehung bestimmt. Wenn wir den Pn098 auf 1000 und den Pn102 auf 1 setzen, bedeutet das, dass der Motor 1000 Puls pro Umdrehung benötigt.

Die mathematische Berechnung lautet:

$$
\frac{360^\circ}{\frac{Pn098}{Pn102}} = \frac{360^\circ}{\frac{1000 Puls}{1}} = \frac{360^\circ}{1000 Puls} = 0,36 \, \text{Grad pro Puls}
$$

Das bedeutet, dass bei jedem Puls, den wir an den Motor senden, dieser sich um 0,36 Grad in die vorgegebene Richtung dreht.