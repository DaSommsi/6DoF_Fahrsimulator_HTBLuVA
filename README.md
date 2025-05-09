# 🎮 6DoF Fahrsimulator – HTBLuVA Salzburg

Ein Schulprojekt zur Wiederinbetriebnahme und Weiterentwicklung eines sechsachsigen Fahrsimulators. Dieses System bietet durch die Bewegungsplattform mit sechs Motoren ein realistisches Fahrgefühl – ideal für Ausbildung, Forschung oder Präsentationen. Die ursprüngliche Software ging verloren, daher entsteht nun eine neue, robuste und wartbare Softwarelösung – mit modernem Entwicklungsworkflow, Versionskontrolle und detaillierter Dokumentation.

---

## 🚀 Projektziel

Das Hauptziel dieses Projekts ist es, den bestehenden Fahrsimulator der HTBLuVA Salzburg technisch und softwareseitig wieder funktionstüchtig zu machen. Neben dem Neuaufbau der Software liegt ein besonderer Fokus auf:

- der Behebung von elektromagnetischen Störungen (EMF),
- der Optimierung der Kommunikationswege zwischen Spiel und Steuerung,
- einer sauberen, dokumentierten Codebasis für zukünftige Erweiterungen,
- und einer funktionalen Dokumentation, die auch Dritten einen schnellen Einstieg ermöglicht.

Langfristig ist geplant, das System modular zu gestalten, sodass neue Funktionen wie VR-Unterstützung, Fahrdynamikanalyse oder Safety-Mechanismen leichter integriert werden können.

---

## 🕹️ Systemübersicht

Der Simulator ist mechanisch in der Lage, Bewegungen in allen sechs Freiheitsgraden (6DoF) auszuführen: Nick (Pitch), Gier (Yaw), Roll, sowie lineare Verschiebungen entlang der X-, Y- und Z-Achse. Die Bewegungen werden durch Fahrdaten aus einer Echtzeitsimulation ausgelöst und direkt auf den Sitz übertragen.

Das Erlebnis ist dadurch nicht nur optisch, sondern auch physisch immersiv – vergleichbar mit professionellen Fahrschul- oder Racing-Simulatoren.

---

## 🔩 Hardwarekomponenten

- **ESP32** – Zentrale Steuerungseinheit für die Plattformbewegung
- **Arduino (zusätzlich)** – Steuert das Gurtstraffsystem (z. B. bei Kollisionen)
- **Frequenzumrichter** – Dient zur präzisen Ansteuerung der Motoren
- **Eigenentwickelte PCBs** – Schalt- und Sicherheitslogik
- **6DoF Plattform** – Robuste mechanische Konstruktion mit sechs Motoren

Die Hardware wurde ursprünglich im Zuge eines Diplomprojekts aufgebaut. Unser aktuelles Ziel ist es, diese zu revitalisieren, Fehlerquellen zu minimieren und klar zu dokumentieren.

---

## 💻 Softwarearchitektur

- **Programmiersprachen:** C / C++
- **Entwicklungsumgebung:** Visual Studio Code mit PlatformIO
- **Versionskontrolle:** Git (GitHub-basiert)
- **Telemetrie-Integration:** SimTools v3 Pro, SimHub
- **Kommunikation:** USB-Verbindung zwischen Gaming-Laptop und ESP32

Das Spiel liefert über SimTools oder SimHub Echtzeitdaten (Geschwindigkeit, G-Kräfte, Position etc.), welche durch die Firmware auf dem ESP32 in Motorbefehle übersetzt werden. Die Regelung erfolgt möglichst deterministisch, um eine Verzögerung zwischen In-Game-Bewegung und Plattformreaktion zu minimieren.

---

## 🗂️ Projektstruktur

```plaintext
/src              → Hauptcode für ESP32 & Gurtstraffer
/documentation    → Detaillierte Projektdokumentation, development.md etc.
README.md         → Projektübersicht
development.md    → Entwicklungslogbuch mit Fortschrittsnotizen
