# Reinforcement Learning zur Steuerung des Cart-Pole-Systems
Yuxiang Su  1644682
Zhangtao Liu    1644679 
Guangqi Zhuang  1619946

## Inhaltsverzeichnis
- [Reinforcement Learning zur Steuerung des Cart-Pole-Systems](#reinforcement-learning-zur-steuerung-des-cart-pole-systems)
  - [Inhaltsverzeichnis](#inhaltsverzeichnis)
  - [Einleitung](#einleitung)
  - [Code-Repository](#code-repository)
  - [Modellierung und Analyse des Inverted-Pendulum-Systems](#modellierung-und-analyse-des-inverted-pendulum-systems)
    - [Systemstruktur des Inverted-Pendulum-Systems](#systemstruktur-des-inverted-pendulum-systems)
    - [Bauteilzeichnung mit Maße](#bauteilzeichnung-mit-maße)
    - [Änderung des Aufbaus und Bauteils](#änderung-des-aufbaus-und-bauteils)
      - [Führungsschienen](#führungsschienen)
      - [Pendel](#pendel)
      - [Sockel](#sockel)
  - [Schaltplan](#schaltplan)
  - [Motorsteuerung](#motorsteuerung)
  - [A4988 Schrittmotor-Treiber-Modul](#a4988-schrittmotor-treiber-modul)
  - [Winkellesen](#winkellesen)
  - [Datenübertragung zwischen Arduino und Laptop](#datenübertragung-zwischen-arduino-und-laptop)
  - [Agent und Kontrolle](#agent-und-kontrolle)
    - [Hyper-Parameter](#hyper-parameter)
    - [Aktualisierung der Q-Tabelle](#aktualisierung-der-q-tabelle)
  - [Verbleibende Probleme und Optimierungsvorschläge](#verbleibende-probleme-und-optimierungsvorschläge)
    - [Geräteoptimierung](#geräteoptimierung)
    - [Vortraining in einer Simulationsumgebung](#vortraining-in-einer-simulationsumgebung)
    - [Verzögerung durch Datenaustausch](#verzögerung-durch-datenaustausch)
    - [Mehr Actionen](#mehr-actionen)
    - [Wegsensor und Belohnungsfunktion](#wegsensor-und-belohnungsfunktion)
  - [Materiallist](#materiallist)
        - [NEUE Openbuilds HPV2 Mini V linear antrieb Effektive reise 200mm Linear modul mit NEMA17 stepper motor für Reprap 3D drucker](#neue-openbuilds-hpv2-mini-v-linear-antrieb-effektive-reise-200mm-linear-modul-mit-nema17-stepper-motor-für-reprap-3d-drucker)
          - [Acryl-Rohr 6mm ID x 8mm](#acryl-rohr-6mm-id-x-8mm)
          - [6-mm-Stift mit 6-mm-Winkelsteckkupplung](#6-mm-stift-mit-6-mm-winkelsteckkupplung)
          - [Acryl Buchstützen](#acryl-buchstützen)
          - [A4988 Schrittmotor-Treiber-Modul](#a4988-schrittmotor-treiber-modul-1)
          - [Präzisions-Potentiometer WDD35D4](#präzisions-potentiometer-wdd35d4)
          - [Arduino Nano](#arduino-nano)

## Einleitung
In diesem Projekt entwickeln wir ein Cart-Pole-System von Grund auf neu. Das System liest ständig den Zustand des Pols, trifft eine Entscheidung durch den Agenten und sendet dann Befehle an den Motor, der sich schließlich bewegt, um das Gleichgewicht des Pols zu erreichen. Das System besteht aus einem Motorsteuerungsmodul, einem Winkelmessmodul und einem Datenaustauschmodul, die in den folgenden Abschnitten einzeln vorgestellt werden.
Der aktuelle Zustand des Projekts sieht wie folgt aus.
![](./3.png)
## Code-Repository
https://github.com/pinz12/CartPole
## Modellierung und Analyse des Inverted-Pendulum-Systems
### Systemstruktur des Inverted-Pendulum-Systems
Das Hardware-Teil des Cart-Pole-Systems besteht hauptsächlich aus dem A4988 als Schrittmotor-Treiber-Modul, dem mechanischen Halterung und den Führungsschienen, dem Schrittmotor, dem Winkelsensor(Potentionsmeter), dem Pendelstab und dem Rotationsarm. 
Die Struktur ist in der Abbildung dargestellt.

![](./1.png)

Da die tatsächlich gekauften Teile etwas von unseren ursprünglichen Plänen für das Modell abweichen, werde wir im Folgenden erläutern, welche Änderungen wir beim tatsächlichen Bau vorgenommen haben, welche Probleme bei der Hardware auftraten und welche Lösungen es möglicherweise gibt.

### Bauteilzeichnung mit Maße
![](./2.png)

### Änderung des Aufbaus und Bauteils
#### Führungsschienen
In Bezug auf die Führungsschiene, da wir kein Werkzeug zum direkten Schneiden von Struktur-Aluminium hatten, haben wir den Schrittmotor oberhalb der Führungsschiene platziert. Für die Kraftübertragung haben wir uns weiterhin dafür entschieden, eine Nocke am Motor anzubringen, die über einen Riemen die Plattform nach links und rechts bewegt. Der effektive Verfahrweg der gleitenden Führungsschiene beträgt ungefähr 400 mm.
![](./3.png)

#### Pendel
Bezüglich des Pendelarms, da dieser mit dem Winkelsensor verbunden werden musste, haben wir einen 90-Grad-Kupplung verwendet, um den Pendelarm mit dem Winkelsensor zu verbinden, anstatt wie im Modell einen integrierten Pendelarm zu verwenden. Die Verbindung wurde mit Klebstoff hergestellt.
![](./4.png)

#### Sockel
Bezüglich des Sockels haben wir, um das Gleichgewicht des Schwerpunkts zu gewährleisten, entschieden, unter den ursprünglich zwei kleinen Holzsockeln eine größere und längere Platte anzubringen. Dadurch wird sichergestellt, dass der Schwerpunkt des gesamten Systems innerhalb des Tischrands bleibt und nicht durch das Schwingen während des Betriebs aus dem Gleichgewicht gerät.

## Schaltplan
![](./wire.png)
## Motorsteuerung
Als Aktuatoren verwenden wir Schrittmotoren. Der Antriebsmodus des Motors ist Vollschritt (Jeder Impuls bewirkt, dass der Motor einen Vollschritt dreht). Sie können ihn nach Bedarf auf 1/2 Schritt oder 1/4 Schritt einstellen. 
Der Ausgang des Mikrocontrollers sendet einen hohen Pegel und dann einen niedrigen Pegel, um einen Impuls zu erzeugen. Der Schrittmotor macht für jeden Impuls einen Schritt.
Der Motor benötigt eine Stromversorgung von 12 V. Wir verwenden acht 1,5-V-Batterien als Stromquelle.
Je höher die Anzahl der Impulse, desto größer der Drehwinkel des Motors. Je kürzer das Zeitintervall zwischen den Impulsen, desto schneller dreht sich der Motor.
Wir haben eine move()-Funktion definiert, mit der man den Motor einfach steuern kann.
```cpp
void move(int step, int delay, bool dir) {
    if (dir) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
    for (int i = 0; i < step; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(delay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(delay);
    }
}
```
Diese Funktion verfügt über drei Parameter: step, delay und dir.
Der Step-Parameter ist die Anzahl der Schritte, die der Motor machen soll, die Gesamtlänge der Führungsschiene beträgt ca. 850 Schritte (Vollschrittmodus). Sie können die entsprechende Anzahl der Schritte selbst einstellen.
Der Delay-Parameter ist das Zeitintervall des Schrittsignals, die Zeit, die jeder Impuls (1 Schritt) benötigt, ist $2×delay$. Je kleiner der Wert der Verzögerung ist, desto schneller dreht der Motor, der Wert der Verzögerung ist normalerweise 600-1000.
Der Dir-Parameter steuert die Drehrichtung des Motors.

Der gesamte Ablauf der Funktion move() ist wie folgt:
Wenn dirPin einen hohen Wert hat, bewegt der Motor den Wagen nach links, wenn dirPin einen niedrigen Wert hat, bewegt der Motor den Wagen nach rechts.
Dann wird eine Step-Schleife geöffnet und jedes Mal, wenn dieser Schleifenkörper ausgeführt wird, dreht sich der Motor um einen Schritt (1,8°).
In jeder Schleife wird zuerst ein High-Pegel auf den StepPin geschrieben, der um eine kleine Verzögerung verzögert ist, dann wird ein Low-Pegel geschrieben, der wiederum um eine kleine Verzögerung verzögert ist. Auf diese Weise wird ein Impuls mit dem Zeitintervall $2×delay$ erzeugt.

## A4988 Schrittmotor-Treiber-Modul
Wir verwenden den A4988 als Motortreiber.
![](A4988-Pin-Layout.jpg)
[Quelle Bild](https://dronebotworkshop.com/stepper-motor-hall-effect/)
STEP ist mit Pin 3 verbunden, um den Schrittbetrieb zu steuern. 
DIR ist mit Pin 2 verbunden, um die Richtung zu steuern. 
VMOT und GND sind die Stromversorgungspins des Motors, die mit der 12V-Batterie verbunden sind. 
1A, 1B, 2A, 2B entsprechen den vier Phasen des Schrittmotors, sie werden entsprechend ihrer Farben verbunden.
VGG und GND sind die Stromversorgungspins des Treiber Moduls, die mit dem 5V-Stromversorgungsanschluss des Arduino verbunden sind.

Es ist zu beachten, dass für jeden Schrittmotortyp das Treiber Modul mit dem Potentiometerknopf auf die entsprechende Vref-Referenzspannung eingestellt werden muss.
Die Vref-Referenzspannung kann gemessen werden, indem man den roten Stift des Multimeters mit dem Potentiometerknopf und den schwarzen Stift mit dem GND-Pin in der rechten unteren Ecke verbindet.
![](6.png)

Die Formel für Vref lautet $$V_{ref} = I_{max}×R_{cs}×8$$.
Das in diesem Projekt verwendete Treiber Modul hat ein Rcs von 0,1Ω und ein Imax von 1,5A, daher wurde Vref auf die gewünschten 1,2V eingestellt.
Wenn Sie das Treiber Modul oder den Motor wechseln, müssen Sie Vref neu berechnen und anpassen.
## Winkellesen
![](5.png)
Wir verwenden ein Potentiometer, um den Winkel und die Winkelgeschwindigkeit des Pendels zu messen.
Ein Potentiometer kann mit einem Schiebewiderstand verglichen werden. Es hat drei Pins, 1, 2 und 3. Wir verbinden Pin 1 mit dem GND-Pin des Arduino-Boards, Pin 3 mit dem 5V-Pin des Arduino-Boards und Pin 2 mit dem analogen Eingang des Arduino (wir haben Pin A0 gewählt), um eine Spannungsteilerschaltung aufzubauen. Wenn der Winkel gedreht wird, ändert sich die Spannung an Pin 2. Je größer der Widerstand ist, desto größer ist die Spannung an Pin 2.
Der Analogeingang des Potentiometers kann mit der Funktion analogRead() ausgelesen werden.
```cpp
int analogInputVal = analogRead(A0);
double a = (analogInputVal % 102) * (360.0 / 102.4) - 200;
```
Mit Hilfe des obigen Codes kann das gelesene Analogsignal einem Winkel im Bereich 0-360° zugeordnet werden.
Das Potentiometermodell ist Vishay Spectrol Precision Potentiometer 2406.
Es hat 10 Quantisierungsbits und 10 effektive Umdrehungen. Das bedeutet, dass dieses Potentiometer bei insgesamt zehn Umdrehungen je nach Winkel einen Wert zwischen 0 und 1023 ausgeben kann.
Die Genauigkeit des Potentiometers ist$\frac{360°×10}{2^{10}} = \ 3.515625°$

## Datenübertragung zwischen Arduino und Laptop
In unserem Projekt ist der Arduino nur für das Daten-Lesen von Staten und die Steuerung der Motoren zuständtig. Die Berechnungen zur Entscheidungsfindung für die Aktionen werden im Compuer mithilfe von Python-Skripten durchgeführt. Es gibt viele Probleme mit dieser Lösung (siehe den Abschnitt über Verzögerung durch Datenaustausch) und dieser Abschnitt bespricht nur den Ablauf.
Für die Datenübertragung haben wir die Serial-Bibliothek für Arduino verwendet. Alle Daten wurden Zeile für Zeile in Form von Strings übertragen.
Der Arduino verwendet `Serial.println()`, um Daten zu übertragen und `Serial.readStringUntil()`, um die Daten zu lesen.
Auf der Computerseite wird `serial.Serial()` verwendet, um die serielle Schnittstelle zu öffnen. Zum Beispiel bedeutet `ser = serial.Serial('COM5', 9600)`, dass wir eine serielle Schnittstelle am Anschluss COM5 öffnen und die Baudrate auf 9600 setzen, je höher die Baudrate, desto schneller die Datenübertragung.
Auf der Computerseite werden die Daten mit `ser.readline().decode(‚utf-8‘).rstrip()` gelesen und mit `ser.write()` übertragen.

## Agent und Kontrolle
Für die Entscheidungsfindung haben wir den Q-Learning-Algorithmus aus dem Reinforcement-Learning verwendet. Leider haben wir das Training nicht abgeschlossen (siehe den Abschnitt über Vortraining in einer Simulationsumgebung).
Wir verwendeten ein 2D-Array (in Python Liste genannt) mit 12.000 Zeilen und 2 Spalten, um die Q-Tabelle zu speichern. 12.000 dieser Zeilen entsprechen den 12.000 Zuständen des Systems. Wir lesen drei Zustände des Systems: Pendelwinkel, Pendelwinkelgeschwindigkeit und Wagenposition. Der Winkel nimmt Werte von 0° bis 360° an, was in 120 Intervallen diskretisiert ist, die Winkelgeschwindigkeit ist -100 bis 100, was in 10 Intervallen diskretisiert ist, und die Position des Wagens ist -400 bis 400, was in 10 Intervallen diskretisiert ist. Insgesamt gibt es also $120 × 10 × 10 = 12.000$ Zustände.
Mit der Funktion `state_to_index(state)` können Sie die Zustandsvariable in den entsprechenden Index in der Q-Tabelle umwandeln.
Die 2 Spalten der Q-Tabelle stehen für 2 Aktionen. Spalte 0 steht für 1 Schritt nach links und Spalte 1 für 1 Schritt nach rechts. Wenn eine Entscheidung getroffen wird, wird die Zeile, die dem aktuellen Zustand entspricht, abgefragt und die Spalte mit dem größeren Q-Wert wird ausgegeben. Wenn z. B. der Index eines Zustands 7000 ist und in der Zeile 7000 der Wert der Spalte 0 1,2 und der Wert der Spalte 1 0,3 ist, trifft der Agent eine Entscheidung und führt die Aktion 0 aus, d. h. einen Schritt nach links.
In der Funktion `move()` des Arduinos können Sie weitere Bewegungen hinzufügen oder die Bewegungsdistanz und Geschwindigkeit der aktuellen Bewegung nach Bedarf ändern.
### Hyper-Parameter
Wir definieren drei Hyperparameter, die da wären:

Alpha (α) - die Lernrate. α bestimmt, wie viele neue Informationen bei jeder Aktualisierung alte Informationen ersetzen. Für dieses Projekt wird der Wert 0,1 angenommen.

Gamma (γ) - Abzinsungsfaktor. γ bestimmt, wie wichtig zukünftige Belohnungen für die aktuelle Entscheidung sind, wenn das Q aktualisiert wird. Wenn γ nahe bei 1 liegt, konzentriert sich der Algorithmus mehr auf langfristige Gewinne; wenn γ nahe bei 0 liegt, konzentriert sich der Algorithmus mehr auf kurzfristige Gewinne. Für dieses Projekt wird der Wert 0,9 angenommen.

Epsilon (ε) - Explorationsrate. ε ist ein Wert zwischen 0 und 1. Eine zufällige Aktion wird mit einer Wahrscheinlichkeit von ε ausgewählt. Die aktuell als optimal betrachtete Aktion wird mit einer Wahrscheinlichkeit von 1-ε ausgewählt. Die Explorationsrate nimmt in der Regel im Laufe des Trainings ab, so dass der Algorithmus die Umgebung zu Beginn stärker erkunden kann, um potenziell bessere Strategien zu entdecken, und später mehr Gebrauch von den vorhandenen Erfahrungen machen kann. Für dieses Projekt wird 0,1 angenommen.
### Aktualisierung der Q-Tabelle
Nach der Ausführung einer von einem Agenten ausgelösten Aktion nimmt das System einmalig einen neuen Zustand ein. Die Q-Tabelle wird entsprechend den beiden Zuständen davor und danach aktualisiert.
```python
def update_Q_table(s, s_new, a):
    print("updating Q Table...")
    index = state_to_index(s)
    r = reward(s)
    Q_table[index,a] = Q_table[index,a] + ALPHA * (r + GAMMA * get_max_Q_value(s_new) -Q_table[index,a])
    print(f"The updated Q Value of Q_table[{index},{a}] is {Q_table[index,a]}")
```

## Verbleibende Probleme und Optimierungsvorschläge 
Unser aktueller Prototyp weist noch einige ungelöste Probleme auf. Im Folgenden werden das Problem und die Optimierungsvorschläge dargestellt.  

### Geräteoptimierung
Wir gehen davon aus, dass unser Prototyp Schwierigkeiten haben könnte, die Stange im Gleichgewicht zu halten, hauptsächlich aus zwei Gründen: Erstens ist die Genauigkeit des Winkelsensors relativ gering. Er kann nur Winkeländerungen von über 3.51 Grad erfassen, was zu einer unzureichenden Sensibilität der Regelung führen könnte. Zweitens ist die Dämpfung an der Rotationsachse relativ hoch, was das Erreichen eines stabilen Gleichgewichts zusätzlich erschweren könnte. Ohne den Winkelsensor oder die Führungsschiene auszutauschen, lautet unser Optimierungsvorschlag, die Länge der Stange zu verlängern oder das Gewicht am oberen Ende der Stange zu erhöhen. Dies trägt dazu bei, das durch die Abweichung der Stange von der Gleichgewichtsposition verursachte Drehmoment deutlicher zu machen und somit den Einfluss der Dämpfung relativ zu verringern. Eine längere Stange oder ein schwereres Gewicht erhöht auch das Trägheitsmoment. Das bedeutet, dass der Stange bei einer Abweichung langsamer kippt, wodurch die System mehr Zeit hat, Korrekturen vorzunehmen und das Gleichgewicht zu halten. Es ist darauf zu achten, dass das für das Gleichgewicht erforderliche Drehmoment nicht das maximale Drehmoment des Motors überschreitet. 

### Vortraining in einer Simulationsumgebung
Wir haben bereits die algorithmische Struktur für das Q-Learning entwickelt, aber beim Aufbau des virtuellen Modells sind wir auf Probleme gestoßen. Daher haben wir das Training nicht auf dem virtuellen Modell durchgeführt, sondern direkt auf dem realen Modell. Für Algorithmen des Reinforcement Learning ist es jedoch offensichtlich unklug, das Training auf dem realen Modell durchzuführen, da es nicht möglich ist, das reale Modell tausende Male ununterbrochen laufen zu lassen. Deshalb bleibt der Aufbau eines virtuellen Modells weiterhin notwendig.  

### Verzögerung durch Datenaustausch
In unserem Programm werden die von den Sensoren erzeugten Signale und die an den Motor gesendeten Signale nicht direkt vom Arduino-Chip verarbeitet, sondern müssen zusätzlich über den PC verarbeitet werden, was zu erheblichen Verzögerungen in der Verarbeitung führt. Um dieses Problem zu lösen, müssen wir das Q-Learning zunächst auf dem PC durchführen und das trainierte Modell(die Q-Tabelle) anschließend auf den Arduino-Chip hochladen. Der Arduino-Chip sollte dann ausschließlich die Signale empfangen, die Daten verarbeiten und den Motor steuern. 

### Mehr Actionen
In unserer aktuellen Q-Tabelle gibt es nur zwei Ausgabetypen, die jeweils das Bewegen nach links oder rechts mit einer vorgegebenen Geschwindigkeit über eine vorgegebene Distanz repräsentieren. Dies könnte während des tatsächlichen Trainings zu suboptimalen Ergebnissen führen. Wir könnten mehr Ausgabetypen hinzufügen, wie z. B. schnelles oder langsames Bewegen nach links, schnelles oder langsames Bewegen nach rechts, sowie Position halten. Dadurch hätte das System in verschiedenen Gleichgewichtssituationen unterschiedliche Anpassungsmöglichkeiten, was zur Steigerung der Trainingseffizienz beitragen könnte. 

### Wegsensor und Belohnungsfunktion 
Basierend auf unserem aktuellen Reinforcement-Learning-Algorithmus erwarten wir, dass es das Gleichgewicht halten kann, aber die Dauer wird nicht sehr lang sein, da es während des Gleichgewichtshaltens möglicherweise weiterhin nach links oder nach rechts bewegt, bis es das Ende der Führungsschiene erreicht. Wir müssen noch einen Belohnungskoeffizient so einstellen, dass es dazu neigt, in der Mitte der Schiene zu bleiben. Da unser Prototyp keinen Positionssensor besitzt, können wir die Position der Plattform, die die Stange trägt, nur anhand der Schrittsignale das Schrittmotors bestimmen. Dies erfordert, dass die Anfangsposition der Plattform vor jedem Systemlauf konsistent ist. Mit einem Positionssensor könnte dieses Problem gelöst werden. 
## Materiallist
##### [NEUE Openbuilds HPV2 Mini V linear antrieb Effektive reise 200mm Linear modul mit NEMA17 stepper motor für Reprap 3D drucker](https://de.aliexpress.com/item/1005004263776581.html?spm=a2g0o.detail.pcDetailTopMoreOtherSeller.2.5915ilscilscR3&gps-id=pcDetailTopMoreOtherSeller&scm=1007.40050.354490.0&scm_id=1007.40050.354490.0&scm-url=1007.40050.354490.0&pvid=4df1412a-357f-4eb9-8ca3-8188f0bf4881&_t=gps-id:pcDetailTopMoreOtherSeller,scm-url:1007.40050.354490.0,pvid:4df1412a-357f-4eb9-8ca3-8188f0bf4881,tpp_buckets:668%232846%238115%232000&pdp_npi=4%40dis%21EUR%2153.18%2137.22%21%21%21409.43%21286.60%21%40210388c917240080696511976eafea%2112000028564482549%21rec%21DE%214888794309%21XZ&utparam-url=scene%3ApcDetailTopMoreOtherSeller%7Cquery_from%3A)
###### [Acryl-Rohr 6mm ID x 8mm](https://www.amazon.de/-/en/sourcing-Clear-Rigid-Acrylic-305mm/dp/B09F6CNJ1W/ref=sr_1_7?adgrpid=77964784544&dib=eyJ2IjoiMSJ9.NznjAHTK0Lj42fq3xglxn9X6VGSxoKroX9Z4TEUURSadBsGBQHA0GY3ssK98c_hHHJ0b3Ay4wetLawvm8DRecVyZIJ6VISUj98ArpEF0Z2t3E5HegzDaRDxkN6_-kk1JubYW8zGsavCuLQbNL1svsplYJksEvgCQN_paNn6-Y5ha94xFIIBaNKUqSConODfAl11qC5H_dtyc7CyLt8YoFYoG3sNsaGtFGIGtTpfD1qoXYAWTR8yToI_Hpxw-rsVjr-cfXeVADjX-9i17fsn5gM5v4b1-nXbx3VeoOL7ez84.F_nAdXdJFZr4xBhTffSoIt9SCutoUy7hXVZ4nnWdw3w&dib_tag=se&hvadid=606633938289&hvdev=c&hvlocphy=9068126&hvnetw=g&hvqmt=e&hvrand=10159296915190359259&hvtargid=kwd-313511956759&hydadcr=18740_2293463&keywords=kunststoff+rohr+6mm&qid=1714949813&sr=8-7)
###### [6-mm-Stift mit 6-mm-Winkelsteckkupplung](https://www.hupenshop.de/6mm-stift-met-6mm-haakse-insteekkkoppeling-)
###### [Acryl Buchstützen](https://www.amazon.de/dp/B0CQR64X72/ref=sspa_dk_detail_6?psc=1&pf_rd_p=ae2317a0-2175-4285-af64-66539858231f&pf_rd_r=AT3Q9MT2ZJZDG0SZA3YQ&pd_rd_wg=8ahhU&pd_rd_w=aCGBk&content-id=amzn1.sym.ae2317a0-2175-4285-af64-66539858231f&pd_rd_r=bb410982-5df8-4c45-a179-dd8b02d3f457&s=kitchen&sp_csd=d2lkZ2V0TmFtZT1zcF9kZXRhaWw)
###### [A4988 Schrittmotor-Treiber-Modul](https://www.amazon.de/AZDelivery-Schritt-komplett-Stiftleisten-K%C3%BChlk%C3%B6rper/dp/B01N9QOJ99/ref=sr_1_3?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=3IKDEL774F3YG&dib=eyJ2IjoiMSJ9.4Y2E44LXoFY_nlLSuqQjAz7stuDfUGY3NDzK44VvcD-xWOUzq4AsP87tNdCR69-KU4nPltPSQ2g51_7l90VEnQe8USB_vKJEZrtO_Yd4ylywFYfo4iP4MhhbUEepmrlUMd7Vg5gVmGVMDVYZxOQKstbNSjrcYGJYtny6MPBo-BnyDjWdujXURf5W66iBB35pNu73CiXpFTbOycFXo9mAicAynaAEDl2xDGyCBPzVEMhZnHIlBnsPSq9dqyrJHFBEH_jKQ7znI8QfobkmgpXE8EeQnGl2uWjRp75J33E8o50.ZqEJgT-Iv4KYrjnf611mzExpn37e1BG1r8GW7ckz_80&dib_tag=se&keywords=Chip%2BA4988&qid=1715000689&s=industrial&sprefix=chip%2Ba4988%2Cindustrial%2C108&sr=1-3&th=1)
###### Präzisions-Potentiometer WDD35D4
###### Arduino Nano