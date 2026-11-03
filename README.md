# BeanNG

Online-Panzerduell mit Soft-Body-Physik im Stil von BeamNG, in C++20 mit SDL3 und OpenGL 3.3.

![Die Basis: alle Panzer und Autos, dahinter die Kommandozentrale](docs/screenshots/basis.jpg)

Zwei Spieler, eine große Karte, und alles, was sich bewegt oder kaputtgeht, ist ein Soft Body aus Knoten und Balken bei 2000 Physikschritten pro Sekunde. Du startest zu Fuß mit Pistole und Messer und entscheidest selbst, wie du den Gegner erwischst: mit dem Panzer, dem Auto, aus 800 m durchs Zielfernrohr oder per Lenkrakete quer über die Karte.

| | |
|---|---|
| ![Fluss mit Brücke](docs/screenshots/bruecke.jpg) | ![Ein Haus im Dorf](docs/screenshots/haus.jpg) |
| ![Liegend hinter dem Scharfschützengewehr](docs/screenshots/liegend.jpg) | ![Durchs Zielfernrohr: Einschlagmarke und Wind](docs/screenshots/zielfernrohr.jpg) |
| ![Eine Lenkrakete im Flug](docs/screenshots/lenkrakete.jpg) | ![Die Karte mit markiertem Ziel](docs/screenshots/karte.jpg) |
| ![Am Hummer: E steigt ein, R repariert](docs/screenshots/hummer.jpg) | ![Wer stirbt, fällt](docs/screenshots/gefallen.jpg) |

## Bauen und starten

    ./run.sh

Auf Laptops mit zwei GPUs nutzt das Spiel automatisch die dedizierte Karte. Mit `BEANNG_GPU=integrated ./run.sh` läuft es auf der integrierten.

Abhängigkeiten: SDL3, libepoxy, CMake, ein C++20-Compiler, Python 3 mit numpy, scipy und shapely.

## Online-Duell

Zwei Spieler auf einer eigenen Karte: je eine Basis im Westen und im Osten, dazwischen Dörfer, Hügel, Wälder, ein Fluss mit drei Brücken und zwei hohe Sperrmauern, über die nur die Artillerie schießen kann. Die Physik läuft allein auf dem Server, die Spieler schicken nur ihre Eingaben.

```sh
./run.sh --help                 # baut auch den Server
build/beanng-server 27015       # auf dem Rechner, den beide erreichen können
```

Im Spiel: **Online spielen**, Benutzername, Adresse und Port eintragen, **Verbinden**. Oder direkt mit `./run.sh --connect 1.2.3.4:27015`. Der Port ist UDP und muss von außen erreichbar sein.

**Ablauf:** Sind beide da, wählt jeder seine Ausrüstung (20 Slots, 60 Sekunden): Tretmine 2 Slots, Panzermine 5, Reparaturset 10, Sturmgewehr 2, 60 Schuss Kleinmunition 1. Du startest zu Fuß mit Pistole und Messer in deiner Basis. Wer stirbt, verliert; nach 8 Sekunden geht es von vorn los. Ausrüstung und Beute gelten für ein Match.

**Karte:** 2 km groß, ringsum Berge. Dazwischen Dörfer, Hügel, Wälder, Hecken, Erdwälle, Felsgruppen, alte Panzerwracks, ein Fluss mit drei Brücken zwei flache Furten und zwei 10 m hohe Sperrmauern, über die nur die Artillerie schießt.

![Die Karte von oben](docs/screenshots/karte-von-oben.jpg)

**Fahrzeuge:** In jeder Basis stehen alle vier Panzer sowie Xantia Activa, Koenigsegg One:1 und Hummer H1. Einsteigen kannst du in jedes freie Fahrzeug, auch beim Gegner. Reparieren geht nur von außen mit einem Set.

![Alle vier Türme](docs/screenshots/panzer.jpg)

| Turm | Waffe | Besonderheit |
|---|---|---|
| 105-mm-Kanone | 8 Granaten | fünf Volltreffer bis zur Zerstörung |
| Leichte Kanone | 120 Schuss, schnell | leicht, wenig Schaden pro Schuss |
| Artillerie | 8 Granaten, Steilfeuer auf einen Kartenpunkt | ein Treffer zerstört jeden Panzer und reißt einen großen Krater; schwer und langsam |
| Lenkraketen | 8 Raketen und ein schwaches Geschütz | Rakete wird mit der Maus über die ganze Karte geflogen |

**Schaden am Panzer:** Motor, Kanone, Turmkranz, Munition und Ketten nehmen einzeln Schaden. Schwachstellen wie Optik, Entfernungsmesser, Winkelspiegel, Kühler, Treibstoffleitung, Höhenrichtanlage und Antenne erreicht auch das Scharfschützengewehr, und jede nimmt dem Panzer etwas anderes.

**Scharfschützengewehr:** drei Sekunden Aufbau im Liegen, Zoom 6x bis 24x, Geschossabfall, Wind und Flugzeit. Auf 1000 m sind das gut eine Sekunde und 5,5 m Abfall. Im Auto trifft es, was im Weg liegt: Tank, Ölwanne, Motor, Kühler, Getriebe, Reifen oder den Fahrer.

**Finden:** In Häusern und bei Wracks stehen Kisten mit Munition, Scharfschützengewehr, Raketenwerfer, Reparatursets, Minen, Verbandskästen und Bombencodes. Ein Code holt vier ungenaue Raketen auf einen Kartenpunkt.

**Computer:** Im Kommandozentrum des Gegners lässt sich der Hauptcomputer in 5 Sekunden anzapfen. Danach zeigt die Karte alle 30 Sekunden auf Anfrage ungefähr, wo er ist.

**Serverkonsole:** Text ohne `/` geht in den Chat. Dazu `/give <spieler> <sache> [anzahl]`, `/kill`, `/kick`, `/players`, `/restart`, `/wind`, `/help` und `/quit`. TAB ergänzt Befehle, Spieler und Sachen. `--give-all` gibt beiden von Anfang an alle Waffen, zum Ausprobieren.

## Steuerung

| Aktion | Tastatur/Maus | Controller |
|---|---|---|
| Gas / Bremse, im Stand rückwärts | W / S | RT / LT |
| Lenken | A / D | linker Stick |
| Bremse | Leertaste | A |
| Zielen, Kamera | Maus, C, Mausrad | rechter Stick, Y |
| Feuer | linke Maustaste, F | – |
| Rakete starten, Feuer sprengt sie | G | – |
| Einsteigen, aussteigen | E | – |
| Rennen, springen | Umschalt, Leertaste | – |
| Reparaturset, zu Fuß neben dem Fahrzeug | R | – |
| Karte: Klick markiert das Ziel | K | – |
| Waffe wählen | 1 bis 5 | – |
| Anvisieren, auch im Panzer | rechte Maustaste halten | – |
| Tretmine, Panzermine legen | M, N | – |
| Bombencode auf den markierten Kartenpunkt: vier Raketen, ungenau | K, Klick, B | – |
| Scharfschützengewehr aufbauen, Zielfernrohr, Zoom 6x bis 24x | X, rechte Maustaste, Mausrad | – |
| Beim Gegner anzapfen, dann Standort abfragen (alle 30 s) | E am Computer, J | – |
| Visier auf Entfernung einstellen | Pfeil hoch, runter | – |
| Chat | Enter | – |
| Screenshot | F12 | – |

## Aufbau

- `src/duel.*`: ein Match ohne Grafik: Karte, Fahrzeuge, Spieler, Waffen, Minen, Loot, Ragdolls
- `src/server.cpp`: der Server mit Konsole; `src/net.*`, `src/duelnet.h`: die Pakete über UDP
- `src/main.cpp`, `src/render.*`, `src/audio.*`: der Spieler-Client: Netz, Eingabe, Kamera, Grafik und Ton
- `src/sim.*`: Node-Beam-Solver mit 2 kHz, elastoplastische Beams, Kollisionen, Schlafzustand
- `src/tank.*`: Panzer, Türme, Module und Schwachstellen; `src/vehicle.*` mit `xantia`, `one1`, `hummer`: die Autos
- `src/props.*`: Häuser, Bäume und anderes Zerstörbare; `src/world.*`: Karte, Gelände, Straßen, Fluss, Mauern
- `src/carmesh.*`: bindet die Modelle an die Physik-Nodes; `tools/gen_*.py` erzeugt sie (`./run.sh` macht das von selbst)
- `src/tests.cpp`: Prüfstand ohne Grafik, auf der alten Teststrecke

## Prüfstand

    ./build/beanng-test                    # alles
    ./build/beanng-test crash skidpad      # einzelne Tests

Einzelne Sondertests: `split`, `torn`, `slow`, `winding`. `BEANNG_PROFILE=1 ./run.sh` zeigt die Zeit pro Bild.

Tests: `statics bounce accel brake skidpad crash rough jitter response compliance wheel powertrain rest props perf`

Die Fahrzeugdaten sind angenäherte Werte, keine Herstellerdaten.
