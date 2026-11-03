# BeanNG

Online-Panzerduell mit Soft-Body-Physik im Stil von BeamNG, in C++20 mit SDL3 und OpenGL 3.3.

## Fahrzeuge

Vier Fahrzeuge stehen zur Wahl: Citroën Xantia Activa, Leopard 1, Koenigsegg One:1 und ein Geländewagen.

Das Spiel startet im Hauptmenü, **ESC** öffnet es jederzeit wieder. Dort findest du Fahrzeugwahl,
Fahrzeug-Einstellungen, Spiel-Einstellungen und die Tastenbelegung. Direkt erreichst du die Fahrzeugwahl
mit **TAB** und die Fahrzeug-Einstellungen mit **O**.

Maustaste). Beim Leopard zielt die Maus: der Turm folgt dem Fadenkreuz, der Ring zeigt, wohin das Rohr
gerade wirklich zeigt und wie weit das Nachladen ist.

Spiel-Einstellungen bleiben in `settings.cfg` gespeichert.

`tools/gen_xantia.py`, `tools/gen_leopard.py`, `tools/gen_one1.py` und `tools/gen_ui.py` erzeugen die Modelle und die
Schrift unter `assets/`. Das macht `./run.sh` von selbst (braucht numpy, scipy, shapely und Pillow).
Testläufe: `beanng-test buckle`, `tree`, `bomb`, `tank`, `tankcrash`, `tankkill`, `rocks`, `one1`, `esp`, `hummersteer`.

Am Spawn steht von jedem Modell eines. Sie schlafen, bis etwas sie trifft, und lassen sich anfahren und
beschießen; ein Wrack bleibt auch beim Fahrzeugwechsel eines.

Der Leopard ist nicht unzerstörbar: Ein direkter Treffer richtet etwa die Hälfte des Schadens an, der ihn
vernichtet, und eine Explosion nahe am Laufwerk wirft die Kette ab. Am Ende geht die Munition hoch und der
Turm fliegt davon. Harte Einschläge zählen ebenfalls. Der Zustand steht in der Anzeige unten links.

Felsbrocken zerbrechen in Explosionen zu kleineren Brocken, Mauersteine direkt daneben werden pulverisiert.

## Bauen und starten

    ./run.sh

Auf Laptops mit zwei GPUs nutzt das Spiel automatisch die dedizierte Karte. Mit `BEANNG_GPU=integrated ./run.sh` läuft es auf der integrierten.

Abhängigkeiten: SDL3, libepoxy, CMake, ein C++20-Compiler, Python 3 mit numpy, scipy und shapely.

## Online-Duell

Zwei Spieler auf einer eigenen Karte: je eine Basis im Westen und im Osten, dazwischen Dörfer, ein Fluss mit drei Brücken und zwei hohe Sperrmauern, über die nur die Artillerie schießen kann. Die Physik läuft allein auf dem Server, die Spieler schicken nur ihre Eingaben.

```sh
./run.sh --help                 # baut auch den Server
build/beanng-server 27015       # auf dem Rechner, den beide erreichen können
```

Im Spiel: **Online spielen**, Adresse und Port eintragen, **Verbinden**. Oder direkt mit `./run.sh --connect 1.2.3.4:27015`. Der Port ist UDP und muss von außen erreichbar sein. Sind beide da, wählt jeder Turm und Ausrüstung (20 Slots, 60 Sekunden Zeit). Wessen Panzer zerstört wird, verliert; nach 8 Sekunden geht es von vorn los.

| Turm | Waffe | Besonderheit |
|---|---|---|
| 105-mm-Kanone | 8 Granaten | fünf Volltreffer bis zur Zerstörung |
| Leichte Kanone | 120 Schuss, schnell | leicht, wenig Schaden pro Schuss |
| Artillerie | 8 Granaten, Steilfeuer | schießt auf den Punkt auf der Karte, schwer und langsam |
| Lenkraketen | 8 Raketen und ein schwaches Geschütz | Rakete wird mit der Maus über die ganze Karte geflogen |

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
| Tretmine, Panzermine legen | M, N | – |
| Bombencode auf den markierten Kartenpunkt: vier Raketen, ungenau | K, Klick, B | – |
| Scharfschützengewehr aufbauen, Zielfernrohr, Zoom 6x bis 24x | X, rechte Maustaste, Mausrad | – |
| Chat | Enter | – |
| Screenshot | F12 | – |

## Aufbau

- `src/sim.*`: Node-Beam-Solver mit 2 kHz, elastoplastische Beams, Kollisionen, Schlafzustand
- `src/vehicle.*`: Reifen, Antrieb, ABS, Hydropneumatik, Hydractive, Activa, Schadenslogik
- `src/xantia.cpp`: Struktur, Fahrwerk und Massen des Xantia
- `tools/gen_xantia.py`: erzeugt das 3D-Modell (`assets/xantia.mesh`, braucht numpy, scipy, shapely)
- `src/carmesh.*`: bindet das Modell an die Physik-Nodes
- `src/props.*`: zerstörbare Objekte, `src/world.*`: Gelände, Straßen, statische Objekte
- `src/render.*`, `src/main.cpp`: Grafik, Eingabe, Kameras, Physik-Thread
- `src/tests.cpp`: Prüfstand ohne Grafik

## Prüfstand

    ./build/beanng-test                    # alles
    ./build/beanng-test crash skidpad      # einzelne Tests

Einzelne Sondertests: `split`, `torn`, `slow`, `winding`. `BEANNG_PROFILE=1 ./run.sh` zeigt die Zeit pro Bild.

Tests: `statics bounce accel brake skidpad crash rough jitter response compliance wheel powertrain rest props perf`

Die Fahrzeugdaten sind angenäherte Werte, keine Herstellerdaten.
