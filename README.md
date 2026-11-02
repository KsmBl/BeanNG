# BeanNG

Soft-Body-Fahrsimulation im Stil von BeamNG, in C++20 mit SDL3 und OpenGL 3.3.
Fahrzeug: Citroën Xantia Activa (3.0 V6).

## Bauen und starten

    ./run.sh

Auf Laptops mit zwei GPUs nutzt das Spiel automatisch die dedizierte Karte. Mit `BEANNG_GPU=integrated ./run.sh` läuft es auf der integrierten.

Abhängigkeiten: SDL3, libepoxy, CMake, ein C++20-Compiler, Python 3 mit numpy, scipy und shapely.

## Steuerung

| Aktion | Tastatur/Maus | Controller |
|---|---|---|
| Gas / Bremse | W / S oder Pfeiltasten | RT / LT |
| Lenken | A / D oder Pfeiltasten, M: Mauslenkung | linker Stick |
| Rückwärts (Automatik) | S im Stand halten | LT im Stand halten |
| Handbremse | Leertaste | A |
| Gang runter / hoch | Q / E | LB / RB |
| Automatik / manuell | T | – |
| Hydractive Sport | G | X |
| Niveau | H | Steuerkreuz hoch |
| Activa an/aus | V | Steuerkreuz runter |
| Zündung | I | – |
| Kamera | C, rechte Maustaste ziehen, Mausrad | Y, rechter Stick |
| Auto packen | linke Maustaste ziehen | – |
| Reparieren / Neustart | F5 / R | B / Back |
| Pause / Zeitlupe | P / `[` `]` | Start / Steuerkreuz links, rechts |
| Beam-Ansicht, Röntgen-Ansicht, Hilfe, Screenshot | F2, F3, F1, F12 | – |

## Teststrecke

Du startest bei x = -90 und blickst auf die Hindernisspur: Schilder, Holzzaun, Pylonen,
Steinmauer, Leitplanke, Betonblöcke, Endwand. Daneben liegen Crashwand (Spur y = -30),
Sprungschanze (y = 40, ab etwa 80 km/h), Elchtest-Gasse (y = -60), Kopfsteinpflaster, Waschbrett
und Bordsteine (y = 70 bis 100) sowie die Landstraße rund um die Fläche.

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
