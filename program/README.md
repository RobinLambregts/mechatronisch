# Bier Inkap Robot

## Bestandsstructuur

```
bier_robot/
├── main.py          ← Startpunt, vul-algoritme, UI
├── imus.py          ← IMU uitlezen (complementary filter)
├── calibration.py   ← Kalibratieprocedure
├── motors.py        ← GPIO, PWM, PI-regelaar
├── camera.py        ← Schuimdetectie via OpenCV
└── input.py         ← Terminal commando-verwerking
```

## Starten

```bash
cd bier_robot
python3 main.py
```

## Commando's (in terminal)

| Commando | Actie |
|---|---|
| `start` | Begin automatisch inkappen |
| `stop` | Stop alles onmiddellijk |
| `hoek` | Toon huidige IMU-hoeken |
| `calibrate` | Herstart kalibratieprocedure |
| `M2:45 M3:-70` | Handmatige hoekdoelen per motor |
| `30` | Beide motoren naar 30° |
| Enter (leeg) | Annuleer actieve doelen |
| `exit` / `q` | Programma afsluiten |

## Motorindeling

| Motor | Functie | IMU | Hoekbereik |
|---|---|---|---|
| Motor 1 | Draaien flesje (handmatig) | — | vrij |
| Motor 2 | Kantelen flesje | MPU1 (0x68) | 0° … 50° |
| Motor 3 | Voor/achteruit glas | MPU2 (0x69) | -90° … -40° |

## Kalibratiepositie

Bij opstarten (of `calibrate`):
- **Flesje horizontaal** → Motor 2 = 0°
- **Glas verticaal** → Motor 3 = -90°

## Schuimregeling

De camera analyseert continu de verhouding schuim/bier:
- **Schuim < 15%** → glas rechter (meer schuim)
- **Schuim > 25%** → glas schuiner (minder schuim)
- **Overflow risico** → flesje terug, stop

## Camera instellen

In `camera.py` pas je aan:
- `CAMERA_INDEX` — welke /dev/video je gebruikt
- `ROI` — het venster (x, y, breedte, hoogte) in pixels dat het glas omvat
- `BIER_HSV_LAAG/HOOG` — kleurdrempels voor bier
- `SCHUIM_HSV_LAAG/HOOG` — kleurdrempels voor schuim

Tip: gebruik `cv2.imshow` op de ROI tijdens testen om de drempelwaarden bij te stellen.

## Afhankelijkheden

```bash
pip3 install opencv-python numpy RPi.GPIO smbus
```