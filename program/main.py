"""
main.py - Bier inkap robot
==========================
Start: python3 main.py

Workflow:
  1. Init IMUs + motoren + camera
  2. Automatische kalibratie (flesje 45°, glas -90°)
  3. Wacht op "start" commando in terminal
  4. Vul-routine:
       a. Flesje (Motor 2) naar beginstand FLESJE_VULDOEL
       b. Gierlus:
            • Camera ziet GEEN verandering in schuim/bier
              → flesje zachtjes verder kantelen (FLESJE_STAP per cyclus)
            • Camera ziet WEL verandering
              → flesje stilhouden, glas bijsturen op foam_ratio:
                  - te veel schuim  → glas schuiner  (richting -90°)
                  - te weinig schuim → glas rechter  (richting -40°)
                  - overflow         → flesje terug, stop
       c. Flesje leeg (bier_hoogte_px < drempel) → eindfase
       d. Flesje terug, glas rechtop
  5. UI venster toont live status
"""

import time
import threading
import cv2
import numpy as np

import imus
import motors
import camera
import input as inp
from calibration import voer_kalibratie_uit

# ==========================================
# Inkap-algoritme parameters
# ==========================================
FLESJE_BEGIN     = 30.0    # beginstand flesje bij start gieten
FLESJE_MAX       = 50.0    # maximale kantelhoek flesje
FLESJE_START     = 0.0     # parkeerstand flesje (na afloop)
FLESJE_STAP      = 1.5     # graden per stap als bier stabiel is

GLAS_START       = -90.0   # startpositie glas (schuin)
GLAS_RECHTOP     = -45.0   # eindpositie glas (bijna rechtop)
GLAS_STAP_GROOT  = 3.0     # bijstap te veel schuim
GLAS_STAP_KLEIN  = 1.5     # bijstap te weinig schuim

# Glas grenzen
GLAS_MIN = -90.0
GLAS_MAX = -40.0

# Tijd tussen camera-checks tijdens gieten
CAMERA_CHECK_INTERVAL = 0.3   # seconden

# Flesje leeg: bierhoogte onder dit aantal pixels gedurende N checks
LEEG_BIER_PX    = 10
LEEG_TELLER_MAX = 10

# ==========================================
# Globale vlaggen
# ==========================================
_vul_bezig  = False
_stop_vlag  = threading.Event()
_stop_prog  = False


# ==========================================
# Inkap routine
# ==========================================
def vul_routine():
    global _vul_bezig
    if _vul_bezig:
        print("[Vul] Al bezig met inkappen.")
        return

    _vul_bezig = True
    _stop_vlag.clear()

    print("\n" + "="*55)
    print(" INKAPPEN GESTART")
    print("="*55)

    # Huidige glashoek als startpunt
    huidige_glas_hoek   = imus.get_angle(imus.MPU2_ADDR) or GLAS_START
    huidige_flesje_hoek = FLESJE_BEGIN
    leeg_teller         = 0

    # ---- Stap 1: flesje naar beginstand ----
    print(f"[1/4] Flesje naar beginstand {FLESJE_BEGIN}°…")
    motors.stel_doel_in(2, FLESJE_BEGIN)
    if not motors.wacht_op_doel(2, timeout=15):
        print("  !! Flesje timeout — doorgaan…")

    if _stop_vlag.is_set():
        _einde_vul()
        return

    # ---- Stap 2: gierlus ----
    print("[2/4] Gierlus gestart (camera stuurt bij)…")

    while not _stop_vlag.is_set():
        cam   = camera.get_camera_data()
        actie = camera.schuim_actie()
        stabiel = camera.inhoud_stabiel()

        schuim_h = cam.get('schuim_hoogte_px', 0)
        bier_h   = cam.get('bier_hoogte_px', 0)
        foam_pct = round(cam['foam_ratio'] * 100, 1)

        print(
            f"  schuim:{foam_pct}%  S:{schuim_h}px B:{bier_h}px  "
            f"{'STABIEL→flesje' if stabiel else 'verandering→glas'}  "
            f"actie:{actie}  glas:{imus.get_angle(imus.MPU2_ADDR)}°  "
            f"flesje:{imus.get_angle(imus.MPU1_ADDR)}°",
            end='\r'
        )

        # -- Overflow: meteen stoppen --
        if actie == 'overflow':
            print("\n  !! OVERFLOW RISICO — flesje terug!")
            motors.stel_doel_in(2, FLESJE_START)
            break

        if not cam['geldig']:
            # Geen beeld: flesje stilhouden, wachten
            time.sleep(CAMERA_CHECK_INTERVAL)
            continue

        if stabiel:
            # Inhoud verandert niet → flesje verder kantelen
            huidige_flesje_hoek = min(huidige_flesje_hoek + FLESJE_STAP, FLESJE_MAX)
            motors.stel_doel_in(2, huidige_flesje_hoek)

        else:
            # Inhoud verandert → flesje stilhouden, glas bijsturen
            # Annuleer actief flesje-doel zodat motor stopt op huidige positie
            with motors.doel_lock:
                motors.motor_doel[2]['actief'] = False

            if actie == 'meer_schuim':
                huidige_glas_hoek = max(huidige_glas_hoek - GLAS_STAP_GROOT, GLAS_MIN)
                motors.stel_doel_in(3, huidige_glas_hoek)

            elif actie == 'minder_schuim':
                huidige_glas_hoek = min(huidige_glas_hoek + GLAS_STAP_KLEIN, GLAS_MAX)
                motors.stel_doel_in(3, huidige_glas_hoek)

        # -- Flesje leeg detectie --
        if cam['geldig'] and bier_h < LEEG_BIER_PX:
            leeg_teller += 1
            if leeg_teller >= LEEG_TELLER_MAX:
                print("\n  Flesje lijkt leeg — gieten stoppen.")
                break
        else:
            leeg_teller = max(0, leeg_teller - 1)

        time.sleep(CAMERA_CHECK_INTERVAL)

    if _stop_vlag.is_set():
        _einde_vul()
        return

    # ---- Stap 3: flesje terugplaatsen ----
    print(f"\n[3/4] Flesje terug naar {FLESJE_START}°…")
    motors.stel_doel_in(2, FLESJE_START)
    motors.wacht_op_doel(2, timeout=15)

    # ---- Stap 4: glas rechtop ----
    print(f"[4/4] Glas rechtop naar {GLAS_RECHTOP}°…")
    motors.stel_doel_in(3, GLAS_RECHTOP)
    motors.wacht_op_doel(3, timeout=20)

    print("\n✓ Inkappen klaar! Geniet van uw pint.")
    print("="*55 + "\n")
    _vul_bezig = False   # ← bug fix: was ontbrekend na normale afronding


def _einde_vul():
    global _vul_bezig
    motors.annuleer_alle_doelen()
    print("\n[Vul] Gestopt door gebruiker.")
    _vul_bezig = False


def stop_alles():
    _stop_vlag.set()
    motors.annuleer_alle_doelen()


# ==========================================
# CV2 UI
# ==========================================
def teken_ui(glas_hoek, flesje_hoek, cam_data):
    frame = np.zeros((400, 580, 3), dtype=np.uint8)

    def t(txt, y, kleur=(220, 220, 220), schaal=0.52):
        cv2.putText(frame, txt, (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, schaal, kleur, 1, cv2.LINE_AA)

    # Titel
    cv2.rectangle(frame, (0, 0), (580, 40), (30, 30, 30), -1)
    cv2.putText(frame, "BIER INKAP ROBOT", (140, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 190, 255), 2, cv2.LINE_AA)

    # IMU
    t(f"Flesje (Motor 2): {flesje_hoek}°  [0° … 50°]",   70, (180, 230, 255))
    t(f"Glas   (Motor 3): {glas_hoek}°  [-90° … -40°]",  100, (180, 230, 255))

    # Camera zones
    if cam_data['geldig']:
        fp    = round(cam_data['foam_ratio'] * 100, 1)
        ov    = "JA" if cam_data['overflow_risk'] else "nee"
        sh    = cam_data.get('schuim_hoogte_px', 0)
        bh    = cam_data.get('bier_hoogte_px', 0)
        sv    = cam_data.get('schuim_veranderd', False)
        bv    = cam_data.get('bier_veranderd', False)
        stabiel = not sv and not bv

        kleur_fp = (0, 255, 100) if 15 <= fp <= 25 else (0, 120, 255)
        t(f"Schuim: {fp}%  (ideaal 15-25%)   S:{sh}px  B:{bh}px", 140, kleur_fp)
        t(f"Overflow: {ov}", 168,
          (0, 60, 255) if cam_data['overflow_risk'] else (160, 160, 160))
        status_cam = "STABIEL → flesje kantelt" if stabiel else "VERANDERING → glas stuurt bij"
        t(status_cam, 196, (0, 220, 140) if stabiel else (0, 160, 255))
    else:
        t("Camera: geen geldig beeld", 140, (80, 80, 80))

    # Inkap status
    status_txt   = "Inkappen BEZIG" if _vul_bezig else "Wacht op 'start'"
    status_kleur = (0, 255, 160) if _vul_bezig else (120, 120, 120)
    t(status_txt, 230, status_kleur, schaal=0.6)

    # Ingebedde camera ROI
    roi = cam_data.get('roi_frame')
    if roi is not None:
        try:
            roi_small = cv2.resize(roi, (160, 210))
            frame[155:365, 395:555] = roi_small
            cv2.rectangle(frame, (395, 155), (555, 365), (60, 60, 60), 1)
            t("Camera ROI", 380, (80, 80, 80))
        except Exception:
            pass

    # Footer
    t("Terminal: start | stop | hoek | calibrate | exit", 380, (100, 100, 100), schaal=0.44)

    return frame


# ==========================================
# MAIN
# ==========================================
def main():
    global _stop_prog

    print("=" * 55)
    print(" BIER INKAP ROBOT — opstarten")
    print("=" * 55)

    # 1. Init
    print("[1/4] IMUs initialiseren…")
    imus.init_all()

    print("[2/4] Motoren initialiseren…")
    motors.init_motoren()
    motor_thread = motors.start_motor_thread()

    print("[3/4] Camera starten…")
    camera.start_camera()

    # 2. Kalibratie
    print("[4/4] Automatische kalibratie starten…")
    voer_kalibratie_uit(motors.pwm_motoren, motors.motor_doel, motors.doel_lock)

    # 3. Input thread
    inp.registreer_callbacks(vul_routine, stop_alles)
    input_thread = inp.start_input_thread()

    # 4. UI loop
    cv2.namedWindow("Bier Robot", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Bier Robot", 580, 400)

    print("\nSysteem klaar. Typ 'start' in de terminal om te beginnen.")
    print("Sluit af via 'exit' in terminal of ESC in het venster.\n")

    try:
        while not _stop_prog:
            g_hoek = imus.get_angle(imus.MPU2_ADDR)
            f_hoek = imus.get_angle(imus.MPU1_ADDR)
            cam    = camera.get_camera_data()

            frame = teken_ui(g_hoek, f_hoek, cam)
            cv2.imshow("Bier Robot", frame)

            key = cv2.waitKey(100) & 0xFF
            if key == 27:   # ESC
                break

            if not input_thread.is_alive():
                break

    except KeyboardInterrupt:
        print("\nOnderbroken door gebruiker.")

    finally:
        print("\nAfsluiten…")
        _stop_prog = True
        stop_alles()
        camera.stop_camera()
        time.sleep(0.3)
        motors.motor_systeem_actief = False
        motor_thread.join(timeout=1.5)
        cv2.destroyAllWindows()
        motors.cleanup_motoren()
        print("Klaar. Tot de volgende pint!")


if __name__ == '__main__':
    main()