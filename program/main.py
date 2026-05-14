"""
main.py - Bier inkap robot (MANUEEL FLESJE)
==========================================
In deze versie:
- Robot stuurt alleen het GLAS (Motor 3).
- Gebruiker kapt zelf het flesje in.
- Camera-feedback past nog steeds de glashoek aan.
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
# Flesje parameters (niet meer in gebruik voor motor, enkel referentie voor UI)
FLESJE_START     = 0.0 

GLAS_START       = -90.0   # startpositie glas (schuin)
GLAS_RECHTOP     = -45.0   # eindpositie glas (bijna rechtop)
GLAS_STAP_GROOT  = 3.0     # bijstap te veel schuim
GLAS_STAP_KLEIN  = 1.5     # bijstap te weinig schuim

# Glas grenzen
GLAS_MIN = -90.0
GLAS_MAX = -40.0

# Tijd tussen camera-checks tijdens gieten
CAMERA_CHECK_INTERVAL = 0.3 

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
# Inkap routine (Handmatig flesje)
# ==========================================
def vul_routine():
    global _vul_bezig
    if _vul_bezig:
        print("[Vul] Al bezig met inkappen.")
        return

    _vul_bezig = True
    _stop_vlag.clear()

    print("\n" + "="*55)
    print(" INKAPPEN GESTART (Houd flesje vast!)")
    print("="*55)

    # Huidige glashoek als startpunt
    huidige_glas_hoek = imus.get_angle(imus.MPU2_ADDR) or GLAS_START
    leeg_teller       = 0

    # ---- Stap 1: Flesje overslaan ----
    print("[1/4] Wachten op handmatige start (kap het bier in)...")

    # ---- Stap 2: Gierlus (Alleen glassturing) ----
    print("[2/4] Gierlus gestart (camera stuurt GLAS bij)…")

    while not _stop_vlag.is_set():
        cam   = camera.get_camera_data()
        actie = camera.schuim_actie()
        stabiel = camera.inhoud_stabiel()

        schuim_h = cam.get('schuim_hoogte_px', 0)
        bier_h   = cam.get('bier_hoogte_px', 0)
        foam_pct = round(cam['foam_ratio'] * 100, 1)

        print(
            f"  schuim:{foam_pct}%  S:{schuim_h}px B:{bier_h}px  "
            f"{'STABIEL' if stabiel else 'AANPASSEN'}  "
            f"actie:{actie}  glas:{imus.get_angle(imus.MPU2_ADDR)}°",
            end='\r'
        )

        # -- Overflow: Waarschuwing (flesje moet je zelf terugtrekken!) --
        if actie == 'overflow':
            print("\n  !! OVERFLOW RISICO — Stop met gieten!")
            # Geen motor 2 commando meer hier

        if not cam['geldig']:
            time.sleep(CAMERA_CHECK_INTERVAL)
            continue

        # -- Glas bijsturen op basis van camera --
        if not stabiel:
            if actie == 'meer_schuim':
                huidige_glas_hoek = max(huidige_glas_hoek - GLAS_STAP_GROOT, GLAS_MIN)
                motors.stel_doel_in(3, huidige_glas_hoek)

            elif actie == 'minder_schuim':
                huidige_glas_hoek = min(huidige_glas_hoek + GLAS_STAP_KLEIN, GLAS_MAX)
                motors.stel_doel_in(3, huidige_glas_hoek)

        # -- Flesje leeg detectie (om routine af te sluiten) --
        if cam['geldig'] and bier_h < LEEG_BIER_PX:
            leeg_teller += 1
            if leeg_teller >= LEEG_TELLER_MAX:
                print("\n  Flesje lijkt leeg — routine afronden.")
                break
        else:
            leeg_teller = max(0, leeg_teller - 1)

        time.sleep(CAMERA_CHECK_INTERVAL)

    if _stop_vlag.is_set():
        _einde_vul()
        return

    # ---- Stap 3: Flesje terugplaatsen (Overslaan) ----
    print("\n[3/4] Flesje handmatig wegzetten…")

    # ---- Stap 4: Glas rechtop ----
    print(f"[4/4] Glas rechtop naar {GLAS_RECHTOP}°…")
    motors.stel_doel_in(3, GLAS_RECHTOP)
    motors.wacht_op_doel(3, timeout=20)

    print("\n✓ Klaar! Geniet van je handmatig ingeschonken pint.")
    print("="*55 + "\n")
    _vul_bezig = False 


def _einde_vul():
    global _vul_bezig
    motors.annuleer_alle_doelen()
    print("\n[Vul] Gestopt door gebruiker.")
    _vul_bezig = False


def stop_alles():
    _stop_vlag.set()
    motors.annuleer_alle_doelen()


# ==========================================
# CV2 UI (Houdt wel de hoeken bij voor feedback)
# ==========================================
def teken_ui(glas_hoek, flesje_hoek, cam_data):
    frame = np.zeros((400, 580, 3), dtype=np.uint8)

    def t(txt, y, kleur=(220, 220, 220), schaal=0.52):
        cv2.putText(frame, txt, (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, schaal, kleur, 1, cv2.LINE_AA)

    cv2.rectangle(frame, (0, 0), (580, 40), (30, 30, 30), -1)
    cv2.putText(frame, "BIER ROBOT - SEMI-AUTO", (140, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 190, 255), 2, cv2.LINE_AA)

    t(f"Flesje (MANUEEL): {flesje_hoek}°",   70, (100, 100, 255))
    t(f"Glas   (Motor 3): {glas_hoek}°",  100, (180, 230, 255))

    if cam_data['geldig']:
        fp = round(cam_data['foam_ratio'] * 100, 1)
        t(f"Schuim: {fp}%   S:{cam_data.get('schuim_hoogte_px',0)}px", 140, (0, 255, 100))
    
    status_txt   = "Inkappen BEZIG (Giet nu!)" if _vul_bezig else "Wacht op 'start'"
    t(status_txt, 230, (0, 255, 160) if _vul_bezig else (120, 120, 120), schaal=0.6)

    return frame


# ==========================================
# MAIN
# ==========================================
def main():
    global _stop_prog
    imus.init_all()
    motors.init_motoren()
    motor_thread = motors.start_motor_thread()
    camera.start_camera()

    # Kalibratie: Let op, Motor 2 zal mogelijk nog proberen te kalibreren
    # tenzij je dit in calibration.py ook hebt uitgezet.
    print("[Kalibratie] Starten...")
    voer_kalibratie_uit(motors.pwm_motoren, motors.motor_doel, motors.doel_lock)

    inp.registreer_callbacks(vul_routine, stop_alles)
    input_thread = inp.start_input_thread()

    cv2.namedWindow("Bier Robot", cv2.WINDOW_NORMAL)

    try:
        while not _stop_prog:
            g_hoek = imus.get_angle(imus.MPU2_ADDR)
            f_hoek = imus.get_angle(imus.MPU1_ADDR) # Leest nog steeds de hoek uit voor UI
            cam    = camera.get_camera_data()

            frame = teken_ui(g_hoek, f_hoek, cam)
            cv2.imshow("Bier Robot", frame)

            if cv2.waitKey(100) & 0xFF == 27 or not input_thread.is_alive():
                break
    finally:
        _stop_prog = True
        stop_alles()
        camera.stop_camera()
        motors.motor_systeem_actief = False
        motor_thread.join(timeout=1.5)
        cv2.destroyAllWindows()
        motors.cleanup_motoren()

if __name__ == '__main__':
    main()