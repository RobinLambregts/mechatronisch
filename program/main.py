"""
camera_monitor.py — Standalone camera monitor
==============================================
Start: python3 camera_monitor.py

Print alleen iets naar de terminal als de actie verandert.
Geen motoren, geen IMUs, geen kalibratie.
"""

import time
import camera

CAMERA_CHECK_INTERVAL = 0.3  # seconden

def main():
    print("Camera starten…")
    camera.start_camera()
    print("Camera actief. Druk Ctrl+C om te stoppen.\n")

    vorige_actie = None

    try:
        while True:
            cam    = camera.get_camera_data()
            stabiel = camera.inhoud_stabiel()
            actie  = camera.schuim_actie()

            if not cam['geldig']:
                huidige_actie = "geen_beeld"
            elif actie == 'overflow':
                huidige_actie = "overflow"
            elif stabiel:
                huidige_actie = "stabiel_flesje_kantelen"
            elif actie == 'meer_schuim':
                huidige_actie = "meer_schuim_glas_schuiner"
            elif actie == 'minder_schuim':
                huidige_actie = "minder_schuim_glas_rechter"
            else:
                huidige_actie = "onbekend"

            if huidige_actie != vorige_actie:
                foam_pct = round(cam.get('foam_ratio', 0) * 100, 1)
                schuim_h = cam.get('schuim_hoogte_px', 0)
                bier_h   = cam.get('bier_hoogte_px', 0)
                print(
                    f"[ACTIE] {huidige_actie:<30}"
                    f"  schuim:{foam_pct}%  S:{schuim_h}px  B:{bier_h}px"
                )
                vorige_actie = huidige_actie

            time.sleep(CAMERA_CHECK_INTERVAL)

    except KeyboardInterrupt:
        print("\nGestopt.")

    finally:
        camera.stop_camera()


if __name__ == '__main__':
    main()