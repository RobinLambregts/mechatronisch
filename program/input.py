"""
input.py - Terminal commando-verwerking

Beschikbare commando's:
  start           → start het automatisch inkappen
  stop            → stop / annuleer alles
  hoek            → lees beide IMU-hoeken uit
  calibrate       → start IMU-kalibratieprocedure
  M2:XX M3:YY     → stel handmatig hoekdoel in per motor
  <getal>         → stel hetzelfde hoekdoel in voor beide motoren
  Enter (leeg)    → annuleer alle actieve doelen
  exit / q        → programma afsluiten
"""

import threading
from imus import (
    MPU1_ADDR, MPU2_ADDR,
    get_angle,
)
from motors import (
    motor_doel, doel_lock, pwm_motoren,
    stel_doel_in, annuleer_alle_doelen,
)
from calibration import voer_kalibratie_uit

_input_actief  = True
_vul_callback  = None   # wordt ingesteld door main.py
_stop_callback = None


def registreer_callbacks(vul_cb, stop_cb):
    global _vul_callback, _stop_callback
    _vul_callback  = vul_cb
    _stop_callback = stop_cb


def terminal_input_worker():
    global _input_actief

    MENU = (
        "\nCommando's:\n"
        "  start           → begin automatisch inkappen\n"
        "  stop            → stop alles\n"
        "  hoek            → toon huidige hoeken\n"
        "  calibrate       → IMU-kalibratie\n"
        "  M2:XX M3:YY     → handmatige hoekdoelen\n"
        "  <getal>         → beide motoren naar die hoek\n"
        "  Enter           → annuleer actieve doelen\n"
        "  exit / q        → afsluiten\n"
        "> "
    )

    while _input_actief:
        try:
            invoer = input(MENU).strip()
        except (EOFError, KeyboardInterrupt):
            break

        low = invoer.lower()

        # ---- Afsluiten ----
        if low in ('exit', 'q'):
            print("Afsluiten...")
            _input_actief = False
            if _stop_callback:
                _stop_callback()
            break

        # ---- Leeg = annuleer ----
        if invoer == '':
            annuleer_alle_doelen()
            print("Alle doelen geannuleerd.")
            continue

        # ---- Start inkappen ----
        if low == 'start':
            if _vul_callback:
                print("Inkappen gestart…")
                threading.Thread(target=_vul_callback, daemon=True).start()
            else:
                print("Geen vul-callback geregistreerd.")
            continue

        # ---- Stop ----
        if low == 'stop':
            annuleer_alle_doelen()
            if _stop_callback:
                _stop_callback()
            print("Gestopt.")
            continue

        # ---- Hoek uitlezen ----
        if low == 'hoek':
            h1 = get_angle(MPU1_ADDR)
            h2 = get_angle(MPU2_ADDR)
            print(f"  IMU1 (flesje / Motor 2): {h1}°")
            print(f"  IMU2 (glas   / Motor 3): {h2}°")
            continue

        # ---- Kalibratie ----
        if low == 'calibrate':
            voer_kalibratie_uit(pwm_motoren, motor_doel, doel_lock)
            continue

        # ---- Handmatige hoekdoelen ----
        if invoer.upper().startswith('M'):
            doelen = {}
            try:
                for deel in invoer.split():
                    deel = deel.upper()
                    if deel.startswith('M') and ':' in deel:
                        m_str, h_str = deel[1:].split(':')
                        m_id = int(m_str)
                        if m_id in motor_doel:
                            doelen[m_id] = stel_doel_in(m_id, float(h_str))
                        else:
                            print(f"  Motor {m_id} heeft geen IMU-koppeling.")
            except ValueError:
                print("  Ongeldige invoer. Voorbeeld: M2:30 M3:-70")
                continue

            for m_id, doel in doelen.items():
                imu = MPU1_ADDR if m_id == 2 else MPU2_ADDR
                print(f"  [Motor {m_id}] → {doel}° | huidig: {get_angle(imu)}°")
            continue

        # ---- Enkelvoudig getal → beide motoren ----
        try:
            hoek = float(invoer)
            d2 = stel_doel_in(2, hoek)
            d3 = stel_doel_in(3, hoek)
            print(f"  Motor 2 → {d2}° | Motor 3 → {d3}°")
            continue
        except ValueError:
            pass

        print("  Onbekend commando. Typ Enter om het menu te zien.")


def start_input_thread():
    t = threading.Thread(target=terminal_input_worker, daemon=True)
    t.start()
    return t