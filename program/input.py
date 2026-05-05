import imus
import motors
import calibration

def terminal_input_worker():
    while motors.motor_systeem_actief:
        try:
            invoer = input(
                "\nCommando's:\n"
                "  • Één getal    → beide motoren naar die hoek (bijv. '30')\n"
                "  • M2:XX M3:YY → elk een eigen hoek (bijv. 'M2:45 M3:-20')\n"
                "  • 'hoek'       → lees huidige hoek van beide IMUs uit\n"
                "  • 'calibrate'  → start IMU kalibratieprocedure\n"
                "  • Enter        → annuleer alle actieve doelen\n> "
            ).strip()

            # Annuleer alle doelen
            if invoer == "":
                with motors.doel_lock:
                    for m_id in motors.motor_doel:
                        motors.motor_doel[m_id]['actief'] = False
                        motors.pwm_motoren[m_id].ChangeDutyCycle(0)
                print("Alle doelen geannuleerd.")
                continue

            # Huidige hoek uitlezen
            if invoer.lower() == "hoek":
                print(f"Huidige hoek IMU1 (Motor 2): {imus.get_angle(imus.MPU1_ADDR)}°")
                print(f"Huidige hoek IMU2 (Motor 3): {imus.get_angle(imus.MPU2_ADDR)}°")
                continue

            # IMU kalibratie
            if invoer.lower() == "calibrate":
                calibration.voer_kalibratie_uit()
                continue

            doelen_parsed = {}

            if invoer.upper().startswith("M"):
                for deel in invoer.split():
                    deel = deel.upper()
                    if deel.startswith("M") and ":" in deel:
                        m_str, h_str = deel[1:].split(":")
                        m_id = int(m_str)
                        if m_id in motors.motor_doel:
                            doelen_parsed[m_id] = float(h_str)
                        else:
                            print(f"Motor {m_id} heeft geen IMU-koppeling, overgeslagen.")
            else:
                hoek = float(invoer)
                doelen_parsed = {2: hoek, 3: hoek}

            if not doelen_parsed:
                print("Geen geldige invoer herkend.")
                continue

            with motors.doel_lock:
                for m_id, doel in doelen_parsed.items():
                    motors.motor_doel[m_id]['doel']   = doel
                    motors.motor_doel[m_id]['actief'] = True

            for m_id, doel in doelen_parsed.items():
                huidige_hoek = imus.get_angle(imus.MPU1_ADDR if m_id == 2 else imus.MPU2_ADDR)
                print(f"[Motor {m_id}] Naar {doel}° | Huidige hoek: {huidige_hoek}°")

        except ValueError:
            print("Ongeldige invoer, probeer opnieuw (bijv. '30' of 'M2:45 M3:-20').")
        except EOFError:
            break