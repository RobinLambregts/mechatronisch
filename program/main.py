#AANPASSING GEMAAKT VANOP LAPTOP

import time
import threading
import cv2
import numpy as np
import smbus
import math
import RPi.GPIO as GPIO

# ==========================================
# 0. IMU SETUP
# ==========================================
bus = smbus.SMBus(1)
MPU1_ADDR = 0x68
MPU2_ADDR = 0x69

# Calibratie offsets (worden ingesteld door kalibratieprocedure)
imu_offsets = {
    MPU1_ADDR: {'acc_y': 0.0, 'acc_z': 0.0},
    MPU2_ADDR: {'acc_y': 0.0, 'acc_z': 0.0},
}

def init_mpu(addr):
    try:
        bus.write_byte_data(addr, 0x6B, 0)
    except:
        print(f"IMU {hex(addr)} niet gevonden")

def read_word(addr, reg):
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    val  = (high << 8) + low
    if val >= 0x8000:
        val = -((65535 - val) + 1)
    return val

def get_angle(addr):
    """
    Rotatie rond de X-as: atan2(acc_y, acc_z)
    Registers:
      acc_x = 0x3B / 0x3C
      acc_y = 0x3D / 0x3E  ← gebruikt
      acc_z = 0x3F / 0x40  ← gebruikt
    Calibratie-offsets worden afgetrokken van de ruwe waarden.
    """
    try:
        raw_acc_y = read_word(addr, 0x3D) / 16384.0
        raw_acc_z = read_word(addr, 0x3F) / 16384.0
        acc_y = raw_acc_y - imu_offsets[addr]['acc_y']
        acc_z = raw_acc_z - imu_offsets[addr]['acc_z']
        angle = math.degrees(math.atan2(acc_y, acc_z))
        return round(angle, 2)
    except Exception as e:
        print(f"IMU {hex(addr)} leesfout: {e}")
        return None

def kalibreer_imu(addr, target_angle=0, num_samples=200, vertraging=0.01):
    """
    Kalibratieprocedure voor één IMU:
    - target_angle: de hoek die de IMU moet aangeven na kalibratie (bijv. 0 of 90)
    """
    print(f"  Kalibreren IMU {hex(addr)} naar {target_angle}° ({num_samples} samples)...", end='', flush=True)
    som_y = 0.0
    som_z = 0.0
    gelezen = 0
    for _ in range(num_samples):
        try:
            som_y += read_word(addr, 0x3D) / 16384.0
            som_z += read_word(addr, 0x3F) / 16384.0
            gelezen += 1
        except Exception as e:
            print(f"\n  Leesfout tijdens kalibratie IMU {hex(addr)}: {e}")
        time.sleep(vertraging)

    if gelezen == 0:
        print(f" MISLUKT (geen leesbare samples).")
        return False

    gem_y = som_y / gelezen
    gem_z = som_z / gelezen

    # Bereken de verwachte acc waarden op basis van de doelhoek
    # Bij 0°: y=0, z=1 | Bij 90°: y=1, z=0
    rad = math.radians(target_angle)
    expected_y = math.sin(rad)
    expected_z = math.cos(rad)

    # Offset = gemeten gemiddelde - verwachte waarde
    imu_offsets[addr]['acc_y'] = gem_y - expected_y
    imu_offsets[addr]['acc_z'] = gem_z - expected_z

    print(f" Klaar.")
    return True

def voer_kalibratie_uit():
    """
    Volledige kalibratieprocedure. 
    Motor 2 (IMU1) wordt op 0 graden gezet.
    Motor 3 (IMU2) wordt op 90 graden gezet.
    """
    print("\n" + "="*50)
    print("IMU KALIBRATIE GESTART")
    print("="*50)
    print("! Zorg dat de robot in de KALIBRATIEPOSITIE staat.")
    print("  (Motor 2 horizontaal, Motor 3 verticaal/90 graden)")
    print("  Wacht 3 seconden...")

    with doel_lock:
        for m_id in motor_doel:
            motor_doel[m_id]['actief'] = False
            pwm_motoren[m_id].ChangeDutyCycle(0)

    for i in range(3, 0, -1):
        print(f"  {i}...", end='', flush=True)
        time.sleep(1)
    print()

    # Hier passen we de doelhoeken aan: IMU1 = 0°, IMU2 = 90°
    succes1 = kalibreer_imu(MPU1_ADDR, target_angle=0)
    succes2 = kalibreer_imu(MPU2_ADDR, target_angle=90)

    print()
    if succes1 and succes2:
        print("✓ Kalibratie geslaagd.")
        hoek1 = get_angle(MPU1_ADDR)
        hoek2 = get_angle(MPU2_ADDR)
        print(f"  Gecalibreerde hoek IMU1 (M2): {hoek1}° (verwacht ≈ 0°)")
        print(f"  Gecalibreerde hoek IMU2 (M3): {hoek2}° (verwacht ≈ 90°)")
    else:
        print("✗ Kalibratie deels mislukt.")
    print("="*50 + "\n")

init_mpu(MPU1_ADDR)
init_mpu(MPU2_ADDR)

# ==========================================
# 1. GPIO SETUP
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

config = {
    1: {'dir': 17, 'pulse': 22},
    2: {'dir': 23, 'pulse': 24},
    3: {'dir': 20, 'pulse': 21}
}

PWM_FREQ = 100   # Hz — verlaagd van 400 naar 100

pwm_motoren = {}
for m_id, pins in config.items():
    GPIO.setup(pins['dir'], GPIO.OUT)
    GPIO.setup(pins['pulse'], GPIO.OUT)
    pwm = GPIO.PWM(pins['pulse'], PWM_FREQ)
    pwm.start(0)
    pwm_motoren[m_id] = pwm

# ==========================================
# 2. DRAAIEN-NAAR-HOEK CONFIGURATIE (PD-regelaar)
# ==========================================
TOLERANTIE  = 1.0   # graden — binnen deze marge = "op doel"
MAX_DC      = 40    # maximale duty cycle
MIN_DC      = 8     # minimale duty cycle (onder deze drempel staat motor stil)

# PD-parameters — pas deze aan voor jouw mechanica:
#   KP  : hoe harder de motor duwt bij grote fout (proportioneel)
#   KD  : dempingskracht op basis van hoe snel de fout verandert (voorkomt overshoot/jitter)
KP = 2.5    # verhoog als te traag, verlaag als te veel overshoot
KD = 8.0    # verhoog als nog te veel jitter/oscillatie

# Koppeling: welke IMU meet welke motor
IMU_MOTOR_MAP = {
    MPU1_ADDR: 2,   # IMU1 stuurt Motor 2
    MPU2_ADDR: 3,   # IMU2 stuurt Motor 3
}

# PD-toestand per motor (vorige fout + tijdstip)
pd_staat = {
    2: {'vorige_fout': 0.0, 'vorige_tijd': None},
    3: {'vorige_fout': 0.0, 'vorige_tijd': None},
}

# ==========================================
# 3. THREADING
# ==========================================
motor_systeem_actief = True
laatste_toets_tijd   = time.time()

motor_statussen = {1: None, 2: None, 3: None}

motor_doel = {
    2: {'doel': None, 'actief': False},
    3: {'doel': None, 'actief': False},
}

doel_lock = threading.Lock()

def bereken_pd_dc(m_id, fout):
    """
    PD-regelaar: output = KP*fout + KD*dfout/dt
    Begrensd tussen MIN_DC en MAX_DC.
    Geeft (duty_cycle, richting_bool) terug.
    """
    nu = time.time()
    staat = pd_staat[m_id]

    if staat['vorige_tijd'] is None:
        d_fout = 0.0
    else:
        dt = nu - staat['vorige_tijd']
        dt = max(dt, 0.001)  # voorkom deling door nul
        d_fout = (fout - staat['vorige_fout']) / dt

    staat['vorige_fout'] = fout
    staat['vorige_tijd'] = nu

    pd_output = KP * abs(fout) - KD * (d_fout if fout > 0 else -d_fout)
    pd_output = max(0.0, pd_output)  # nooit negatief

    dc = int(MIN_DC + (MAX_DC - MIN_DC) * min(pd_output / (KP * 30.0), 1.0))
    dc = max(MIN_DC, min(MAX_DC, dc))
    return dc

def motor_worker():
    global motor_systeem_actief, motor_statussen, laatste_toets_tijd

    while motor_systeem_actief:

        # --- Automatische IMU-sturing (PD) ---
        with doel_lock:
            actieve_doelen = {
                m_id: info.copy()
                for m_id, info in motor_doel.items()
                if info['actief'] and info['doel'] is not None
            }

        for imu_addr, m_id in IMU_MOTOR_MAP.items():
            if m_id not in actieve_doelen:
                # Reset PD-toestand als motor niet actief is
                pd_staat[m_id]['vorige_tijd'] = None
                pd_staat[m_id]['vorige_fout'] = 0.0
                continue

            doel = actieve_doelen[m_id]['doel']
            hoek = get_angle(imu_addr)

            if hoek is None:
                pwm_motoren[m_id].ChangeDutyCycle(0)
                continue

            fout = doel - hoek

            if abs(fout) <= TOLERANTIE:
                pwm_motoren[m_id].ChangeDutyCycle(0)
                pd_staat[m_id]['vorige_tijd'] = None
                pd_staat[m_id]['vorige_fout'] = 0.0
                with doel_lock:
                    motor_doel[m_id]['actief'] = False
                print(f"[Motor {m_id}] Doel bereikt: {hoek}° ≈ {doel}°")
            else:
                dc       = bereken_pd_dc(m_id, fout)
                richting = GPIO.HIGH if fout > 0 else GPIO.LOW
                GPIO.output(config[m_id]['dir'], richting)
                pwm_motoren[m_id].ChangeDutyCycle(dc)

        # --- Handmatige toets-sturing ---
        if time.time() - laatste_toets_tijd > 0.15:
            motor_statussen = {1: None, 2: None, 3: None}

        for m_id, richting in motor_statussen.items():
            with doel_lock:
                if m_id in motor_doel and motor_doel[m_id]['actief']:
                    continue

            if richting == 1:
                GPIO.output(config[m_id]['dir'], GPIO.HIGH)
                pwm_motoren[m_id].ChangeDutyCycle(MAX_DC)
            elif richting == 0:
                GPIO.output(config[m_id]['dir'], GPIO.LOW)
                pwm_motoren[m_id].ChangeDutyCycle(MAX_DC)
            else:
                pwm_motoren[m_id].ChangeDutyCycle(0)

        time.sleep(0.05)

motor_thread = threading.Thread(target=motor_worker, daemon=True)
motor_thread.start()

# ==========================================
# 4. TERMINAL INPUT THREAD
# ==========================================
def terminal_input_worker():
    while motor_systeem_actief:
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
                with doel_lock:
                    for m_id in motor_doel:
                        motor_doel[m_id]['actief'] = False
                        pwm_motoren[m_id].ChangeDutyCycle(0)
                print("Alle doelen geannuleerd.")
                continue

            # Huidige hoek uitlezen
            if invoer.lower() == "hoek":
                hoek1 = get_angle(MPU1_ADDR)
                hoek2 = get_angle(MPU2_ADDR)
                print(f"Huidige hoek IMU1 (Motor 2): {hoek1}°")
                print(f"Huidige hoek IMU2 (Motor 3): {hoek2}°")
                continue

            # IMU kalibratie
            if invoer.lower() == "calibrate":
                voer_kalibratie_uit()
                continue

            # Parsen: "M2:45 M3:-20" of gewoon "30"
            doelen_parsed = {}

            if invoer.upper().startswith("M"):
                for deel in invoer.split():
                    deel = deel.upper()
                    if deel.startswith("M") and ":" in deel:
                        m_str, h_str = deel[1:].split(":")
                        m_id = int(m_str)
                        if m_id in motor_doel:
                            doelen_parsed[m_id] = float(h_str)
                        else:
                            print(f"Motor {m_id} heeft geen IMU-koppeling, overgeslagen.")
            else:
                hoek = float(invoer)
                doelen_parsed = {2: hoek, 3: hoek}

            if not doelen_parsed:
                print("Geen geldige invoer herkend.")
                continue

            with doel_lock:
                for m_id, doel in doelen_parsed.items():
                    motor_doel[m_id]['doel']   = doel
                    motor_doel[m_id]['actief'] = True

            for m_id, doel in doelen_parsed.items():
                huidige_hoek = get_angle(MPU1_ADDR if m_id == 2 else MPU2_ADDR)
                print(f"[Motor {m_id}] Naar {doel}° | Huidige hoek: {huidige_hoek}°")

        except ValueError:
            print("Ongeldige invoer, probeer opnieuw (bijv. '30' of 'M2:45 M3:-20').")
        except EOFError:
            break

input_thread = threading.Thread(target=terminal_input_worker, daemon=True)
input_thread.start()

# ==========================================
# 5. UI
# ==========================================
cv2.namedWindow("Robot Besturing")
print("="*50)
print("Handmatige sturing (CV2-venster actief houden):")
print("  Motor 1: [A] Vooruit | [Q] Achteruit")
print("  Motor 2: [E] Vooruit | [D] Achteruit")
print("  Motor 3: [T] Vooruit | [G] Achteruit")
print("Terminal: geef doelhoek, typ 'hoek' of 'calibrate'")
print("Druk op 'ESC' om af te sluiten.")
print("="*50)

# ==========================================
# 6. MAIN LOOP
# ==========================================
try:
    while True:
        with doel_lock:
            doel_m2 = motor_doel[2].copy()
            doel_m3 = motor_doel[3].copy()

        hoek1 = get_angle(MPU1_ADDR)
        hoek2 = get_angle(MPU2_ADDR)

        # Toon of kalibratie actief is (niet-nul offsets)
        kal_actief = any(
            imu_offsets[a]['acc_y'] != 0.0 or imu_offsets[a]['acc_z'] != 0.0
            for a in [MPU1_ADDR, MPU2_ADDR]
        )

        frame = np.zeros((320, 500, 3), dtype=np.uint8)

        def tekst(frame, txt, y, kleur=(255, 255, 255)):
            cv2.putText(frame, txt, (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, kleur, 1)

        tekst(frame, "Robot Besturing", 30, (0, 200, 255))
        tekst(frame, f"IMU1 (Motor 2): {hoek1}°", 70)
        tekst(frame, f"IMU2 (Motor 3): {hoek2}°", 100)

        if doel_m2['actief']:
            fout2 = round(doel_m2['doel'] - (hoek1 or 0), 1)
            tekst(frame, f"Doel M2: {doel_m2['doel']}°  fout: {fout2}°", 135, (0, 255, 100))
        else:
            tekst(frame, "Doel M2: inactief", 135, (150, 150, 150))

        if doel_m3['actief']:
            fout3 = round(doel_m3['doel'] - (hoek2 or 0), 1)
            tekst(frame, f"Doel M3: {doel_m3['doel']}°  fout: {fout3}°", 165, (0, 255, 100))
        else:
            tekst(frame, "Doel M3: inactief", 165, (150, 150, 150))

        kal_kleur = (0, 255, 180) if kal_actief else (100, 100, 100)
        kal_tekst = "Kalibratie: actief" if kal_actief else "Kalibratie: niet uitgevoerd"
        tekst(frame, kal_tekst, 200, kal_kleur)

        tekst(frame, f"PWM:{PWM_FREQ}Hz | DC:{MIN_DC}-{MAX_DC}% | KP:{KP} KD:{KD}", 240, (100, 100, 255))
        tekst(frame, "ESC=afsluiten | Terminal: doelhoek/'hoek'/'calibrate'", 280, (180, 180, 180))

        cv2.imshow("Robot Besturing", frame)
        key = cv2.waitKey(100) & 0xFF

        if key != 255:
            laatste_toets_tijd = time.time()
            if key == ord('a'):   motor_statussen[1] = 1
            elif key == ord('q'): motor_statussen[1] = 0
            elif key == ord('e'): motor_statussen[2] = 1
            elif key == ord('d'): motor_statussen[2] = 0
            elif key == ord('t'): motor_statussen[3] = 1
            elif key == ord('g'): motor_statussen[3] = 0
            elif key == 27:
                break

except KeyboardInterrupt:
    print("Onderbroken door gebruiker.")

# ==========================================
# 7. CLEANUP
# ==========================================
finally:
    print("Systeem afsluiten, motoren stoppen...")
    motor_systeem_actief = False
    motor_statussen = {1: None, 2: None, 3: None}
    motor_thread.join(timeout=1.0)
    cv2.destroyAllWindows()
    for pwm in pwm_motoren.values():
        pwm.stop()
    GPIO.cleanup()
    print("GPIO succesvol opgeruimd.")