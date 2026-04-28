import time
import threading
import cv2
import numpy as np
import smbus
import math
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# ==========================================
# CAMERA SETUP (from beer.py)
# ==========================================
picam2 = Picamera2()
config_cam = picam2.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}
)
picam2.configure(config_cam)
picam2.set_controls({"AwbEnable": True})
picam2.start()
time.sleep(1)

print("Camera gestart")

# Green color detection HSV
lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])

# ==========================================
# 0. IMU SETUP
# ==========================================
bus = smbus.SMBus(1)
MPU1_ADDR = 0x68
MPU2_ADDR = 0x69

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
    """
    try:
        acc_y = read_word(addr, 0x3D) / 16384.0
        acc_z = read_word(addr, 0x3F) / 16384.0
        angle = math.degrees(math.atan2(acc_y, acc_z))
        return round(angle, 2)
    except Exception as e:
        print(f"IMU {hex(addr)} leesfout: {e}")
        return None

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

PWM_FREQ = 100   # Hz

pwm_motoren = {}
for m_id, pins in config.items():
    GPIO.setup(pins['dir'], GPIO.OUT)
    GPIO.setup(pins['pulse'], GPIO.OUT)
    pwm = GPIO.PWM(pins['pulse'], PWM_FREQ)
    pwm.start(0)
    pwm_motoren[m_id] = pwm

# ==========================================
# 2. DRAAIEN-NAAR-HOEK CONFIGURATIE
# ==========================================
TOLERANTIE = 1.0   # graden
MAX_DC     = 40    # duty cycle
MIN_DC     = 8     # minimale duty cycle
AFREM_ZONE = 15.0  # graden

IMU_MOTOR_MAP = {
    MPU1_ADDR: 2,
    MPU2_ADDR: 3,
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
green_detected = False

def bereken_duty_cycle(fout):
    """Lineaire afremming in de afremzone."""
    abs_fout = abs(fout)
    if abs_fout >= AFREM_ZONE:
        return MAX_DC
    dc = MIN_DC + (MAX_DC - MIN_DC) * (abs_fout / AFREM_ZONE)
    return int(dc)

def motor_worker():
    global motor_systeem_actief, motor_statussen, laatste_toets_tijd

    while motor_systeem_actief:

        # --- Automatische IMU-sturing ---
        with doel_lock:
            actieve_doelen = {
                m_id: info.copy()
                for m_id, info in motor_doel.items()
                if info['actief'] and info['doel'] is not None
            }

        for imu_addr, m_id in IMU_MOTOR_MAP.items():
            if m_id not in actieve_doelen:
                continue

            doel = actieve_doelen[m_id]['doel']
            hoek = get_angle(imu_addr)

            if hoek is None:
                pwm_motoren[m_id].ChangeDutyCycle(0)
                continue

            fout = doel - hoek

            if abs(fout) <= TOLERANTIE:
                pwm_motoren[m_id].ChangeDutyCycle(0)
                with doel_lock:
                    motor_doel[m_id]['actief'] = False
                print(f"[Motor {m_id}] Doel bereikt: {hoek}° ≈ {doel}°")
            else:
                dc       = bereken_duty_cycle(fout)
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
# 4. CAMERA WORKER THREAD
# ==========================================
def camera_worker():
    global green_detected

    while motor_systeem_actief:
        try:
            frame = picam2.capture_array()

            # Color correction
            b, g, r = cv2.split(frame)
            r = cv2.addWeighted(r, 1.05, r, 0, 0)
            g = cv2.addWeighted(g, 1.0, g, 0, 0)
            b = cv2.addWeighted(b, 0.95, b, 0, 0)
            frame = cv2.merge([b, g, r])

            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Detect green
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            # Find contours
            contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            green_found = False
            for cnt in contours:
                if cv2.contourArea(cnt) > 500:  # Filter small noise
                    green_found = True
                    break

            # If green detected, set motors to 0 degrees
            if green_found:
                if not green_detected:
                    green_detected = True
                    print("[CAMERA] Green detected! Setting motors to 0 degrees")
                    with doel_lock:
                        motor_doel[2]['doel'] = 0
                        motor_doel[2]['actief'] = True
                        motor_doel[3]['doel'] = 0
                        motor_doel[3]['actief'] = True
            else:
                green_detected = False

        except Exception as e:
            print(f"Camera worker error: {e}")

        time.sleep(0.1)

camera_thread = threading.Thread(target=camera_worker, daemon=True)
camera_thread.start()

# ==========================================
# 5. TERMINAL INPUT THREAD
# ==========================================
def terminal_input_worker():
    while motor_systeem_actief:
        try:
            invoer = input(
                "\nCommando's:\n"
                "  • Één getal    → beide motoren naar die hoek (bijv. '30')\n"
                "  • M2:XX M3:YY → elk een eigen hoek (bijv. 'M2:45 M3:-20')\n"
                "  • 'hoek'       → lees huidige hoek van beide IMUs uit\n"
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
# 6. UI
# ==========================================
cv2.namedWindow("Robot Besturing")
print("="*50)
print("Handmatige sturing (CV2-venster actief houden):")
print("  Motor 1: [A] Vooruit | [Q] Achteruit")
print("  Motor 2: [E] Vooruit | [D] Achteruit")
print("  Motor 3: [T] Vooruit | [G] Achteruit")
print("Terminal: geef doelhoek of typ 'hoek'")
print("Camera: groen zichtbaar → motoren naar 0 graden")
print("Druk op 'ESC' om af te sluiten.")
print("="*50)

# ==========================================
# 7. MAIN LOOP
# ==========================================
try:
    while True:
        with doel_lock:
            doel_m2 = motor_doel[2].copy()
            doel_m3 = motor_doel[3].copy()

        hoek1 = get_angle(MPU1_ADDR)
        hoek2 = get_angle(MPU2_ADDR)

        frame = np.zeros((310, 500, 3), dtype=np.uint8)

        def tekst(frame, txt, y, kleur=(255, 255, 255)):
            cv2.putText(frame, txt, (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, kleur, 1)

        tekst(frame, "Robot Besturing + Camera", 30, (0, 200, 255))
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

        if green_detected:
            tekst(frame, "GREEN DETECTED - Motors to 0°", 200, (0, 255, 0))
        else:
            tekst(frame, "No green detected", 200, (100, 100, 100))

        tekst(frame, f"PWM: {PWM_FREQ}Hz | Max DC: {MAX_DC}% | Min DC: {MIN_DC}%", 235, (100, 100, 255))
        tekst(frame, "ESC = afsluiten | Terminal = doelhoek / 'hoek'", 270, (180, 180, 180))

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
# 8. CLEANUP
# ==========================================
finally:
    print("Systeem afsluiten, motoren stoppen...")
    motor_systeem_actief = False
    motor_statussen = {1: None, 2: None, 3: None}
    motor_thread.join(timeout=1.0)
    camera_thread.join(timeout=1.0)
    cv2.destroyAllWindows()
    picam2.stop()
    for pwm in pwm_motoren.values():
        pwm.stop()
    GPIO.cleanup()
    print("GPIO succesvol opgeruimd.")
