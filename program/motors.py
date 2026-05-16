"""
motors.py - GPIO motor aansturing (3 stappenm/DC motoren via PWM)

Motor 1: draaien flesje   (geen IMU, handmatig)
Motor 2: draaien glas     (IMU1 / MPU1_ADDR)  — bereik: 0° … 50°
Motor 3: voor/achteruit glas (IMU2 / MPU2_ADDR) — bereik: -90° … -40°
"""

import time
import threading
import RPi.GPIO as GPIO

from imus import (
    MPU1_ADDR, MPU2_ADDR,
    get_angle, angle_difference,
)

# ==========================================
# GPIO Config
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

CONFIG = {
    1: {'dir': 17, 'pulse': 22},
    2: {'dir': 23, 'pulse': 24},
    3: {'dir': 20, 'pulse': 21},
}

PWM_FREQ = 40
TOLERANTIE = 1.0       # graden
MAX_DC     = 40        # maximale duty cycle %
MIN_DC     = {2: 0, 3: 0}

# ---- Hoekgrenzen ----
GRENS_FLESJE = (0.0, 50.0)     # Motor 2
GRENS_GLAS   = (-90.0, -40.0)  # Motor 3

# ---- PI parameters ----
KP    = 5.0
KI    = 0.2
MAX_I = 15.0

IMU_MOTOR_MAP = {MPU1_ADDR: 2, MPU2_ADDR: 3}

# ==========================================
# Gedeelde toestand
# ==========================================
pwm_motoren = {}
motor_doel = {
    2: {'doel': None, 'actief': False},
    3: {'doel': None, 'actief': False},
}
motor_statussen = {1: None, 2: None, 3: None}   # handmatige sturing
doel_lock = threading.Lock()
laatste_toets_tijd = time.time()

pi_staat = {
    2: {'integraal': 0.0, 'vorige_tijd': None},
    3: {'integraal': 0.0, 'vorige_tijd': None},
}

motor_systeem_actief = True

def zet_aantal_stappen(m_id, stappen):
    """Direct handmatig aantal stappen zetten voor motor m_id."""
    if m_id not in motor_statussen:
        print(f"  Ongeldige motor ID: {m_id}")
        return
    motor_statussen[m_id] = 1 if stappen > 0 else -1 if stappen < 0 else 0
    global laatste_toets_tijd
    laatste_toets_tijd = time.time()

def init_motoren():
    """Initialiseer GPIO en start PWM voor alle motoren."""
    for m_id, pins in CONFIG.items():
        GPIO.setup(pins['dir'],   GPIO.OUT)
        GPIO.setup(pins['pulse'], GPIO.OUT)
        pwm = GPIO.PWM(pins['pulse'], PWM_FREQ)
        pwm.start(0)
        pwm_motoren[m_id] = pwm
    print("  Motoren geïnitialiseerd.")


def cleanup_motoren():
    for pwm in pwm_motoren.values():
        pwm.stop()
    GPIO.cleanup()
    print("  GPIO opgeruimd.")


# ==========================================
# PI regelaar
# ==========================================
def bereken_pi_dc(m_id, fout):
    nu = time.time()
    staat = pi_staat[m_id]
    dt = 0.0 if staat['vorige_tijd'] is None else (nu - staat['vorige_tijd'])
    staat['vorige_tijd'] = nu

    staat['integraal'] += fout * dt
    if KI != 0:
        lim = MAX_I / KI
        staat['integraal'] = max(min(staat['integraal'], lim), -lim)

    pi_output = KP * fout + KI * staat['integraal']
    richting  = GPIO.HIGH if pi_output > 0 else GPIO.LOW
    output_abs = abs(pi_output)

    base = MIN_DC[m_id]
    dc = int(base + (MAX_DC - base) * min(output_abs / 15.0, 1.0))
    dc = max(base, min(MAX_DC, dc))
    return dc, richting


def _reset_pi(m_id):
    pi_staat[m_id]['vorige_tijd'] = None
    pi_staat[m_id]['integraal']   = 0.0


# ==========================================
# Hoekgrenzen bewaking
# ==========================================
def klamp_doel(m_id, doel):
    if m_id == 2:
        lo, hi = GRENS_FLESJE
    elif m_id == 3:
        lo, hi = GRENS_GLAS
    else:
        return doel
    geklampt = max(lo, min(hi, doel))
    if geklampt != doel:
        print(f"  [Motor {m_id}] Doel {doel}° buiten bereik → geklampt naar {geklampt}°")
    return geklampt


def stel_doel_in(m_id, doel):
    """Stel een geklampt hoekdoel in voor motor m_id."""
    doel = klamp_doel(m_id, doel)
    with doel_lock:
        motor_doel[m_id]['doel']   = doel
        motor_doel[m_id]['actief'] = True
    return doel


def annuleer_alle_doelen():
    with doel_lock:
        for m_id in motor_doel:
            motor_doel[m_id]['actief'] = False
            pwm_motoren[m_id].ChangeDutyCycle(0)


def wacht_op_doel(m_id, timeout=10.0):
    """Blokkeer totdat motor m_id zijn doel bereikt heeft (of timeout)."""
    start = time.time()
    while time.time() - start < timeout:
        with doel_lock:
            if not motor_doel[m_id]['actief']:
                return True
        time.sleep(0.05)
    return False


# ==========================================
# Motor worker thread
# ==========================================
def motor_worker():
    global motor_systeem_actief
    while motor_systeem_actief:

        # --- PI sturing ---
        with doel_lock:
            actief = {m: info.copy() for m, info in motor_doel.items() if info['actief']}

        for imu_addr, m_id in IMU_MOTOR_MAP.items():
            if m_id not in actief:
                _reset_pi(m_id)
                continue

            doel  = actief[m_id]['doel']
            hoek  = get_angle(imu_addr)
            if hoek is None:
                continue

            fout = angle_difference(doel, hoek)

            if abs(fout) <= TOLERANTIE:
                pwm_motoren[m_id].ChangeDutyCycle(0)
                _reset_pi(m_id)
                with doel_lock:
                    motor_doel[m_id]['actief'] = False
            else:
                dc, richting = bereken_pi_dc(m_id, fout)
                GPIO.output(CONFIG[m_id]['dir'], richting)
                pwm_motoren[m_id].ChangeDutyCycle(dc)

        # --- Handmatige sturing (keyboard) ---
        if time.time() - laatste_toets_tijd > 0.15:
            for m_id, richting in motor_statussen.items():
                if m_id in motor_doel and motor_doel[m_id]['actief']:
                    continue
                if richting == 1:
                    GPIO.output(CONFIG[m_id]['dir'], GPIO.HIGH)
                    pwm_motoren[m_id].ChangeDutyCycle(MAX_DC)
                elif richting == 0:
                    GPIO.output(CONFIG[m_id]['dir'], GPIO.LOW)
                    pwm_motoren[m_id].ChangeDutyCycle(MAX_DC)
                else:
                    pwm_motoren[m_id].ChangeDutyCycle(0)

        time.sleep(0.05)


def start_motor_thread():
    t = threading.Thread(target=motor_worker, daemon=True)
    t.start()
    return t