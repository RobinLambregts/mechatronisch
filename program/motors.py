import RPi.GPIO as GPIO
import threading
import time
import imus

# Configuratie
TOLERANTIE = 1.0   
MAX_DC     = 40    
MIN_DC     = 8     
AFREM_ZONE = 15.0  
PWM_FREQ   = 100   

config = {
    1: {'dir': 17, 'pulse': 22},
    2: {'dir': 23, 'pulse': 24},
    3: {'dir': 20, 'pulse': 21}
}

IMU_MOTOR_MAP = {
    imus.MPU1_ADDR: 2,
    imus.MPU2_ADDR: 3,
}

# State Variabelen
motor_systeem_actief = True
laatste_toets_tijd   = time.time()
motor_statussen = {1: None, 2: None, 3: None}
motor_doel = {
    2: {'doel': None, 'actief': False},
    3: {'doel': None, 'actief': False},
}
doel_lock = threading.Lock()
pwm_motoren = {}

def init_motors():
    global pwm_motoren
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for m_id, pins in config.items():
        GPIO.setup(pins['dir'], GPIO.OUT)
        GPIO.setup(pins['pulse'], GPIO.OUT)
        pwm = GPIO.PWM(pins['pulse'], PWM_FREQ)
        pwm.start(0)
        pwm_motoren[m_id] = pwm

def bereken_duty_cycle(fout):
    abs_fout = abs(fout)
    if abs_fout >= AFREM_ZONE:
        return MAX_DC
    dc = MIN_DC + (MAX_DC - MIN_DC) * (abs_fout / AFREM_ZONE)
    return int(dc)

def motor_worker():
    global motor_systeem_actief, motor_statussen, laatste_toets_tijd

    while motor_systeem_actief:
        with doel_lock:
            actieve_doelen = {
                m_id: info.copy()
                for m_id, info in motor_doel.items()
                if info['actief'] and info['doel'] is not None
            }

        # Automatische IMU-sturing
        for imu_addr, m_id in IMU_MOTOR_MAP.items():
            if m_id not in actieve_doelen:
                continue

            doel = actieve_doelen[m_id]['doel']
            hoek = imus.get_angle(imu_addr)

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
                dc = bereken_duty_cycle(fout)
                richting = GPIO.HIGH if fout > 0 else GPIO.LOW
                GPIO.output(config[m_id]['dir'], richting)
                pwm_motoren[m_id].ChangeDutyCycle(dc)

        # Handmatige toets-sturing
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