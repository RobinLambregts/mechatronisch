import RPi.GPIO as GPIO
import time
import threading
import imus

# GPIO Config
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

config = {
    1: {'dir': 17, 'pulse': 22},
    2: {'dir': 23, 'pulse': 24},
    3: {'dir': 20, 'pulse': 21}
}

PWM_FREQ = 100
TOLERANTIE = 1.0
MAX_DC = 40
MIN_DC = 8

# PI Parameters
KP = 1.2   # Iets verhoogd voor snellere reactie
KI = 0.5   # De integraal-factor: bouwt kracht op als het doel niet bereikt wordt
MAX_I = 15 # Anti-windup: de maximale bijdrage van de I-term aan de duty cycle

IMU_MOTOR_MAP = {imus.MPU1_ADDR: 2, imus.MPU2_ADDR: 3}

pwm_motoren = {}
for m_id, pins in config.items():
    GPIO.setup(pins['dir'], GPIO.OUT)
    GPIO.setup(pins['pulse'], GPIO.OUT)
    pwm = GPIO.PWM(pins['pulse'], PWM_FREQ)
    pwm.start(0)
    pwm_motoren[m_id] = pwm

motor_systeem_actief = True
laatste_toets_tijd = time.time()
motor_statussen = {1: None, 2: None, 3: None}
motor_doel = {
    2: {'doel': None, 'actief': False},
    3: {'doel': None, 'actief': False},
}

# Staat voor de PI regelaar
pi_staat = {
    2: {'integraal': 0.0, 'vorige_tijd': None},
    3: {'integraal': 0.0, 'vorige_tijd': None},
}
doel_lock = threading.Lock()

def bereken_pi_dc(m_id, fout):
    """
    PI-regelaar: output = KP*fout + KI * integraal(fout * dt)
    """
    nu = time.time()
    staat = pi_staat[m_id]
    
    if staat['vorige_tijd'] is None:
        dt = 0.0
    else:
        dt = nu - staat['vorige_tijd']
    
    staat['vorige_tijd'] = nu

    # Bereken Integraal (alleen als de motor niet al op volle kracht staat/windup preventie)
    # We integreren de signed fout zodat de I-term ook de andere kant op werkt
    staat['integraal'] += fout * dt
    
    # Anti-windup: Begrens de integraal-term
    # We zorgen dat de KI * integraal niet meer dan MAX_I kan worden
    if KI != 0:
        limiet = MAX_I / KI
        staat['integraal'] = max(min(staat['integraal'], limiet), -limiet)

    # PI Output berekening
    p_term = KP * fout
    i_term = KI * staat['integraal']
    
    pi_output = p_term + i_term
    
    # Richting bepalen (sign van de pi_output)
    richting = GPIO.HIGH if pi_output > 0 else GPIO.LOW
    
    # Duty cycle schalen (gebruik absolute waarde van de output)
    output_abs = abs(pi_output)
    
    # Schaling naar Duty Cycle (0-100)
    # We mappen 0-30 graden fout naar MIN_DC tot MAX_DC
    dc = int(MIN_DC + (MAX_DC - MIN_DC) * min(output_abs / 30.0, 1.0))
    
    return max(MIN_DC, min(MAX_DC, dc)), richting

def motor_worker():
    global motor_systeem_actief, motor_statussen
    while motor_systeem_actief:
        # PI Sturing
        with doel_lock:
            actieve_doelen = {m: info.copy() for m, info in motor_doel.items() if info['actief']}

        for imu_addr, m_id in IMU_MOTOR_MAP.items():
            if m_id not in actieve_doelen:
                pi_staat[m_id]['vorige_tijd'] = None
                pi_staat[m_id]['integraal'] = 0.0 # Reset integraal als niet actief
                continue
            
            doel = actieve_doelen[m_id]['doel']
            hoek = imus.get_angle(imu_addr)
            if hoek is None: continue
            
            fout = doel - hoek
            
            if abs(fout) <= TOLERANTIE:
                pwm_motoren[m_id].ChangeDutyCycle(0)
                pi_staat[m_id]['integraal'] = 0.0 # Reset bij bereiken doel
                with doel_lock: 
                    motor_doel[m_id]['actief'] = False
            else:
                dc, richting = bereken_pi_dc(m_id, fout)
                GPIO.output(config[m_id]['dir'], richting)
                pwm_motoren[m_id].ChangeDutyCycle(dc)

        # Handmatige sturing (Toetsenbord)
        if time.time() - laatste_toets_tijd > 0.15:
            for m_id, richting in motor_statussen.items():
                if not (m_id in motor_doel and motor_doel[m_id]['actief']):
                    if richting == 1:
                        GPIO.output(config[m_id]['dir'], GPIO.HIGH)
                        pwm_motoren[m_id].ChangeDutyCycle(MAX_DC)
                    elif richting == 0:
                        GPIO.output(config[m_id]['dir'], GPIO.LOW)
                        pwm_motoren[m_id].ChangeDutyCycle(MAX_DC)
                    else:
                        pwm_motoren[m_id].ChangeDutyCycle(0)
        time.sleep(0.05)