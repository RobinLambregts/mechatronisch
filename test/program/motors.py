import time
import RPi.GPIO as GPIO

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

def init_motoren():
    """Stel de GPIO pins in als uitgang."""
    for m_id, pins in CONFIG.items():
        GPIO.setup(pins['dir'], GPIO.OUT)
        GPIO.setup(pins['pulse'], GPIO.OUT)
        GPIO.output(pins['pulse'], GPIO.LOW)
    print("  Motoren geïnitialiseerd (Simpele modus).")

def zet_aantal_stappen(m_id, stappen):
    if m_id not in CONFIG:
        print(f"  Ongeldige motor ID: {m_id}")
        return
    
    if (m_id == 1):
        STAP_PAUZE = 0.001
    elif (m_id == 2):
        STAP_PAUZE = 0.05
    else:
        STAP_PAUZE = 0.01

    pins = CONFIG[m_id]
    
    # Richting bepalen (positief = HIGH, negatief = LOW)
    richting = GPIO.HIGH if stappen > 0 else GPIO.LOW
    GPIO.output(pins['dir'], richting)

    # Het daadwerkelijke aantal pulsen sturen
    aantal = abs(stappen)
    for _ in range(aantal):
        GPIO.output(pins['pulse'], GPIO.HIGH)
        time.sleep(STAP_PAUZE)
        GPIO.output(pins['pulse'], GPIO.LOW)
        time.sleep(STAP_PAUZE)

    print(f"  Motor {m_id}: {stappen} stappen gezet.")