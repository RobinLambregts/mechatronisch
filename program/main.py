from Motor import Motor, Mode
import RPi.GPIO as GPIO
import math
import sys
import tty
import termios
import threading
import time

# =========================================================
# MOVE FUNCTIONS
# =========================================================

def move_function_1(current_position, doelpositie):
    verschil = doelpositie - current_position
    return (verschil / 3.0) * 4000

def move_function_2(current_position, doelpositie):
    def naar_hoek_rad(x):
        graden = 60.0 - (x + 1) * 6.0
        graden = max(0.0, min(60.0, graden))
        return math.radians(graden)

    hoek_huidig = naar_hoek_rad(current_position)
    hoek_doel   = naar_hoek_rad(doelpositie)
    delta_rad   = hoek_doel - hoek_huidig
    return round(delta_rad / (math.pi / 2) * 460)
    
def move_function_3(current_position, doelpositie):
    def naar_hoek_rad(x):
        graden = 60.0 - (x + 1) * 6.0
        graden = max(0.0, min(60.0, graden))
        return math.radians(graden)
    hoek_huidig = naar_hoek_rad(current_position)
    hoek_doel   = naar_hoek_rad(doelpositie)
    delta_rad   = hoek_doel - hoek_huidig
    return round(delta_rad / (math.pi / 2) * 460)


# =========================================================
# GPIO SETUP
# =========================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

motor1 = Motor(17, 22, 0.001, move_function_1, 3, 0, 0)
motor2 = Motor(20, 21, 0.01, move_function_2, 8, 8, -1) 
motor3 = Motor(23, 24, 0.05, move_function_3, 6, -2, -2)


# =========================================================
# CONFIG & ORIGIN SETUP
# =========================================================

MODE_SCHUIN_AFSTAND = 1
MODE_RECHT_AFSTAND = 8
STAP = 0.05 

lock = threading.Lock()
huidige_mode = Mode.MODE_SCHUIN

# === NIEUW: Sla de exacte beginposities op van alle motoren als 'oorsprong' ===
OORSPRONG_M1 = motor1.current_position
OORSPRONG_M2 = motor2.current_position
OORSPRONG_M3 = motor3.current_position

motor3_pos = OORSPRONG_M3

automatisch_actief = False
s_ingedrukt = False


# =========================================================
# HELPERS
# =========================================================

def clamp(v, min_v, max_v):
    return max(min_v, min(max_v, v))


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        c = sys.stdin.read(1)
        if c == '\x1b':
            c += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

    return c


# =========================================================
# CORE ROBOT LOGICA
# =========================================================

def compensate(doel3):
    p1 = 0
    p2 = MODE_SCHUIN_AFSTAND if huidige_mode == Mode.MODE_SCHUIN else MODE_RECHT_AFSTAND
    
    fout = doel3 - (p1 + p2)
    
    # Zolang we te hoog zitten (fout is negatief), p2 verlagen
    while fout < 0:
        if p2 <= motor2.min_positie:  # We zitten op het minimum, we kunnen niet lager!
            break
        p2 -= 0.5
        fout += 0.5
        
    # Zolang we te laag zitten (fout > 3, p1 kan maximaal 3 opvangen), p2 verhogen
    while fout > 3:
        if p2 >= motor2.max_positie:  # We zitten op het maximum, we kunnen niet hoger!
            break
        p2 += 0.5
        fout -= 0.5  # Correctie: 1 stap omhoog betekent dat de fout 1 kleiner wordt
        
    p1_final = p1
    
    # De resterende fout (als die tussen 0 en 3 valt) wijzen we toe aan p1
    if 0 <= fout <= 3:
        p1_final = fout
        
    p2_final = p2
        
    return p1_final, p2_final


def beweeg_sync(doel3):
    global motor3_pos

    p1, p2 = compensate(doel3)
    motor3_pos = clamp(doel3, motor3.min_positie, motor3.max_positie)

    with lock:
        t1 = threading.Thread(target=motor1.beweeg_naar, args=(p1,))
        t2 = threading.Thread(target=motor2.beweeg_naar, args=(p2,))
        t3 = threading.Thread(target=motor3.beweeg_naar, args=(motor3_pos,))

        t1.start(); t2.start(); t3.start()
        t1.join(); t2.join(); t3.join()


# === NIEUW: Functie om gecontroleerd terug te keren naar de startpositie ===
def terug_naar_oorsprong():
    global motor3_pos
    print("\n[RESET] Proces beëindigd. Systeem wacht 5 seconden...")
    time.sleep(5)
    
    print("[RESET] Motoren keren nu gelijktijdig terug naar de oorspronkelijke startpositie...")
    with lock:
        t1 = threading.Thread(target=motor1.beweeg_naar, args=(OORSPRONG_M1,))
        t2 = threading.Thread(target=motor2.beweeg_naar, args=(OORSPRONG_M2,))
        t3 = threading.Thread(target=motor3.beweeg_naar, args=(OORSPRONG_M3,))

        t1.start(); t2.start(); t3.start()
        t1.join(); t2.join(); t3.join()
        
    motor3_pos = OORSPRONG_M3
    print("[RESET] Alle motoren zijn veilig teruggekeerd naar de oorsprong.\n")


# =========================================================
# AUTOMATISCHE PROCEDURE (THREAD)
# =========================================================

def automatische_loop():
    global motor3_pos, huidige_mode, s_ingedrukt, automatisch_actief
    
    automatisch_actief = True
    print(f"\n[START] Automatisch proces gestart. M3 beweegt vloeiend naar {motor3.max_positie}...")

    while motor3_pos < motor3.max_positie and automatisch_actief:
        
        # INTERRUPT: Als er op 's' is geduwd
        if s_ingedrukt:
            print("\n[INTERRUPT] 's' gedetecteerd! M3 gaat nu 50 stappen omhoog...")
            
            STAPPEN_50_DELTA = 1.63
            motor3_pos = clamp(motor3_pos + STAPPEN_50_DELTA, motor3.min_positie, motor3.max_positie)
            beweeg_sync(motor3_pos)
            
            huidige_mode = Mode.MODE_RECHT if huidige_mode == Mode.MODE_SCHUIN else Mode.MODE_SCHUIN
            print(f"[INFO] Mode geswitcht. Systeem pauzeert 5 seconden...")
            
            time.sleep(5)
            s_ingedrukt = False
            print("[INFO] Pauze voorbij. M3 vervolgt zijn weg...\n")

        # Reguliere piepkleine stap richting eindpunt 8
        motor3_pos = clamp(motor3_pos + STAP, motor3.min_positie, motor3.max_positie)
        beweeg_sync(motor3_pos)
        
        time.sleep(0.005)

    if motor3_pos >= motor3.max_positie:
        print(f"\n[SUCCES] Uiterste punt van M3 ({motor3.max_positie}) is bereikt.")
    
    automatisch_actief = False
    
    # === NIEUW: Na het succesvol afronden van de loop, ga terug naar af ===
    terug_naar_oorsprong()


# =========================================================
# MAIN LOOP
# =========================================================

print("\n=== Robot Besturing Geactiveerd ===")
print("Druk op [SPATIE] of [ENTER] om het proces te starten.")
print("Druk op [S] tijdens het proces voor de interrupt.")
print("Druk op [Q] om te stoppen.")

beweeg_sync(motor3_pos)

try:
    while True:
        k = get_key()

        if k in ('q', 'Q', '\x03'):
            if automatisch_actief:
                automatisch_actief = False
                # De thread stopt nu uit zichzelf, we vangen de reset op in het 'finally' blok hieronder
            else:
                print("\nProgramma handmatig gestopt.")
                terug_naar_oorsprong()
            break

        elif k in (' ', '\r', '\n'):
            if not automatisch_actief:
                threading.Thread(target=automatische_loop, daemon=True).start()
            else:
                print("[AANWIJZING] Het proces loopt al.")

        elif k in ('s', 'S'):
            if automatisch_actief:
                s_ingedrukt = True
            else:
                huidige_mode = Mode.MODE_RECHT if huidige_mode == Mode.MODE_SCHUIN else Mode.MODE_SCHUIN
                print("Mode handmatig gewijzigd naar:", huidige_mode)
                beweeg_sync(motor3_pos)

finally:
    # Als de gebruiker tijdens het proces hard afbreekt (met Q of Ctrl+C), 
    # zorgt dit ervoor dat hij alsnog netjes terugloopt voor de GPIO cleanup.
    if automatisch_actief:
        terug_naar_oorsprong()
    GPIO.cleanup()