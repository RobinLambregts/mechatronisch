import cv2
import numpy as np
from flask import Flask, Response
from picamera2 import Picamera2
from Motor import Motor, Mode
import RPi.GPIO as GPIO
import math
import sys
import tty
import termios
import threading
import time

# =========================================================
# 1. CAMERA & ANALYSE KLASSEN
# =========================================================

class BeerGlassAnalyzer:
    def __init__(self, config=None):
        self.config = {
            # Gebruik de geüpdatete kleuren voor écht bier
            'liquid': {'lower': np.array([10, 80, 50]), 'upper': np.array([35, 255, 255])},
            'foam': {'lower': np.array([0, 0, 200]), 'upper': np.array([180, 50, 255])},
            'glass_box_padding': 10
        }
        if config:
            self.config.update(config)
        self.raw_image = None
        self.preprocessed_image = None
        self.results = {}

    def _preprocess(self, image):
        self.raw_image = image.copy()
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    def _detect_glass(self, hsv_image):
        _, brightness_thresh = cv2.threshold(hsv_image[:,:,2], 100, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(brightness_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        glass_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(glass_contour)
        p = self.config['glass_box_padding']
        x_pad = max(0, x - p)
        y_pad = max(0, y - p)
        w_pad = min(self.raw_image.shape[1] - x_pad, w + 2*p)
        h_pad = min(self.raw_image.shape[0] - y_pad, h + 2*p)

        return (x_pad, y_pad, w_pad, h_pad), glass_contour

    def analyze(self, camera_frame=None):
        if camera_frame is None:
            return {'status': 'fout', 'message': 'Geen frame'}

        self.preprocessed_image = self._preprocess(camera_frame)
        glass_detect_result = self._detect_glass(self.preprocessed_image)
        
        if not glass_detect_result:
            self.results = {'status': 'fout', 'message': 'Geen glas'}
            return self.results
        
        glass_box, glass_contour = glass_detect_result
        x, y, w, h = glass_box
        glass_roi = self.preprocessed_image[y:y+h, x:x+w]
        
        liquid_mask = cv2.inRange(glass_roi, self.config['liquid']['lower'], self.config['liquid']['upper'])
        foam_mask = cv2.inRange(glass_roi, self.config['foam']['lower'], self.config['foam']['upper'])
        
        total_glass_area_pixels = cv2.contourArea(glass_contour)
        liquid_area_pixels = cv2.countNonZero(liquid_mask)
        foam_area_pixels = cv2.countNonZero(foam_mask)
        
        if total_glass_area_pixels == 0:
            return {'status': 'fout'}

        beer_volume_percent = (liquid_area_pixels / total_glass_area_pixels) * 100
        foam_volume_percent = (foam_area_pixels / total_glass_area_pixels) * 100
        
        self.results = {
            'status': 'succes',
            'data': {
                'beer_volume_percent': beer_volume_percent,
                'foam_volume_percent': foam_volume_percent,
                'total_volume_percent': beer_volume_percent + foam_volume_percent
            }
        }
        return self.results

    def get_visual_result(self):
        # [Korte versie van visualisatie voor overzichtelijkheid, dit is hetzelfde als voorheen]
        if self.raw_image is None: return None
        visual_img = self.raw_image.copy()
        h, w, _ = visual_img.shape
        
        glass_detect_result = self._detect_glass(self.preprocessed_image)
        if glass_detect_result:
            glass_box, _ = glass_detect_result
            gx, gy, gw, gh = glass_box
            cv2.rectangle(visual_img, (gx, gy), (gx+gw, gy+gh), (0, 255, 0), 2)
            glass_roi = self.preprocessed_image[gy:gy+gh, gx:gx+gw]
            liquid_mask = cv2.inRange(glass_roi, self.config['liquid']['lower'], self.config['liquid']['upper'])
            foam_mask = cv2.inRange(glass_roi, self.config['foam']['lower'], self.config['foam']['upper'])

            liquid_contours, _ = cv2.findContours(liquid_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if liquid_contours:
                cv2.drawContours(visual_img[gy:gy+gh, gx:gx+gw], [max(liquid_contours, key=cv2.contourArea)], -1, (255, 0, 0), 2)

            foam_contours, _ = cv2.findContours(foam_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if foam_contours:
                cv2.drawContours(visual_img[gy:gy+gh, gx:gx+gw], [max(foam_contours, key=cv2.contourArea)], -1, (255, 255, 255), 2)

        overlay_w, overlay_h = 280, 160
        ox, oy = max(0, w - overlay_w - 20), 20
        overlay_bg = visual_img.copy()
        cv2.rectangle(overlay_bg, (ox, oy), (ox + overlay_w, oy + overlay_h), (50, 50, 50), -1)
        cv2.addWeighted(overlay_bg, 0.75, visual_img, 0.25, 0, visual_img)

        font, curr_y = cv2.FONT_HERSHEY_SIMPLEX, oy + 30
        cv2.putText(visual_img, "Bier Analyse", (ox + 15, curr_y), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        curr_y += 35
        
        if self.results.get('status') == 'succes':
            res = self.results['data']
            cv2.putText(visual_img, f"Schuim: {res['foam_volume_percent']:.1f}%", (ox + 15, curr_y), font, 0.6, (255, 255, 255), 2); curr_y += 30
            cv2.putText(visual_img, f"Bier: {res['beer_volume_percent']:.1f}%", (ox + 15, curr_y), font, 0.6, (255, 0, 0), 2); curr_y += 30
            cv2.putText(visual_img, f"Totaal: {res['total_volume_percent']:.1f}%", (ox + 15, curr_y), font, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(visual_img, "STATUS: ZOEKEN...", (ox + 15, curr_y), font, 0.6, (0, 0, 255), 2)

        return visual_img

class CameraServer:
    def __init__(self, width=640, height=480):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (width, height)})
        self.picam2.configure(config)
        self.picam2.start()
        self.analyzer = BeerGlassAnalyzer()
        
        # Globale variabelen die de motor-loop kan uitlezen
        self.huidig_totaal_pct = 0.0
        self.huidig_schuim_pct = 0.0
        self.detectie_status = "wachten"

    def generate_frames(self):
        while True:
            try:
                frame_rgb = self.picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                
                res = self.analyzer.analyze(camera_frame=frame_bgr)
                
                # Update de publieke variabelen voor de robot logica
                if res['status'] == 'succes':
                    self.huidig_totaal_pct = res['data']['total_volume_percent']
                    self.huidig_schuim_pct = res['data']['foam_volume_percent']
                    self.detectie_status = "succes"
                else:
                    self.detectie_status = "fout"

                processed_frame = self.analyzer.get_visual_result()
                if processed_frame is None: processed_frame = frame_bgr

                ret, buffer = cv2.imencode('.jpg', processed_frame)
                if not ret: continue
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            except Exception as e:
                continue

camera = CameraServer()
app = Flask(__name__)

@app.route('/')
def video_feed():
    return Response(camera.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Start de webserver in een APARTE achtergrond thread zodat de robot loop niet blokkeert
def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

flask_thread = threading.Thread(target=run_flask, daemon=True)
flask_thread.start()


# =========================================================
# 2. ROBOT SETUP & MOVE FUNCTIONS (Oorspronkelijk)
# =========================================================

def move_function_1(current_position, doelpositie):
    verschil = doelpositie - current_position
    return (verschil / 3.0) * 4000

def move_function_2(current_position, doelpositie):
    def naar_hoek_rad(x):
        graden = 60.0 - (x + 1) * 6.0
        return math.radians(max(0.0, min(60.0, graden)))
    return round((naar_hoek_rad(doelpositie) - naar_hoek_rad(current_position)) / (math.pi / 2) * 460)
    
def move_function_3(current_position, doelpositie):
    def naar_hoek_rad(x):
        graden = 60.0 - (x + 1) * 6.0
        return math.radians(max(0.0, min(60.0, graden)))
    return round((naar_hoek_rad(doelpositie) - naar_hoek_rad(current_position)) / (math.pi / 2) * 460)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

motor1 = Motor(17, 22, 0.001, move_function_1, 3, 0, 0)
motor2 = Motor(20, 21, 0.01, move_function_2, 8, 8, -1) 
motor3 = Motor(23, 24, 0.05, move_function_3, 6, -2, -2)

MODE_SCHUIN_AFSTAND = 1
MODE_RECHT_AFSTAND = 8
STAP = 0.05 

lock = threading.Lock()
huidige_mode = Mode.MODE_SCHUIN

OORSPRONG_M1 = motor1.current_position
OORSPRONG_M2 = motor2.current_position
OORSPRONG_M3 = motor3.current_position
motor3_pos = OORSPRONG_M3

automatisch_actief = False

def clamp(v, min_v, max_v): return max(min_v, min(max_v, v))

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        c = sys.stdin.read(1)
        if c == '\x1b': c += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return c

def compensate(doel3):
    p1 = 0
    p2 = MODE_SCHUIN_AFSTAND if huidige_mode == Mode.MODE_SCHUIN else MODE_RECHT_AFSTAND
    fout = doel3 - (p1 + p2)
    
    while fout < 0:
        if p2 <= motor2.min_positie: break
        p2 -= 0.5
        fout += 0.5
        
    while fout > 3:
        if p2 >= motor2.max_positie: break
        p2 += 0.5
        fout -= 0.5
        
    p1_final = fout if 0 <= fout <= 3 else p1
    return p1_final, p2

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

def terug_naar_oorsprong():
    global motor3_pos
    print("\n[RESET] Systeem wacht 3 seconden en keert terug naar startpositie...")
    time.sleep(3)
    with lock:
        t1 = threading.Thread(target=motor1.beweeg_naar, args=(OORSPRONG_M1,))
        t2 = threading.Thread(target=motor2.beweeg_naar, args=(OORSPRONG_M2,))
        t3 = threading.Thread(target=motor3.beweeg_naar, args=(OORSPRONG_M3,))
        t1.start(); t2.start(); t3.start()
        t1.join(); t2.join(); t3.join()
    motor3_pos = OORSPRONG_M3
    print("[RESET] Motoren terug in oorsprong.\n")


# =========================================================
# 3. CAMERA GEBASEERDE AUTOMATISCHE LOGICA
# =========================================================

def automatische_loop():
    global motor3_pos, huidige_mode, automatisch_actief
    
    automatisch_actief = True
    print(f"\n[START] Schenkproces gestart. M3 beweegt...")

    # CONFIGURATIE CAMERA-LOGICA
    MAX_VOLUME_THRESHOLD = 85.0 # Als de pint voor 85% vol zit, moet hij stoppen/corrigeren
    MIN_SCHUIM_THRESHOLD = 15.0 # Minder dan 15% schuim? Pint rechterop zetten.
    MAX_SCHUIM_THRESHOLD = 25.0 # Meer dan 25% schuim? Pint schuiner zetten.

    while motor3_pos < motor3.max_positie and automatisch_actief:
        
        # --- CAMERA EVALUATIE ---
        if camera.detectie_status == "succes":
            vol = camera.huidig_totaal_pct
            schuim = camera.huidig_schuim_pct

            # 1. Bijna overlopen logica (Vervangt de 's' toets)
            if vol > MAX_VOLUME_THRESHOLD:
                print(f"\n[ALARM] Pint is voor {vol:.1f}% vol! M3 trekt omhoog en pint gaat recht.")
                STAPPEN_50_DELTA = 1.63
                motor3_pos = clamp(motor3_pos + STAPPEN_50_DELTA, motor3.min_positie, motor3.max_positie)
                
                # Forceer recht voor de laatste milliliters zodat het mooi afschuimt
                huidige_mode = Mode.MODE_RECHT 
                beweeg_sync(motor3_pos)
                
                print("[INFO] Systeem wacht 4 seconden om het schuim te laten bezinken...")
                time.sleep(4) 
                print("[INFO] Verder met inkappen...")
                continue # Skip de rest van deze loop-iteratie, haal een nieuw camera frame

            # 2. Schuim bijsturen logica
            vorige_mode = huidige_mode
            if schuim < MIN_SCHUIM_THRESHOLD:
                huidige_mode = Mode.MODE_RECHT
            elif schuim > MAX_SCHUIM_THRESHOLD:
                huidige_mode = Mode.MODE_SCHUIN
            
            if vorige_mode != huidige_mode:
                print(f"[CAMERA] Schuim is nu {schuim:.1f}%. Motor past hoek aan naar: {huidige_mode}")


        # --- MOTOR BEWEGING ---
        # Reguliere piepkleine stap richting eindpunt (schenken)
        motor3_pos = clamp(motor3_pos + STAP, motor3.min_positie, motor3.max_positie)
        beweeg_sync(motor3_pos)
        
        time.sleep(0.005)

    if motor3_pos >= motor3.max_positie:
        print(f"\n[SUCCES] Uiterste punt van M3 bereikt. Pint is getapt!")
    
    automatisch_actief = False
    terug_naar_oorsprong()

# =========================================================
# 4. MAIN PROGRAMMA
# =========================================================

print("\n=== Robot Besturing Geactiveerd ===")
print("Webcam stream actief op: http://[JOUW_PI_IP]:5000")
print("Druk op [SPATIE] of [ENTER] om het automatisch schenken te starten.")
print("Druk op [Q] om te stoppen.")

beweeg_sync(motor3_pos)

try:
    while True:
        k = get_key()

        if k in ('q', 'Q', '\x03'):
            if automatisch_actief:
                automatisch_actief = False
            else:
                print("\nProgramma handmatig gestopt.")
                terug_naar_oorsprong()
            break

        elif k in (' ', '\r', '\n'):
            if not automatisch_actief:
                threading.Thread(target=automatische_loop, daemon=True).start()
            else:
                print("[AANWIJZING] Het proces loopt al.")

finally:
    if automatisch_actief:
        terug_naar_oorsprong()
    GPIO.cleanup()