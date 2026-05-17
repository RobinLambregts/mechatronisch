import cv2
from flask import Flask, Response
from picamera2 import Picamera2
import numpy as np

# =========================================================
# ANALYZER KLASSE
# =========================================================

class BeerGlassAnalyzer:
    def __init__(self, config=None):
        self.config = {
            'liquid': {
                # Aangepast voor de kleur van echt bier (Geel/Goud/Bruin)
                # Hue rond de 10-35 pakt oranje tot geel.
                'lower': np.array([10, 80, 50]), 
                'upper': np.array([35, 255, 255])
            },
            'foam': {
                # V-waarde (helderheid) verhoogd van 160 naar 200.
                # Dit zorgt ervoor dat zwakkere reflecties op het glas genegeerd worden 
                # en hij alleen echt helder wit schuim pakt.
                'lower': np.array([0, 0, 200]),   
                'upper': np.array([180, 50, 255])
            },
            'glass_box_padding': 10
        }
        if config:
            self.config.update(config)
        self.raw_image = None
        self.preprocessed_image = None
        self.results = {}

    def _preprocess(self, image):
        self.raw_image = image.copy()
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return hsv_image

    def _detect_glass(self, hsv_image):
        _, brightness_thresh = cv2.threshold(hsv_image[:,:,2], 100, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(brightness_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # FIX: cv2.contourArea in plaats van cv2.CONTOUR_AREA
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
            return {'status': 'fout', 'message': 'Geen frame ontvangen'}

        self.preprocessed_image = self._preprocess(camera_frame)
        glass_detect_result = self._detect_glass(self.preprocessed_image)
        
        if not glass_detect_result:
            self.results = {'status': 'fout', 'message': 'Glas niet gedetecteerd'}
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
            self.results = {'status': 'fout', 'message': 'Glas oppervlak te klein'}
            return self.results

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
        if self.raw_image is None:
            return None
            
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
                # FIX: cv2.contourArea
                largest_liquid = max(liquid_contours, key=cv2.contourArea)
                cv2.drawContours(visual_img[gy:gy+gh, gx:gx+gw], [largest_liquid], -1, (255, 0, 0), 2)

            foam_contours, _ = cv2.findContours(foam_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if foam_contours:
                # FIX: cv2.contourArea
                largest_foam = max(foam_contours, key=cv2.contourArea)
                cv2.drawContours(visual_img[gy:gy+gh, gx:gx+gw], [largest_foam], -1, (255, 255, 255), 2)

        # Overlay configuratie
        overlay_w, overlay_h = 280, 160
        margin = 20
        ox, oy = w - overlay_w - margin, margin
        
        if ox < 0: ox = 0
        if oy < 0: oy = 0

        overlay_bg = visual_img.copy()
        cv2.rectangle(overlay_bg, (ox, oy), (ox + overlay_w, oy + overlay_h), (50, 50, 50), -1)
        alpha = 0.75
        cv2.addWeighted(overlay_bg, alpha, visual_img, 1 - alpha, 0, visual_img)

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 1
        text_margin = 15
        line_height = 30

        curr_y = oy + text_margin + 15
        cv2.putText(visual_img, "Bier Analyse", (ox + text_margin, curr_y), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        curr_y += line_height + 5
        
        if self.results.get('status') == 'succes':
            res = self.results['data']
            cv2.putText(visual_img, f"Bier:", (ox + text_margin, curr_y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
            cv2.putText(visual_img, f"{res['beer_volume_percent']:.1f}%", (ox + 120, curr_y), font, font_scale, (255, 0, 0), 2, cv2.LINE_AA)
            curr_y += line_height

            cv2.putText(visual_img, f"Schuim:", (ox + text_margin, curr_y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
            cv2.putText(visual_img, f"{res['foam_volume_percent']:.1f}%", (ox + 120, curr_y), font, font_scale, (255, 255, 255), 2, cv2.LINE_AA)
            curr_y += line_height

            cv2.putText(visual_img, f"Totaal:", (ox + text_margin, curr_y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
            cv2.putText(visual_img, f"{res['total_volume_percent']:.1f}%", (ox + 120, curr_y), font, font_scale, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(visual_img, "STATUS: FOUT", (ox + text_margin, curr_y), font, font_scale, (0, 0, 255), 2, cv2.LINE_AA)
            curr_y += line_height
            cv2.putText(visual_img, self.results.get('message', 'Onbekende fout'), (ox + text_margin, curr_y), font, 0.5, (0, 0, 255), thickness, cv2.LINE_AA)

        return visual_img


# =========================================================
# FLASK & CAMERA STREAM
# =========================================================

class Camera:
    def __init__(self, width=640, height=480):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (width, height)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        
        self.analyzer = BeerGlassAnalyzer()

    def generate_frames(self):
        while True:
            try:
                frame_rgb = self.picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                
                self.analyzer.analyze(camera_frame=frame_bgr)
                processed_frame = self.analyzer.get_visual_result()
                
                if processed_frame is None:
                    processed_frame = frame_bgr

                ret, buffer = cv2.imencode('.jpg', processed_frame)
                if not ret:
                    continue
                    
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' +
                    buffer.tobytes() +
                    b'\r\n'
                )
            except Exception as e:
                print(f"Frame error: {e}")
                continue

camera = Camera()
app = Flask(__name__)

@app.route('/')
def video_feed():
    return Response(
        camera.generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)