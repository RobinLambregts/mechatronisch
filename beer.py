import cv2
import numpy as np
from picamera2 import Picamera2
import time

# -----------------------------
# Camera setup
# -----------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}  # RGB888 voor correcte kleuren
)
picam2.configure(config)

# Zet automatische witbalans aan
picam2.set_controls({"AwbEnable": True})

picam2.start()
time.sleep(1)  # laat camera opstarten

print("Camera gestart, druk 'q' om te stoppen")

# -----------------------------
# Kleurgrenzen HSV
# -----------------------------
# Bier geel/bruin
lower_beer = np.array([15, 50, 50])
upper_beer = np.array([35, 255, 255])

# Schuim wit
lower_foam = np.array([0, 0, 200])
upper_foam = np.array([180, 50, 255])

while True:
    # Capture frame en converteer naar BGR voor OpenCV
    frame = picam2.capture_array()

    # Optionele kleurcorrectie als geel nog niet goed is
    b, g, r = cv2.split(frame)
    r = cv2.addWeighted(r, 1.05, r, 0, 0)  # rood iets versterken
    g = cv2.addWeighted(g, 1.0, g, 0, 0)
    b = cv2.addWeighted(b, 0.95, b, 0, 0)  # blauw iets verlagen
    frame = cv2.merge([b, g, r])

    # Convert naar HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Maak maskers
    mask_beer = cv2.inRange(hsv, lower_beer, upper_beer)
    mask_foam = cv2.inRange(hsv, lower_foam, upper_foam)

    combined_mask = cv2.bitwise_or(mask_beer, mask_foam)

    # Vind contouren van het glas
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    overlay = frame.copy()  # overlay zodat originele kleuren zichtbaar blijven

    for cnt in contours:
        if cv2.contourArea(cnt) > 500:  # filter kleine ruis
            # Teken exacte contour van het glas
            cv2.drawContours(overlay, [cnt], -1, (0, 255, 0), 2)

            # Maak masker van deze contour
            mask_cnt = np.zeros_like(combined_mask)
            cv2.drawContours(mask_cnt, [cnt], -1, 255, -1)

            # Bier en schuim apart binnen de contour
            beer_pixels = cv2.bitwise_and(frame, frame, mask=cv2.bitwise_and(mask_cnt, mask_beer))
            foam_pixels = cv2.bitwise_and(frame, frame, mask=cv2.bitwise_and(mask_cnt, mask_foam))

            # Overlay toevoegen zonder andere kleuren te maskeren
            overlay = cv2.addWeighted(overlay, 1.0, beer_pixels, 0.7, 0)
            overlay = cv2.addWeighted(overlay, 1.0, foam_pixels, 0.7, 0)

    cv2.imshow("Bierglas Detectie Contour", overlay)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
