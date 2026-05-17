from Motor import Motor, Mode
import RPi.GPIO as GPIO
import time
import math

def main():
    print("Start programma...")
    
    def move_function_1(current_position, doelpositie):
        verschil = doelpositie - current_position
        return (verschil / 3.0) * 4000

    def move_function_2(current_position, doelpositie):
        def naar_hoek_rad(x):
            graden = 60.0 - (x + 1) * 6.0
            graden = max(0.0, min(60.0, graden))  # clamp
            return math.radians(graden)

        hoek_huidig = naar_hoek_rad(current_position)
        hoek_doel   = naar_hoek_rad(doelpositie)

        delta_rad   = hoek_doel - hoek_huidig

        return round(delta_rad / (math.pi / 2) * 460)

    def move_function_3(current_position, doelpositie):
        def naar_hoek_rad(x):
            graden = 60.0 - (x + 1) * 6.0
            graden = max(0.0, min(60.0, graden))  # clamp
            return math.radians(graden)

        hoek_huidig = naar_hoek_rad(current_position)
        hoek_doel   = naar_hoek_rad(doelpositie)

        delta_rad   = hoek_doel - hoek_huidig

        return round(delta_rad / (math.pi / 2) * 460)
        
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    motor1 = Motor(dir_pin=17, pulse_pin=22, step_size=0.001, move_function=move_function_1, max_positie=3, startpositie=0, min_positie=-3)
    motor2 = Motor(dir_pin=20, pulse_pin=21, step_size=0.01, move_function=move_function_2, max_positie=8, startpositie=8, min_positie=0)
    motor3 = Motor(dir_pin=23, pulse_pin=24, step_size=0.1, move_function=move_function_3, max_positie=9, startpositie=-1, min_positie=-1)

    motor1.beweeg_naar(-3)

if __name__ == "__main__":
    main()