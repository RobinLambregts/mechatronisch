import time
import RPi.GPIO as GPIO
from enum import Enum


class Mode(Enum):
    MODE_SCHUIN = 1
    MODE_RECHT = 2


class Motor:
    def __init__(self, dir_pin, pulse_pin, step_size, move_function, max_positie, startpositie, min_positie):
        self.dir_pin = dir_pin
        self.pulse_pin = pulse_pin
        self.step_size = step_size
        self.move_function = move_function
        self.max_positie = max_positie
        self.min_positie = min_positie

        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.pulse_pin, GPIO.OUT)
        GPIO.output(self.pulse_pin, GPIO.LOW)

        self.current_position = startpositie
        self.mode = Mode.MODE_RECHT

    def zet_aantal_stappen(self, stappen):
        
        print(f"Motor: Beweeg {stappen} stappen.")

        richting = GPIO.HIGH if stappen > 0 else GPIO.LOW
        GPIO.output(self.dir_pin, richting)

        aantal = int(abs(stappen))

        for _ in range(aantal):
            GPIO.output(self.pulse_pin, GPIO.HIGH)
            time.sleep(self.step_size)
            GPIO.output(self.pulse_pin, GPIO.LOW)
            time.sleep(self.step_size)

        print(f"Motor: {stappen} stappen gezet.")

    def set_mode(self, mode):
        self.mode = mode

    def beweeg_naar(self, doelpositie):

        if doelpositie < self.min_positie or doelpositie > self.max_positie:
            raise ValueError("Doelpositie buiten bereik")

        if self.mode == Mode.MODE_SCHUIN:
            stappen = self.move_function(self.current_position, doelpositie)

        elif self.mode == Mode.MODE_RECHT:
            stappen = self.move_function(self.current_position, doelpositie)

        else:
            raise ValueError("Ongeldige modus")

        self.zet_aantal_stappen(stappen)

        self.current_position = doelpositie