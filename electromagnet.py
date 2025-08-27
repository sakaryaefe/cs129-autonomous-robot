import time
import traceback
import pigpio
import numpy as np
import constants

class ElectroMagnet:
    def __init__(self, io):
        self.io = io
        # Fix 1: Fixed typo - "constansts" should be "constants"
        self.pin = constants.PIN_MAGNET # GPIO PIN 1 (physical pin 28)
        
        # Fix 2: Use pigpio.OUTPUT instead of self.io.OUTPUT
        io.set_mode(self.pin, pigpio.OUTPUT)
        
        self.off()
        
    def on(self):
        self.io.write(self.pin, 1)
        print(f"Electromagnet ON - Pin {self.pin} set to HIGH")
        
    def off(self):
        self.io.write(self.pin, 0)
        print(f"Electromagnet OFF - Pin {self.pin} set to LOW")