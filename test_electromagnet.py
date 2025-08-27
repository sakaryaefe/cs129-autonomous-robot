import pigpio
import time
from electromagnet import ElectroMagnet  # Assuming this is your class file
import constants

def test():
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon not running. Run: sudo pigpiod")
        return
    
    magnet = ElectroMagnet(pi)
    
    print("Magnet ON")
    magnet.on()
    time.sleep(2)
    
    print("Magnet OFF")
    magnet.off()
    time.sleep(2)
    
    print("Test done")
    pi.stop()

if __name__ == "__main__":
    test()