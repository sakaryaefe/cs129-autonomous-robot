import board
import busio
from adafruit_pn532.i2c import PN532_I2C
import threading
import time
import traceback

class NFC_Sensor:
	def __init__(self):
		# Initialize the hardware.
		self.i2c = busio.I2C(board.SCL, board.SDA)
		self.pn532 = PN532_I2C(self.i2c, debug=False)
		self.pn532.SAM_configuration()
		# Clear the last read data.
		self.last_read = None
		# Start the worker thread.
		#print("Starting NFC thread...")
		self.reading = True
		self.thread = threading.Thread(name="NFCThread", target=self.run)
		self.thread.start()
	
	def run(self):
		while self.reading:
			# Attempt an NFC read.
			uid = self.pn532.read_passive_target(timeout=0.2)
			# Only save if we have something.
			if uid is not None:
				self.last_read = hash(tuple(uid))
			# Wait 50ms before re-attempting.
			time.sleep(0.05)
	
	def read(self):
		# Grab and then clear the last read.
		data = self.last_read
		self.last_read = None
		# Return the data.
		return data
	
	def shutdown(self):
		self.reading = False
		#print("Waiting for NFC thread to finish...")
		self.thread.join()
		#print("NFC thread returned.")
	
if __name__ == "__main__":
	
	nfc = NFC_Sensor()
	try:
		while True:
			print(nfc.last_read)
	except BaseException as ex:
		print(f"Ending due to exception: {repr(ex)}")
		traceback.print_exc()
	nfc.shutdown()