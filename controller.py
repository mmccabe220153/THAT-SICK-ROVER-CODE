import RPi.GPIO as GPIO
import math
import xbox
import time
import threading

'''
GPIO_LED_GREEN = 22
GPIO_LED_BLUE = 27
GPIO_LED_RED = 17

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(GPIO_LED_GREEN, GPIO.OUT)
GPIO.setup(GPIO_LED_RED, GPIO.OUT)
GPIO.setup(GPIO_LED_BLUE, GPIO.OUT)

led_state_green = GPIO.LOW
led_state_red = GPIO.LOW
led_state_blue = GPIO.LOW
'''

class controller():

	def __init__(self):
		self.drive_speed = 0
		self.turn_radius = 99999
		self.green = 0
		self.blue = 0
		self.red = 0
		self.thread_run = True
		self.direction = "Stationary"

	def dataListener(self,joystick):
		while self.thread_run:
			try:
				if not (joy.connected()):
					self.thread_run = False
					break
				speed = 100 * joy.leftY()
				if speed < -25: self.direction = 'Reversing'; self.drive_speed = speed
				if -25 < speed < 25: self.direction = 'Stationary'; self.drive_speed = 0
				if speed > 25: self.direction = 'Forward'; self.drive_speed = speed

				turn = 10*joy.rightX()
				if turn > 1: self.turn_radius = (turn*(-23))+250
				if -1 < turn < 1: self.turn_radius = 250
				if turn < -1: self.turn_radius = (turn*(-23))-250

				if(joy.A()):
					self.green = 1
				else:
					self.green = 0
				if(joy.B()):
					self.red = 1
				else:
					self.red = 0
				if(joy.X()):
					self.blue = 1
				else:
					self.blue = 0
			except KeyboardInterrupt:
				self.thread_run = False
'''
def GreenLightOn():
	led_state_green = GPIO.HIGH
	GPIO.output(GPIO_LED_GREEN, led_state_green)
def RedLightOn():
	led_state_red = GPIO.HIGH
	GPIO.output(GPIO_LED_RED, led_state_red)
def BlueLightOn():
	led_state_blue = GPIO.HIGH
	GPIO.output(GPIO_LED_BLUE, led_state_blue)
def GreenLightOff():
	led_state_green = GPIO.LOW
	GPIO.output(GPIO_LED_GREEN, led_state_green)
def RedLightOff():
	led_state_red = GPIO.LOW
	GPIO.output(GPIO_LED_RED, led_state_red)
def BlueLightOff():
	led_state_blue = GPIO.LOW
	GPIO.output(GPIO_LED_BLUE, led_state_blue)
'''

def driveUpdater(Controller):

	print "Drive: %d, Direction: %s Turn:%d" %(int(Controller.drive_speed),(Controller.direction),int(Controller.turn_radius))
	
	'''
	GreenLightOn() if Controller.green == 1	else GreenLightOff()
	RedLightOn() if Controller.red == 1 else RedLightOff()
	BlueLightOn() if Controller.blue == 1 else BlueLightOff()
	'''
	
def main():
	myController = controller()
	dataThread = threading.Thread(target = myController.dataListener, args =(joy, ))
	thread_manager = True

	while True:
		try:
			myController.thread_run = True
			if (joy.connected() and thread_manager):
				dataThread.start()
				thread_manager = False
			if (joy.connected()):
				driveUpdater(myController)
			else:
				thread_manager = True
				myController.thread_run = False
			time.sleep(0.05)
		except KeyboardInterrupt:
			myController.thread_run = False

	joy.close()


if __name__ == "__main__":
	joy = xbox.Joystick()
	main()
