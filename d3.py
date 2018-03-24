import RPi.GPIO as GPIO
import time 
from threading import Thread
from multiprocessing import Process
GPIO.setmode(GPIO.BCM)

TRIG = 18
ECHO = 23
TRIG2 = 24
ECHO2 = 25

'''
LED_R = 13
LED_Y = 12
LED_G = 16
LED_B = 5
LED_W = 6
LED_R2 = 17
LED_Y2 = 27
LED_G2 = 22
LED_B2 = 4
LED_W2 = 26
'''

try:
	class Dist():
		def __init__(self):
			self.distFlag = 0
			self.count = 1
			self.lower = 30
			self.upper = 50
			self.prev = -1
			self.thresh = 50
			
			GPIO.setwarnings(False)
			
			GPIO.setup(TRIG,GPIO.OUT)
			GPIO.setup(ECHO,GPIO.IN)
			
			GPIO.setup(TRIG2,GPIO.OUT)
			GPIO.setup(ECHO2,GPIO.IN)
			
			'''
			GPIO.setup(LED_R,GPIO.OUT)
			GPIO.setup(LED_Y,GPIO.OUT)
			GPIO.setup(LED_G,GPIO.OUT)
			GPIO.setup(LED_B,GPIO.OUT)
			GPIO.setup(LED_W,GPIO.OUT)	
			GPIO.setup(LED_R2,GPIO.OUT)
			GPIO.setup(LED_Y2,GPIO.OUT)
			GPIO.setup(LED_G2,GPIO.OUT)
			GPIO.setup(LED_B2,GPIO.OUT)
			GPIO.setup(LED_W2,GPIO.OUT)	
			'''

		def distSense(self):
			
			GPIO.output(TRIG, False)
			time.sleep(0.000001)
			GPIO.output(TRIG, True)
			time.sleep(0.00001)
			GPIO.output(TRIG, False)
			
			GPIO.output(TRIG2, False)
			time.sleep(0.000001)
			GPIO.output(TRIG2, True)
			time.sleep(0.00001)
			GPIO.output(TRIG2, False)
			
			while GPIO.input(ECHO)==0 or GPIO.input(ECHO2)==0:
				StartTime = time.time()
			while GPIO.input(ECHO)==1 or GPIO.input(ECHO2)==1:
				StopTime = time.time()
				
			TimeElapsed = StopTime - StartTime
			distance = TimeElapsed * 17150
			distance = round(distance, 2)

			return distance
			
		def hazAvoid(self):
			
			distance = self.distSense()
			
			if type(distance) != float:
				return -1

			if distance > self.upper:
				self.count = 0
			
			elif distance < self.lower:
				self.count += 1
			
			if self.count >= self.thresh:
				self.flag = 1
			else:
				self.flag = 0
				
			if distance >= self.upper:
				'''
				GPIO.output(LED_R, GPIO.LOW)
				GPIO.output(LED_Y, GPIO.LOW)
				GPIO.output(LED_G, GPIO.LOW)
				GPIO.output(LED_B, GPIO.LOW)
				GPIO.output(LED_W, GPIO.LOW)
				GPIO.output(LED_R2, GPIO.LOW)
				GPIO.output(LED_Y2, GPIO.LOW)
				GPIO.output(LED_G2, GPIO.LOW)
				GPIO.output(LED_B2, GPIO.LOW)
				GPIO.output(LED_W2, GPIO.LOW)
				'''
			else:
				'''
				GPIO.output(LED_R, GPIO.HIGH)
				GPIO.output(LED_Y, GPIO.HIGH)
				GPIO.output(LED_G, GPIO.HIGH)
				GPIO.output(LED_B, GPIO.HIGH)
				GPIO.output(LED_W, GPIO.HIGH)
				GPIO.output(LED_R2, GPIO.HIGH)
				GPIO.output(LED_Y2, GPIO.HIGH)
				GPIO.output(LED_G2, GPIO.HIGH)
				GPIO.output(LED_B2, GPIO.HIGH)
				GPIO.output(LED_W2, GPIO.HIGH)
				'''
			

			return self.flag

	mySensor = Dist()
	while True:
		x = mySensor.hazAvoid()
		if x == 1:
			print "too close!"
			break
		elif x == -1:
			print "sensor error!"
			break
			time.sleep(0.005)
except:
	GPIO.cleanup()
	raise
