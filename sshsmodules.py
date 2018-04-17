import smbus
import math
import xbox
import threading
import RPi.GPIO as GPIO
from threading import Thread
from multiprocessing import Process
GPIO.setmode(GPIO.BCM)
import subprocess
import os
import select
import time

class accel():

	# Register 
	power_mgmt_1 = 0x6b
	power_mgmt_2 = 0x6c

	def read_byte(reg):
		return bus.read_byte_data(address, reg) 
		
	def read_word(reg):
		h = bus.read_byte_data(address, reg)
		l = bus.read_byte_data(address, reg+1)
		value = (h << 8) + 1
		return value
		
	def read_word_2c(reg):
		val = read_word(reg) 
		if (val >= 0x8000): 
			return -((65535 - val) + 1) 
		else: 
			return val
				
	def dist(a,b):
		return math.sqrt((a*a)+(b*b))
		
	def get_y_rotation(x,y,z):
		radians = math.atan2(x, dist(y,z))
		return -math.degrees(radians)

	def get_x_rotation(x,y,z):
		radians = math.atan2(y, dist(x,z))
		return math.degrees(radians)
		try:
			bus = smbus.SMBus(1) # bus = smbus.SUBus(0) fuer Revision 1
			address = 0x68		 # via i2cdetect 

			# Aktivieren, um das Modul ansprechen zu koennen 
			bus. write_byte_data(address, power_mgmt_1, 0)
			
			print "Gyroscope"
			print "--------"

			gyroscope_xout = read_word_2c(0x43)
			gyroscope_yout = read_word_2c(0x45)
			gyroscope_zout = read_word_2c(0x47)

			print "gyroscope_xout: ", ("%5d" % gyroscope_xout), "scaled: ", (gyroscope_xout / 131)
			print "gyroscope_yout: ", ("%5d" % gyroscope_yout), "scaled: ", (gyroscope_yout / 131)
			print "gyroscope_zout: ", ("%5d" % gyroscope_zout), "scaled: ", (gyroscope_zout / 131) 

			print 
			print "accelelerationssensor"
			print "---------------------"
			
			acceleleration_xout = read_word_2c(0x3b) 
			acceleleration_yout = read_word_2c(0x3d) 
			acceleleration_zout = read_word_2c(0x3f) 
			
			acceleleration_xout_scaled = acceleleration_xout / 16384.0 
			acceleleration_yout_scaled = acceleleration_yout / 16384.0 
			acceleleration_zout_scaled = acceleleration_zout / 16384.0
			
			print "X Rotation: " , get_x_rotation(acceleleration_xout_scaled, acceleleration_yout_scaled, acceleleration_zout_scaled)
			print "WHY Rotation: " , get_y_rotation(acceleleration_xout_scaled, acceleleration_yout_scaled, acceleleration_zout_scaled)
		except KeyboardInterrupt:
			print("suh")
			break
		except:
			print("error yo")
			break
		
		


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

	def getVals():
		v = 100 * joy.leftY()
		turn = 10*joy.rightX()
		if turn > 1: r = (turn*(-23))+250
		if -1 < turn < 1: r = 250
		if turn < -1: r = (turn*(-23))-250
		return v,r

	joy.close()


	if __name__ == "__main__":
		joy = xbox.Joystick()
	
	
	

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

try:
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
	
	
""" Xbox 360 controller support for Python
11/9/13 - Steven Jacobs
This class module supports reading a connected xbox controller.
It requires that xboxdrv be installed first:
    sudo apt-get install xboxdrv
See http://pingus.seul.org/~grumbel/xboxdrv/ for details on xboxdrv
Example usage:
    import xbox
    joy = xbox.Joystick()         #Initialize joystick
    
    if joy.A():                   #Test state of the A button (1=pressed, 0=not pressed)
        print 'A button pressed'
    x_axis   = joy.leftX()        #X-axis of the left stick (values -1.0 to 1.0)
    (x,y)    = joy.leftStick()    #Returns tuple containing left X and Y axes (values -1.0 to 1.0)
    trigger  = joy.rightTrigger() #Right trigger position (values 0 to 1.0)
    
    joy.close()                   #Cleanup before exit
"""



class Joystick:

    """Initializes the joystick/wireless receiver, launching 'xboxdrv' as a subprocess
    and checking that the wired joystick or wireless receiver is attached.
    The refreshRate determines the maximnum rate at which events are polled from xboxdrv.
    Calling any of the Joystick methods will cause a refresh to occur, if refreshTime has elapsed.
    Routinely call a Joystick method, at least once per second, to avoid overfilling the event buffer.
 
    Usage:
        joy = xbox.Joystick()
    """
    def __init__(self,refreshRate = 30):
        self.proc = subprocess.Popen(['xboxdrv','--no-uinput','--detach-kernel-driver'], stdout=subprocess.PIPE)
        self.pipe = self.proc.stdout
        #
        self.connectStatus = False  #will be set to True once controller is detected and stays on
        self.reading = '0' * 140    #initialize stick readings to all zeros
        #
        self.refreshTime = 0    #absolute time when next refresh (read results from xboxdrv stdout pipe) is to occur
        self.refreshDelay = 1.0 / refreshRate   #joystick refresh is to be performed 30 times per sec by default
        #
        # Read responses from 'xboxdrv' for upto 2 seconds, looking for controller/receiver to respond
        found = False
        waitTime = time.time() + 2
        while waitTime > time.time() and not found:
            readable, writeable, exception = select.select([self.pipe],[],[],0)
            if readable:
                response = self.pipe.readline()
                # Hard fail if we see this, so force an error
                if response[0:7] == 'No Xbox':
                    raise IOError('No Xbox controller/receiver found')
                # Success if we see the following
                if response[0:12].lower() == 'press ctrl-c':
                    found = True
                # If we see 140 char line, we are seeing valid input
                if len(response) == 140:
                    found = True
                    self.connectStatus = True
                    self.reading = response
        # if the controller wasn't found, then halt
        if not found:
            self.close()
            raise IOError('Unable to detect Xbox controller/receiver - Run python as sudo')

    """Used by all Joystick methods to read the most recent events from xboxdrv.
    The refreshRate determines the maximum frequency with which events are checked.
    If a valid event response is found, then the controller is flagged as 'connected'.
    """
    def refresh(self):
        # Refresh the joystick readings based on regular defined freq
        if self.refreshTime < time.time():
            self.refreshTime = time.time() + self.refreshDelay  #set next refresh time
            # If there is text available to read from xboxdrv, then read it.
            readable, writeable, exception = select.select([self.pipe],[],[],0)
            if readable:
                # Read every line that is availabe.  We only need to decode the last one.
                while readable:
                    response = self.pipe.readline()
                    # A zero length response means controller has been unplugged.
                    if len(response) == 0:
                        raise IOError('Xbox controller disconnected from USB')
                    readable, writeable, exception = select.select([self.pipe],[],[],0)
                # Valid controller response will be 140 chars.  
                if len(response) == 140:
                    self.connectStatus = True
                    self.reading = response
                else:  #Any other response means we have lost wireless or controller battery
                    self.connectStatus = False

    """Return a status of True, when the controller is actively connected.
    Either loss of wireless signal or controller powering off will break connection.  The
    controller inputs will stop updating, so the last readings will remain in effect.  It is
    good practice to only act upon inputs if the controller is connected.  For instance, for
    a robot, stop all motors if "not connected()".
    
    An inital controller input, stick movement or button press, may be required before the connection
    status goes True.  If a connection is lost, the connection will resume automatically when the
    fault is corrected.
    """
    def connected(self):
        self.refresh()
        return self.connectStatus

    # Left stick X axis value scaled between -1.0 (left) and 1.0 (right) with deadzone tolerance correction
    def leftX(self,deadzone=4000):
        self.refresh()
        raw = int(self.reading[3:9])
        return self.axisScale(raw,deadzone)

    # Left stick Y axis value scaled between -1.0 (down) and 1.0 (up)
    def leftY(self,deadzone=4000):
        self.refresh()
        raw = int(self.reading[13:19])
        return self.axisScale(raw,deadzone)

    # Right stick X axis value scaled between -1.0 (left) and 1.0 (right)
    def rightX(self,deadzone=4000):
        self.refresh()
        raw = int(self.reading[24:30])
        return self.axisScale(raw,deadzone)

    # Right stick Y axis value scaled between -1.0 (down) and 1.0 (up)
    def rightY(self,deadzone=4000):
        self.refresh()
        raw = int(self.reading[34:40])
        return self.axisScale(raw,deadzone)

    # Scale raw (-32768 to +32767) axis with deadzone correcion
    # Deadzone is +/- range of values to consider to be center stick (ie. 0.0)
    def axisScale(self,raw,deadzone):
        if abs(raw) < deadzone:
            return 0.0
        else:
            if raw < 0:
                return (raw + deadzone) / (32768.0 - deadzone)
            else:
                return (raw - deadzone) / (32767.0 - deadzone)

    # Dpad Up status - returns 1 (pressed) or 0 (not pressed)
    def dpadUp(self):
        self.refresh()
        return int(self.reading[45:46])
        
    # Dpad Down status - returns 1 (pressed) or 0 (not pressed)
    def dpadDown(self):
        self.refresh()
        return int(self.reading[50:51])
        
    # Dpad Left status - returns 1 (pressed) or 0 (not pressed)
    def dpadLeft(self):
        self.refresh()
        return int(self.reading[55:56])
        
    # Dpad Right status - returns 1 (pressed) or 0 (not pressed)
    def dpadRight(self):
        self.refresh()
        return int(self.reading[60:61])
        
    # Back button status - returns 1 (pressed) or 0 (not pressed)
    def Back(self):
        self.refresh()
        return int(self.reading[68:69])

    # Guide button status - returns 1 (pressed) or 0 (not pressed)
    def Guide(self):
        self.refresh()
        return int(self.reading[76:77])

    # Start button status - returns 1 (pressed) or 0 (not pressed)
    def Start(self):
        self.refresh()
        return int(self.reading[84:85])

    # Left Thumbstick button status - returns 1 (pressed) or 0 (not pressed)
    def leftThumbstick(self):
        self.refresh()
        return int(self.reading[90:91])

    # Right Thumbstick button status - returns 1 (pressed) or 0 (not pressed)
    def rightThumbstick(self):
        self.refresh()
        return int(self.reading[95:96])

    # A button status - returns 1 (pressed) or 0 (not pressed)
    def A(self):
        self.refresh()
        return int(self.reading[100:101])
        
    # B button status - returns 1 (pressed) or 0 (not pressed)
    def B(self):
        self.refresh()
        return int(self.reading[104:105])

    # X button status - returns 1 (pressed) or 0 (not pressed)
    def X(self):
        self.refresh()
        return int(self.reading[108:109])

    # Y button status - returns 1 (pressed) or 0 (not pressed)
    def Y(self):
        self.refresh()
        return int(self.reading[112:113])

    # Left Bumper button status - returns 1 (pressed) or 0 (not pressed)
    def leftBumper(self):
        self.refresh()
        return int(self.reading[118:119])

    # Right Bumper button status - returns 1 (pressed) or 0 (not pressed)
    def rightBumper(self):
        self.refresh()
        return int(self.reading[123:124])

    # Left Trigger value scaled between 0.0 to 1.0
    def leftTrigger(self):
        self.refresh()
        return int(self.reading[129:132]) / 255.0
        
    # Right trigger value scaled between 0.0 to 1.0
    def rightTrigger(self):
        self.refresh()
        return int(self.reading[136:139]) / 255.0

    # Returns tuple containing X and Y axis values for Left stick scaled between -1.0 to 1.0
    # Usage:
    #     x,y = joy.leftStick()
    def leftStick(self,deadzone=4000):
        self.refresh()
        return (self.leftX(deadzone),self.leftY(deadzone))

    # Returns tuple containing X and Y axis values for Right stick scaled between -1.0 to 1.0
    # Usage:
    #     x,y = joy.rightStick() 
    def rightStick(self,deadzone=4000):
        self.refresh()
        return (self.rightX(deadzone),self.rightY(deadzone))

    # Cleanup by ending the xboxdrv subprocess
    def close(self):
        os.system('pkill xboxdrv')
        
        
        
