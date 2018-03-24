#!/usr/bin/python
import smbus
import math

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
while True:	
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
