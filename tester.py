from sshsmudles import Dist

mySensor = Dist()
while True:
	x = mySensor.hazAvoid()
	if x == 1:
		print "too close!"
	elif x == -1:
		print "sensor error!"
		break
		time.sleep(0.005)
	time.sleep(0.05)
