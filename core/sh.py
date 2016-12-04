import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import thread

#SETTINGS_FILE = "RTIMULib"

#print("Using settings file " + SETTINGS_FILE + ".ini")
#if not os.path.exists(SETTINGS_FILE + ".ini"):
#  print("Settings file does not exist, will be created")

#s = RTIMU.Settings(SETTINGS_FILE)
#imu = RTIMU.RTIMU(s)

#print("IMU Name: " + imu.IMUName())

#if (not imu.IMUInit()):
#    print("IMU Init Failed")
#    sys.exit(1)
#else:
#    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters

#imu.setSlerpPower(0.02)
#imu.setGyroEnable(True)
#imu.setAccelEnable(True)
#imu.setCompassEnable(True)

#poll_interval = imu.IMUGetPollInterval()
#print("Recommended Poll Interval: %dmS\n" % poll_interval)

#while True:
#  if imu.IMURead():
    # x, y, z = imu.getFusionData()
    # print("%f %f %f" % (x,y,z))
#    data = imu.getIMUData()
#    fusionPose = data["fusionPose"]
#    fusionQPose = data["fusionQPose"]
#    compass = data["compass"]
#    #Accel+Gyro 2 FusionPose
#    print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
    #Compass value
    #print("r: %f p: %f y: %f" % (math.degrees(fusionQPose[0]), math.degrees(fusionQPose[1]), math.degrees(fusionQPose[2])))
    #Compass direct value
    #print("Compass: %f %f %f" % (compass[0], compass[1], compass[2]))

    #print(data)
#    time.sleep(poll_interval*1.0/1000.0)

class SensorHandler():
	SETTINGS_FILE = "RTIMULib"
	s = ""
	imu = 0

	rotation=[0,0,0]
	relatedrotation=[-2.4,0.15,7.255]
	relatedgyro=[0,0,0]
	compass=0.0
	gyro=[0,0,0]
	relatedcompass=0
	altitude=0.0
	groundheight=0

	doupdate=False
	poll_interval = 0

	lastgyro=[[0,0,0],[0,0,0],[0,0,0]]
	onefifteen=1.0/15
	
	def init(self):
		self.s = RTIMU.Settings(self.SETTINGS_FILE)
		self.imu = RTIMU.RTIMU(self.s)
		if (not self.imu.IMUInit()):
			print("IMU Init Failed")
			
		else:
			print("IMU Init Succeeded")
			self.imu.setSlerpPower(0.02)
			self.imu.setGyroEnable(True)
			self.imu.setAccelEnable(True)
			self.imu.setCompassEnable(True)
			self.doupdate=True
			time.sleep(1)
			self.poll_interval = self.imu.IMUGetPollInterval()
			thread.start_new_thread(self.update_thread,(self,))

	def __init__(self):
		pass
	def update_thread(self,donotuse):
		while self.doupdate:
			#print("du qu")
			if self.imu.IMURead():
				data = self.imu.getIMUData()
				#print(data)
				fusionPose = data["fusionPose"]
				fusionQPose = data["fusionQPose"]
				self.rotation=(math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2]))
				self.compass= (math.degrees(fusionQPose[0]), math.degrees(fusionQPose[1]), math.degrees(fusionQPose[2]))
				
				for i in range(0,2):
					for j in range(0,3):
						self.lastgyro[i][j]=self.lastgyro[i+1][j]
				self.lastgyro[2]=self.gyro

				self.gyro=data["gyro"]
				
			time.sleep(self.poll_interval*1.0/1000.0)
	def getRotation(self):
		rtrotation=[0,0,0]
		for i in range(0,3):
			rtrotation[i]=self.rotation[i]+self.relatedrotation[i]
			if rtrotation[i]>180:
				rtrotation[i]=rtrotation[i]-360
			if rtrotation[i]<-180:
				rtrotation[i]=360+rtrotation[i]
			rtrotation[i]=-rtrotation[i]
		return rtrotation
	def getRotationOriginal(self):
		return self.rotation
	def getCompassPose(self):
		print(self.compass)
	def getAltitude(self):
		pass
	def getGyroOriginal(self):
		return self.gyro
	def getGyro(self):
		rtn=[0,0,0]
		rtn[0]=self.lastgyro[0][0]*self.onefifteen+self.lastgyro[1][0]*self.onefifteen*2+self.lastgyro[2][0]*self.onefifteen*4+self.gyro[0]*self.onefifteen*8+self.relatedgyro[0]
		rtn[1]=self.lastgyro[0][1]*self.onefifteen+self.lastgyro[1][1]*self.onefifteen*2+self.lastgyro[2][1]*self.onefifteen*4+self.gyro[1]*self.onefifteen*8+self.relatedgyro[1]
		rtn[2]=self.lastgyro[0][2]*self.onefifteen+self.lastgyro[1][2]*self.onefifteen*2+self.lastgyro[2][2]*self.onefifteen*4+self.gyro[2]*self.onefifteen*8+self.relatedgyro[2]
		rtn[0]=-rtn[0]
		rtn[1]=-rtn[1]
		rtn[2]=-rtn[2]
		return rtn
	def shutdown(self):
		self.doupdate=False

#sh=SensorHandler()
#sh.init()
#time.sleep(1)
#poll_interval = sh.imu.IMUGetPollInterval()
#print(poll_interval*1.0/100.0)
#while True:
	#sh.update()
	#print(sh.getRotation())
	#print(sh.getGyro())
	#sh.getCompassPose()

