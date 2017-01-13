import thread
import time
import math
import sh
import mc

class INET_err_Out():
	import socket
	socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
	addr=("",0)
	def __init__(self,addr,port):
		self.addr=(addr,port)
	def send(self,data):
		self.socket.sendto(str(data)+"\n",self.addr)

class PID_Attitude():
	KpI=0
	KiI=0
	KdI=0
	KpO=0
	KiO=0
	KdO=0
	KpS=0
	KiS=0
	KdS=0

	errorIRange=10
	errorORange=10

	errorSumRangeO=30
	errorSumRangeI=3
	errorSumRangeS=20

	PID_P_Range=100

	errorSumO=0
	errorSumI=0
	errorSumS=0
	errorGyroLast=0
	lastGyro=0
	errorAngleLast=0

	dbg=INET_err_Out("192.168.1.125",726)

	def PID_Gyro(self,aimd,current):
		errorI = aimd - current
		PID_I_P = self.KpI * errorI
		if errorI > self.errorIRange :
			self.errorSumI = self.errorSumI + self.errorIRange
		elif errorI < -self.errorIRange :
			self.errorSumI = self.errorSumI - self.errorIRange
		else :
			self.errorSumI = self.errorSumI + errorI			
		if self.errorSumI > self.errorSumRangeI :
			self.errorSumI = self.errorSumRangeI
		elif self.errorSumI < -self.errorSumRangeI :
			self.errorSumI = -self.errorSumRangeI
		PID_I_I = self.KiI * self.errorSumI
		gyrodelta = errorI - self.errorGyroLast
		self.errorGyroLast = errorI
		PID_I_D = self.KdI * gyrodelta
		PID_I = PID_I_P + PID_I_I + PID_I_D
		return PID_I

	def PID_Rotation(self,aimd,current,gyro,dt):
		errorO = aimd - current
		PID_O_P = self.KpO * errorO
		if errorO > self.errorORange :
			self.errorSumO = self.errorSumO + self.errorORange
		elif errorO < -self.errorORange :
			self.errorSumO = self.errorSumO - self.errorORange
		else :
			self.errorSumO = self.errorSumO + errorO
		if self.errorSumO > self.errorSumRangeO :
			self.errorSumO = self.errorSumRangeO
		elif self.errorSumO < -self.errorSumRangeO :
			self.errorSumO = -self.errorSumRangeO
		PID_O_I = self.KiO * self.errorSumO
		#angledelta = errorO - self.errorAngleLast
		#self.errorAngleLast = errorO
		angledelta = gyro * dt
		PID_O_D = self.KdO * angledelta
		PID_O = PID_O_P + PID_O_I +PID_O_D

		return PID_O

	def PID_Single(self,aimd,current,gyro,dt):
		errorS = aimd - current
		PID_S_P = self.KpS * errorS
		if PID_S_P > self.PID_P_Range :
			PID_S_P = self.PID_P_Range
		if PID_S_P < -self.PID_P_Range :
			PID_S_P = -self.PID_P_Range
		self.dbg.send(PID_S_P)
		self.errorSumS = self.errorSumS + errorS
		if self.errorSumS > self.errorSumRangeS :
			self.errorSumS = self.errorSumRangeS
		if self.errorSumS < -self.errorSumRangeS :
			self.errorSumS = -self.errorSumRangeS
		PID_S_I = self.KiS * self.errorSumS
		gyrodelta = 0 - ( gyro - self.lastGyro )
		self.lastGyro = gyro
		PID_S_D = self.KdS * gyrodelta
		#angledelta = errorS - self.errorAngleLast
		#self.errorAngleLast = errorS
		#PID_S_D = self.KdS * angledelta
		PID_S = PID_S_P + PID_S_I + PID_S_D
		return PID_S


class PID_Yaw():
	offset=0

	def PID_Yaw(self,aimd,current):
		return offset

class PID_Altitude():
	pass

class PID_RelatedPosition():
	pass

class PID_GPS():
	pass

class FlightCalculator():

	isInited=False
	isRunning=False
	isBasePowerCTL=True
	isGPSAltitudeEnabled=False
	isSonarHeightEnabled=False
	isPressureAltitudeEnabled=False
	isGPSPositionEnabled=False
	isAimdYawEnabled=False
	isProtected=True

	aimdRotation=[0,0,0]#[Pitch,Roll,Yaw]
	aimdPosition=[0,0,0]#[Longitude,Latitude,Altitude | Height]
	aimdPowerBase=0
	aimdPowerYaw=0

	sensor=sh.SensorHandler()
	motor=mc.MotorController()

	pitchPID=PID_Attitude()
	rollPID=PID_Attitude()
	yawPID=PID_Yaw()
	
	def init(self):
		self.isInited=True
		self.sensor.init()
		thread.start_new_thread(self.update_thread,(self,))
		self.motor.initPWM()

	def shutdown(self):
		self.setAimdPowerBase(0)
		self.isInited=False
		self.isRunning=False
		self.pitchPID.errorSumO=0
		self.pitchPID.errorSumI=0
		self.pitchPID.errorSumS=0
		self.pitchPID.errorGyroLast=0
		self.pitchPID.errorAngleLast=0
		self.yawPID.errorSumO=0
		self.yawPID.errorSumI=0
		self.yawPID.errorSumS=0
		self.yawPID.errorGyroLast=0
		self.yawPID.errorAngleLast=0
		self.sensor.shutdown()
		self.motor.shutdownPWM()

	def run(self):
		if self.isInited :
			self.isRunning=True
			self.setAimdPowerBase(20)

	def stop(self):
		if self.isInited :
			self.setAimdPowerBase(0)
			self.isRunning=False

	def setAimdRotation(self,pitch,roll):
		self.aimdRotation[0]=pitch
		self.aimdRotation[1]=roll

	def setAimdYaw(self,yaw):
		self.isAimdYawEnabled=True
		self.aimdRotation[2]=yaw

	def setAimdPowerYaw(self,power):
		self.isAimdYawEnabled=False
		self.aimdPowerYaw=power

	def setAimdHeight(self,height):
		pass

	def setAimdAltitude(self,altitude):
		pass

	def setAimdPosition(self,x,y):
		pass

	def setAimdPowerBase(self,power):
		if self.isRunning :
			self.isBasePowerCTL=True
			self.aimdPowerBase=power

	def update_thread(self,dontuse):
		dt=0.02
		while self.isInited:
			timewhenstarted = time.time()
			
			output=[0,0,0,0]
			
			pitchgyro = self.pitchPID.PID_Rotation(self.aimdRotation[0],self.sensor.getRotation()[0],self.sensor.getGyro()[0],dt)
			pitchmotorvalue = self.pitchPID.PID_Gyro(pitchgyro,self.sensor.getGyro()[0])
			#pitchmotorvalue = self.pitchPID.PID_Gyro(self.aimdRotation[0],self.sensor.getGyro()[0])#Debugging for gyroPID
			#pitchmotorvalue = self.pitchPID.PID_Single(self.aimdRotation[0],self.sensor.getRotation()[0],self.sensor.getGyro()[0],dt)
			output[0]=output[0]+int(pitchmotorvalue)
			output[1]=output[1]+int(pitchmotorvalue)
			output[2]=output[2]-int(pitchmotorvalue)
			output[3]=output[3]-int(pitchmotorvalue)

			rollgyro = self.rollPID.PID_Rotation(self.aimdRotation[1],self.sensor.getRotation()[1],self.sensor.getGyro()[1],dt)
			rollmotorvalue = self.rollPID.PID_Gyro(rollgyro,self.sensor.getGyro()[1])
			#rollmotorvalue = self.rollPID.PID_Gyro(self.aimdRotation[1],self.sensor.getGyro()[1])#Debugging for gyroPID
			#rollmotorvalue = self.rollPID.PID_Single(self.aimdRotation[1],self.sensor.getRotation()[1],self.sensor.getRotation()[1],dt)
			output[0]=output[0]-int(rollmotorvalue)
			output[1]=output[1]+int(rollmotorvalue)
			output[2]=output[2]+int(rollmotorvalue)
			output[3]=output[3]-int(rollmotorvalue)

			#Codes below are not in confortable reading sense. Needing recoding.

			if self.isBasePowerCTL :
				for i in range(0,4):
					output[i]=output[i]+self.aimdPowerBase*10
					if output[i]<100 :
						output[i]=100
					if output[i]>1000 :
						output[i]=1000
			if not self.aimdPowerBase==0 :
				#print(output)
				self.motor.changePower(8,output[0]*0.1)
				self.motor.changePower(1,output[1]*0.1)
				self.motor.changePower(4,output[2]*0.1)
				self.motor.changePower(2,output[3]*0.1)
			else :
				self.motor.changePower(15,0)
			
			timecost = time.time() - timewhenstarted
			
			if timecost > 0.02 :
				print("Tick Too Slowly!")
				dt=timecost
			else :
				time.sleep(0.02 - timecost)
				dt=0.02
			
			if (abs(self.sensor.getRotation()[0])>50) or (abs(self.sensor.getRotation()[1])>50) and self.isProtected :
				self.stop()

	def setKp(self,value):
		#self.pitchPID.KpS=value
		self.rollPID.KpS=value
	def setKi(self,value):
		#self.pitchPID.KiS=value
		self.rollPID.KiS=value
	def setKd(self,value):
		#self.pitchPID.KdS=value
		self.rollPID.KdS=value
	def setPR(self,value):
		self.rollPID.PID_P_Range=value

