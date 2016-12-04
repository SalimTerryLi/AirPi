import RPIO as GPIO
from RPIO import PWM
import pigpio
import time
import sys

class MotorController:
	servo=0
	servo1=0
	isInited=False
	pi=0

	def __init__(self):
		#PWM.set_loglevel(PWM.LOG_LEVEL_ERRORS)
		self.pi=pigpio.pi()
		
	def initPWM(self):
		#self.servo=PWM.Servo(0,20000,1)
		#self.servo1=PWM.Servo(0,20000,1)
		#self.servo.set_servo(18,0)
		#self.servo.set_servo(12,0)
		#self.servo1.set_servo(13,0)
		#self.servo1.set_servo(19,0)
		self.pi.set_servo_pulsewidth(18,1000)
		self.pi.set_servo_pulsewidth(12,1000)
		self.pi.set_servo_pulsewidth(13,1000)
		self.pi.set_servo_pulsewidth(19,1000)

		self.isInited=True
		time.sleep(1)

	def initMotor(self,power):
		if self.isInited:
			print("")

	def shutdownPWM(self):
		if self.isInited:
			#self.servo.stop_servo(18)
			#self.servo.stop_servo(12)
			#self.servo1.stop_servo(13)
			#self.servo1.stop_servo(19)
			self.pi.set_servo_pulsewidth(18,0)
			self.pi.set_servo_pulsewidth(12,0)
			self.pi.set_servo_pulsewidth(13,0)
			self.pi.set_servo_pulsewidth(19,0)

			self.isInited=False
	def myprint(self,msg):
		sys.stdout.write(msg+"\n");
	
	
	def changePower(self,channel,power):
		if not self.isInited:
			return
		##Motor1=1,Motor2=2,Motor3=4,Motor4=8,sum for the input
		if channel>=8:
			##Motor4
			channel=channel-8
			#self.servo1.set_servo(19,power*10+1000)
			self.pi.set_servo_pulsewidth(19,1000+power*10)
		if channel>=4:
			##Motor3
			channel=channel-4
			#self.servo1.set_servo(13,power*10+1000)
			self.pi.set_servo_pulsewidth(13,1000+power*10)
		if channel>=2:
			##Motor2
			channel=channel-2
			#self.servo.set_servo(12,power*10+1000)			
			self.pi.set_servo_pulsewidth(12,1000+power*10)
		if channel>=1:
			##Motor1
			channel=channel-1
			#self.servo.set_servo(18,power*10+1000)			
			self.pi.set_servo_pulsewidth(18,1000+power*10)
		if channel!=0:
			self.myprint("Channel number error!")
		##self.servo.stop_servo(18)
		##self.servo.stop_servo(12)
		##self.servo.stop_servo(13)
		##self.servo.stop_servo(19)
	def setupPowerArrange(self):
		pass
	def testPowerOutput(self):
		cnt=0
		timeb=time.time()
		for dc in range(0, 333):
			cnt=cnt+1
			self.changePower(0,dc)
			#self.myprint("dc:"+str(dc))
			time.sleep(0.05);
		time.sleep(1)
		print("\n")
		print(cnt)
		print((time.time()-timeb))

#motorctl=MotorController()
#motorctl.initPWM()
#motorctl.setupPowerArrange()
#motorctl.testPowerOutput()
#motorctl.shutdownPWM()
