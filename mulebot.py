#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time
#import datetime
#channel = 6

interruptLeftCount  = -2
interruptRightCount = -2
startTimeLeft  = 0
startTimeRight = 0
lastTimeLeft   = 0
lastTimeRight  = 0

# Begin GPIO
# From http://learn.sparkfun.com/tutorials/
#                raspberry-gpio/python-rpiigpio-example

import RPi.GPIO as GPIO

pwmEnablePin       = 16 # Broadcom pin 16
motor1DirectionPin = 20 # Broadcom pin 20
motor2DirectionPin = 21 # Broadcom pin 21

motorForward = GPIO.HIGH
motorReverse = GPIO.LOW


dcMotorLeftMotor  = 0
dcMotorRightMotor = 1
    
dcMotorPWMDurationLeft = 0
dcMotorPWMDurationRight = 0

laserDetectLeftPin  = 6
laserDetectRightPin = 5

def motorDirection(motorPin, direction):
#  print "motorPin: ", motorPin
#  print "direction: ",  direction
  GPIO.output(motorPin, direction)


def motorsDirection(direction):
  print direction
  if direction == 'r' or direction == 'R':
    motorDirection(motor1DirectionPin, motorReverse)
    motorDirection(motor2DirectionPin, motorReverse)
    print "Direction reverse"
  else:
    motorDirection(motor1DirectionPin, motorForward)
    motorDirection(motor2DirectionPin, motorForward)
    print "Direction forward"

def dcMotorLeftTurn(duration):
  print "From dcMotorLeftTurn: ", dcMotorPWMDurationLeft
  tempPWMDurationLeft = dcMotorPWMDurationLeft * 70 / 100  # 98
  pwm.setPWM(dcMotorLeftMotor, 0, tempPWMDurationLeft)

  # Duration of the turn  
  time.sleep(duration)

  # Go straight
  pwm.setPWM(dcMotorLeftMotor, 0, dcMotorPWMDurationLeft)


def dcMotorRightTurn(duration):
  tempPWMDurationRight = dcMotorPWMDurationRight * 70 / 100
  pwm.setPWM(dcMotorRightMotor, 0, tempPWMDurationRight)

  # Duration of the turn  
  time.sleep(duration)

  # Go straight
  pwm.setPWM(dcMotorRightMotor, 0, dcMotorPWMDurationRight)


motorMaxRPM = 12
def motorSpeed(speedRPM):

  global dcMotorPWMDurationLeft
  global dcMotorPWMDurationRight

#  print "motorSpeed RPM: ", speedRPM
  if speedRPM > motorMaxRPM:
    speedRPM = 12
  if speedRPM < 0:
    speedRPM = 0

# Left motor
  pwmDuration = 4096 * speedRPM / motorMaxRPM - 1
  pwm.setPWM(dcMotorLeftMotor, 0, pwmDuration)
  dcMotorPWMDurationLeft = pwmDuration

# Right motor
  #Adjust for right motor being faster
  pwmDuration = pwmDuration * 9851 / 10000  # x97.019779 percent 98.519113
  pwm.setPWM(dcMotorRightMotor, 0, pwmDuration)
  dcMotorPWMDurationRight = pwmDuration


def myInt(channel):
  global interruptLeftCount
  global interruptRightCount
  global startTimeLeft
  global startTimeRight


  now = time.time()

  if channel == laserDetectLeftPin:
    interruptLeftCount += 1
    elapsedTime = 0.0
#    print channel, interruptLeftCount

    if interruptLeftCount == 1:
      startTimeLeft = now

    if interruptLeftCount > 0:
      elapsedTime = now - startTimeLeft
    print "Left ", channel, now, interruptLeftCount, elapsedTime


  if channel == laserDetectRightPin:
    interruptRightCount += 1
    elapsedTime = 0.0
#    print channel, interruptRightCount

    if interruptRightCount == 1:
      startTimeRight = now

    if interruptRightCount > 0:
      elapsedTime = now - startTimeRight
    print "Right ", channel, now, interruptRightCount, elapsedTime



# Begin main code
dcMotorPWMDurationLeft = 0
dcMotorPWMDurationRight = 0

# Pin Setup:
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
GPIO.setup(pwmEnablePin,       GPIO.OUT)
GPIO.setup(motor1DirectionPin, GPIO.OUT)
GPIO.setup(motor2DirectionPin, GPIO.OUT)

GPIO.output(pwmEnablePin,       GPIO.LOW )




GPIO.setup(laserDetectLeftPin,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(laserDetectRightPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#GPIO.add_event_detect(laserDetectLeftPin,  GPIO.FALLING, callback=myInt)
#GPIO.add_event_detect(laserDetectRightPin, GPIO.FALLING, callback=myInt)



def setMotorsDirection(direction):
  if direction == 'f' or direction == 'F':
    motorDirection(motor1DirectionPin, motorForward)
    motorDirection(motor2DirectionPin, motorForward)
  elif direction == 'r' or direction == 'R':
    motorDirection(motor1DirectionPin, motorReverse)
    motorDirection(motor2DirectionPin, motorReverse)
  else:
    print "ERROR: setMotorsDirection bad parameter: " + direction


def shutdown():
  count = 0
  pwm.setPWM(0, 0, count)
  pwm.setPWM(1, 0, count)

  ### How to use the Enable Pin???
  GPIO.output(pwmEnablePin, GPIO.HIGH)
  GPIO.cleanup()
  print
  print "Bye!"  


def test():
  laserOn = 0
  lastLaserOn = -1

  startTime = time.time()  # This will get overwritten
  finishTime = time.time()
  lastTime = time.time()

  count = 0
  maxEvents = 20
  while count < maxEvents:
    laserOn = GPIO.input(laserDetectLeftPin)
    #print laserOn

    if lastLaserOn == 0:
      if laserOn ==1:
        now = time.time()
        deltaTime = now - lastTime
        if deltaTime > 3.8:
          print deltaTime
          lastTime = now
          if deltaTime > 4.3:
            startTime = now
          else:
            count += 1
            finishTime = now

    lastLaserOn = laserOn

    time.sleep(.01)

  totalDeltaTime = finishTime - startTime
  singleDeltaTime = totalDeltaTime / maxEvents
  print singleDeltaTime, " * ", maxEvents, " = ", totalDeltaTime


#setMotorsDirection('f')
# End GPIO

# ===========================================================================
# Example Code
# ===========================================================================

# Initialise the PWM device using the default address
pwm = PWM(0x40)
# Note if you'd like more debug output you can instead run:
#pwm = PWM(0x40, debug=True)



#servoMin = 4096 / 12  # Min pulse length out of 4096
#servoMax = 4095       # Max pulse length out of 4096

def setServoPulse(channel, pulse):
  pulseLength = 1000000                   # 1,000,000 us per second
  pulseLength /= 60                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000
  pulse /= pulseLength
  pwm.setPWM(channel, 0, pulse)

#count = 1
pwm.setPWMFreq(1000)                        # Set frequency to 1000 Hz






doContinue = True

try:
  while (doContinue):

    # Change speed of motors
    cmd = raw_input(":;<  Command, f/r 0..9, E.g. f5: ")


    if cmd[0] == 'h':
      doContinue = False
    elif cmd[0] == 'p':
      dcMotorLeftTurn (  ord(cmd[1]) - ord('0')  )
    elif cmd[0] == 's':
      dcMotorRightTurn(  ord(cmd[1]) - ord('0')  )
    elif cmd[0] == 't':
      test()
    else:
      direction = cmd[0]
      print direction
      setMotorsDirection(direction)

#      print cmd[1]
      count = ord(cmd[1]) - ord('0')
      motorSpeed(count)
#      time.sleep(1)
#      count += 1
#    if count > motorMaxRPM:
#      count = 0

except KeyboardInterrupt:
  # This statement is meaningless other than it allows the program to
  # drop down to the next line.
  count = 0



shutdown()
# exception keyboard
# cleanup pwm
#pwm.cleanup()
