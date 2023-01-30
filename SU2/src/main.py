#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
leftMotorGroup_motor_a = Motor(Ports.PORT18, GearSetting.RATIO_6_1, True)
leftMotorGroup_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
leftMotorGroup = MotorGroup(leftMotorGroup_motor_a, leftMotorGroup_motor_b)
leftMotor = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
rightMotorGroup_motor_a = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)
rightMotorGroup_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)
rightMotorGroup = MotorGroup(rightMotorGroup_motor_a, rightMotorGroup_motor_b)
rightMotor = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)
tilter = DigitalOut(brain.three_wire_port.a)
intake = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
endgame = DigitalOut(brain.three_wire_port.b)
motor_6 = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
inertial = Inertial(Ports.PORT1)
rotation_sensor = Encoder(brain.three_wire_port.c)


# wait for rotation sensor to fully initialize
wait(30, MSEC)
#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode V5 Python Project
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code
################
#CONSTANTS
################
maxVol = 11.5

################
#AUTON FUNCTIONS
################
def drivePID(dist, maxSpd):
    kP = 
    kD = 
    prevError = 0
    distDeg = (dist / 3.25) * 360

    while(True):
        currentPos = (leftMotorGroup.position + leftMotor.position + rightMotorGroup.position + rightMotor.position) / 4
        
        #proportional
        error = distDeg - currentPos

        #derivative
        der = error - prevError

        preError = error
    
        #voltage calculation
        vol = (error * kP) + (der * kD)
        
        if vol > maxSpd:
            vol = maxSpd

        #set voltage to motors
        leftMotorGroup.spin(FORWARD, vol, VOLT)
        leftMotor.spin(FORWARD, vol, VOLT)
        rightMotorGroup.spin(FORWARD, vol, VOLT)
        rightMotor.spin(FORWARD, vol, VOLT)

        #print values
        print(currentPos, error, vol)

        #break when reach target
        if error <  1 and error > -1:
            break
        
    leftMotorGroup.set_position(0, DEGREES)
    leftMotor.set_position(0, DEGREES)
    rightMotorGroup.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)
    return

def turnPID(theta, lSpd, rSpd):
    kP = 
    kI = 
    kD = 
    prevError = 0

    while(True):
        currentDeg = inertial.rotation(DEGREES)

        #proportional
        error = currentDeg - theta

        #derivative
        der = error - prevError

        prevError = error

        #integral
        totalError += error

        #voltage calculation
        vol = (error * kP) + (der * kD) + (totalError * kI)
        
        lVol = vol
        
        if lVol > lSpd:
            lVol = lSpd

        rVol = vol

        if rVol > rSpd:
            rVol = rSpd
        
        #set voltage to motors
        leftMotorGroup.spin(FORWARD, lVol, VOLT)
        leftMotor.spin(FORWARD, lVol, VOLT)
        rightMotorGroup.spin(REVERSE, rVol, VOLT)
        rightMotor.spin(REVERSE, rVol, VOLT)

        #print values
        print(currentDeg, error, lVol, rVol)

        #break when reach target
        if error < 0.1 and error > - 0.1:
            break
    inertial.set_rotation(0, DEGREES)
    return

def intakeSpin(deg)
    while(True):
        intake.spin(REVERSE, 10, VOLT)
        if rotation_sensor.position(DEGREES) == deg - 3
            intake.stop()

################
#DRIVE FUNCTIONS
################
def driveControl():
    while(True):
        forSpd = controller_1.axis3.position() / 100
        curv = controller_1.axis1.position() / 100

        leftSpd = forSpd + abs(forSpd) * curv
        rightSpd = forSpd - abs(forSpd) * curv

        maxSpd = max(leftSpd, rightSpd)

        #normalizes output
        if (maxSpd > 1):
            leftSpd /= maxSpd
            rightSpd /= maxSpd

        #allow point turn
        if forSpd == 0:
            leftSpd = curv
            rightSpd = -curv

        print(rightSpd * maxVol, leftSpd * maxVol)
        rightMotorGroup.spin(FORWARD, rightSpd * maxVol, VOLT)
        rightMotor.spin(FORWARD, rightSpd * maxVol, VOLT)
        leftMotorGroup.spin(FORWARD, leftSpd * maxVol, VOLT)
        leftMotor.spin(FORWARD, leftSpd * maxVol, VOLT)

        wait(20,MSEC)

def tilterControl():
    while(True):
        if controller_1.buttonL1.pressing():
            tilter.set(True)
        elif controller_1.buttonL2.pressing():
            tilter.set(False)
        
        wait(20,MSEC)

def intakeControl():
    while(True):
        if controller_1.buttonR1.pressing():
            intake.spin(FORWARD, 10, VOLT)
        
        if controller_1.buttonR2.pressing():
            intake.spin(REVERSE, 10, VOLT)

        wait(20,MSEC)

def flywheelControl():
    a = True

def endgameControl():
    while(True):
        if controller_1.buttonB.pressing() and controller_1.buttonY.pressing():
            endgame.set(True)
        
        wait(20,MSEC)

################
#COMPETITION
################
def pre_autonomous():
    leftMotorGroup.set_stopping(HOLD)
    leftMotor.set_stopping(HOLD)
    rightMotorGroup.set_stopping(HOLD)
    rightMotor.set_stopping(HOLD)
    intake.set_stopping(COAST)

def autonomous():
    #drivePID(dist, maxVol) ; turnPID(theta, lSpd, rSpd) ; intakeSpin(deg)
    # flywheelThread = Thread(flywheelControl)
    intake.spin(FORWARD, 11, VOLT)
    drivePID(48, 10)
    intake.stop()

def driver_control():
    driveThread = Thread(driveControl)
    tilterThread = Thread(tilterControl)
    intakeThread = Thread(intakeControl)
    flywheelThread = Thread(flywheelControl)
    endgameThread = Thread(endgameControl)

competition = Competition(driver_control, autonomous)
pre_autonomous()
