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
intake = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)
endgame = DigitalOut(brain.three_wire_port.b)
inertial = Inertial(Ports.PORT1)
rotation_sensor = Encoder(brain.three_wire_port.c)
flywheel = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)


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
kP = 0.1
kD = 0.015
kI = 0
turnkP = 0
turnkI = 0
turnkD = 0

################
#AUTON FUNCTIONS
################
def drivePID(dist, maxSpd):
    prevError = 0
    distDeg = (dist / 3.25) * 360
    totalError = 0
    leftMotorGroup.set_position(0, DEGREES)
    leftMotor.set_position(0, DEGREES)
    rightMotorGroup.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)
    counter = 0
    while(True):
        currentPos = (leftMotorGroup.position() + leftMotor.position() + rightMotorGroup.position() + rightMotor.position()) / 4
        
        #proportional
        error = distDeg - currentPos

        #derivative
        der = error - prevError

        totalError += error
        #voltage calculation
        vol = (error * kP) + (der * kD)+ (totalError * kI)
        
        if vol > maxSpd:
            vol = maxSpd

        #set voltage to motors
        leftMotorGroup.spin(FORWARD, vol, VOLT)
        leftMotor.spin(FORWARD, vol, VOLT)
        rightMotorGroup.spin(FORWARD, vol, VOLT)
        rightMotor.spin(FORWARD, vol, VOLT)

        #print values
        print(currentPos/360*3.25, error, vol, rightMotorGroup.position(DEGREES), leftMotorGroup.position(DEGREES))

        #break when reach target
        if error <  1 and error > -1 and error:
            if counter >=20:
                break
            else:
                counter +=1
        else:
            counter = 0
        if controller_1.buttonB.pressing():
            break
        
        prevError = error
        

    return

def turnPID(theta, lSpd, rSpd):
    prevError = 0
    totalError = 0
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
        vol = (error * turnkP) + (der * turnkD) + (totalError * turnkI)
        
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
        brain.screen.clear_screen()
        brain.screen.set_cursor(0,0)
        brain.screen.print(currentDeg, error, lVol, rVol)

        #break when reach target
        if error < 0.1 and error > - 0.1:
            break
    inertial.set_rotation(0, DEGREES)
    return

def intakeSpin(deg):
    while(True):
        intake.spin(REVERSE, 10, VOLT)
        if rotation_sensor.position(DEGREES) == deg - 3:
            intake.stop()




################
#COMPETITION
################
def pre_autonomous():
    leftMotorGroup.set_stopping(BRAKE)
    leftMotor.set_stopping(BRAKE)
    rightMotorGroup.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)
    intake.set_stopping(BRAKE)

def autonomous():
    #drivePID(dist, maxVol) ; turnPID(theta, lSpd, rSpd) ; intakeSpin(deg)
    # flywheelThread = Thread(flywheelControl)
    s=True

def driver_control():
    while(True):
        
        if controller_1.buttonA.pressing():
            intake.spin(FORWARD, 11, VOLT)
            drivePID(24, 8)
            intake.stop()
        
        if controller_1.buttonUp.pressing():
            kP += 0.05
        if controller_1.buttonDown.pressing():
            kP -= 0.05
        if controller_1.buttonRight.pressing():
            kD += 0.05
        if controller_1.buttonLeft.pressing():
            kD -= 0.05
competition = Competition(driver_control, autonomous)
pre_autonomous()
