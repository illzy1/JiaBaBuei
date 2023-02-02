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
inertial = Inertial(Ports.PORT11)
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

################
#AUTON FUNCTIONS
################
def drivePID(dist, maxSpd):
    kP = 0.05
    kI = 0
    kD = 0.34
    prevError = 0
    distDeg = (dist / 3.25) * 360
    totalError = 0
    leftMotorGroup.set_position(0, DEGREES)
    leftMotor.set_position(0, DEGREES)
    rightMotorGroup.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)
    startvol = 2
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
        if startvol < vol:
            startvol += 0.3
            vol = startvol

        #set voltage to motors
        leftMotorGroup.spin(FORWARD, vol, VOLT)
        leftMotor.spin(FORWARD, vol, VOLT)
        rightMotorGroup.spin(FORWARD, vol, VOLT)
        rightMotor.spin(FORWARD, vol, VOLT)

        #print values
        print(currentPos/360*3.25, error, vol, startvol, der)

        #break when reach target
        if error <  1 and error > -1 and error:
            if counter >= 20:
                break
            else:
                counter +=1
        else:
            counter = 0
        if controller_1.buttonB.pressing():
            break
        
        prevError = error

        wait(10, MSEC)
        
    return

def turnPID(theta, lSpd, rSpd):
    turnkP = 0.1
    turnkI = 0.00
    turnkD = 0.045
    prevError = 0
    totalError = 0
    counter = 0
    sign = theta/abs(theta)
    inertial.set_heading(0, DEGREES)
    while(True):
        wait(10, MSEC)
        currentDeg = inertial.heading(DEGREES)
        if currentDeg > 180:
            currentDeg = abs(currentDeg - 360)
        if theta != 0:
            sign = theta/abs(theta)
        #proportional
        error = abs(theta) - currentDeg

        #derivative
        if (error != 0):
            der = error - prevError
        else:
            der = 0
            
        prevError = error

        #integral
        totalError += error

        #calculate motor voltage
        lVol = (error * turnkP) + (der * turnkD) + (totalError * turnkI)
        rVol = (error * turnkP) + (der * turnkD) + (totalError * turnkI)
        
        if lVol > lSpd:
            lVol = lSpd
        
        if rVol > rSpd:
            rVol = rSpd
        
        #set voltage to motors
        if (theta > 0):
            leftMotorGroup.spin(FORWARD, lVol, VOLT)
            leftMotor.spin(FORWARD, lVol, VOLT)
            rightMotorGroup.spin(REVERSE, rVol, VOLT)
            rightMotor.spin(REVERSE, rVol, VOLT)
        elif (sign < 0):
            leftMotorGroup.spin(REVERSE, lVol, VOLT)
            leftMotor.spin(REVERSE, lVol, VOLT)
            rightMotorGroup.spin(FORWARD, rVol, VOLT)
            rightMotor.spin(FORWARD, rVol, VOLT)  

        #print values
        print(currentDeg, error, lVol, rVol)
        brain.screen.clear_screen()
        brain.screen.set_cursor(0,0)
        brain.screen.print(inertial.heading(DEGREES), error, lVol, rVol)

        #break when reach target
        if error < 0.2 and error > - 0.2:
            if counter >= 20:
                break
            else:
                counter +=1
        else:
                counter = 0
    inertial.set_rotation(0, DEGREES)
    return

def intakeSpin(deg):
    while(True):
        intake.spin(REVERSE, 10, VOLT)
        if rotation_sensor.position(DEGREES) == deg - 3:
            intake.stop()

def turn(deg, Spd):
    if deg < 0:
        deg = deg + 360
    while(True):
        currentDeg = inertial.heading(DEGREES)
        if currentDeg > 180:
            currentDeg = abs(currentDeg - 360)
        if currentDeg < abs(deg) and deg > 0:
            leftMotorGroup.spin(FORWARD, Spd, VOLT)
            leftMotor.spin(FORWARD, Spd, VOLT)
            rightMotorGroup.spin(REVERSE, Spd, VOLT)
            rightMotor.spin(REVERSE, Spd, VOLT)
            print(currentDeg)
            wait(10,MSEC)
        elif currentDeg < abs(deg) and deg < 0:
            leftMotorGroup.spin(REVERSE, Spd, VOLT)
            leftMotor.spin(REVERSE, Spd, VOLT)
            rightMotorGroup.spin(FORWARD, Spd, VOLT)
            rightMotor.spin(FORWARD, Spd, VOLT)
        else:
            return

def angCorrect(ang):
    if (ang < 0):
        return 360 + ang
    elif(ang >= 360):
        return ang-360
    else:
        return ang

def drive(vol):
        leftMotorGroup.spin(FORWARD, vol, VOLT)
        leftMotor.spin(FORWARD, vol, VOLT)
        rightMotorGroup.spin(FORWARD, vol, VOLT)
        rightMotor.spin(FORWARD, vol, VOLT)  

def stop():
        leftMotorGroup.stop()
        leftMotor.stop()
        rightMotorGroup.stop()
        rightMotor.stop()

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
            intake.spin(FORWARD, 11, VOLT)
        elif controller_1.buttonR2.pressing():
            intake.spin(REVERSE, 11, VOLT)
        else:
            intake.stop()

        wait(20,MSEC)

def flywheelControl():
    targetMin =  330
    targetShot = 300
    flyvar = 7
    while True:
        # bing bing controller
        currentRPM = flywheel.velocity(RPM)
        if currentRPM < targetShot:  # below margin of error
            vol = flyvar+3
        elif currentRPM < targetMin:  # below target rpm
            vol = flyvar
        else:  # maintain rpm
            vol = flyvar-1
        flywheel.spin(FORWARD, vol, VOLT)
        
        #changing target voltage
        if controller_1.buttonUp.pressing() and flyvar < 9:
            flyvar += 0.25
            targetMin+= 10
            targetShot+=10
        if controller_1.buttonDown.pressing() and flyvar > 1:
            flyvar -= 0.25
            targetMin -= 10
            targetShot -=10
        #print values
        controller_1.screen.clear_screen()
        controller_1.screen.set_cursor(1,1)
        controller_1.screen.print(vol)

def endgameControl():
    while(True):
        if controller_1.buttonB.pressing() and controller_1.buttonY.pressing():
            endgame.set(True)
        
        wait(20,MSEC)

################
#COMPETITION
################
def pre_autonomous():
    leftMotorGroup.set_stopping(BRAKE)
    leftMotor.set_stopping(BRAKE)
    rightMotorGroup.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)
    intake.set_stopping(BRAKE)
    intake.set_velocity(500, RPM)
    inertial.calibrate()

def autonomous():
    #drivePID(dist, maxVol) ; turnPID(theta, lSpd, rSpd) ; intakeSpin(deg)
    wait(3, SECONDS)
    flywheelThread = Thread(flywheelControl)
    drive(3)                                        #drive to roller
    wait(0.3, SECONDS)
    intake.spin_to_position(360, DEGREES, False)    #turn roller
    wait(2, SECONDS)
    stop()                                          #stop drive
    drive(-3)                                       #drive back
    wait(0.4, SECONDS)
    stop()
    turn(-170, 3)                                   #turn to goal
    wait(0.5, SECONDS)
    intake.spin_to_position(-60, DEGREES)           #index
    wait(1, SECONDS)
    intake.spin_to_position(-60, DEGREES)           #index
    wait(1, SECONDS)
    intake.spin_to_position(-60, DEGREES)           #index
    wait(1, SECONDS)
    

def driver_control():
    leftMotorGroup.set_stopping(COAST)
    leftMotor.set_stopping(COAST)
    rightMotorGroup.set_stopping(COAST)
    rightMotor.set_stopping(COAST)
    driveThread = Thread(driveControl)
    tilterThread = Thread(tilterControl)
    intakeThread = Thread(intakeControl)
    flywheelThread = Thread(flywheelControl)
    endgameThread = Thread(endgameControl)

competition = Competition(driver_control, autonomous)
pre_autonomous()
