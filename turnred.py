# region VEXcode Generated Robot Configuration
from vex import *
import urandom
# pyright: reportMissingModuleSource=false
# Brain should be defined by default
brain = Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
BackL_motor_a = Motor(Ports.PORT16, GearSetting.RATIO_18_1, False)
BackL_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
backL = MotorGroup(BackL_motor_a, BackL_motor_b)
BackR_motor_a = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
BackR_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
backR = MotorGroup(BackR_motor_a, BackR_motor_b)
frontL = Motor(Ports.PORT15, GearSetting.RATIO_18_1, False)
frontR = Motor(Ports.PORT21, GearSetting.RATIO_18_1, True)
flywheel = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)
intake = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
endgame = DigitalOut(brain.three_wire_port.a)
inertial = Inertial(Ports.PORT6)
# vex-vision-config:begin
#eyes__REDPLATE = Signature(1, 10547, 11941, 11244,-2595, -1927, -2261,6.6, 0)
#eyes__REDPLATE = Signature(1, -4207, -3591, -3899,9055, 10013, 9534,6.6, 0)
eyes__REDPLATE = Signature(1, 11889, 13107, 12498,-1517, -625, -1071,6.8, 0)
eyes__BLUEPLATE = Signature(2, -3479, -2907, -3193,8863, 9947, 9405,8.2, 0)
eyes = Vision(Ports.PORT17, 50, eyes__REDPLATE, eyes__BLUEPLATE)
# vex-vision-config:end


# wait for rotation sensor to fully initialize
wait(30, MSEC)
# endregion VEXcode Generated Robot Configuration

# ------------------------------------------
#
# 	Project:
# 	Author:
# 	Created:
# 	Configuration:
#
# ------------------------------------------

# Library imports
from vex import *

# Begin project code


def pre_autonomous():
    # actions to do when the program starts
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")

    intake.set_velocity(100, PERCENT)
    
    backL.set_stopping(BRAKE)
    frontL.set_stopping(BRAKE)
    backR.set_stopping(BRAKE)
    frontR.set_stopping(BRAKE)
    
    
def motorSpin(a,b):
    backL.spin(a)
    frontL.spin(a)
    backR.spin(b)
    frontR.spin(b)
    
def motorStop():
    backL.stop()
    frontL.stop()
    backR.stop()
    frontR.stop()

def motorVel(a, b):
    backL.set_velocity(a, PERCENT)
    frontL.set_velocity(a, PERCENT)
    backR.set_velocity(b, PERCENT)
    frontR.set_velocity(b, PERCENT)

def autonFlywheel():
    targetShot = 375
    targetMin = 415
    flywheel.spin(FORWARD, 8.5, VOLT)
    currentRPM = flywheel.velocity(RPM)
    while(True):
        if currentRPM < targetShot:  # below margin of error
            vol = 12
        elif currentRPM < targetMin:  # below target rpm
            vol = 7.4
        else:  # maintain rpm
            vol = 6.4
        print(currentRPM)
    
def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    #flywheel thread
    flywheel = Thread(autonFlywheel)
    #turn to roller
    motorVel(43, 12)
    motorSpin(FORWARD, FORWARD)
    wait(2, SECONDS)
    #turn roller
    motorVel(15, 15)
    motorSpin(FORWARD, FORWARD)
    wait(0.5, SECONDS)
    intake.spin_for(FORWARD, 350, DEGREES)
    #turn towards middle
    motorVel(30, 30)
    motorSpin(REVERSE, REVERSE)
    wait(0.5, SECONDS)
    motorStop()
    motorSpin(FORWARD, REVERSE)
    wait(1.102, SECONDS)
    motorStop()
    wait(0.55, SECONDS)
    intake.spin(REVERSE)
    wait(0.2, SECONDS)
    intake.stop()
    wait(1.3, SECONDS)
    intake.spin(REVERSE)
    
    
def flywheelthread():  # Thread for flywheel
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

def drive():
    
    # Begin project code
    maxVol = 11  # maximum voltage given to motors
    Kp = 0.4  # constant for P
    Ki = 0  # constant for I
    Kd = 0.2  # constant for d
    Goal = 0  # declaring goal as zero because the below line will end in failure if we don't for unknown reasons
    Goal = eyes.take_snapshot(
        eyes__REDPLATE
    )  # using vision sensor to take a snapshot and find red object
    if Goal is not None:  # to prevent errors when red object is not found
        AbsError = (
            eyes.largest_object().centerX - 190
        )  # position of red object in vision sensor screen after adjustments for aiming
    else:
        AbsError = 0
    # declaring variables used later for aiming
    PastError = AbsError
    ErrorI = 0
    TotalError = 0
    AimAmount = 0
    ErrorP = 0
    ErrorI = 0
    ErrorD = 0
    while True:
        #intake controls
        if (controller_1.buttonL1.pressing()):
            intake.spin(REVERSE,11,VOLT)
        elif (controller_1.buttonR1.pressing()):
            intake.spin(FORWARD,11,VOLT)
        else:
            intake.stop()
        
        # endgame release
        if controller_1.buttonA.pressing() and controller_1.buttonB.pressing():
            endgame.set(True)
        else:
            endgame.set(False)
        # PID calculations for aiming
        if controller_1.buttonL2.pressing():
            Goal = eyes.take_snapshot(eyes__REDPLATE)
            print("RED")
        elif controller_1.buttonR2.pressing():
            Goal = eyes.take_snapshot(eyes__BLUEPLATE)
            print("BLUE")
            
        if controller_1.buttonL2.pressing() and Goal is not None:

            AbsError = (eyes.largest_object().centerX - 175) / 8
            ErrorP = Kp * AbsError
            TotalError += PastError
            if TotalError >= 5:
                ErrorI = Ki * TotalError
            else:
                ErrorI = 0

            ErrorD = Kd * (AbsError - PastError)
            PastError = AbsError

            AimAmount = ErrorP + ErrorI + ErrorD

            if AimAmount >= 5:
                AimAmount = 5

            if AimAmount <= -5:
                AimAmount = -5
        else:
            AimAmount = 0
            AbsError = 0
        # Math used for curvature driving
        forSpd = controller_1.axis3.position() / 100
        curv = controller_1.axis1.position() / 100

        leftSpd = forSpd + abs(forSpd) * curv
        rightSpd = forSpd - abs(forSpd) * curv

        maxSpd = max(leftSpd, rightSpd)

        # normalizes output
        if maxSpd > 1:
            leftSpd /= maxSpd
            rightSpd /= maxSpd
            

        # allow point turn
        if forSpd < 0.1 and forSpd>-0.1:
            leftSpd = curv
            rightSpd = -curv
        # debug testing code
        backR.spin(FORWARD, rightSpd * maxVol - AimAmount, VOLT)
        frontR.spin(FORWARD, rightSpd * maxVol - AimAmount, VOLT)
        backL.spin(FORWARD, leftSpd * maxVol + AimAmount, VOLT)
        frontL.spin(FORWARD, leftSpd * maxVol + AimAmount, VOLT)

        wait(20, MSEC)
def user_control():
    # declaring and activating threads
    backL.set_stopping(COAST)
    frontL.set_stopping(COAST)
    backR.set_stopping(COAST)
    frontR.set_stopping(COAST)
    flywheelThread = Thread(flywheelthread)
    driveThread = Thread(drive) 


comp = Competition(user_control, autonomous)
pre_autonomous()
