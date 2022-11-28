#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
BackL_motor_a = Motor(Ports.PORT16, GearSetting.RATIO_18_1, False)
BackL_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
BackL = MotorGroup(BackL_motor_a, BackL_motor_b)
BackR_motor_a = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
BackR_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
BackR = MotorGroup(BackR_motor_a, BackR_motor_b)
FrontL = Motor(Ports.PORT15, GearSetting.RATIO_18_1, False)
FrontR = Motor(Ports.PORT21, GearSetting.RATIO_18_1, False)
Flywheel = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
Intake = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
Endgame = DigitalOut(brain.three_wire_port.a)
# vex-vision-config:begin
eyes__REDPLATE = Signature(1, -2835, -2341, -2588,8907, 9987, 9447,4, 0)
eyes = Vision(Ports.PORT17, 50, eyes__REDPLATE)
# vex-vision-config:end


# wait for rotation sensor to fully initialize
wait(30, MSEC)
#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:
#	Author:
#	Created:
#	Configuration:
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code

def pre_autonomous():
    # actions to do when the program starts
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")
    wait(1, SECONDS)
    # BackL.set_velocity(100,PERCENT)
    # FrontL.set_velocity(100,PERCENT)    
    # BackR.set_velocity(100,PERCENT)
    # FrontR.set_velocity(100,PERCENT)
    # Intake.set_velocity(100,PERCENT)

    BackL.set_velocity(30,PERCENT)
    FrontL.set_velocity(30,PERCENT)    
    BackR.set_velocity(7,PERCENT)
    FrontR.set_velocity(7,PERCENT)
    Intake.set_velocity(100,PERCENT)

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here
    # BackL.spin(FORWARD)
    # FrontL.spin(FORWARD) 
    # BackR.spin(REVERSE)
    # FrontR.spin(REVERSE)
    # wait(0.6,SECONDS)
    # Intake.spin_for(FORWARD,400,DEGREES)
    # BackL.stop()
    # FrontL.stop()
    # BackR.stop()
    # FrontR.stop()
    
    BackL.spin(FORWARD)
    FrontL.spin(FORWARD) 
    BackR.spin(REVERSE)
    FrontR.spin(REVERSE)
    wait(5,SECONDS)
    BackR.set_velocity(30,PERCENT)
    FrontR.set_velocity(30,PERCENT)
    Intake.spin_for(FORWARD,350,DEGREES)
    BackL.stop()
    FrontL.stop()
    BackR.stop()
    FrontR.stop()

def flywheelintakethread(): # Thread for flywheel and intake control
    
    targetMin = 380
    targetShot = 340 

    while True:
        #bing bing controller 
        currentRPM = Flywheel.velocity(RPM)
        if currentRPM < targetShot: #below margin of error
            vol = 10 
        elif currentRPM < targetMin: #below target rpm
            vol = 8
        else: # maintain rpm
            vol = 7
        Flywheel.spin(FORWARD, vol, VOLT)

        #endgame release
        if controller_1.buttonA.pressing() and controller_1.buttonB.pressing():
            Endgame.set(True)
        else:
            Endgame.set(False)

    
def drivethread():
    # Begin project code
    maxVol = 11 #maximum voltage given to motors
    Kp = 0.3 # constant for P
    Ki = 0  # constant for I
    Kd = 0.05 # constant for d
    Goal = 0 # declaring goal as zero because the below line will end in failure if we don't for unknown reasons
    Goal = eyes.take_snapshot(eyes__REDPLATE) #using vision sensor to take a snapshot and find red object
    if Goal is not None: #to prevent errors when red object is not found
        AbsError = (eyes.largest_object().centerX - 190) #position of red object in vision sensor screen after adjustments for aiming
    else:
        AbsError = 0
    #declaring variables used later for aiming
    PastError = AbsError 
    ErrorI = 0
    TotalError = 0
    AimAmount = 0
    ErrorP = 0
    ErrorI = 0
    ErrorD = 0
    while(True):
        #PID calculations for aiming
        Goal = eyes.take_snapshot(eyes__REDPLATE) 
        if controller_1.buttonX.pressing() and Goal is not None:

            AbsError = (eyes.largest_object().centerX - 190)/8
            ErrorP = Kp * AbsError
            TotalError += PastError
            if (TotalError >= 5 ):
                ErrorI = Ki * TotalError
            else:
                ErrorI = 0
                
            ErrorD = Kd * (AbsError - PastError)
            PastError = AbsError
                
            AimAmount = ErrorP + ErrorI + ErrorD

            if (AimAmount >= 5):
                AimAmount = 5
                
            if (AimAmount <= -5):
                AimAmount = -5
        else: 
            AimAmount = 0
            AbsError = 0
        #Math used for curvature driving
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
        #debug testing code
        print(rightSpd * maxVol, leftSpd * maxVol)
        BackR.spin(FORWARD, rightSpd * maxVol-AimAmount, VOLT)
        FrontR.spin(FORWARD, rightSpd * maxVol-AimAmount, VOLT)
        BackL.spin(FORWARD, leftSpd * maxVol+AimAmount, VOLT)
        FrontL.spin(FORWARD, leftSpd * maxVol+AimAmount, VOLT)


        wait(20,MSEC)
        
def user_control():
    # declaring and activating threads
    flyintakethread=Thread(flywheelintakethread) 
    driverthread=Thread(drivethread)
comp = Competition(user_control, autonomous)
pre_autonomous()
