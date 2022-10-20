#!/usr/bin/env python
from math import copysign, pi
import time
from megapi import MegaPi
from hw1.msg import Control
import rospy

MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left

STRAIGHT_SCALE = 1.5
ROTATE_SCALE = 3.15/pi

class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left   

    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) + 
              " MBR: " + repr(MBR) + 
              " MFL: " + repr(MFL))


    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) + 
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))
        try:
            self.bot.motorRun(self.mfl,vfl)
            self.bot.motorRun(self.mfr,vfr)
            self.bot.motorRun(self.mbl,vbl)
            self.bot.motorRun(self.mbr,vbr)
        except Exception as e:
            print(e)


    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(-speed, speed, -speed, speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        self.setFourMotors(speed, speed, speed, speed)
        


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speed, speed, -speed, -speed)

    
    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )
    
    def close(self):
        self.bot.close()
        self.bot.exit()
    


def control_callback(control_msg):
    print control_msg, control_msg.action
    if control_msg.action == "Straight":
        mpi_ctrl.carStraight(100)
        time.sleep(control_msg.value*STRAIGHT_SCALE)
        mpi_ctrl.carStop()
    elif control_msg.action == "Rotate":
        rotate_speed = int(copysign(46,control_msg.value))
        if rotate_speed > 0:
            
            mpi_ctrl.carRotate(rotate_speed+1)
        else:
            mpi_ctrl.carRotate(rotate_speed)
        time.sleep(abs(control_msg.value)*ROTATE_SCALE)
        mpi_ctrl.carStop()
    

if __name__ == "__main__":
    
    rospy.init_node('megapi_controller')
    mpi_ctrl = MegaPiController()
    time.sleep(1)
    rospy.Subscriber('/waypoints', Control, control_callback, queue_size=20) 
    rospy.spin()