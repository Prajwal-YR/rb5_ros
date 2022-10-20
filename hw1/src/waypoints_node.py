#!/usr/bin/env python
from math import sqrt, atan, pi
import rospy
from hw1.msg import Control
import time

pub_joy = rospy.Publisher("/waypoints", Control, queue_size=20)

class Position:
    def __init__(self,x=0,y=0,theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        
    def __str__(self):
        return "x = %4.1f; y = %4.1f; theta = %5.2f"% (self.x, self.y, self.theta)
    
    def is_equal(self, __o):
        return self.x == __o.x and self.y == __o.y and self.theta == __o.theta
    
    def is_same_point(self, __o):
        return self.x == __o.x and self.y == __o.y
    
    def is_vertical(self, __o):
        return self.x == __o.x
    
    def is_same_angle(self, __o):
        return self.theta == __o.theta
    
    def find_distance(self, __o):
        return sqrt((self.x-__o.x)**2+(self.y-__o.y)**2)
    
def find_orientation(initial_position, final_position):
    if initial_position.is_vertical(final_position): # x2-x1 = 0 
        print initial_position,"and",final_position,"are vertical"
        if final_position.y >= initial_position.y: 
            theta = pi/2 
        else: 
            theta = -pi/2
    elif initial_position.is_same_point(final_position):
        theta = 0
    else:
        theta = atan((final_position.y - initial_position.y) / (final_position.x - initial_position.x))
    # print "theta:",theta
    if final_position.x >= initial_position.x and final_position.y >= initial_position.y:
        # print "Q1"
        orientation = theta
    
    elif final_position.x <= initial_position.x and final_position.y >= initial_position.y:
        # print "Q2"
        orientation = pi - theta
        
    elif final_position.x <= initial_position.x and final_position.y <= initial_position.y:
        # print "Q3"
        orientation = -pi + theta
    
    elif final_position.x >= initial_position.x and final_position.y <= initial_position.y:
        # print "Q4"
        orientation = theta
        
    return round(orientation,2)
    
def move(current_bot_position, next_bot_position):
    print "Moving from",current_bot_position,"to",next_bot_position
    if not current_bot_position.is_equal(next_bot_position):
        distance = current_bot_position.find_distance(next_bot_position)
        orientation = find_orientation(current_bot_position, next_bot_position)
        print "Orientation: ",orientation
        rotate_msg = Control(action="Rotate")
        rotate_msg.value = orientation - current_bot_position.theta
        print "Rotate", rotate_msg.value # Rotate this much
        pub_joy.publish(rotate_msg)
        time.sleep(1)
        move_msg = Control(action="Straight")
        move_msg.value = distance
        print "Move",distance # move distance
        pub_joy.publish(move_msg)
        time.sleep(1)
        current_bot_position.x = next_bot_position.x
        current_bot_position.y = next_bot_position.y
        if abs(orientation - next_bot_position.theta) > 0.05: # Rotate further
            rotate_msg = Control(action="Rotate")
            rotate_msg.value = next_bot_position.theta - orientation
            print "Rotate again", rotate_msg.value
            pub_joy.publish(rotate_msg)
            time.sleep(1)
        current_bot_position.theta = next_bot_position.theta
        
        
    
if __name__ == "__main__":
    positions = []
    with open('/root/rosws/src/rb5_ros/hw1/src/waypoints.txt') as f:
        lines = f.readlines()
        for line in lines:
            pos = list(map(float,line.split(',')))
            if len(pos) != 3:
                raise Exception("Check file format!")
            positions.append(Position(*pos))
    # print *positions
    current_bot_position = Position() # Starting 0,0,0
    rospy.init_node("waypoints")
    for next_bot_position in positions:
        move(current_bot_position, next_bot_position)
        time.sleep(2)

        