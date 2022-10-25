#!/usr/bin/env python
import sys
from turtle import position
import rospy
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray
import numpy as np
from tf.transformations import euler_from_matrix, quaternion_matrix
"""
The class of the pid controller.
"""


class PIDcontroller:

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.timestep = 0.05
        self.maximumValue = 0.5

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.target = np.array(state)

    def get_current_state_from_april_tag(self, currentState):
        """
        get the current state of the bot from april tags
        by using transformations if tags are detected else echo
        the currenState
        """
        global detections
        for detection in detections:
            quaternion = np.array([
                detection.pose.orientation.x, detection.pose.orientation.y,
                detection.pose.orientation.z, detection.pose.orientation.w
            ])
            at_t = np.array([[detection.pose.position.x],
                             [detection.pose.position.y],
                             [detection.pose.position.z]])
            tag_id = detection.id
            c_rot = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
            c_t = np.array([[0.05], [0.015], [0.145]])
            rTc = np.vstack([np.hstack([c_rot, c_t]), [0, 0, 0, 1]])
            at_rot = quaternion_matrix(quaternion)[:3, :3]
            cTat = np.vstack([np.hstack([at_rot, at_t]), [0, 0, 0, 1]])
            atTr = np.linalg.inv(np.dot(rTc, cTat))
            print(tag_id)
            if tag_id == 8:
                at8_rot = c_rot
                at8_t = np.array([[2], [0], [0]])
                wTat = np.vstack([np.hstack([at8_rot, at8_t]), [0, 0, 0, 1]])
            elif tag_id == 5:
                at5_rot = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])
                at5_t = np.array([[0], [2], [0]])
                wTat = np.vstack([np.hstack([at5_rot, at5_t]), [0, 0, 0, 1]])
            elif tag_id == 3:
                # if len(detections) > 1: continue
                at3_rot = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
                at3_t = np.array([[1], [2.85], [0]])
                wTat = np.vstack([np.hstack([at3_rot, at3_t]), [0, 0, 0, 1]])
            elif tag_id == 4:
                at4_rot = np.array([[-1, 0, 0], [0, 0, -1], [0, -1, 0]])
                wTat = np.vstack(
                    [np.hstack([at4_rot, [[0], [-1], [0]]]), [0, 0, 0, 1]])
            else:
                print("Uknown April tag " + str(tag_id))
                return currentState
            wTr = np.dot(wTat, atTr)
            [x, y, z] = np.dot(wTr, np.array([[0], [0], [0],
                                              [1]]))[:3].flatten()
            theta = euler_from_matrix(wTr[:3, :3])[-1]
            # print([x,y,theta])
            currentState = np.array([x, y, theta])
            print "cs in get_cs" + str(currentState)
        return currentState

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        currentState = self.get_current_state_from_april_tag(currentState)
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)
        # print "e in update" + str(e)
        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value.
        resultNorm = np.linalg.norm(result)
        if (resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result


def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0]
    twist_msg.linear.y = desired_twist[1]
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg


def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]),
                   np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]),
                   np.cos(current_state[2]), 0.0], [0.0, 0.0, 1.0]])
    return np.dot(J, twist)


def april_tag_handler(message):
    """
    Callback handler for april tags. Sets global variable detections
    """
    global detections
    detections = message.detections


if __name__ == "__main__":
    global detections
    detections = []
    # import time
    rospy.init_node("hw2")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)
    sub_april_tag = rospy.Subscriber("/apriltag_detection_array",
                                     AprilTagDetectionArray,
                                     april_tag_handler,
                                     queue_size=1)
    # rospy.spin()
    rate = rospy.Rate(15)

    waypoint = np.array([
                         [0.0, 0.0, 0],
                         [1.0, 0.0, 0.0],
                         [1.0, 2.0, np.pi],
                         [0.0, 0.0, 0.0]
                         ])
    wp = 0
    # init pid controller
    pid = PIDcontroller(0.0185, 0.0015, 0.10)

    # init current state
    current_state = np.array([0.0, 0.0, 0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough,
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print "move to way point", wp
        pub_twist.publish(genTwistMsg([0, 0, 0]))
        rospy.sleep(3)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        rospy.sleep(0.05)

        # Code used to tune April tags position
        
        # while True:
        #     current_state = pid.get_current_state_from_april_tag(current_state)
        #     print(current_state)
        #     rospy.sleep(1)

        # update the current state
        current_state += update_value

        while (
                np.linalg.norm(pid.getError(current_state, wp)[:2]) >= 0.15

        ):  # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            # pub_twist.publish(genTwistMsg([0, 0, 0]))
            #print(coord(update_value, current_state))
            rate.sleep()
            # update the current state
            current_state += update_value
            current_state = pid.get_current_state_from_april_tag(current_state)
            print(current_state)

    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.0])))
