#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import math
import tf
import tf2_ros
from tf.transformations import quaternion_matrix
import matplotlib.pyplot as plt

UPDATE_EVERY_STEPS = 3
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
        self.timestep = 0.1
        self.maximumValue = 0.02
        self.S = np.zeros((3, 1))
        self.sigma = np.zeros((3, 3))
        self.current_detections = {}
        self.tag_to_row = {}
        self.q = 0.01
        self.r = 0.01

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

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
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

    def kalman_predict(self, twist_cord, l):

        # twist_cord[2] = (self.S[2] + np.pi) % (2 * np.pi) - np.pi
        # print("Twist in predict",twist_cord, twist_cord.shape)
        detections = filter(lambda i: 'camera_' in i, l.getFrameStrings())
        for detection in detections:
            try:
                (trans, rot) = l.lookupTransform(detection,
                                                 "marker_" + detection[-1],
                                                 rospy.Time())
                # print("Trans [0]",trans[0])
                matrix = quaternion_matrix(rot)
                angle = math.atan2(matrix[1][2], matrix[0][2])
                self.current_detections[detection] = np.array([[trans[0]],
                                                               [trans[1]],
                                                               [angle]])
                if detection not in self.tag_to_row:
                    self.tag_to_row[detection] = len(self.S)
                    x_r, y_r, theta_r = self.S[0:3, 0]
                    s_for_new = np.array([[
                        trans[0] * np.cos(theta_r) +
                        trans[1] * np.cos(theta_r) + x_r
                    ],
                                          [
                                              -trans[0] * np.sin(theta_r) +
                                              trans[1] * np.cos(theta_r) + y_r
                                          ], [angle + theta_r]])
                    print(s_for_new.shape, self.S.shape)
                    self.S = np.vstack([self.S, s_for_new])
                    self.sigma = np.hstack([
                        np.vstack([self.sigma,
                                   np.zeros((3, len(self.sigma)))]),
                        np.zeros((len(self.sigma) + 3, 3))
                    ])
                    print("After predict", self.S.shape, self.sigma.shape)

            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException, tf2_ros.TransformException):
                print("Lookup error")

        twist_matrix = np.zeros(self.S.shape)
        twist_matrix[0:3, 0] = twist_cord
        self.S = self.S + twist_matrix
        self.sigma = self.sigma + self.q * np.eye(len(self.sigma))
        self.S[2, 0] = (self.S[2, 0] + np.pi) % (2 * np.pi) - np.pi

    def kalman_update(self):
        if len(self.current_detections) == 0:
            return
        z_t = np.vstack(self.current_detections.values())
        H = np.zeros((len(z_t), len(self.S)))
        theta = self.S[2]
        T_robot = np.array([[-np.cos(theta), -np.sin(theta), 0],
                            [np.sin(theta), -np.cos(theta), 0], [0, 0, -1]])
        for i, detection in enumerate(self.current_detections.keys()):
            H[i * 3:(i + 1) * 3, 0:3] = T_robot
            j = self.tag_to_row[detection]
            H[i * 3:(i + 1) * 3, j:j + 3] = -T_robot

        S_INV = np.dot(np.dot(H, self.sigma), H.T) + self.r * np.eye(len(z_t))
        K = np.dot(np.dot(self.sigma, H.T), S_INV)
        self.S = self.S + np.dot(K, z_t - np.dot(H, self.S))
        self.sigma = np.dot(np.eye(len(self.S)) - np.dot(K, H), self.sigma)

    def getCurrentPos(self):
        return self.S[0:3, 0]


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
    """
    Convert the twist into the car coordinate
    """
    J = np.array([[np.cos(current_state[2]),
                   np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]),
                   np.cos(current_state[2]), 0.0], [0.0, 0.0, 1.0]])
    return np.dot(J, twist)

def plot_ground_truth():
    pass

def plot_state(state):
    plt.plot(state[0,0],state[1,0],marker='o',color = 'skyblue')
    for i in range(3, len(state), 3):
        plt.plot(state[i,0], state[i+1,0], marker='x', color = 'maroon')
    plt.savefig('/root/rosws/src/rb5_ros/rb5_control/src/state_graph.png', dpi=120)


if __name__ == "__main__":
    import time
    rospy.init_node("hw2")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    listener = tf.TransformListener()

    waypoint = np.array([
        [0.0, 0.0, 0.0],
        #  [1.0, 0.0, 0.0],
        [1.0, 0.0, np.pi / 2],
        #  [1.0, 1.0, np.pi / 2],
        [1.0, 1.0, np.pi],
        #  [0.0, 1.0, np.pi],
        [0.0, 1.0, -np.pi / 2],
        #  [0.0, 0.0, -np.pi / 2],
        [0.0, 0.0, 0.0]
    ])

    # init pid controller
    pid = PIDcontroller(0.1, 0.005, 0.005)

    # init current state
    current_state = np.array([0.0, 0.0, 0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough,
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        pid.kalman_predict(update_value, listener)
        current_state += update_value
        # current_state = pid.getCurrentPos()
        i = 0
        while (
                np.linalg.norm(pid.getError(current_state, wp)) > 0.05
        ):  # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            i += 1
            pid.kalman_predict(update_value, listener)
            if i % UPDATE_EVERY_STEPS == 0:
                pid.kalman_update()
            current_state += update_value
            # current_state = pid.getCurrentPos()
            plot_state(pid.S)
            print(pid.S, pid.S.shape)
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.0])))
