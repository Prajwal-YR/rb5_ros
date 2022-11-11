#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import math
import tf
import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_from_euler
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import matplotlib.cm as cm

UPDATE_EVERY_STEPS = 5
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
        self.S = np.array([1 / 3.0, 0, 0]).reshape(-1, 1)  #np.zeros((3, 1))  #
        self.sigma = np.zeros((3, 3))
        self.current_detections = {}
        self.seen_timestamps = {}
        self.tag_to_row = {}
        self.q = 5e-4
        self.r = 5e-4

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

    def kalman_predict(self, twist_cord, l, br):

        # twist_cord[2] = (self.S[2] + np.pi) % (2 * np.pi) - np.pi
        # print("Twist in predict",twist_cord, twist_cord.shape)
        # detections = filter(lambda i: 'marker_' in i, l.getFrameStrings())

        self.current_detections = {}
        tags = []
        for tag_id in [str(i) for i in range(0, 10)]:
            try:
                latest_timestamp = l.getLatestCommonTime(
                    "robot", "marker_" + tag_id)
                if tag_id not in self.seen_timestamps or self.seen_timestamps[
                        tag_id] != latest_timestamp:
                    self.seen_timestamps[tag_id] = latest_timestamp
                    tags.append(tag_id)
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException, tf2_ros.TransformException):
                # print("Can't get timestamp for tag_"+str(tag_id))
                pass
        for tag_id in tags:
            try:
                # print("Tag id", tag_id)
                (trans,
                 rot) = l.lookupTransform("robot", "marker_" + tag_id,
                                          self.seen_timestamps[str(tag_id)])
                # print("Trans [0]",trans[0])

                matrix = quaternion_matrix(rot)
                angle = math.atan2(matrix[1][2], matrix[0][2])
                # print("Seen", self.seen_timestamps.keys(), trans, angle)
                # exit()
                self.current_detections[tag_id] = np.array([[trans[0]],
                                                            [trans[1]],
                                                            [angle]])
                # print("tag to Row", self.tag_to_row,
                #       self.current_detections[tag_id])
                if tag_id not in self.tag_to_row:
                    br.sendTransform((self.S[0, 0], self.S[1, 0], 0),
                                     quaternion_from_euler(0, 0, self.S[2, 0]),
                                     rospy.Time.now(), "robot", "world")
                    l.waitForTransform("world", "marker_" + tag_id,
                                       rospy.Time(), rospy.Duration(2.0))
                    s_trans, s_rot = l.lookupTransform("world",
                                                       "marker_" + tag_id,
                                                       rospy.Time())
                    matrix = quaternion_matrix(s_rot)
                    s_theta = math.atan2(matrix[1][2], matrix[0][2])
                    s_for_new = [[s_trans[0]], [s_trans[1]], [s_theta]]
                    # x_r, y_r, theta_r = self.S[0:3, 0]
                    # T_mat = np.array(
                    #     [[-np.cos(theta_r), -np.sin(theta_r), x_r],
                    #      [np.sin(theta_r), -np.cos(theta_r), y_r], [0, 0, 1]])

                    # s_for_new = np.dot(T_mat,
                    #                    np.array([[trans[0]], [trans[1]], [1]]))
                    # s_for_new[-1, 0] = angle + theta_r
                    self.tag_to_row[tag_id] = len(self.S)
                    # print("Updated tag to row", self.tag_to_row)
                    self.S = np.vstack([self.S, s_for_new])
                    self.sigma = np.hstack([
                        np.vstack([self.sigma,
                                   np.zeros((3, len(self.sigma)))]),
                        np.zeros((len(self.sigma) + 3, 3))
                    ])
                    # print("After predict", self.S.shape, self.sigma.shape)

            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException,
                    tf2_ros.TransformException) as e:
                print("Lookup error for " + tag_id, e)

        twist_matrix = np.zeros(self.S.shape)
        twist_matrix[0:3, 0] = twist_cord
        # print(twist_matrix)
        self.S = self.S + twist_matrix
        self.sigma = self.sigma + self.q * np.eye(len(self.sigma))

        for i in range(0, len(self.S), 3):
            self.S[i + 2, 0] = (self.S[i + 2, 0] + np.pi) % (2 * np.pi) - np.pi

    def kalman_update(self, br):
        if len(self.current_detections) == 0:
            return
        z_t = np.vstack(self.current_detections.values())
        # print('z_t', z_t)
        H = np.zeros((len(z_t), len(self.S)))
        # print("H shape", H.shape)
        theta = self.S[2]
        T_robot = np.array([[-np.cos(theta), -np.sin(theta), 0],
                            [np.sin(theta), -np.cos(theta), 0], [0, 0, -1]])
        # print("T_robot shape", T_robot.shape)
        # print("Tag_to_row", self.tag_to_row)
        for i, detection in enumerate(self.current_detections.keys()):
            H[i * 3:(i + 1) * 3, 0:3] = T_robot
            j = self.tag_to_row[detection]
            H[i * 3:(i + 1) * 3, j:j + 3] = -T_robot
        # print("H value",H)
        # exit()

        S_INV = np.linalg.inv(
            np.dot(np.dot(H, self.sigma), H.T) + self.r * np.eye(len(z_t)))
        K = np.dot(np.dot(self.sigma, H.T), S_INV)
        diff = (z_t - np.dot(H, self.S))
        # print("diff", diff)
        self.S = self.S + np.dot(K, diff)
        self.sigma = np.dot((np.eye(len(self.S)) - np.dot(K, H)), self.sigma)

        for i in range(0, len(self.S), 3):
            self.S[i + 2, 0] = (self.S[i + 2, 0] + np.pi) % (2 * np.pi) - np.pi

        # br.sendTransform(
        #     (self.S[0, 0], self.S[1, 0], 0),
        #     tf.transformations.quaternion_from_euler(0, 0, self.S[2, 0]),
        #     rospy.Time.now(), "robot", "world")

    def getCurrentPos(self):
        return self.S[0:3, 0].copy()


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
    # colors = cm.rainbow(np.linspace(0, 1, 8))
    # plt.plot(, marker='x', color = colors[0])
    # plt.text(,"tag_0", color = colors[0])
    pass


def plot_states(states, tag_to_row, name="state_graph"):
    tags = tag_to_row.keys()
    row_indices = tag_to_row.values()
    for state in states:
        plt.plot(state[0, 0], state[1, 0], marker='o', color='orange')
    if len(states) > 0:
        state = states[-1]
        colors = iter(cm.rainbow(np.linspace(0, 1, len(state) // 3)))
        for i in range(3, len(state), 3):
            color = next(colors)
            plt.plot(state[i, 0], state[i + 1, 0], marker='x', color='maroon')
            plt.text(state[i, 0],
                     state[i + 1, 0],
                     tags[row_indices.index(i)],
                     color='black')

    print(tag_to_row)
    plt.savefig('/root/rosws/src/rb5_ros/rb5_control/src/' + name + '.png',
                dpi=120)


def plot_final_states(state, cov, tag_to_row):
    fig = plt.figure(dpi=120)
    tags = tag_to_row.keys()
    row_indices = tag_to_row.values()
    ax = plt.subplot(111)
    colors = iter(cm.rainbow(np.linspace(0, 1, len(state) // 3)))
    for i in range(1, len(state) // 3):
        # print("i",i)
        cov_mat = cov[3 * i:3 * (i + 1) - 1, 3 * i:3 * (i + 1) - 1]
        x, y = state[3 * i, 0], state[3 * i + 1, 0]
        # print("cov_mat",cov_mat)
        lambda_, v = np.linalg.eig(cov_mat)
        # print("v",v,"arccos",np.arccos(v[0,0]))
        lambda_ = np.sqrt(lambda_)

        for j in range(1, 2):
            ell = Ellipse(xy=(x, y),
                          width=lambda_[0] * j * 2,
                          height=lambda_[1] * j * 2,
                          edgecolor='red',
                          angle=np.rad2deg((np.arccos(v[0, 0]) + 0j).real))
            ell.set_facecolor('none')
            ax.add_artist(ell)
        color = next(colors)
        plt.plot(x, y, marker='x', color='maroon')
        plt.text(x, y, tags[row_indices.index(3 * i)], color='black')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    fig.savefig('/root/rosws/src/rb5_ros/rb5_control/src/tag_map.png', dpi=120)


if __name__ == "__main__":
    import time
    rospy.init_node("hw2")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    waypoint = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 0.0, np.pi / 2],
        [1.0, 1.0, np.pi / 2],
        [1.0, 1.0, np.pi],
        [0.0, 1.0, np.pi],
        [0.0, 1.0, -np.pi / 2],
        [0.0, 0.0, -np.pi / 2],
        # [0.0, 0.0, 0.0],
        # [1.0, 0.0, 0.0],
        # [1.0, 0.0, np.pi / 2],
        # [1.0, 1.0, np.pi / 2],
        # [1.0, 1.0, np.pi],
        # [0.0, 1.0, np.pi],
        # [0.0, 1.0, -np.pi / 2],
        # [0.0, 0.0, -np.pi / 2],
        [0.0, 0.0, 0.0],
    ])

    waypoint = np.array(
        [
            [1 / 3.0, 0, 0],
            [2 / 3.0, 0.0, 0],
            [2 / 3.0, 0, np.pi / 4],
            [1.0, 1 / 3.0, np.pi / 4],
            [1.0, 1 / 3.0, np.pi / 2],
            [1.0, 2 / 3.0, np.pi / 2],
            [1.0, 2 / 3.0, 3 * np.pi / 4],
            [2 / 3.0, 1.0, 3 * np.pi / 4],
            [2 / 3.0, 1.0, -np.pi],
            [1 / 3.0, 1.0, -np.pi],
            [1 / 3.0, 1.0, -3 * np.pi / 4],
            [0.0, 2 / 3.0, -3 * np.pi / 4],
            [0.0, 2 / 3.0, -np.pi / 2],
            [0.0, 1 / 3.0, -np.pi / 2],
            [0.0, 1 / 3.0, -np.pi / 4],
            [1 / 3.0, 0.0, -np.pi / 4],
            [1 / 3.0, 0.0, 0],
            # [2 / 3.0, 0.0, 0],
            # [2 / 3.0, 0, np.pi / 4],
            # [1.0, 1 / 3.0, np.pi / 4],
            # [1.0, 1 / 3.0, np.pi / 2],
            # [1.0, 2 / 3.0, np.pi / 2],
            # [1.0, 2 / 3.0, 3 * np.pi / 4],
            # [2 / 3.0, 1.0, 3 * np.pi / 4],
            # [2 / 3.0, 1.0, -np.pi],
            # [1 / 3.0, 1.0, -np.pi],
            # [1 / 3.0, 1.0, -3 * np.pi / 4],
            # [0.0, 2 / 3.0, -3 * np.pi / 4],
            # [0.0, 2 / 3.0, -np.pi / 2],
            # [0.0, 1 / 3.0, -np.pi / 2],
            # [0.0, 1 / 3.0, -np.pi / 4],
            # [1 / 3.0, 0.0, -np.pi / 4],
            # [1 / 3.0, 0.0, 0],
        ],
        dtype=float)

    # init pid controller
    pid = PIDcontroller(0.05, 0.007, 0.005)

    # init current state
    current_state = np.array([1 / 3.0, 0, 0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough,
    # the current way point will be updated with a new point.
    for index, wp in enumerate(waypoint):
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
        pid.kalman_predict(update_value, listener, broadcaster)
        current_state += update_value
        current_state = pid.getCurrentPos()
        i = 0
        S_history = []
        while (
                np.linalg.norm(pid.getError(current_state, wp)) > 0.1
        ):  # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            i += 1
            pid.kalman_predict(update_value, listener, broadcaster)
            if i % UPDATE_EVERY_STEPS == 0:
                pid.kalman_update(broadcaster)
            current_state += update_value
            S_history.append(pid.S.copy())
            current_state = pid.getCurrentPos()
            print("S in main", pid.S[0:3, 0], pid.S.shape)
            print("Sigma in main", np.linalg.norm(pid.sigma), pid.sigma.shape)
        pub_twist.publish(genTwistMsg([0, 0, 0]))
        if index % 2 == 1:
            print("Plotting for wp ", wp)
            plot_states(S_history, pid.tag_to_row)
        time.sleep(0.5)
    # stop the car and exit
    np.save("cov.npy", pid.sigma)
    plot_final_states(pid.S.copy(), pid.sigma, pid.tag_to_row)
    pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.0])))
