#!/usr/bin/env python

import rospy
import tf
import math
import random
import numpy as np
import zmq
import json
from au_core.msg import DynamicsState
from au_core_utils.load_topics import load_topics


class SonarSim:
    SIGNAL_AMPLITUDE = 1.6  # 1.6V is simulated
    DURATION = 1.4e-3  # 1.4ms ping period

    def __init__(self, pinger_freq, sampling_freq, noise, pinger_loc):
        """
        initializes the sonar sim
        :param pinger_freq: frequency of pinger (Hz)
        :param sampling_freq: adc sampling frequency (Hz)
        :param noise: std dev of heading noise (deg)
        :param pinger_loc: x,y NED world coods of simulated pinger (m)
        """
        self.pinger_freq = pinger_freq
        self.sampling_freq = sampling_freq
        self.noise = noise * math.pi / 180.0
        self.pinger_loc = pinger_loc

    def gen_ping(self, robot_position, robot_yaw):
        """
        generates a ping using robot_position and yaw
        :param robot_position: x,y world coods of the robot (m)
        :param robot_yaw: robot yaw (rad)
        :return: ping data as a numpy matrix of size 4 x n, calculated heading for the ping
        """
        heading = self.__get_heading(robot_position, robot_yaw)
        # calculate shifts for heading
        ratio = math.tan(heading)
        shift_b = 69.0 * math.pi / 180.0  # choose shift b arbitrarily
        shift_a = ratio * shift_b
        # generate sin waves
        ref = self.__generate_sin()
        a = self.__generate_sin(shift_a)
        b = self.__generate_sin(shift_b)
        return np.vstack((a, ref, b, ref)), heading

    def __get_heading(self, robot_position, robot_yaw):
        """
        calculates the pinger heading based on robot and pinger position.
        adds Gaussian noise to calc heading
        :param robot_position: x,y world coods of the robot (m)
        :param robot_yaw: robot yaw (rad)
        :return: pinger heading relative to the robot (rad)
        """
        abs_heading = math.atan2(self.pinger_loc[1] - robot_position[1],
                                 self.pinger_loc[0] - robot_position[0])
        return self.normalize(
            abs_heading - robot_yaw + random.gauss(0, self.noise))

    def __generate_sin(self, phase_shift=0):
        """
        generates a sin wave as an estimation for the ping signal
        :param phase_shift: phase shift for the sine wave (rad)
        :return: numpy array representing the sine wave
        """
        amp = self.SIGNAL_AMPLITUDE
        fs = self.sampling_freq
        duration = self.DURATION
        f = self.pinger_freq
        return (amp * np.sin(2 * np.pi * np.arange(fs * duration) * f / fs +
                             phase_shift)).astype(np.float32)

    @staticmethod
    def normalize(angle, limit=math.pi):
        """
        circular normalization of an value between -limit to limit
        :param angle: value to normalize
        :param limit: +/- limit of value
        :return: normalized value
        """
        return (angle + limit) % (2 * limit) - limit


class SonarSimNode:
    def __init__(self):
        """
        initializes the ros node with all pubs and subs
        initializes the simulator object
        """
        topics = load_topics()

        # init node
        rospy.init_node('sonar_sim', anonymous=False)

        # load params
        sampling_freq = rospy.get_param("~sampling_freq", 1e6)
        pinger_freq = rospy.get_param("~pinger_freq", 27e3)
        noise = rospy.get_param("~noise_stdev", 20)
        pinger_x = rospy.get_param("~x", 0.0)
        pinger_y = rospy.get_param("~y", 0.0)
        rate = rospy.get_param("~rate", 1)
        zmq_port = rospy.get_param("~data_port", "tcp://*:12345")
        rospy.loginfo("Initialized sonar sim")
        rospy.loginfo("Sampling freq: {:.0f} Hz".format(sampling_freq))
        rospy.loginfo("Pinger freq: {:.0f} Hz".format(pinger_freq))
        rospy.loginfo("Noise stdev: {:.1f} deg".format(noise))
        rospy.loginfo("Pinger location: {:.1f}, {:.1f}".format(
            pinger_x, pinger_y))
        rospy.loginfo("Ping rate: {:.1f} Hz".format(rate))
        rospy.loginfo("ZMQ data port: {}".format(zmq_port))

        # robot pose sub
        state_topic = topics['/topic/sensor/dynamics_state']
        self._state_sub = rospy.Subscriber(
            state_topic, DynamicsState, self.__state_cb, queue_size=1)
        rospy.loginfo('Subscribing for state to ' + state_topic)
        self.state = None

        # init zmq
        context = zmq.Context()
        self.__socket = context.socket(zmq.PUB)
        self.__socket.bind(zmq_port)
        rospy.loginfo('Publishing sonar data to ' + zmq_port)

        # init sim
        self.sim = SonarSim(
            pinger_freq=pinger_freq,
            sampling_freq=sampling_freq,
            noise=noise,
            pinger_loc=(pinger_x, pinger_y))

        # set timer
        self.rate = rospy.Rate(rate)

    def run(self):
        """
        runs the simulator. gets the ping data at each interval and
        sends it over ZMQ
        """
        while not rospy.is_shutdown():
            if self.state is not None:
                # get robot state
                x = self.state.pose.position.x
                y = self.state.pose.position.y
                quat = [
                    self.state.pose.orientation.x,
                    self.state.pose.orientation.y,
                    self.state.pose.orientation.z,
                    self.state.pose.orientation.w
                ]
                eulers = tf.transformations.euler_from_quaternion(quat, 'sxyz')
                # generate ping and send
                ping, heading = self.sim.gen_ping((x, y), eulers[2])
                self.__socket.send(self.__create_msg(ping))
                rospy.logdebug(
                    "Sent ping with heading: " + str(np.rad2deg(heading)))
            self.rate.sleep()

    def __state_cb(self, data):
        """
        sets the robot state
        :param data: robot state
        """
        self.state = data

    def __create_msg(self, ping):
        """
        generates json message from ping data
        :param ping: ping matrix generated by SonarSim.gen_ping()
        :return: json data to send
        """
        now = rospy.get_rostime()
        output = {
            "info": {},
            "timestamp": int(now.secs * 1e3 + now.nsecs * 1e-6),
            "data": ping.T.tolist()
        }
        return json.dumps(output)


if __name__ == '__main__':
    try:
        sonar_node = SonarSimNode()
        sonar_node.run()
    except rospy.ROSInterruptException:
        pass
