#! /usr/bin/env python

import sys
import os
import time
import signal
import json
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

from au_core_utils.load_topics import load_topics
from au_sonar.msg import SonarDebug, FilterControl, Sonar
from sonar.sonar_hardware import SonarHardware
from sonar.signal_analysis import fft, normalize_angle
from sonar.particle_filter import ParticleFilter


class SonarNode:
    def __init__(self, output_topic, debug_topic, filter_control_topic,
                 robot_state_topic, zmq_data_port, zmq_cmd_port, params_file,
                 freq, num_particles, sense_noise):
        # initialize pubs and subs
        self.output_pub = rospy.Publisher(output_topic, Sonar, queue_size=1)
        rospy.loginfo('Sonar output publishing on: ' + output_topic)
        self.debug_pub = rospy.Publisher(debug_topic, SonarDebug, queue_size=1)
        rospy.loginfo('Sonar debug publishing on: ' + debug_topic)
        self.filter_control_sub = rospy.Subscriber(
            filter_control_topic, FilterControl, self.__control_cb)
        rospy.loginfo(
            'Sonar filter ctrl listening on: ' + filter_control_topic)

        # initialize filter object
        self.num_particles = num_particles
        self.sense_noise = sense_noise
        self.freq = freq
        self.filter = None  # set up when init message received

        # init sonar hardware
        self.sonar_hardware = SonarHardware(
            state_topic=robot_state_topic, zmq_port=zmq_data_port)
        # if sonar_hardware.load_params(filename=params_file, server_port=zmq_cmd_port, freq=freq):
        #    rospy.loginfo('Preprocesser params loaded')
        # else:
        #    rospy.logerr(
        #        'Could not load params from preprocessor at file ' + params_file)
        #    exit(1)

        self.raw_signals = None  # raw sonar signals
        self.robot_pose = None  # robot 2d pose (x,y,theta)
        self.current_freq = None  # dominant freq in signal
        self.fft_data = None  # fft data used for debugging
        self.shift_a = None  # signal shift a
        self.shift_b = None  # signal shift b
        self.rel_heading = None  # heading of pinger relative to robot yaw
        self.abs_heading = None  # heading of pinger relative to world axis

    def run(self):
        """
        waits for sonar data and runs the algorithm
        :return: sonar data
        """
        sonar_data = self.sonar_hardware.get_data()  # blocking call
        if sonar_data is not None:
            self.raw_signals = np.float32(sonar_data['data'])

            #  get robot pose
            quat = sonar_data['pose']['orientation']
            _, _, robot_yaw = np.rad2deg(
                euler_from_quaternion(
                    [quat['x'], quat['y'], quat['z'], quat['w']], 'sxyz'))
            self.robot_pose = (sonar_data['pose']['position']['x'],
                               sonar_data['pose']['position']['y'], robot_yaw)

            # calc fft for debuging
            self.current_freq, _, self.fft_data = fft(self.raw_signals[:, 0])
            if abs(self.freq - self.current_freq) > 200:
                rospy.logwarn(
                    "Target freq {}Hz and actual freq do not match {}Hz".
                    format(self.freq, self.current_freq))

            # calculate heading
            self.__calc_heading(self.raw_signals, robot_yaw)
            rospy.logdebug('Got ping with abs heading: {}'.format(
                self.abs_heading))

            # run sonar filter and publish output
            if self.filter is not None:
                self.filter.update_filter(
                    self.abs_heading, [self.robot_pose[0], self.robot_pose[1]])
                self.output_pub.publish(self.__mk_output_msg(self.filter))

            # publish debug message
            self.debug_pub.publish(self.__mk_debug_msg())
        return sonar_data

    def __calc_heading(self, raw_signals, robot_yaw):
        """
        calculates the pinger heading with raw signal and robot pose
        :param raw_signals: raw signals from all four hydrophones
        :param robot_pose: 2d robot pose
        """
        # calc phase shifts
        phase_angles = [0 for _ in range(4)]
        for i in range(0, 4):
            _, phase_angles[i], _ = fft(raw_signals[:, i])
        # calc shift from reference
        self.shift_a = normalize_angle(
            phase_angles[0] - phase_angles[1])  # a - refa
        self.shift_b = normalize_angle(
            phase_angles[2] - phase_angles[3])  # b - refb
        # calculate heading
        self.rel_heading = np.rad2deg(np.arctan2(self.shift_a, self.shift_b))
        self.abs_heading = normalize_angle(self.rel_heading - robot_yaw)

    def __control_cb(self, data):
        """
        filter control subscriber callback. used to initialize and stop sonar filter
        :param data: control message (enable, x, y, stdev)
        """
        x = data.x
        y = data.y
        stdev = data.stdev
        if data.enable:
            if self.filter is not None:  # already initialized
                rospy.loginfo(
                    "Reinitializing filter with initial est. {}, {} and stdev {}".
                    format(x, y, stdev))
            else:
                rospy.loginfo(
                    "Initializing filter with initial est. {}, {} and stdev {}".
                    format(x, y, stdev))
            self.filter = ParticleFilter([x, y], stdev, self.num_particles,
                                         self.sense_noise)
        else:  # stop filter
            if self.filter is None:
                rospy.logwarn("Filter is not running")
            else:
                rospy.loginfo("Filter stopped")
            self.filter = None

    @staticmethod
    def __mk_output_msg(filter):
        """
        generates output message with sonar data
        :param filter: sonar filter object
        """
        msg = Sonar()
        msg.header.stamp = rospy.Time.now()
        best = filter.get_best_particle()
        cov = filter.get_covariance()
        msg.x = best[0]
        msg.y = best[1]
        msg.covariance = cov.flatten().tolist()
        return msg

    def __mk_debug_msg(self):
        """
        generate debug message by publishing specific class properties
        :return: SonarDebug message
        """
        msg = SonarDebug()
        msg.header.stamp = rospy.Time.now()
        # raw data
        msg.h_a = self.raw_signals[:, 0].tolist()
        msg.h_refa = self.raw_signals[:, 1].tolist()
        msg.h_b = self.raw_signals[:, 2].tolist()
        msg.h_refb = self.raw_signals[:, 3].tolist()
        # fft data
        msg.fft_freq = self.fft_data[0].tolist()
        msg.fft_mag = self.fft_data[1].tolist()
        # signal stats
        msg.actual_freq = self.current_freq
        msg.shift_a = self.shift_a
        msg.shift_b = self.shift_b
        msg.rel_heading = self.rel_heading
        msg.abs_heading = self.abs_heading
        # robot pose
        msg.robot_pose.x = self.robot_pose[0]
        msg.robot_pose.y = self.robot_pose[1]
        msg.robot_pose.theta = self.robot_pose[2]
        # particle filter
        if self.filter is not None:
            msg.particles_x = self.filter.particles[:, 0]
            msg.particles_y = self.filter.particles[:, 1]
            msg.pinger_location = self.filter.get_best_particle()
            msg.filter_cov = self.filter.get_covariance().flatten().tolist()
            msg.filter_running = True
        else:
            msg.filter_running = False
        return msg


def signal_handler(sig, frame):
    print("Exiting!!")
    rospy.signal_shutdown("SIGINT")
    sys.exit(0)


def mk_log_file(log_dir):
    """
    create a log file with current date file
    :param log_dir: directory location for file
    :return: open file handler
    """
    save_file = None
    if log_dir:
        log_dir = os.path.expanduser(log_dir)
        if not os.path.exists(log_dir):
            rospy.loginfo('Making new directory for logs in ' + log_dir)
            os.makedirs(log_dir)
        filename = os.path.join(log_dir, time.strftime("%Y-%m-%d_%H-%M.txt"))
        rospy.loginfo('Saving sonar data to :' + filename)
        save_file = open(filename, "w+")
    return save_file


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    # initialize ROS
    rospy.init_node('sonar_node')
    # read params
    topics = load_topics()
    freq = rospy.get_param("~freq", 27.0)
    num_particles = rospy.get_param("~num_particles", 1000)
    sense_noise = rospy.get_param("~sense_noise", 10)
    log_dir = rospy.get_param("~log_dir", None)
    data_port = rospy.get_param("~data_port", 'tcp://127.0.0.1:12345')
    cmd_port = rospy.get_param("~cmd_port", 'tcp://127.0.0.1:12346')
    params_file = rospy.get_param("~params_file", None)

    # initialize sonar
    sonar = SonarNode(
        output_topic=topics['/topic/sensor/sonar/output'],
        debug_topic=topics['/topic/sensor/sonar/debug'],
        filter_control_topic=topics['/topic/sensor/sonar/ctrl'],
        robot_state_topic=topics['/topic/sensor/dynamics_state'],
        zmq_data_port=data_port,
        zmq_cmd_port=cmd_port,
        params_file=params_file,
        freq=freq,
        num_particles=num_particles,
        sense_noise=sense_noise)

    # setup file based logging
    log_file = mk_log_file(log_dir)

    while not rospy.is_shutdown():
        sonar_data = sonar.run()

        if sonar_data is not None and log_file is not None:
            log_file.write(json.dumps(sonar_data) + '\n')
            log_file.flush()
