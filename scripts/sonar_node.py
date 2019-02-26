#! /usr/bin/env python

import json
import signal
import sys
import os
import time
import zmq
import yaml

import rospy
from au_core.msg import DynamicsState
from au_core_utils.load_topics import load_topics


def signal_handler(sig, frame):
    print("Exiting!!")
    rospy.signal_shutdown("SIGINT")
    sys.exit(0)


def load_params(filename, server_port, freq):
    if os.path.exists(filename):
        params = yaml.load(open(filename))
        if freq is not None:
            params['centerFreq'] = freq

        # setup client for ZMQ server
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect(server_port)
        rospy.loginfo('Connected to param server at ' + server_port)

        for param, value in params.items():
            cmd = "$set " + param + " " + str(value)
            socket.send_string(cmd)
            reply = socket.recv()
            if cmd[1:] == reply:
                rospy.loginfo('Wrote param ' + param + '=' + str(value))
            else:
                rospy.logerr('Incorrect response for ' + cmd + ' -> ' + reply)
                return False
        return True
    else:
        return False


class SonarHardware():
    def __init__(self, state_topic, zmq_port):
        # current state
        self.__current_state = None

        # initialize state sub
        self.__state_sub = rospy.Subscriber(
            state_topic, DynamicsState, self.__state_cb, queue_size=1)
        rospy.loginfo('Subscribing for state to ' + state_topic)

        # initialize zmq
        context = zmq.Context()
        self.__socket = context.socket(zmq.SUB)
        self.__socket.connect(zmq_port)
        self.__socket.setsockopt(zmq.SUBSCRIBE, '')
        rospy.loginfo('Subscribing for sonar data to ' + zmq_port)

    def get_data(self):
        if self.__current_state is None:
            return None

        message = json.loads(self.__socket.recv())
        state_timestamp = int(self.__current_state.header.stamp.secs * 1e3 +
                              self.__current_state.header.stamp.nsecs * 1e-6)
        if message['timestamp'] - state_timestamp > 200:
            rospy.logwarn('Sonar data and robot state out of sync by ' +
                          str(message['timestamp'] - state_timestamp) + 'ms')
        message['pose'] = self.__cvt_pose(self.__current_state.pose)
        return message

    def __state_cb(self, state):
        self.__current_state = state

    def __cvt_pose(self, state):
        output = {
            'position': {
                'x': state.position.x,
                'y': state.position.y,
                'z': state.position.z
            },
            'orientation': {
                'x': state.orientation.x,
                'y': state.orientation.y,
                'z': state.orientation.z,
                'w': state.orientation.w
            }
        }
        return output


if __name__ == '__main__':
    # sigint handler
    signal.signal(signal.SIGINT, signal_handler)

    # initialize ROS
    rospy.init_node('sonar_node')
    # read params
    topics = load_topics()
    freq = rospy.get_param("~freq", None)
    log_dir = rospy.get_param("~log_dir", None)
    data_port = rospy.get_param("~data_port", 'tcp://127.0.0.1:12345')
    cmd_port = rospy.get_param("~cmd_port", 'tcp://127.0.0.1:12346')
    params_file = rospy.get_param("~params_file", None)

    save_file = None
    # save file
    if log_dir:
        log_dir = os.path.expanduser(log_dir)
        if not os.path.exists(log_dir):
            rospy.loginfo('Making new directory for logs in ' + log_dir)
            os.makedirs(log_dir)
        filename = os.path.join(log_dir, time.strftime("%Y-%m-%d_%H-%M.txt"))
        rospy.loginfo('Saving sonar data to :' + filename)
        save_file = open(filename, "w+")

    # init sonar hardware
    # if load_params(filename=params_file, server_port=cmd_port, freq=freq):
    #    rospy.loginfo('Preprocesser params loaded')
    # else:
    #    rospy.logerr(
    #        'Could not load params from preprocessor at file ' + params_file)
    #    exit(1)
    sonar = SonarHardware(
        state_topic=topics['/topic/sensor/dynamics_state'], zmq_port=data_port)

    while not rospy.is_shutdown():
        sonar_data = sonar.get_data()
        if sonar_data is not None:
            rospy.loginfo("Got sonar ping")
            if save_file is not None:
                save_file.write(json.dumps(sonar_data) + '\n')
                save_file.flush()
