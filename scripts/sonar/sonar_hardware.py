import os
import json
import yaml
import zmq

import rospy
from au_core.msg import DynamicsState


class SonarHardware():
    """
    retrieves sonar pings from ZMQ data server, and syncs current pose with it
    """

    def __init__(self, state_topic, zmq_port):
        """
        intializes sonar hardware connections
        :param state_topic: port name for robot state
        :param zmq_port: zmq data port for sonar ping
        """
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
        """
        waits for sonar data over ZMQ, and syncs it with robot state (blocking call)
        :return: dictionary containing sonar data (keys: timestamp, data, info, pose)
        """
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

    @staticmethod
    def load_params(filename, server_port, freq):
        """
        loads sonar preprocessor parameters from a yaml file and
        sends it over zmq command port to preprocessor
        :param filename: sonar parameter yaml file
        :param server_port: zmq port name
        :param freq: target frequency overrided
        :return: True if successful
        """
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
                    rospy.logerr(
                        'Incorrect response for ' + cmd + ' -> ' + reply)
                    return False
            return True
        else:
            return False

    def __state_cb(self, state):
        """
        robot state subscriber callback
        :param state: robot state
        """
        self.__current_state = state

    def __cvt_pose(self, state):
        """
        converts robot pose dynamic state message to dictionary
        :param state: robot state
        :return: dictionary containing position and orientation (quaternion)
        """
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
