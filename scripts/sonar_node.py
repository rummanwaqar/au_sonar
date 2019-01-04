#! /usr/bin/env python

import signal
import time
import os
import Queue
import argparse
import numpy as np

from sonar import PingReader
from sonar import PreprocessorComm
from sonar import PhaseShiftAnalysis

import rospy
import actionlib
import au_sonar.msg
from au_sonar.msg import SonarFeedback, SonarGoal, SonarResult
from au_core.msg import PingStatus
from std_msgs.msg import Float32


class SonarAction(object):
    # create messages that are used to publish feedback/result
    _feedback = SonarFeedback()
    _result = SonarResult()

    def __init__(self, name, noOfPoints, noOfInterations):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            au_sonar.msg.SonarAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self._as.start()
        self._isRunning = False
        self._pings = []
        self._noOfPoints = noOfPoints
        self._noOfIterations = noOfInterations

    def add_heading(self, data):
        if self._isRunning:
            self._pings.append(data)

    def execute_cb(self, goal):
        self._isRunning = True
        self._pings = []

        # helper variables
        r = rospy.Rate(0.5)
        success = True

        # publish info to the console for the user
        rospy.loginfo('{}: Executing, waiting for {} points'.format(
            self._action_name, self._noOfPoints))

        # start executing the action
        while len(self._pings) <= self._noOfPoints:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self._isRunning = False
                break
            self._feedback.status = "{} of {}".format(
                len(self._pings), self._noOfPoints)
            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            heading = None
            print('------------------------------------------------')
            print('initial data: {}'.format(self._pings))
            for i in range(self._noOfIterations):
                mean, std_dev, self._pings = PhaseShiftAnalysis.outlier_elimination(
                    self._pings)
                heading = np.mean(self._pings)
                print('output {}: heading {} with data {}'.format(
                    i, heading, self._pings))
            print('------------------------------------------------')
            self._result.rel_heading = heading

            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

        self._isRunning = False


if __name__ == '__main__':
    rospy.init_node('sonar')

    pingReader = None
    preprocessor = None

    freq = rospy.get_param("~freq", 27)
    storage_path = rospy.get_param("~storage_path", None)
    port = rospy.get_param("~port", '/dev/arvp/sonar')
    noOfPoints = int(rospy.get_param("~noOfPoints", 8))
    noOfIterations = int(rospy.get_param("~noOfIterations", 2))

    if storage_path == "":
        storage_path = None

    print "Frequency: {}".format(freq)
    print "Storage path: {}".format(storage_path)
    print "Preprocessor port: {}".format(port)
    print "No of analysis points: {}".format(noOfPoints)
    print "No of iterations for outlier detection: {}".format(noOfIterations)

    # ping status pub
    ping_status_pub = rospy.Publisher('~ping_status', PingStatus, queue_size=1)
    gain_pub = rospy.Publisher('~gain', Float32, queue_size=2)

    server = SonarAction(rospy.get_name(), noOfPoints, noOfIterations)

    print "Initialized!"

    data_queue = Queue.Queue()
    ping_info_queue = Queue.Queue()

    source_path = os.path.dirname(os.path.abspath(__file__))
    preprocessor = PreprocessorComm(port, \
                                    os.path.join(source_path, '../params/preprocessor.yaml'), \
                                    ping_info_queue)
    preprocessor.start()
    if preprocessor.write_current_params():
        print('All params loaded')
    else:
        print('Param loading failed')

    pingReader = PingReader(data_queue)
    pingReader.start()
    store = False

    preprocessor.send_param('centerFreq', int(freq), timeout=0.5)

    if storage_path:
        print('activated storage')
        if os.path.exists(storage_path):
            print('storing files {}'.format(storage_path))
            store = True

    ping = None
    ping_status = None

    while True:
        try:
            ping = data_queue.get(timeout=0.1)
        except Queue.Empty:
            pass
        else:
            ping_status = PingStatus()
            ping_status.header.stamp = rospy.Time.now()
            ping_status.isGood = 0
            # wait for info here
            if ping.update_ping_info(ping_info_queue):
                ping_status.latency = ping.ping_info['timestamp'] - ping.timestamp
                ping_status.mean = ping.ping_info['avgPkLv']
                ping_status.variance = ping.ping_info['variance']
                ping_status.gain = ping.ping_info['gain']
                if ping.ping_info['cal']:
                    gain_pub.publish(data=ping.ping_info['gain'])
                    if store:
                        ping.to_csv(storage_path)

                    phase_analysis = PhaseShiftAnalysis(
                        ping_data=ping, target_frequency=int(freq) * 1000)
                    try:
                        if phase_analysis.get_windowed_shift():
                            heading = phase_analysis.get_heading()
                            ping_status.heading = heading
                            ping_status.freq = phase_analysis.debug['freq']
                            ping_status.shift_x = phase_analysis.debug[
                                'shiftX']
                            ping_status.shift_y = phase_analysis.debug[
                                'shiftY']
                            ping_status.isGood = 1
                            server.add_heading(heading)
                        else:
                            ping_status.error = 'window_check_failed'
                    except RuntimeWarning, e:
                        ping_status.error = str(e)
                else:
                    ping_status.error = "uncalibrated"
            else:
                ping_status.error = "missing_info"
            data_queue.task_done()
            ping_status_pub.publish(ping_status)
