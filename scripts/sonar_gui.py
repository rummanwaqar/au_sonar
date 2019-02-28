#! /usr/bin/env python

import sys
import numpy as np

import rospy
from au_sonar.msg import SonarDebug, FilterControl
from au_core_utils.load_topics import load_topics

from PyQt5 import QtGui, QtWidgets
import pyqtgraph as pg

from widgets.control_panel import ControlPanel
from widgets.ping_plotter import PingPlotter
from widgets.particle_plotter import ParticlePlotter
from widgets.status_bar import StatusBar

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


class SonarGui(QtWidgets.QWidget):
    def __init__(self, debug_topic, ctrl_topic):
        super(SonarGui, self).__init__()

        # initialize UI
        self.control_panel = ControlPanel(self.__start_filter,
                                          self.__stop_filter)
        self.ping_plotter = PingPlotter()
        self.particle_plotter = ParticlePlotter()
        self.status_bar = StatusBar()
        self.__init_ui()

        # initialize pubs and subs
        self.debug_sub = rospy.Subscriber(debug_topic, SonarDebug,
                                          self.__debug_cb)
        rospy.loginfo('Listening on sonar debug topic: ' + debug_topic)
        self.ctrl_pub = rospy.Publisher(
            ctrl_topic, FilterControl, queue_size=1)
        rospy.loginfo('Sonar ctrl publishing on: ' + ctrl_topic)

        self.show()

    def __init_ui(self):
        self.resize(786, 1024)
        self.setWindowTitle('Sonar GUI')

        # add everything to layout
        layout = QtGui.QVBoxLayout()
        self.setLayout(layout)
        layout.addWidget(self.control_panel)
        plotter_size_policy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        plotter_size_policy.setVerticalStretch(1)
        self.ping_plotter.setSizePolicy(plotter_size_policy)
        layout.addWidget(self.ping_plotter)
        layout.addWidget(self.particle_plotter)
        layout.addWidget(self.status_bar)
        layout.setContentsMargins(10, 10, 10, 0)

    def __debug_cb(self, data):
        raw_data = [data.h_a, data.h_refa, data.h_b, data.h_refb]
        fft_data = [data.fft_freq, data.fft_mag]
        self.ping_plotter.update_data(raw_data, fft_data, data.actual_freq,
                                      data.shift_a, data.shift_b,
                                      data.rel_heading, data.abs_heading)
        robot_pos = [data.robot_pose.x, data.robot_pose.y]
        particle_data = np.vstack([data.particles_x, data.particles_y])
        self.particle_plotter.update(robot_pos, particle_data)
        if data.filter_running:
            self.control_panel.disable_fields(True)
            self.status_bar.filter_on(data.pinger_location, data.filter_cov)
        else:
            self.control_panel.disable_fields(False)
            self.status_bar.filter_off()

    def __start_filter(self, x, y, stdev):
        msg = FilterControl()
        msg.enable = True
        msg.x = x
        msg.y = y
        msg.stdev = stdev
        if self.ctrl_pub is not None:
            self.ctrl_pub.publish(msg)
        rospy.logdebug("starting filter with {},{} with dev {}".format(
            x, y, stdev))

    def __stop_filter(self):
        msg = FilterControl()
        msg.enable = False
        if self.ctrl_pub is not None:
            self.ctrl_pub.publish(msg)
        rospy.logdebug("stopping filter")


if __name__ == '__main__':
    rospy.init_node('sonar_gui')
    topics = load_topics()

    # initalize gui
    app = QtGui.QApplication([])
    gui = SonarGui(
        debug_topic=topics['/topic/sensor/sonar/debug'],
        ctrl_topic=topics['/topic/sensor/sonar/ctrl'])

    sys.exit(app.exec_())
