from PyQt5 import QtWidgets
import pyqtgraph as pg
import numpy as np


class ParticlePlotter(QtWidgets.QGroupBox):
    """
    plots the particles in a scatter plot
    plot the position of the robot
    """

    def __init__(self):
        """
        initializer
        """
        super(ParticlePlotter, self).__init__()
        self.__init_ui()

    def __init_ui(self):
        """
        initializes UI and makes the plot
        :return:
        """
        view = pg.PlotWidget(title="Particle Filter")
        view.setLabel('left', "X", units='m')
        view.setLabel('bottom', "Y", units='m')
        view.showGrid(x=True, y=True)
        view.setXRange(-20, 20)
        view.setYRange(-20, 20)
        view.setAspectLocked(True)

        self.particle_plot = pg.ScatterPlotItem(
            size=7,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(0, 0, 0, 120),
            pxMode=True)
        self.robot = pg.ScatterPlotItem(
            size=10,
            pen=pg.mkPen(None),
            brush=pg.mkBrush(255, 0, 0, 200),
            pxMode=True)
        view.addItem(self.particle_plot)
        view.addItem(self.robot)

        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)
        layout.addWidget(view)

    def update(self, robot_pos, particles):
        """
        updates the data in the plot
        :param robot_pos: robot position (x,y)
        :param particles: array of particles
        :return:
        """
        self.robot.setData([{'pos': [robot_pos[1], robot_pos[0]]}])

        _, n = particles.shape
        self.particle_plot.setData([{
            'pos': [particles[1, i], particles[0, i]]
        } for i in range(n)])
