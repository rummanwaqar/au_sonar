from PyQt5 import QtWidgets
import pyqtgraph as pg


class StatusBar(QtWidgets.QStatusBar):
    def __init__(self):
        super(StatusBar, self).__init__()
        self.__init_ui()

    def __init_ui(self):
        self.label = QtWidgets.QLabel('')
        self.addWidget(self.label)
        self.filter_off()

    def filter_off(self):
        self.label.setText('Filter not running')

    def filter_on(self, position, cov):
        self.label.setText(
            'Running | Pinger: {:2.1f},{:2.1f} | Cov_X: {:2.2f} | Cov_Y: {:2.2f} | Cov_XY: {:2.2f}'.
            format(position[0], position[1], cov[0], cov[3], cov[1]))
