from PyQt5 import QtWidgets
import pyqtgraph as pg


class ControlPanel(QtWidgets.QGroupBox):
    """
    Creates a control panel to start and stop the filter
    """

    def __init__(self, start_cb, stop_cb):
        """
        initializes control panel
        :param start_cb: start filter callback
        :param stop_cb: stop filter callback
        """
        super(ControlPanel, self).__init__()
        self.start_cb = start_cb
        self.stop_cb = stop_cb
        self.__init_ui()

    def __init_ui(self):
        """
        initializes UI
        """
        # create controls
        self.startBtn = QtWidgets.QPushButton('Initialize filter')
        self.startBtn.clicked.connect(self.__start_filter)
        self.stopBtn = QtWidgets.QPushButton('Stop filter')
        self.stopBtn.clicked.connect(self.__stop_filter)

        self.xText = QtWidgets.QLineEdit()
        self.xText.setPlaceholderText('x (m)')
        self.yText = QtWidgets.QLineEdit()
        self.yText.setPlaceholderText('y (m)')
        self.stdevText = QtWidgets.QLineEdit()
        self.stdevText.setPlaceholderText('stdev (m)')

        # make layout
        layout = QtWidgets.QHBoxLayout()
        self.setLayout(layout)
        layout.addWidget(self.startBtn)
        layout.addWidget(self.xText)
        layout.addWidget(self.yText)
        layout.addWidget(self.stdevText)
        layout.addStretch(1)
        layout.addWidget(self.stopBtn)

        # set up buttons
        self.disable_fields(False)

    def disable_fields(self, enable):
        """
        disables enable button and input fields
        :param enable: boolean true to disable buttons
        """
        self.startBtn.setEnabled(not enable)
        self.stopBtn.setEnabled(enable)
        self.xText.setEnabled(not enable)
        self.yText.setEnabled(not enable)
        self.stdevText.setEnabled(not enable)

    def __start_filter(self):
        """
        callback for start button
        calls the start_cb callback
        """
        x = self.validate_input(self.xText.text())
        y = self.validate_input(self.yText.text())
        stdev = self.validate_input(self.stdevText.text())

        if x is None or y is None or stdev is None:
            print('Bad input')
        else:
            self.disable_fields(True)
            self.start_cb(x, y, stdev)

    def __stop_filter(self):
        """
        callback for stop button
        calls the stop_cb callback
        """
        self.disable_fields(False)
        self.stop_cb()

    @staticmethod
    def validate_input(inp):
        """
        check if input is non-empty and a float
        :param inp: str input
        :return: float output
        """
        if inp == '':
            return None
        try:
            return float(inp)
        except ValueError:
            return None
