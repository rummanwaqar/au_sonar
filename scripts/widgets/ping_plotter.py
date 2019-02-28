from PyQt5 import QtWidgets
import pyqtgraph as pg


class PingPlotter(QtWidgets.QGroupBox):
    """
    plots ping data, fft and ping information
    """

    def __init__(self):
        """
        initializes the plotter
        """
        super(PingPlotter, self).__init__()
        self.data = None
        self.full_graph = []
        self.zoom_graph = []
        self.pens = [
            pg.mkPen(color=x, width=2)
            for x in [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
        ]
        self.__init_ui()

    def update_data(self, data, fft, freq, shift_a, shift_b, rel_heading,
                    abs_heading):
        """
        updates the plots with data
        :param data: ping data
        :param fft: fft plot
        :param freq: signal freq
        :param shift_a: signal shift a
        :param shift_b: signal shift b
        :param rel_heading: relative heading of pinger
        :param abs_heading: abs heading of pinger
        """
        self.data = data
        self.__update_full_plot()
        self.__update_zoom_plot()
        self.__update_fft_plot(fft_mag=fft[1], fft_freq=fft[0])
        self.status.setText(
            self.__mk_status(freq, shift_a, shift_b, rel_heading, abs_heading))

    def __init_ui(self):
        """
        initializes all graphs
        """
        # set up full plot
        full_plot = pg.PlotWidget(name='FullPlot')
        full_plot.setMouseEnabled(x=False, y=False)
        self.region = pg.LinearRegionItem(values=(400, 600))
        self.region.setZValue(10)
        self.region.sigRegionChanged.connect(self.__region_update)
        full_plot.addItem(self.region, ignoreBounds=True)
        full_plot.setAutoVisible(y=True)
        for i in range(4):
            self.full_graph.append(full_plot.plot(pen=self.pens[i]))

        # set up zoom plot
        zoom_plot = pg.PlotWidget(
            name='ZoomPlot',
            title="Zoomed Input Signals",
            labels={
                'left': 'Voltage (V)',
                'bottom': 'Samples'
            })
        zoom_plot.setMouseEnabled(x=False, y=False)
        for i in range(3):
            self.zoom_graph.append(zoom_plot.plot(pen=self.pens[i]))

        # set up fft
        fft_plot = pg.PlotWidget(
            name='FftPlot',
            title="FFT",
            labels={
                'left': 'Magnitude',
                'bottom': 'Frequency (KHz)'
            })
        zoom_plot.setMouseEnabled(x=False, y=False)
        self.fft = fft_plot.plot(pen=pg.mkPen(color='r', width=1))

        # info frame
        info_frame = QtWidgets.QFrame()
        frame_layout = QtWidgets.QHBoxLayout()
        frame_layout.setContentsMargins(0, 0, 0, 0)
        info_frame.setLayout(frame_layout)
        legend_a = QtWidgets.QLabel('signal a')
        legend_a.setStyleSheet('color: red')
        legend_b = QtWidgets.QLabel('signal b')
        legend_b.setStyleSheet('color: blue')
        legend_ref = QtWidgets.QLabel('ref signal')
        legend_ref.setStyleSheet('color: green')
        self.status = QtWidgets.QLabel(self.__mk_status(0, 0, 0, 0, 0))
        frame_layout.addWidget(legend_a)
        frame_layout.addWidget(legend_b)
        frame_layout.addWidget(legend_ref)
        frame_layout.addStretch(1)
        frame_layout.addWidget(self.status)

        # layout
        layout = QtWidgets.QGridLayout()
        self.setLayout(layout)
        layout.addWidget(info_frame, 0, 0, 1, 2)
        layout.addWidget(zoom_plot, 1, 0, 1, 1)
        layout.addWidget(fft_plot, 1, 1, 1, 1)
        layout.addWidget(full_plot, 2, 0, 2, 2)

    def __update_full_plot(self):
        """
        updates data for the full plot
        """
        for i in range(3):
            self.full_graph[i].setData(self.data[i])

    def __update_zoom_plot(self):
        """
        updates data for the zoom plot
        """
        x_min, x_max = [int(x) for x in self.region.getRegion()]
        if x_min < 0:
            x_min = 0
        if x_max > len(self.data[0]):
            x_max = len(self.data[0])
        for i in range(3):
            self.zoom_graph[i].setData(self.data[i][x_min:x_max])

    def __update_fft_plot(self, fft_mag, fft_freq):
        """
        updates fft plot
        :param fft_mag: fft magnitudes (y)
        :param fft_freq: fft_frequencies (x)
        """
        self.fft.setData([x / 1000 for x in fft_freq], fft_mag)

    def __region_update(self, window):
        """
        callback for region selector in full graph
        :param window:
        """
        self.__update_zoom_plot()

    def __mk_status(self, freq, shift_a, shift_b, rel_heading, abs_heading):
        return "Frequency: {:2.1f} KHz | ShiftA: {:.1f} | ShiftB: {:.1f} | Rel Heading: {:.1f} | Abs Heading: {:.1f}".format(
            freq / 1000.0, shift_a, shift_b, rel_heading, abs_heading)
