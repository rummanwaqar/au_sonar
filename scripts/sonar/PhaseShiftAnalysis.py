#!/usr/bin/env python
import os
import math
import argparse
from PingData import PingData
import numpy as np

#import matplotlib.pyplot as plt


class PhaseShiftAnalysis(object):
    def __init__(self,
                 ping_data,
                 target_frequency=27000,
                 sampling_frequency=1e6,
                 normalize=True):
        # target freq
        self.target_frequency = target_frequency
        # sample freq
        self.sampling_frequency = sampling_frequency
        self.ping = ping_data

        self.debug = {'freq': None, 'shiftX': None, 'shiftY': None}

        if normalize:
            self.ping.hydrophoneA = PhaseShiftAnalysis.normalize(
                self.ping.hydrophoneA)
            self.ping.hydrophoneB = PhaseShiftAnalysis.normalize(
                self.ping.hydrophoneB)
            self.ping.refA = PhaseShiftAnalysis.normalize(self.ping.refA)
            self.ping.refB = PhaseShiftAnalysis.normalize(self.ping.refB)

    def get_heading(self):
        try:
            shift_x = self.__phase_diff(self.ping.refB, self.ping.hydrophoneB)
            shift_y = self.__phase_diff(self.ping.refA, self.ping.hydrophoneA)
            self.debug['shiftX'] = shift_x
            self.debug['shiftY'] = shift_y
            heading = np.rad2deg(np.arctan2(shift_x, shift_y))
            if heading < 0:
                heading = 180 + heading
            elif heading > 0:
                heading = heading - 180
            return heading
        except RuntimeWarning as e:
            raise e

    # phase difference A - B (returns in degrees)
    def __phase_diff(self, sigA, sigB):
        (sigA_freq, sigA_angle) = PhaseShiftAnalysis.fft_analysis(
            self.sampling_frequency, signal=sigA)
        (sigB_freq, sigB_angle) = PhaseShiftAnalysis.fft_analysis(
            self.sampling_frequency, signal=sigB)

        # check frequency
        if sigA_freq != sigB_freq:
            raise RuntimeWarning('misaligned_freq')
        self.debug['freq'] = sigA_freq
        # measured frequency within +/- 1 of target freq
        # if abs(sigA_freq - self.target_frequency) > 1000:
        #     raise RuntimeWarning('wrong_freq')
        return self.normalize_angle(sigA_angle - sigB_angle)

    @staticmethod
    def fft_analysis(fs, signal):
        l = len(signal)
        t = 1 / fs
        # fft (right side only)
        fft = np.fft.rfft(signal)
        # calculate fft magnitude
        # normalize fft and multiply by 2 for one side
        fft_mag = 2 * np.abs(fft / l)[1:]  # removing DC component as well
        # calculate fft frequencies
        fft_freq = np.fft.rfftfreq(l, t)[1:]
        #plt.plot(fft_freq, fft_mag)
        #plt.show()
        # find max magnitude frequency and its phase angle
        peak_index = np.argmax(fft_mag)
        peak_freq = fft_freq[peak_index]
        peak_angle = np.angle(fft, deg=1)[peak_index]
        return peak_freq, peak_angle

    @staticmethod
    # normalize angle between -180 to 180
    def normalize_angle(angle, limit=180):
        return (angle + limit) % (2 * limit) - limit

    @staticmethod
    # normalize signal
    def normalize(signal):
        return signal / np.max(signal)

    @staticmethod
    def outlier_elimination(pings=[]):
        if len(pings) > 2:
            if np.any(pings < 0):
                # unwrap angles
                index = np.argwhere(pings < 0)
                if len(index) > 0:
                    pings[index] = pings[index] + 360

            # select pings within one std_dev
            mean = np.mean(pings)
            std_dev = np.std(pings)
            pings = np.array(
                [x for x in pings if (mean - std_dev) < x < (mean + std_dev)])

            # wrap angles
            if np.any(pings > 180):
                index = np.argwhere(pings > 180)
                if len(index) > 0:
                    pings[index] = pings[index] - 360
            return mean, std_dev, pings

    def get_windowed_shift(self):
        shift_x = []
        shift_y = []

        for i in np.arange(5, len(self.ping), 256):
            if len(self.ping) - i < 256:
                continue
            shift_y.append(
                self.__phase_diff(self.ping.refA[i:i + 256],
                                  self.ping.hydrophoneA[i:i + 256]))
            shift_x.append(
                self.__phase_diff(self.ping.refB[i:i + 256],
                                  self.ping.hydrophoneB[i:i + 256]))
        mean_x = np.mean(shift_x)
        mean_y = np.mean(shift_y)

        if np.all([np.abs(shift_y - mean_y) < 50]) and np.all(
            [np.abs(shift_x - mean_x) < 50]):
            return True

        return False


if __name__ == '__main__':
    # read cli params
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="csv file")
    args = parser.parse_args()
    filename = args.file

    if not os.path.exists(filename):
        print('Invalid file specified: {}'.format(filename))
        exit(1)

    ping = PingData.from_csv(filename)
    phase_analysis = PhaseShiftAnalysis(ping)
    print(phase_analysis.get_heading())
    print(phase_analysis.debug)
