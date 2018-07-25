from __future__ import division
import time
import csv
import os

class PingData(object):
    def __init__(self, timestamp=None, hydrophoneA=[], refA=[], hydrophoneB=[], refB=[], angle=None):
        self.timestamp = timestamp
        self.hydrophoneA = hydrophoneA
        self.refA = refA
        self.hydrophoneB = hydrophoneB
        self.refB = refB
        self.angle = angle

    @classmethod
    def from_raw_data(cls, data):
        if len(data) % 4 != 0:
            raise RuntimeWarning('invalid ping data. wrong data length')

        timestamp = int(round(time.time()))
        hydrophoneA = []
        hydrophoneB = []
        refA = []
        refB = []
        for i in range(0, len(data), 4):
            hydrophoneA.append(PingData.__to_voltage(data[i]))
            refA.append(PingData.__to_voltage(data[i+1]))
            hydrophoneB.append(PingData.__to_voltage(data[i+2]))
            refB.append(PingData.__to_voltage(data[i+3]))
        return cls(timestamp=timestamp, hydrophoneA=hydrophoneA, hydrophoneB=hydrophoneB, refA=refA, refB=refB)

    @classmethod
    def from_csv(cls, filename):
        timestamp = os.path.splitext(os.path.basename(filename))[0]
        angle = None
        timestamp_split = timestamp.split('_')
        if len(timestamp_split) == 1:
            # no angle
            timestamp = int(timestamp)
        elif len(timestamp_split) == 2:
            timestamp = int(timestamp_split[0])
            angle = int(timestamp_split[1])
        else:
            print('invalid file name')

        hydrophoneA = []
        hydrophoneB = []
        refA = []
        refB = []

        with open(filename, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            next(reader, None)  # skip the headers
            for row in reader:
                hydrophoneA.append(float(row[0]))
                refA.append(float(row[1]))
                hydrophoneB.append(float(row[2]))
                refB.append(float(row[3]))
        return cls(timestamp=timestamp, hydrophoneA=hydrophoneA, hydrophoneB=hydrophoneB, refA=refA, refB=refB, angle=angle)

    def to_csv(self, path):
        filename = os.path.join(path, str(self.timestamp))
        if self.angle is not None:
            filename = filename + '_{}.csv'.format(self.angle)
        else:
            filename = filename + '.csv'.format(self.angle)
        with open(filename, 'wb') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            writer.writerow(["hydrophoneA", "refA", "hydrophoneB", "refB"])
            writer.writerows([self.hydrophoneA[i], self.refA[i], self.hydrophoneB[i], self.refB[i]]
                             for i in range(len(self.hydrophoneA)))
        print('  wrote ping to {}'.format(filename))

    def plot(self):
        import matplotlib.pyplot as plt
        import numpy as np

        time_axis = np.arange(0.0, len(self) / 1e3, 1e-3)  # time in ms (sampling freq = 1MHz)
        plt.plot(time_axis, self.hydrophoneA, label='hydrophone A')
        plt.plot(time_axis, self.refA, label='ref A')
        plt.plot(time_axis, self.hydrophoneB, label='hydrophone B')
        plt.plot(time_axis, self.refB, label='ref B')
        plt.legend(loc='upper left')
        plt.xlabel('time (ms)')
        plt.ylabel('voltage (V)')

        plt.show()

    def __str__(self):
        if self.timestamp is not None and len(self.hydrophoneA) > 0:
            return "ping received at {0} with {1} points".format(self.timestamp, len(self.hydrophoneA))
        else:
            return "empty ping object"

    def __len__(self):
        return len(self.hydrophoneA)

    @staticmethod
    def __to_voltage(data):
        return data/1024*2.0
