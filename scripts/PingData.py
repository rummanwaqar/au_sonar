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
        self.angle = None

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
            hydrophoneA.append(data[i])
            refA.append(data[i+1])
            hydrophoneB.append(data[i+2])
            refB.append(data[i+3])
        return cls(timestamp=timestamp, hydrophoneA=hydrophoneA, hydrophoneB=hydrophoneB, refA=refA, refB=refB)

    @classmethod
    def from_csv(cls, file):
        return cls()

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

    def __str__(self):
        if self.timestamp is not None and len(self.hydrophoneA) > 0:
            return "ping received at {0} with {1} points".format(self.timestamp, len(self.hydrophoneA))
        else:
            return "empty ping object"
