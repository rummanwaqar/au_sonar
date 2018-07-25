#!/usr/bin/env python
from __future__ import division
import os
import argparse
from PingData import PingData
import matplotlib.pyplot as plt
import numpy as np

def main():
    # read cli params
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="csv file")
    args = parser.parse_args()
    filename = args.file

    if not os.path.exists(filename):
        print('Invalid file specified: {}'.format(filename))
        exit(1)

    ping = PingData.from_csv(filename)
    ping.plot()



if __name__ == '__main__':
    main()