#!/usr/bin/env python
import os
import argparse
from PingData import PingData

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