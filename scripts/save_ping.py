#!/usr/bin/env python

import signal
import time
import os
import Queue
import argparse

from AsyncReadSocket import AsyncReadSocket


class ServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit of all
    running threads and main program
    """
    pass


def service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ServiceExit


def main():
    # register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)

    # read cli params
    parser = argparse.ArgumentParser()
    parser.add_argument("path", help="path to save CSVs")
    parser.add_argument("--angle", help="angle of data", type=int)
    args = parser.parse_args()
    path = args.path
    angle = args.angle

    if not os.path.exists(path):
        print('Invalid path specified: {}'.format(path))
        exit(1)

    data_queue = Queue.Queue()

    # start the job threads
    try:
        readSocketThread = AsyncReadSocket(data_queue)
        readSocketThread.start()
        start = time.time()

        while True:
            ping = data_queue.get()
            if angle is not None:
                ping.angle = angle
            ping.to_csv(path)
            data_queue.task_done()

    except ServiceExit:
        # terminate the threads
        readSocketThread.shutdown_flag.set()

        readSocketThread.join()

    print('Exiting the main program')

if __name__ == '__main__':
    main()