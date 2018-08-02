#!/usr/bin/env python

import signal
import time
import os
import Queue
import argparse

from PingReader import PingReader
from PreprocessorComm import PreprocessorComm


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
    parser.add_argument("port", help="serial port")
    args = parser.parse_args()
    path = args.path
    angle = args.angle
    port = args.port

    if not os.path.exists(path):
        print('Invalid path specified: {}'.format(path))
        exit(1)

    data_queue = Queue.Queue()
    ping_info_queue = Queue.Queue()

    source_path = os.path.dirname(os.path.abspath(__file__))

    # start the job threads
    try:
        preprocessor = PreprocessorComm(port, \
                                        os.path.join(source_path, '../cfg/preprocessor.cfg'), \
                                        ping_info_queue)
        preprocessor.start()
        if preprocessor.write_current_params():
            print('All params loaded')
        else:
            print('Param loading failed')

        pingReader = PingReader(data_queue)
        pingReader.start()
        start = time.time()

        ping = None
        while True:
            try:
                ping = data_queue.get(timeout=0.1)
            except Queue.Empty:
                pass
            else:
                # wait for info here
                if ping.update_ping_info(ping_info_queue):
                    print('calibrated ping')
                    if angle is not None:
                        ping.angle = angle
                    ping.to_csv(path)
                else:
                    print('uncalibrated ping')
                data_queue.task_done()

    except ServiceExit:
        # terminate the threads
        pingReader.shutdown_flag.set()
        pingReader.join()
        preprocessor.shutdown_flag.set()
        preprocessor.join()
    except RuntimeError as e:
        exit(1)

    print('Exiting the main program')

if __name__ == '__main__':
    main()