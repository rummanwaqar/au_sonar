#!/usr/bin/env python

import signal
import os
import Queue
import argparse
import time
import logging

from AsyncReadSerial import AsyncReadSerial


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
    # register signal handler
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)

    # read cli param
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="serial port")
    args = parser.parse_args()
    port = args.port

    data_queue = Queue.Queue()

    # logging
    logging_dir = os.path.expanduser('~/.sonar/')
    if not os.path.exists(logging_dir):
        os.makedirs(logging_dir)
        print(logging_dir)
    logging.basicConfig(filename=os.path.join(logging_dir, 'log.txt'),
                        level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    logger.info('Started')

    try:
        readSerialThread = AsyncReadSerial(port, data_queue)
        readSerialThread.start()

        while True:
            try:
                data = data_queue.get(timeout=1)
            except Queue.Empty:
                pass
            else:
                logging.debug(data)
                data_queue.task_done()
    except ServiceExit:
        readSerialThread.shutdown_flag.set()
        readSerialThread.join()
        logger.info('Finished')

if __name__ == '__main__':
    main()