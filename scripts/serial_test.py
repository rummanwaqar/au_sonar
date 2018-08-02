#!/usr/bin/env python

import signal
import os
import Queue
import argparse
import time
import logging

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
    # register signal handler
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)

    # read cli param
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="serial port")
    args = parser.parse_args()
    port = args.port

    ping_queue = Queue.Queue()

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
        preprocessor = PreprocessorComm(port, '../cfg/preprocessor.cfg', ping_queue)
        preprocessor.start()
        if preprocessor.write_current_params():
            print('All params loaded')
            logger.info('All params loaded')
        else:
            print('Param loading failed')
            logger.error('Param loading failed')

        while True:
            try:
                data = ping_queue.get(timeout=1)
            except Queue.Empty:
                pass
            else:
                #logging.debug(data)

                print(data)
                ping_queue.task_done()
            #print(readSerialThread.read_param('iGain', timeout=10))
            #print(readSerialThread.sonar_params)
            # data = readSerialThread.get_line()
            # if data is not None:
                # print(data)
                # readSerialThread.read_complete()
            pass
    except ServiceExit:
        preprocessor.shutdown_flag.set()
        preprocessor.join()
        logger.info('Finished')
    except RuntimeError as e:
        logging.error(e)
        exit(1)

if __name__ == '__main__':
    main()