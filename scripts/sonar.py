import signal
import time
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

    # start the job threads
    try:
        readSocketThread = AsyncReadSocket()
        readSocketThread.start()

        while True:
            time.sleep(0.5)

    except ServiceExit:
        # terminate the threads
        readSocketThread.shutdown_flag.set()

        readSocketThread.join()

    print('Exiting the main program')

if __name__ == '__main__':
    main()