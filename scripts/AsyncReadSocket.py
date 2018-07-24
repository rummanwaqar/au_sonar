import threading
import time
import zmq
import msgpack

class AsyncReadSocket(threading.Thread):
    def __init__(self, address="10.42.43.125", port="9999"):
        # initialize ZMQ socket
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect("tcp://%s:%s" % (address, port))
        self.socket.setsockopt(zmq.SUBSCRIBE, '')

        self.shutdown_flag = threading.Event()

        threading.Thread.__init__(self)

    def run(self):
        # this is a blocking call. Waits for data to be published
        while not self.shutdown_flag.is_set():
            raw_message = self.socket.recv()
            message = msgpack.unpackb(raw_message)

            millis = int(round(time.time()))
            print(" [%d] Received ping data with %d points." %(millis, len(message)/4))

        print("Exiting ZMQ socket read thread.")
        # socket and context is actually closed by garbage collection