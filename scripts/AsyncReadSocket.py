import threading
import time
import zmq
import msgpack
import Queue
from PingData import PingData

class AsyncReadSocket(threading.Thread):
    def __init__(self, queue, address="10.42.43.125", port="9999"):
        # initialize ZMQ socket
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect("tcp://%s:%s" % (address, port))
        self.socket.setsockopt(zmq.SUBSCRIBE, '')

        self.shutdown_flag = threading.Event()
        self.queue = queue

        threading.Thread.__init__(self)

    def run(self):
        while not self.shutdown_flag.is_set():
            # this is a blocking call. Waits for data to be published
            raw_message = self.socket.recv()
            # convert to PingData and pass to queue
            message = msgpack.unpackb(raw_message)
            try:
                ping = PingData.from_raw_data(message)
                print(str(ping))
                self.queue.put(ping)
            except RuntimeWarning as e:
                print(str(e))

        print("Exiting ZMQ socket read thread.")
        # socket and context is actually closed by garbage collection

