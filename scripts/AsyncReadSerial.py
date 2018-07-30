import threading
import serial
import Queue
import time


class AsyncReadSerial(threading.Thread):
    def __init__(self, port, queue):
        self.serial = serial.Serial(port, timeout=0.5)

        self.shutdown_flag = threading.Event()
        self.buf = bytearray()
        self.queue = queue

        threading.Thread.__init__(self)

    def run(self):
        while not self.shutdown_flag.is_set():
            # this is a blocking call. Waits for a full line to be published
            i = max(1, min(2048, self.serial.in_waiting))
            data = self.serial.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i + 1]
                self.buf[0:] = data[i + 1:]
                self.queue.put(r)
            else:
                self.buf.extend(data)

        self.serial.close()
        print("Exiting Serial socket thread.")

    def write(self, data):
        if self.serial.is_open():
            self.serial.write(data=data)
        else:
            raise RuntimeError("Cannot write. Serial port is closed.")



