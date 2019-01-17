import zmq
from google.protobuf.timestamp_pb2 import Timestamp

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:9999")
socket.setsockopt_string(zmq.SUBSCRIBE, '')

timestamp = Timestamp()

while True:
    raw_message = socket.recv()
    timestamp.ParseFromString(raw_message)
    print('got something')
    print(timestamp)
