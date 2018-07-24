import zmq
import msgpack

context = zmq.Context()
socket = context.socket(zmq.SUB)
port = "9999"
socket.connect("tcp://10.42.43.125:%s" % port)

socket.setsockopt(zmq.SUBSCRIBE, '')

message = socket.recv()
a = msgpack.unpackb(message)
print(a)