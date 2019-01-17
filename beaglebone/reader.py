import zmq
from google.protobuf.timestamp_pb2 import Timestamp
from build.sonardata_pb2 import SonarData

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:9999")
socket.setsockopt_string(zmq.SUBSCRIBE, '')

sonar_data = SonarData()

while True:
    raw_message = socket.recv()
    sonar_data.ParseFromString(raw_message)

    print("{}.{}".format(
        sonar_data.timestamp.seconds,sonar_data.timestamp.nanos))
    print(len(sonar_data.data))
    # print(sonar_data.data[0])
    print(sonar_data.data[0].a)
    print(dict(sonar_data.stats))
