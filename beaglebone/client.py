import zmq

context = zmq.Context()
print("Connecting to server...")
socket = context.socket(zmq.REQ)
socket.connect("tcp://127.0.0.1:5555")

#  Do 10 requests, waiting each time for a response
for request in range (1,10):
    print ("Sending request ", request,"...")
    socket.send_string("$set gain 40.0")
    #  Get the reply.
    message = socket.recv()
    print ("Received reply ", request, "[", message, "]")
