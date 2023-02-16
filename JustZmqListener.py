import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5559")
socket.setsockopt(zmq.SUBSCRIBE, b'')

while True:
    message = socket.recv()
    print(message)