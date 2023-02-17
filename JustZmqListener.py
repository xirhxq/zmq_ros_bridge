import zmq
import struct

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.43.138:5555")
socket.setsockopt(zmq.SUBSCRIBE, b'A_to_B')

while True:
    message = socket.recv()
    data = struct.unpack('f' * (len(message) // 4), message)
    print(data)