import zmq
import struct

context = zmq.Context()
socket = context.socket(zmq.SUB)
ip = input('Input IP(Default: 138): ')
socket.connect("tcp://192.168.43." + ip + ":5555")
topic = input('Input Topic Name(Default: A_to_B): ')
socket.setsockopt(zmq.SUBSCRIBE, topic.encode())

while True:
    message = socket.recv_multipart()[1]
    data = struct.unpack('f' * (len(message) // 4), message)
    print(data)