import zmq
import random
import time
import struct
from datetime import datetime

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")


while True:
    data = [random.uniform(0, 1) for _ in range(3)]
    message = struct.pack('f' * len(data), *data)
    topic = b'A_to_B'

    socket.send_multipart([topic, message])
    print(f"Send Message {data} @ {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")

    time.sleep(3)
