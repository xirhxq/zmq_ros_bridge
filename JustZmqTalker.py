import zmq
import random
import time
import struct
from datetime import datetime

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")


while True:
    # 生成固定长度的浮点数数列
    data = [random.uniform(0, 1) for _ in range(3)]
    message = struct.pack('f' * len(data), *data)

    # 发送消息
    socket.send(message)
    print(f"Send Message {message} @ {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")

    # 等待一段时间后再发送下一条消息
    time.sleep(3)
