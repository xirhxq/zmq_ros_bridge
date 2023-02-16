import zmq
import time
from datetime import datetime
import struct

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

last_time = time.time()  # 记录上一次处理消息的时间戳
loop_time = 1.0  # 设定每次循环的时间

poller = zmq.Poller()
poller.register(socket, zmq.POLLIN)

while True:
    # 检查是否有新消息可用
    socks = dict(poller.poll(1000))
    
    if socket in socks and socks[socket] == zmq.POLLIN:
        while socket.poll(timeout=0, flags=zmq.POLLIN):
            message = socket.recv()
            data = struct.unpack('i', message)
            print(f"Received message: {data} @ {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        # print(f"Got Message @ {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
    else:
        print(f"No New Message @ {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")

    # if socket.poll(1000, zmq.POLLIN):
    #     message = socket.recv()
    #     float_list = struct.unpack('f' * (len(message) // 4), message)
    #     # print(f"Received message: {float_list}")
    #     print(f"Got Message @ {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
    # else:
    #     print(f"No New Message @ {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")

    # 计算已经过去的时间并等待下一次循环
    elapsed_time = time.time() - last_time
    wait_time = loop_time - elapsed_time
    if wait_time > 0:
        time.sleep(wait_time)

    # 更新时间戳
    last_time = time.time()

    # 执行其他任务
    # timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    # print(f"Doing other work @ {timestamp}...")
