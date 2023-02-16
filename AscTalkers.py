import zmq
import time
import random

context = zmq.Context()

# 设置 A 节点
a_socket = context.socket(zmq.PUB)
a_socket.bind("tcp://*:5555")

# 设置 B 节点
b_socket = context.socket(zmq.PUB)
b_socket.bind("tcp://*:5556")

# 设置订阅者
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5557")
subscriber.setsockopt_string(zmq.SUBSCRIBE, '')

poller = zmq.Poller()
poller.register(subscriber, zmq.POLLIN)

last_a_sent = 0
last_b_sent = 0

while True:
    current_time = time.time()
    # A节点发送消息
    if current_time - last_a_sent > 0.5:
        a_msg = [random.random() for _ in range(2)]
        a_socket.send_multipart([b'a', str(a_msg).encode()])
        last_a_sent = current_time
        print(f"[{time.strftime('%H:%M:%S.%f', time.localtime())}] A sent: {a_msg}")

    # B节点发送消息
    if current_time - last_b_sent > 0.333:
        b_msg = [random.random() for _ in range(3)]
        b_socket.send_multipart([b'b', str(b_msg).encode()])
        last_b_sent = current_time
        print(f"[{time.strftime('%H:%M:%S.%f', time.localtime())}] B sent: {b_msg}")

    # 接收消息
    events = dict(poller.poll(100))
    if subscriber in events and events[subscriber] == zmq.POLLIN:
        while subscriber.poll(timeout=0, flags=zmq.POLLIN):
            topic, msg = subscriber.recv_multipart()
            print(f"[{time.strftime('%H:%M:%S.%f', time.localtime())}] Received {msg} from {topic}")

    time.sleep(0.01)