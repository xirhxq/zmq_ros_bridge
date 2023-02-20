import zmq
import rospy
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import struct
from functools import partial
import socket
from datetime import datetime

def now_ms():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]

name_ip_dict = {
    'A': 'tcp://192.168.43.142:5555',
    'B': 'tcp://192.168.43.6:5555'
}

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('8.8.8.8', 80))
my_ip = s.getsockname()[0]

my_name = ''
print('My IP: ', my_ip)
psb_name = [name for name, socket in name_ip_dict.items() if my_ip in socket]

if len(psb_name) != 1:
    print('Can\'t find right name for this IP')
    raise AssertionError
else:
    my_name = psb_name[0]
    print('My name: ' + my_name)


context = zmq.Context()
poller = zmq.Poller()

sub_sockets = {}
for name, socket in name_ip_dict.items():
    if name == my_name:
        continue
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(socket)
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, my_name)
    sub_sockets[name] = sub_socket
    poller.register(sub_socket, zmq.POLLIN)
    print(f'Listen to {name} @ {socket}')

pub_socket = context.socket(zmq.PUB)
pub_socket.bind("tcp://*:5555")
    

def float_array_callback(msg):
    data = msg.data
    who = msg.layout.dim[0].label
    msg = struct.pack('f' * len(data), *data)
    print(f'ROS->TCP | {my_name}(me) to {who} | {now_ms()} | {data}')
    pub_socket.send_multipart([who.encode(), msg])

ros_sub = rospy.Subscriber(
    '/tx',  
    Float32MultiArray, 
    float_array_callback
)
print(f'Listen to ROS topic /tx on {my_name}\'s ROS network')


ros_pub = rospy.Publisher('/rx', Float32MultiArray, queue_size=10)
print(f'Will pub /rx on {my_name}\'s ROS network')

def main_loop():
    while not rospy.is_shutdown():
        socks = dict(poller.poll(10))

        for name, sub_socket in sub_sockets.items():
            if sub_socket in socks and socks[sub_socket] == zmq.POLLIN:
                message = sub_socket.recv_multipart()[1]
                data = struct.unpack('f' * (len(message) // 4), message)
                print(f'TCP->ROS | {name} to {my_name}(me) | {now_ms()} | {data}')
                msg = Float32MultiArray(data=data)
                msg.layout.dim.append(MultiArrayDimension(label=name))
                ros_pub.publish(Float32MultiArray(data=data))

    context.term()

if __name__ == '__main__':
    rospy.init_node('zmq_ros_bridge', anonymous=True)
    main_loop()