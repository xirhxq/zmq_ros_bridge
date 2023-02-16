import zmq
import rospy
from std_msgs.msg import String, Float32MultiArray
import struct
from functools import partial

name_ip_dict = {
    'A': 'tcp://localhost:5555',
    'B': 'tcp://192.168.1.101:5555', 
    'C': 'tcp://192.168.1.102:5555'
}

my_name = 'A'

context = zmq.Context()
poller = zmq.Poller()

# 订阅者套接字
sub_sockets = {}
for name, socket in name_ip_dict.items():
    if name == my_name:
        continue
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(socket)
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, name + '_to_' + my_name)
    sub_sockets[name] = sub_socket
    poller.register(sub_socket, zmq.POLLIN)

# 发布者套接字
pub_socket = context.socket(zmq.PUB)
pub_socket.bind("tcp://*:5555")
    

def float_array_callback(msg, who):
    data = msg.data
    msg = struct.pack('f' * len(data), *data)
    topic = my_name + '_to_' + who
    pub_socket.send_multipart([topic, msg])

# ROS订阅者和发布者
ros_subs = []
for name, socket in name_ip_dict.items():
    if name == my_name:
        continue
    ros_sub = rospy.Subscriber(
        '/' + my_name + '_to_' + name,  
        Float32MultiArray, 
        partial(float_array_callback, who=name)
    )
    ros_subs.append(ros_sub)

ros_pubs = {}
for name, socket in name_ip_dict.items():
    if name == my_name:
        continue
    ros_pub = rospy.Publisher('/' + name + '_to_' + my_name, Float32MultiArray, queue_size=10)
    ros_pubs[name] = ros_pub


def main_loop():
    while not rospy.is_shutdown():
        socks = dict(poller.poll())

        # 监听订阅者
        for name, sub_socket in sub_sockets:
            if sub_socket in socks and socks[sub_socket] == zmq.POLLIN:
                message = sub_socket.recv()
                rospy.loginfo(f"Received message: {message}")
                data = struct.unpack('f' * (len(message) // 4), message)
                ros_pubs[name].publish(Float32MultiArray(data=data))

    context.term()

if __name__ == '__main__':
    rospy.init_node('zmq_ros_bridge', anonymous=True)
    main_loop()