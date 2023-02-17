import zmq
import rospy
from std_msgs.msg import String, Float32MultiArray
import struct
from functools import partial

name_ip_dict = {
    'A': 'tcp://192.168.43.138:5555',
    'B': 'tcp://192.168.43.150:5555'
}

my_name = 'B'
print('My IP', name_ip_dict[my_name])

context = zmq.Context()
poller = zmq.Poller()

sub_sockets = {}
for name, socket in name_ip_dict.items():
    if name == my_name:
        continue
    sub_socket = context.socket(zmq.SUB)
    sub_socket.connect(socket)
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, name + '_to_' + my_name)
    sub_sockets[name] = sub_socket
    poller.register(sub_socket, zmq.POLLIN)
    print(f'Listen to {name} @ {socket}')

pub_socket = context.socket(zmq.PUB)
pub_socket.bind("tcp://*:5555")
    

def float_array_callback(msg, who):
    data = msg.data
    msg = struct.pack('f' * len(data), *data)
    print(f'Got ROS Message {data}')
    topic = my_name + '_to_' + who
    pub_socket.send_multipart([topic.encode(), msg])

ros_subs = []
for name, socket in name_ip_dict.items():
    if name == my_name:
        continue
    ros_sub = rospy.Subscriber(
        '/' + my_name + '_to_' + name,  
        Float32MultiArray, 
        partial(float_array_callback, who=name)
    )
    print(f'Listen to topic /{my_name}_to_{name}')
    ros_subs.append(ros_sub)

ros_pubs = {}
for name, socket in name_ip_dict.items():
    if name == my_name:
        continue
    ros_pub = rospy.Publisher('/' + name + '_to_' + my_name, Float32MultiArray, queue_size=10)
    print(f'Will pub /{name}_to_{my_name}')
    ros_pubs[name] = ros_pub

def main_loop():
    while not rospy.is_shutdown():
        socks = dict(poller.poll())

        for name, sub_socket in sub_sockets.items():
            if sub_socket in socks and socks[sub_socket] == zmq.POLLIN:
                message = sub_socket.recv_multipart()[1]
                data = struct.unpack('f' * (len(message) // 4), message)
                rospy.loginfo(f"Received message: {data}")
                ros_pubs[name].publish(Float32MultiArray(data=data))

    context.term()

if __name__ == '__main__':
    rospy.init_node('zmq_ros_bridge', anonymous=True)
    main_loop()