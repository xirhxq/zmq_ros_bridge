import zmq
import rospy
from std_msgs.msg import String
import struct

context = zmq.Context()
poller = zmq.Poller()

# 订阅者套接字
sub_socket = context.socket(zmq.SUB)
sub_socket.connect(f"tcp://localhost:5555")
sub_socket.setsockopt_string(zmq.SUBSCRIBE, '')
poller.register(sub_socket, zmq.POLLIN)

# 发布者套接字
pub_socket = context.socket(zmq.PUB)
pub_socket.bind("tcp://*:5559")

def callback(data):
    pub_socket.send_string(data.data)

# ROS订阅者和发布者
ros_sub = rospy.Subscriber("ros_topicc", String, callback)
ros_pub = rospy.Publisher("ros_topic", String, queue_size=10)


def main_loop():
    while not rospy.is_shutdown():
        try:
            socks = dict(poller.poll())
        except KeyboardInterrupt:
            break

        # 监听订阅者
        if sub_socket in socks and socks[sub_socket] == zmq.POLLIN:
            message = sub_socket.recv()
            rospy.loginfo(f"Received message: {message}")
            data = struct.unpack('i', message)
            ros_pub.publish(str(data[0]))

    context.term()

if __name__ == '__main__':
    rospy.init_node('zmq_ros_node')
    main_loop()