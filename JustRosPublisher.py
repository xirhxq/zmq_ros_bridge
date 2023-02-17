import rospy
import random
import time
from std_msgs.msg import String, Float32MultiArray

topic = input('Input Topic Name(Default: /B_to_A):')
ros_pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)


rospy.init_node('JustRosPublisher', anonymous=True)

while not rospy.is_shutdown():
    data = [random.uniform(0, 1) for _ in range(2)]
    ros_pub.publish(Float32MultiArray(data=data))
    print(f'Pub {data}')
    time.sleep(1)