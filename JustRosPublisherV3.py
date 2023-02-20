import rospy
import random
import time
from std_msgs.msg import String, Float32MultiArray
from datetime import datetime
import socket

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

other_names = [name for name in name_ip_dict if name != my_name]
if other_names == []:
    print('No Other Names')
    raise AssertionError
other_name = other_names[0]

from_who_input = input(f'From who(Default: {my_name}): ')
from_who = my_name if from_who_input == '' else from_who_input

to_whom_input = input(f'To whom(Default: {other_name}): ')
to_whom = other_name if to_whom_input == '' else to_whom_input

topic = '/' + from_who + '_to_' + to_whom
print('Topic is ' + topic)
ros_pub = rospy.Publisher(topic, Float32MultiArray, queue_size=10)

len_input = input('Data length(Default: 2): ')
len = 2 if len_input == '' else int(len_input)


rospy.init_node('JustRosPublisher', anonymous=True)

while not rospy.is_shutdown():
    data = [random.randint(1, 10) for _ in range(len)]
    ros_pub.publish(Float32MultiArray(data=data))
    print(f'Pub {data} @ {now_ms()}')
    time.sleep(1)