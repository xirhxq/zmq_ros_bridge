import rospy
import time
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import socket
import random

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

from_who_input = input(f'Client Name(Default: {other_name}): ')
from_who = other_name if from_who_input == '' else from_who_input

to_whom_input = input(f'Server Name(Default: {my_name}): ')
to_whom = my_name if to_whom_input == '' else to_whom_input

rospy.init_node('TWTOFV4Server', anonymous=True)

tx_pub = rospy.Publisher('/tx', Float32MultiArray, queue_size=10)

def send_message_to(whom, data):
    msg = Float32MultiArray(data=data)
    msg.layout.dim.append(MultiArrayDimension(label=whom))
    tx_pub.publish(msg)

poll_data = []
def sub_callback(msg):
    if msg.layout.dim[0].label == other_name:
        tbpoll = time.time()
        poll_data = msg.data
        treply = random.uniform(0.5, 1.5)
        poll_data.append(treply)
        time.sleep(treply)
        send_message_to(other_name)

rx_sub = rospy.Subscriber('/rx', Float32MultiArray, sub_callback)

test_res = []

while not rospy.is_shutdown():
    print('Start Server...')