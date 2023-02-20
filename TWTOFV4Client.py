import rospy
import time
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import socket
import statistics

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

from_who_input = input(f'Client Name(Default: {my_name}): ')
from_who = my_name if from_who_input == '' else from_who_input

to_whom_input = input(f'Server Name(Default: {other_name}): ')
to_whom = other_name if to_whom_input == '' else to_whom_input

rospy.init_node('TWTOFV4Client', anonymous=True)

tx_pub = rospy.Publisher('/tx', Float32MultiArray, queue_size=10)

def send_message_to(whom, data):
    msg = Float32MultiArray(data=data)
    msg.layout.dim.append(MultiArrayDimension(label=whom))
    tx_pub.publish(msg)

resp_data = []
def sub_callback(msg):
    if msg.layout.dim[0].label == other_name:
        resp_data = msg.data

rx_sub = rospy.Subscriber('/rx', Float32MultiArray, sub_callback)


def test_once():
    tapoll = time.time()
    send_message_to(other_name, [tapoll])
    time_exceed_flag = False
    time_limit = 3.0
    while resp_data[0] != tapoll and not time_exceed_flag:
        if time.time() > tapoll + time_limit:
            time_exceed_flag = True
    if time_exceed_flag:
        print('Time Exceeded')
        return -1
    print(f'Get response {resp_data}')
    taresp = time.time()
    treply = resp_data[1]
    tround = taresp - tapoll
    return (tround - treply) / 2


test_res = []

while not rospy.is_shutdown():
    operation = input('Continue to test? (y/n): ')
    if operation == '' or operation == 'y':
        res = test_once()
        if res > 0:
            test_res.append(res)
            print(f'New Result {res}')
    else:
        print(f'TWTOF Result: avg[{statistics.mean(test_res)}] | std var[{statistics.stdev(test_res)}] | median[{statistics.median(test_res)}]')