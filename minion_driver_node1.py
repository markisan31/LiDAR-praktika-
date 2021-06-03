#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import serial
import time

stm = ...
sensor_data_shape = [8,8,4,4,4]
my_array_for_publishing = Int32MultiArray()

def clamp(value: float):
	return int (min(max(value, 1000.0), 2000.0))


def find_stm():
	stm_product = 'STM32 STLink'
	stm_dev_port = ''
	ports = list(port_list.comports())
	for p in ports:
		if p.product == stm_product:
			stm_dev_port = p.device

	if stm_dev_port == '':
		raise Exception("STM not found!")
	else:
		print(f"Found STM at {stm_dev_port}")
		return stm_dev_port

# LiDAR start
def clbk_laser(msg):
    
    regions = { #dictionary of ranges
        'right':  min(msg.ranges[0:8]), 
        'fright': min(msg.ranges[9:17]), 
        'front':  min(msg.ranges[18:26]), 
        'fleft':  min(msg.ranges[27:35]), 
        'left':   min(msg.ranges[36:44]), 
    }
    take_action(regions)

def take_action(regions): #v2ga triviaalne

    state_description = ''
    min = 1
    

    velocity = 1500
	angularVelocity = 1500 #1000 vasak, 2000 paremale

    if regions['front'] > min and regions['fleft'] > min and regions['fright'] > min:
        state_description = 'case 1 - drive everywhere'
        velocity = 1560
        angularVelocity = 1500
    elif regions['front'] < min and regions['fleft'] > min and regions['fright'] > min:
        state_description = 'case 2 - turn left or right'
        velocity = 1500
        angularVelocity = 1460 #vasakule
    elif regions['front'] > min and regions['fleft'] > min and regions['fright'] < min:
        state_description = 'case 3 - drive forward or turn left'
        velocity = 1500
        angularVelocity = 1460
    elif regions['front'] > min and regions['fleft'] < min and regions['fright'] > min:
        state_description = 'case 4 - drive forward or turn right'
        velocity = 1500
        angularVelocity = 1560
    elif regions['front'] < min and regions['fleft'] > min and regions['fright'] < min:
        state_description = 'case 5 - turn left'
        velocity = 1500
        angularVelocity = 1460
    elif regions['front'] < min and regions['fleft'] < min and regions['fright'] > min:
        state_description = 'case 6 - turn right'
        velocity = 1500
        angularVelocity = 1560
    elif regions['front'] < min and regions['fleft'] < min and regions['fright'] < min:
        state_description = 'case 7 - stop'
        velocity = 1500
        angularVelocity = 1500
    elif regions['front'] > min and regions['fleft'] < min and regions['fright'] < min:
        state_description = 'case 8 - drive forward'
        velocity = 1560
        angularVelocity = 1500
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
	

	# rospy.loginfo(rospy.get_caller_id() + "In ints %i %i", velocity, angularVelocity)
	try:
		output = b'AA'+ str(velocity).encode('utf-8') + str(angularVelocity).encode('utf-8') 
		stm.write(output)

		# Read only when writing is done
		sensor_data = read_serial()
		if sensor_data != None:
			publish_sensor_data(sensor_data)

	except Exception as e:
		print(e)

# LiDAR end

def read_serial():
	line_in = b''
	try:
		# read whole buffer and find the last line in
		receive = stm.read_all()
		if len(receive) >= 29:
			for i in range (len(receive)-1, 0, -1):
				if receive[i-1:i] == b'\n' and i-29 >=0:
					line_in = receive[i-29:i]
					# print(line_in)
					break

		if len(line_in) == 29:			
			input = []
			index = 0
			for d in sensor_data_shape:
				input.append(int(float(line_in[index:index+d].split(b'\x00',1)[0].decode('utf-8'))))
				index += d
			return input

	except Exception as e:
		print(e)		


def publish_sensor_data(input):
	global my_array_for_publishing
	my_array_for_publishing.data = input
	pub=rospy.Publisher('sensor_vals', Int32MultiArray, queue_size=1)
	pub.publish(my_array_for_publishing)



def main():

    rospy.init_node('minion_driver_node')
    with serial.Serial(find_stm(), 115200, timeout=0) as stm:
		rospy.Subscriber('cmd_vel', Twist, callback)
		time.sleep(1)
		pub=rospy.Publisher('sensor_vals', Int32MultiArray, queue_size=1)
		sub = rospy.Subscriber('scan', LaserScan, clbk_laser)
        # print(stm.readline())
		# stm.reset_input_buffer()
		# stm.reset_output_buffer()
		# while not rospy.is_shutdown():

		rospy.spin()


if __name__ == '__main__':
    main() 

