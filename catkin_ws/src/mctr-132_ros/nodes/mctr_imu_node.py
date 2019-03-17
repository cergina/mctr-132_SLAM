#!/usr/bin/env python
#this scrip is defined to work only if 15 elements are being outputed
# time[ms], accX/Y/Z, gyr X/Y/Z, mag X/Y/Z, quat W/X/Y/Z, Unknown
#author: Maros Cerget

#imports
import os, sys
import serial
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

#ports
ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 6)

#constants
time1 = -1 #time1 == -1 => first reading
time2 = -1 #time1 != -1 && time2 == -1 => second reading
delta = -1 #time1 != -1 && time2 != -1 => time2 - time1
seq = 0 #frame sequence start
rate_in_hertz = 10 # 1 - 100Hz

#functions
# ok
def convert(s):			#try to change the string into float value
	try:
		return float(s)
	except ValueError:
		return s

#ok
def split_imu_array():
	imu_string = ser.readline()
	imu_string = imu_string.rstrip()
	old = imu_string.split(", ")
	new = [convert(i) for i in old]
	return new

# experimental quaternion IMU function
# from: http://www.varesano.net/blog/fabio/simple-gravity-compensation-9-dom-imus#comment-23628
# unknown if q is w,x,y,z or x,y,z,w
def gravity_compensate(q, acc):
  g = [0.0, 0.0, 0.0]
  # get expected direction of gravity
  g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
  g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
  g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
  # compensate accelerometer readings with the expected direction of gravity
  return [acc[0] - g[0], acc[1] - g[1], acc[2] - g[2]]

#publishing f
def publisheris():
	global seq
	global time1
	global time2
	#pub = rospy.Publisher('mctr_imu_read', String, queue_size=1)
	pub = rospy.Publisher('imu', Imu, queue_size=1)
	rospy.init_node('publisheris', anonymous=True)
	# 1-100Hz
	rate = rospy.Rate(rate_in_hertz)
	#fill time2 before loop
	numbers = split_imu_array()
	time2 = numbers[0]
	rate.sleep()
	while not rospy.is_shutdown():
		numbers = split_imu_array()
		print(numbers)
		time1 = time2
		time2 = numbers[0]		#acquire actual time readings from serial
		delta = time2 - time1
		print("Time delta: " + str(delta))
		#
		#pokus s readingami z iMU
		#
		imuMsg = Imu()
		imuMsg.header.stamp = rospy.Time.now()
		imuMsg.header.frame_id = 'base_imu_link'
		imuMsg.header.seq = seq
		seq = seq + 1
		#parse Acceleration data
		imuMsg.linear_acceleration.x = numbers[1] #2
		imuMsg.linear_acceleration.y = numbers[2] #3
		imuMsg.linear_acceleration.z = numbers[3] #4
		# fill acceleration covariance 
		#imuMsg.linear_acceleration_covariance = [
		#0.04 , 0 , 0,
		#0 , 0.04, 0,
		#0 , 0 , 0.04
		#]
		#
		#parse quaternions data
		imuMsg.orientation.w = numbers[10] #11
		imuMsg.orientation.x = numbers[11] #12
		imuMsg.orientation.y = numbers[12] #13
		imuMsg.orientation.z = numbers[13] #14
		# fill quaternions(orientation) covariance
		#imuMsg.orientation_covariance = [
		#0.0025 , 0 , 0,
		#0, 0.0025, 0,
		#0, 0, 0.0025
		#]
		#parse angular velocity data
		imuMsg.angular_velocity.x = numbers[4] #5
		imuMsg.angular_velocity.y = numbers[5] #6
		imuMsg.angular_velocity.z = numbers[6] #7
		# fill angular covariance
		#imuMsg.angular_velocity_covariance = [
		#0.02, 0 , 0,
		#0 , 0.02, 0,
		#0 , 0 , 0.02
		#]
		#
		#koniec pokusu :D
		#
		#rospy.loginfo(imu_string) 	#String
		#pub.publish(imu_string) 	#String
		#rospy.loginfo(imuMsg)		# Imu
		pub.publish(imuMsg)			# Imu
		#
		# experimental part
		quat = [numbers[10], numbers[11], numbers[12], numbers[13]]
		accel = [numbers[1], numbers[2], numbers[3]]
		test = gravity_compensate(quat, accel)
		print("before: " + str(accel[0])+ " " + str(accel[1]) + " " + str(accel[2]))
		print("after: " + str(test[0]) + " " + str(test[1]) + " " + str(test[2]))
		#
		# end of experimental part
		#
		#just to remember how transformation looks
		#static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_ms
		rate.sleep()

# main program
if __name__ == '__main__':
	try:
		publisheris()
	except rospy.ROSInterruptException:
		pass

