#! /usr/bin/env python
#This code is being written by Maros Cerget
#intended for RPLidar A2, on port /dev/ttyUSB0
#prerequsities:
#roscore
#roslaunch rplidar_ros rplidar.launch (topic: /scan)
import rospy
import math
import rospkg
import roslib.packages
import numpy
import octomap_msgs.srv
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point

SRV_NAME = '/octomap_server/clear_bbx'
SRV_INTERFACE = octomap_msgs.srv.BoundingBoxQuery

def callback(msg):
	angle_rad = msg.angle_min		#assign first angle
	angle_deg = angle_rad * 57.2957795	#angle in degrees
	rangingsCount = len(msg.ranges) 	#number of rangings[usually 360]
	print("[" + str(rangingsCount) + "] rangings\n")
	pc_array = []
	for r in range (rangingsCount):	
		#print("[" + str(r) + "] rad: " + str(angle_rad))
		#print("deg: " + str(angle_deg))
		#print("dist: " + str(msg.ranges[r]))
		angle_rad += msg.angle_increment   #new angle calculation
		angle_deg = angle_rad * 57.2957795 #angle in deg
		point_x = math.cos(angle_rad) * msg.ranges[r]
		point_y = math.sin(angle_rad) * msg.ranges[r]
		#print("[X;Y]: [" + str(point_x) + ";" + str(point_y) +"]\n")
		pc_array.append((point_x, point_y, 0))
	sem = pc2.create_cloud_xyz32(msg.header, pc_array)
	#now we need to delete points from map where we aim
	#rospy.wait_for_service('clear_bbx', 1)
	#clear_bbx = rospy.ServiceProxy(SRV_NAME, SRV_INTERFACE)
	#try:
	#	#resp1 = clear_bbx(bounding_box)
	#	resp1 = clear_bbx(min, max)
	#except rospy.ServiceException as exc:
	#	print("Service did not process request: " + str(exc))
	#send new points
	pub.publish(sem)

#def init_eraser():


if __name__ == '__main__':
	try:
		rospy.init_node('scan_values', anonymous=True)
		sub = rospy.Subscriber('/scan', LaserScan, callback)
		pub = rospy.Publisher('Testing', PointCloud2, queue_size=10)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

