#!/usr/bin/env python
import rospy

#import tf_conversions
import tf
import geometry_msgs.msg
from sensor_msgs.msg import Imu

data_imu = "empty"
#data_pc2 = "empty"

def handle_pose(msg):
	#
	data_imu = msg
	#
	#t = geometry_msgs.msg.TransformStamped()
	#t.header.stamp = rospy.Time.now()
	#t.header.frame_id = "map" #should be map
	#t.child_frame_id = "laser"
	#handle translation X,Y,Z
	#t.transform.translation.x = 0.0
	#t.transform.translation.y = 0.0
	#t.transform.translation.z = 0.0
	#handle orientation W,X,Y,Z
	#t.transform.rotation.w = 
	#t.transform.rotation.x = 
	#t.transform.rotation.y = 
	#t.transform.rotation.z = 
	# finish it
	odom_broadcaster.sendTransform(
		(0.0, 0.0, 0.0),
		(0.0, 0.0, 0.0, 1.0),
		rospy.Time.now(),
		"map",
		"world"
	)

	odom_broadcaster.sendTransform(
		(0.0, 0.0, 0.0),
		(data_imu.orientation.x, data_imu.orientation.y,
			data_imu.orientation.z, data_imu.orientation.w,),
		rospy.Time.now(),
		"laser_link",
		"map"
	)

#def pc2_callback(msg):
	#
	#data_pc2 = msg
	#

if __name__ == '__main__':
	try:
		rospy.init_node('tf2_broadcaster', anonymous=True)
		#sub_pc2 = rospy.Subscriber('Testing', PointCloud2, pc2_callback)
		sub_imu = rospy.Subscriber('/imu', Imu, handle_pose)
		#pub_tf = rospy.Publisher('tf2_broadcast', )
		odom_broadcaster = tf.TransformBroadcaster()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

