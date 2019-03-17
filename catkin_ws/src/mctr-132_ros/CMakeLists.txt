cmake_minimum_required(VERSION 2.8.3)
project(mctr-132_ros)

catkin_package()

find_package(catkin REQUIRED rospy std_msgs)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


catkin_install_python(PROGRAMS
	nodes/mctr_imu_node.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
