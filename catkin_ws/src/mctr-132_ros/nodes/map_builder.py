#!/usr/bin/env python
#
# author: Maros Cerget
# e-mail: mcerget@gmail.com
#
# headers
import rospy, datetime
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

# Python globals are limited to a module, they are not true globals
# and not affect whole program, they are less harmful than real globals
file_name = ""
file_inst_obj = None
file_inst_mtl = None
# cube size constant
c_s = 0.05
#
# create OBJ file
#
def create_files():
	global file_name
	global file_inst_obj
	global file_inst_mtl
	# create an original file name	
	file_name = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
	
	# create and open file
	file_inst_obj = open(file_name + ".obj", "w+")
	file_inst_mtl = open(file_name + ".mtl", "w+")
	return True

#
# append to the file
#
def insert_into_obj_file(arg):
	global file_inst_obj
	#
	# do the required action: INSERT
	file_inst_obj.write(arg)

#
# append to the file
#
def insert_into_mtl_file(arg):
	global file_inst_mtl
	#
	# do the required action: INSERT
	file_inst_mtl.write(arg)


#
# close the file if it exists
#
def close_files():
	global file_inst_obj
	global file_inst_mtl
	#
	# only try to close if file is open
	if file_inst_obj is not None:
		file_inst_obj.close()
	#
	# only try to close if file is open
	if file_inst_mtl is not None:
		file_inst_mtl.close()

#
# map extractor from MarkerArray
#
def map_from_marker(msg):
	global file_name
	global file_inst_obj
	global file_inst_mtl
	global c_s
	#
	# we are interested in type 0 (ADD,MODIFY) points, not the type 2 (DELETE)
	# Type 0 points is usually the last index
	no_of_markers = len(msg.markers)	

	# get the last marker
	la_marker = msg.markers[no_of_markers - 1]

	# continue only if it's truly ADD/MODIFY type
	if (la_marker.action != 0):
		print("Last Marker is not ADD or MODIFY, unexpected")
		return

	# get the number of points	
	points_count = len(la_marker.points)
	
	# try to create the file
	create_files()
	if (file_name == "" or file_inst_obj is None):
		print("Unable to open file, exiting...")
		return

	# initialize OBJ header
	insert_into_obj_file("# MCTR-132 ROS (Maros Cerget) OBJ File: ''\r\n")

	# loop through points 'la_marker.points', there are points_count of'em
	print("State: Inserting points")	
	for x in range(points_count):
			pt = la_marker.points[x]
			# insert points into the OBJ file	
			insert_into_obj_file("v " + str(pt.x - c_s) + " " + str(pt.y - c_s) + " " + str(pt.z + c_s) + "\n\r")
			insert_into_obj_file("v " + str(pt.x - c_s) + " " + str(pt.y - c_s) + " " + str(pt.z - c_s) + "\n\r")
			insert_into_obj_file("v " + str(pt.x + c_s) + " " + str(pt.y - c_s) + " " + str(pt.z - c_s) + "\n\r")
			insert_into_obj_file("v " + str(pt.x + c_s) + " " + str(pt.y - c_s) + " " + str(pt.z + c_s) + "\n\r")
			insert_into_obj_file("v " + str(pt.x + c_s) + " " + str(pt.y + c_s) + " " + str(pt.z + c_s) + "\n\r")
			insert_into_obj_file("v " + str(pt.x + c_s) + " " + str(pt.y + c_s) + " " + str(pt.z - c_s) + "\n\r")
			insert_into_obj_file("v " + str(pt.x - c_s) + " " + str(pt.y + c_s) + " " + str(pt.z - c_s) + "\n\r")
			insert_into_obj_file("v " + str(pt.x - c_s) + " " + str(pt.y + c_s) + " " + str(pt.z + c_s) + "\n\r")
		
	# insert points representing [0,0,0] (mapping root)
	pt = [0,0,0]
	c_s = c_s * 2.5
	insert_into_obj_file("v " + str(pt[0] - c_s) + " " + str(pt[1] - c_s) + " " + str(pt[2] + c_s) + "\n\r")
	insert_into_obj_file("v " + str(pt[0] - c_s) + " " + str(pt[1] - c_s) + " " + str(pt[2] - c_s) + "\n\r")
	insert_into_obj_file("v " + str(pt[0] + c_s) + " " + str(pt[1] - c_s) + " " + str(pt[2] - c_s) + "\n\r")
	insert_into_obj_file("v " + str(pt[0] + c_s) + " " + str(pt[1] - c_s) + " " + str(pt[2] + c_s) + "\n\r")
	insert_into_obj_file("v " + str(pt[0] + c_s) + " " + str(pt[1] + c_s) + " " + str(pt[2] + c_s) + "\n\r")
	insert_into_obj_file("v " + str(pt[0] + c_s) + " " + str(pt[1] + c_s) + " " + str(pt[2] - c_s) + "\n\r")
	insert_into_obj_file("v " + str(pt[0] - c_s) + " " + str(pt[1] + c_s) + " " + str(pt[2] - c_s) + "\n\r")
	insert_into_obj_file("v " + str(pt[0] - c_s) + " " + str(pt[1] + c_s) + " " + str(pt[2] + c_s) + "\n\r")	

	print("State: Inserting normals")
	
	# insert verticle normals for cube faces
	insert_into_obj_file("vn 0.0000 -0.0000 1.0000\n\r")
	insert_into_obj_file("vn 1.0000 -0.0000 0.0000\n\r")
	insert_into_obj_file("vn 0.0000 0.0000 -1.0000\n\r")
	insert_into_obj_file("vn -1.0000 0.0000 0.0000\n\r")
	insert_into_obj_file("vn 0.0000 1.0000 0.0000\n\r")
	insert_into_obj_file("vn 0.0000 -1.0000 -0.0000\n\r")

	
	# disable smooth shading and signalize first material (point clouds)
	insert_into_obj_file("usemtl Material\r\ns off\n\r")	

	# do faces magic
	print("State: Inserting faces")
	for x in range(points_count):
		# f - face; str(4+x*8) verticle_number + which iteration // verticle_normal number
		insert_into_obj_file("f "+str(4+x*8) +"//1 "+str(5+x*8)+"//1 "+str(8+x*8)+"//1 "+str(1+x*8)+"//1\r\n")
		insert_into_obj_file("f "+str(3+x*8) +"//2 "+str(6+x*8)+"//2 "+str(5+x*8)+"//2 "+str(4+x*8)+"//2\r\n")
		insert_into_obj_file("f "+str(2+x*8) +"//3 "+str(7+x*8)+"//3 "+str(6+x*8)+"//3 "+str(3+x*8)+"//3\r\n")
		insert_into_obj_file("f "+str(8+x*8) +"//4 "+str(7+x*8)+"//4 "+str(2+x*8)+"//4 "+str(1+x*8)+"//4\r\n")
		insert_into_obj_file("f "+str(5+x*8) +"//5 "+str(6+x*8)+"//5 "+str(7+x*8)+"//5 "+str(8+x*8)+"//5\r\n")
		insert_into_obj_file("f "+str(1+x*8) +"//6 "+str(2+x*8)+"//6 "+str(3+x*8)+"//6 "+str(4+x*8)+"//6\r\n")

	# use second material for LiDAR 
	insert_into_obj_file("usemtl Material2\r\n")

	# insert faces representing the LiDAR cube
	x = points_count
	insert_into_obj_file("f "+str(4+x*8) +"//1 "+str(5+x*8)+"//1 "+str(8+x*8)+"//1 "+str(1+x*8)+"//1\r\n")
	insert_into_obj_file("f "+str(3+x*8) +"//2 "+str(6+x*8)+"//2 "+str(5+x*8)+"//2 "+str(4+x*8)+"//2\r\n")
	insert_into_obj_file("f "+str(2+x*8) +"//3 "+str(7+x*8)+"//3 "+str(6+x*8)+"//3 "+str(3+x*8)+"//3\r\n")
	insert_into_obj_file("f "+str(8+x*8) +"//4 "+str(7+x*8)+"//4 "+str(2+x*8)+"//4 "+str(1+x*8)+"//4\r\n")
	insert_into_obj_file("f "+str(5+x*8) +"//5 "+str(6+x*8)+"//5 "+str(7+x*8)+"//5 "+str(8+x*8)+"//5\r\n")
	insert_into_obj_file("f "+str(1+x*8) +"//6 "+str(2+x*8)+"//6 "+str(3+x*8)+"//6 "+str(4+x*8)+"//6\r\n")
	
	# insert materials stuff
	insert_into_mtl_file("# Blender MTL File: 'None'\r\n")
	insert_into_mtl_file("# Material Count: 2\r\n\r\n")
	insert_into_mtl_file("newmtl Material\r\nNs 96.078431\n\rKa 1.0 1.0 1.0\r\nKd 0.64 0.64 0.64\r\nKs 0.5 0.5 0.5\r\nKe 0.0 0.0 0.0\r\nNi 1.0\r\nd 1.0\r\nillum 2\r\n\r\n")
	insert_into_mtl_file("newmtl Material2\r\nNs 96.078431\n\rKa 1.0 1.0 1.0\r\nKd 0.004739 0.181267 0.0\r\nKs 0.5 0.5 0.5\r\nKe 0.0 0.0 0.0\r\nNi 1.0\r\nd 1.0\r\nillum 2\r\n\r\n")

	# close file
	close_files()
	print("State: Finished")		

#
# MAIN is here
#
if __name__ == '__main__':
	try:
		# important, node initializer
		rospy.init_node('obj_extractor', anonymous=True)
		try:
			#
			# MarkerArray
			print("Waiting for MarkerArray to arrive")
			m = rospy.wait_for_message('/occupied_cells_vis_array', MarkerArray, 1)
			map_from_marker(m)
			#
		except rospy.ROSException:
			# error occured
			print("No message arrived within specified time")
	except rospy.ROSInterruptException:
		pass

