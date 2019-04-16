#!/usr/bin/env python
#
# author: Maros Cerget
# e-mail: mcerget@gmail.com
#
# headers
import sys, rospy, array, math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

#
# Information about how to run the script
#
def provide_help():
	print("\n\rScript that determines a distance to an obstacle in 3D direction of Non-Zero vector")
	print("\n\rrun: 'rosrun mctr-132_ros distance_getter.py [-h] [a] l n [pr]'\n\r")
	print("[-h]:		Help")
	print("[-a]:		Debug")
	print("l 		Level (int)")
	print("n		Position  (int)")
	print("[pr]		Tolerance (float), default is: 0.04")


def show_all(msg):
	print("\n\rThis is a debugging tool that shows all points with its corresponding positions\n\r")
	# Number of markers
	no_of_markers = len(msg.markers)

	# Get the last marker
	la_marker = msg.markers[no_of_markers - 1]
	
	# Continue only if it's add/modify type
	if (la_marker.action != 0):
		print("Last Marker is not ADD or MODIFY, unexpected")
		return

	# get the number of points	
	points_count = len(la_marker.points)

	print("Set of [" + str(points_count) + "] points")

	for x in range(points_count):
		print("pt: " + str(la_marker.points[x].x) + "/" + str(la_marker.points[x].y) + "/" + str(la_marker.points[x].z))


# 
# error output
#
def not_enough_arguments():
	print("Not enough argument's. Check help: '-h' for the script.")

def incorrect_arguments():
	print("Vector has to be set with 0-2 level, 0-7 arrow number. Check help: '-h' for the script.")

#
# description of returning values
#  return is < 0: different direction of vector or total miss
#  return is 0: every vector can get to point at 0, if you multiply the vector by 0
#  return is > 0: point is indeed in the direction of vector, it's the matter of ratio
#
def get_ratio(pt_coord, vct_coord, prec):
	# 0 vector can never meet point if it's not at 0 too
	if (vct_coord == 0.0):
		if ((pt_coord >= 0.0 - prec) and (pt_coord <= 0.0 + prec)):
			return 0.0 #ZERO_HIT
		else:
			return -1.0 #ZERO_MISS (return -1 to signalize as if it was different angle)
	
	# the probability of hit exists, return the ratio
	return pt_coord / vct_coord

# 3D vector has to be passed
def calculate_distance(vec_x, vec_y, vec_z):
	return math.sqrt ( (vec_x*vec_x) + (vec_y*vec_y) + (vec_z*vec_z) )

#
# distance determinator
#
def determine_obstacle(msg, v1, v2, v3, prec):

	# Number of markers
	no_of_markers = len(msg.markers)

	# Get the last marker
	la_marker = msg.markers[no_of_markers - 1]
	
	# Continue only if it's add/modify type
	if (la_marker.action != 0):
		print("Last Marker is not ADD or MODIFY, unexpected")
		return

	print("\r\nObstacle detection started, with precision: " + str(prec))
	print("Vector(" + str(v1) +" "+ str(v2) +" "+ str(v3) +")")
	
	# get the number of points	
	points_count = len(la_marker.points)
		
	print("Set of [" + str(points_count) + "] points") 
	
	# empty array which will be filled with relevant points, atc = array_to_consider	
	atc = []
	
	# example: P={4.0 0.0 2.0} V={1.0 0.0 0.5}
	# Points is ok if, 4 / 1.0 = 4 and V * 4 is equal to Point
	# 
	for x in range(points_count):
		# get ratios for each coordinate
		ratio_x = get_ratio(la_marker.points[x].x, v1, prec)
		ratio_y = get_ratio(la_marker.points[x].y, v2, prec)
		ratio_z = get_ratio(la_marker.points[x].z, v3, prec)
		
		if (ratio_x < 0.0 or ratio_y < 0.0 or ratio_z < 0.0):
			continue	#this is total miss
		
		#whether to ignore some axies based on the 0 hit
		ratio_rel = ratio_x
		if (ratio_rel == 0.0):
			ratio_rel = ratio_y
		if (ratio_rel == 0.0):
			ratio_rel = ratio_z
		# if ratio is still 0.0 it means they are equal and it's ok		
		
		# calculate new point coordinates
		new_x = v1 * ratio_rel
		new_y = v2 * ratio_rel
		new_z =	v3 * ratio_rel
		
		# add them into the array to consider if they're within precision
		if (abs(new_x - la_marker.points[x].x) > prec):
			continue # not within tolerance
		if (abs(new_y - la_marker.points[x].y) > prec):
			continue # not within tolerance
		if (abs(new_z - la_marker.points[x].z) > prec):
			continue # not within tolerance
		
		# finally add
		atc.append(la_marker.points[x])

	print("\r\nNumber of Obstacles in way: " + str(len(atc)))
	
	for c in range(len(atc)):
		print(" [" + str(c) +"]:  {"+str(atc[c].x)+" "+str(atc[c].y)+" "+str(atc[c].z)+" in distance " + str(calculate_distance(atc[c].x, atc[c].y, atc[c].z))+"}\r\n")

	print("Obstacle detection finished")

#
# WHICH vector: level[0-2], chosen_arrow[0-7]
#
def vector_to_use(level, chosen_arrow, offset):
	#these values are bullshit's right now
	switcher = {
		0:  [ 1.0 , 0.0, -1.0],     # lowest level, head
		1:  [ 1.0, 1.0 , -1.0],   # right in 45degrees
		2:  [ 0.0 , 1.0 , -1.0],   # right in 90degrees
		3:  [ -1.0 , 1.0 ,-1.0],   # right in 135degrees
		4:  [ -1.0 , 0.0 , -1.0],   # right in 180degrees
		5:  [ -1.0 , -1.0 , -1.0],   # right in 225degrees
		6:  [ 0.0 , -1.0 , -1.0],   # right in 270degrees
		7:  [ 1.0 , -1.0 , -1.0],   # right in 315degrees
		
		8:  [ 1.0 , 0.0 , 0.0],  # mid level, head
		9:  [ 1.0, 1.0 , 0.0],  	 # right in 45degrees
		10: [ 0.0 , 1.0 , 0.0],	 # right in 90degrees
		11: [ -1.0 , 1.0 ,0.0],	 # right in 135degrees
		12: [ -1.0 , 0.0 ,0.0],	 # right in 180degrees
		13: [ -1.0 , -1.0 ,0.0], # right in 225degrees
		14: [ 0.0 , -1.0 ,0.0],	 # right in 270degrees
		15: [ 1.0 , -1.0 ,0.0],	 # right in 315degrees
		
		16: [ 1.0 , 0.0 , 1.0],  #up level, head
		17: [ 1.0, 1.0, 1.0],    # right in 45degrees
		18: [ 0.0 , 1.0 , 1.0],   # right in 90degrees
		19: [ -1.0 , 1.0 , 1.0],   # right in 135degrees
		20: [ -1.0 , 0.0 , 1.0],   # right in 180degrees
		21: [ -1.0 , -1.0 , 1.0],   # right in 225degrees
		22: [ 0.0 , -1.0 , 1.0],   # right in 270degrees
		23: [ 1.0 , -1.0 , 1.0],   # right in 315degrees
	}

	# eg.: level 1, 6th arrow 1*8 + 6 = 14
	vector_num = level * 8 + chosen_arrow
	
	to_return = switcher.get((level*8 + chosen_arrow))
	#x = to_return[0] + (-1 * offset[0])
	x = to_return[0] + (offset[0])
	#y = to_return[1] + (-1 * offset[1])
	y = to_return[1] + (offset[1])
	#z = to_return[2] + (-1 * offset[2])
	z = to_return[2] + (offset[2])

	return [x, y, z]

# BACKUP
# WHICH vector: level[0-2], chosen_arrow[0-7]
#
#def vector_to_use(level, chosen_arrow):
#	switcher = {
#		0: [ 0.1 , 0.5 ,0.5],
#		1: [ 0.2 , 0.5 ,0.5],
#		2: [ 0.3 , 0.5 ,0.5],
#		3: [ 0.4 , 0.5 ,0.5],
#		4: [ 0.5 , 0.5 ,0.5],
#		5: [ 0.6 , 0.5 ,0.5],
#		6: [ 0.7 , 0.5 ,0.5],
#		7: [ 0.8 , 0.5 ,0.5],
#		8: [ 0.5 , 0.1 ,0.5],
#		9: [ 0.5, 0.2 ,0.5],
#		10: [ 0.5 , 0.3 ,0.5],
#		11: [ 0.5 , 0.4 ,0.5],
#		12: [ 0.5 , 0.5 ,0.5],
#		13: [ 0.5 , 0.6 ,0.5],
#		14: [ 0.5 , 0.7 ,0.5],
#		15: [ 0.5 , 0.8 ,0.5],
#		16: [ 0.5 , 0.5 ,0.1],
#		17: [ 0.5 , 0.5,0.2],
#		18: [ 0.5 , 0.5 ,0.3],
#		19: [ 0.5 , 0.5 ,0.4],
#		20: [ 0.5 , 0.5 ,0.5],
#		21: [ 0.5 , 0.5 ,0.6],
#		22: [ 0.5 , 0.5 ,0.7],
#		23: [ 0.5 , 0.5 ,0.8],
#	}
#
#	# eg.: level 1, 6th arrow 1*8 + 6 = 14
#	vector_num = level * 8 + chosen_arrow
#	
#	return switcher.get((level*8 + chosen_arrow))


#
# MAIN is here
#
if __name__ == '__main__':
	debug = False

	# get information about the number of passed arguments
	arg_count = len(sys.argv)
	
	# does the user want help?
	if (arg_count > 1 and sys.argv[1] == "-h"):
		provide_help()
		sys.exit()
	
	# does the user want to see set of all points?
	if (arg_count > 1 and sys.argv[1] == '-a'):
		debug = True	
	
	if (debug == False):
		# did user send less arguments that we want?
		if (debug == False and arg_count < 3):
			not_enough_arguments()
			sys.exit()
	
		# user wants to specify also the precision resolution if arg_count > 4
		def_precision = "0.04"
		pr = None
	
		if (arg_count > 3):
			pr = sys.argv[3]
		else:
			pr = def_precision

		# user sent ok number of arguments, but are they ok?
		try:
			l = int(sys.argv[1])	# level  0,1,2
			n = int(sys.argv[2])	# number 0,1,2,3,4,5,6,7
			
			if (l < 0 or l > 2 or n < 0 or n > 7):
				incorrect_arguments()
				sys.exit()
			
			#set precision
			pr = float(pr)
		except ValueError:
			incorrect_arguments()
			sys.exit()
		
		# get the vector MAROS. The offset is a hope, that's static offset 
		# which does not change over time. needs to be tested and hopefuly it
		# will work for every direction
		a = vector_to_use(l,n, [0,0,0])

		if (a == "nothing"):
			incorrect_arguments()
			sys.exit()

		# all-zero vector is not supported
		if (a[0] == 0 and a[1] == 0 and a[2] == 0):
			incorrect_arguments()
			sys.exit()
	
	try:
		#important, node initializer
		rospy.init_node('distance_getter', anonymous=True)
		try:
			#MarkerArray
			print("\r\nWaiting for MarkerArray to arrive")
			m = rospy.wait_for_message('/occupied_cells_vis_array', MarkerArray, 1)
			if (debug == False):
				determine_obstacle(m, a[0], a[1], a[2], pr)
			else:
				show_all(m)
			#
		except rospy.ROSException:
			#error occured
			print("No message arrived within specified time")
	except rospy.ROSInterruptException:
		pass

