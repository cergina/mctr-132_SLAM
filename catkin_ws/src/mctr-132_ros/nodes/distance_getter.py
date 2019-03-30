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
	print("\n\rrun: 'rosrun mctr-132_ros distance_getter.py -h [a1] [a2] [a3]'\n\r")
	print("-h:		Help")
	print("[a1] 		Vector X (float)")
	print("[a2]		Vector Y (float)")
	print("[a3]		Vector Z (float)")

def not_enough_arguments():
	print("Not enough argument's. Check help: '-h' for the script.")

def incorrect_arguments():
	print("Vector has to be float values and at least 1 of the 3 axies has to be non zero. Check help: '-h' for the script.")

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
		#print("pt: " + str(la_marker.points[x].x) + "/" + str(la_marker.points[x].y) + "/" + str(la_marker.points[x].z) + " VS " + "Vector(" + str(v1) +" "+ str(v2) +" "+ str(v3) +")")
		#print("ratios: " + str(ratio_x) +"/" + str(ratio_y) +"/" + str(ratio_z))
		
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
# MAIN is here
#
if __name__ == '__main__':

	# get information about the number of passed arguments
	arg_count = len(sys.argv)
	
	# does the user want help?
	if (arg_count > 1 and sys.argv[1] == "-h"):
		provide_help()
		sys.exit()
	
	# did user send less arguments that we want?
	if (arg_count < 4):
		not_enough_arguments()
		sys.exit()
	
	# user wants to specify also the precision resolution if arg_count > 4
	def_precision = "0.04"
	pr = None
	#	
	if (arg_count > 4):
		pr = sys.argv[4]
	else:
		pr = def_precision

	# user sent ok number of arguments, but are they ok?
	try:
		a1 = float(sys.argv[1])
		a2 = float(sys.argv[2])
		a3 = float(sys.argv[3])
		pr = float(pr)
	except ValueError:
		incorrect_arguments()
		sys.exit()
	

	# all-zero vector is not supported
	if (a1 == 0 and a2 == 0 and a3 == 0):
		incorrect_arguments()
		sys.exit()
	
	try:
		#important, node initializer
		rospy.init_node('distance_getter', anonymous=True)
		try:
			#
			#MarkerArray
			print("\r\nWaiting for MarkerArray to arrive")
			m = rospy.wait_for_message('/occupied_cells_vis_array', MarkerArray, 1)
			determine_obstacle(m, a1, a2, a3, pr)
			#
		except rospy.ROSException:
			#error occured
			print("No message arrived within specified time")
	except rospy.ROSInterruptException:
		pass

