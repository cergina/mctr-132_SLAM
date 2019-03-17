#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    sub = 
    pub = rospy.Publisher('noise_out', String, queue_size=1)
    rospy.init_node('noise_out', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        
        hello_str = "publishing"
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
