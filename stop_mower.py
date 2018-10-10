#!/usr/bin/env python

#*************************************************************************#
#   Description :                                                         #
#                 Script for sending command to stop the lawn mower       #
#                                                                         #
#   Parameters :                                                          #
#                                                                         #
#                                                                         #
#**************************V - 1.0****************************************#
#   Date : 08-Oct-2018   By: Gautam                                       #
#   Initial draft of the script                                           #
#                                                                         #
#*************************************************************************#



import rospy ,rospkg
from std_msgs.msg import Float64 ,String

def stop_mower():
    pub = rospy.Publisher('stopMower', String, queue_size=1)
    
    rospy.init_node('stopMower') 
    
    pub.publish("stop")
    
    
    
       
if __name__ == '__main__':
    try:
        # initialize ros node
        
        stop_mower()
    except rospy.ROSInterruptException:
	    rospy.loginfo("stop_mower.py : ERROR while publishing stop command ")
    pass



