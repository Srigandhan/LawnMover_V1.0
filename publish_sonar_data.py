#!/usr/bin/env python

#*************************************************************************#
#   Description :                                                         #
#                 Script for collecting data from sonar sensor            #
#                 and publishing it over a topic for it to be consumed    #
#   Parameters :                                                          #
#                < port Name >                                            #
#                                                                         #
#**************************V - 1.0****************************************#
#   Date : 08-Oct-2018   By: Gautam  & Atul                               #
#   Initial draft of the script                                           #
#                                                                         #
#*************************************************************************#


import rospy
from std_msgs.msg import String
import serial


def fetch_and_publish(portname):
    # Adding constants

    min = 828 
    max = 37630 # value given by sensor vendor itself in inches
    
    #converting it into cms along with getting sonar constant
    sonar_const = ( 254 * 2.54 ) / ( max - min ) 
    
       
    pub = rospy.Publisher('sonarData_1', String, queue_size=5)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        ser = serial.Serial(port=portname,baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
        data = ser.read_until(" ")
        measurement = str((float(data) - min) * sonar_const +17) # added  offset of 17 , which is derived from hit and trail method
        pub.publish(measurement)
        rate.sleep()

if __name__ == '__main__':
    try:
        # initialize ros node
        rospy.init_node('sonarDataPublishNode') 
        portname= rospy.get_param('~portname', '/dev/ttyACM0')
        fetch_and_publish(portname)
    except rospy.ROSInterruptException:
	    rospy.loginfo("publish_sonar_data.py : ERROR while publishing sonar data ")
    pass
