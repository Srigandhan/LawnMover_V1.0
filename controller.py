#!/usr/bin/env python
import rospy ,rospkg
import math
# from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import numpy as np
from std_msgs.msg import Float64 ,String
import ast
from lawn_mower.srv import *

import serial
rospack = rospkg.RosPack()

class controller:

    def __init__(self):
        self.tgt_lat = -1
        self.tgt_long =-1
        self.curr_lat =-1
        self.curr_long =-1

        #Compass
        self.heading = 0.0
        self.DEC_ANGLE = 0.2225

        self.WAYPOINT_DIST_TOLERANE = 0.05 # value in meters
        self.HEADING_TOLERANCE = 50 # value in ?
        self.TWO_PI = 6.2831855
        self.NUMBER_WAYPOINTS = 5 # static value , this describes number to waypoints to be combined for processing
        self.WAYPOINT_LIST = [] # value to be populated from the file 
        self.distanceToTarget = -1
        self.originalDistanceToTarget= -1
        self.targetHeading = -1
        self.currentHeading = -1
        self.turnDirection = "STOP"
        #populate waypoints to list
        self.getWayPoints()
        self.compassInteruppt = False
        self.WAYPOINT_INDEX = -1
        self.usingInteruppt = False
        # subscribe to RTK-GPS topic /vo
        #self.odom_sub = rospy.Subscriber('/vo', Odometry, self.sendCL)
        self.odom_sub = rospy.Subscriber('/fix', NavSatFix, self.sendCL)
        self.sub =  rospy.Subscriber('/compass_angle', Float64, self.getHeading)
        
	#Roatate till it does not have correct heading
        self.rotationFlag = False
        
    
        


    # get current lat , long and orientation
    def sendCL(self,data):
        if self.usingInteruppt : 
            print "******--- Fetching : GPS Data ---****** \n"
            '''
            self.curr_lat = data.pose.pose.position.x
            self.curr_long = data.pose.pose.position.y
            '''
            self.curr_lat = data.latitude
            self.curr_long = data.longitude
            self.usingInteruppt = False
    
    # call service to fetch next waypoint / read through file and store in list
    def getWayPoints(self):
        abs_path = rospack.get_path("lawn_mower") + "/script/ServerRequestHandler/" + "saved_points_sim.txt"
        with open(abs_path,'r') as pts:
	        data = pts.readlines() 
        print data
        for i in data:
            #x,y = i.split(';')
            co = ast.literal_eval(i)
            #self.WAYPOINT_LIST.append([float(x),float(y)])
            self.WAYPOINT_LIST.append([co[0],co[1]])

    def getNextWayPoints(self):
        print '@@ Get Next waypoint is called @@'
        self.WAYPOINT_INDEX = self.WAYPOINT_INDEX + 1

        if (self.WAYPOINT_INDEX >= len(self.WAYPOINT_LIST) ):
            print("controller.py : Info : Processing last waypoint")
            rospy.signal_shutdown("Node Completed")
            return
            # while (1) :
            #     pass

        self.tgt_lat = self.WAYPOINT_LIST[self.WAYPOINT_INDEX][0]
        self.tgt_long= self.WAYPOINT_LIST[self.WAYPOINT_INDEX][1]

        

        self.processGPS() 
        print 'Waiting for GPS Signal and Compass Reading..'
        #while self.usingInteruppt or self.compassInteruppt:
        #    pass
        print ' GPS Signal and Compass Reading Recieved..'
        self.distanceToTarget =  self.distanceToWaypoint()
        self.courseToWaypoint()       


    def processGPS(self):

        self.usingInteruppt = True
        self.compassInteruppt = True
        # update the course and distance to waypoint based on our new position
        # self.distanceToWaypoint()
        # self.courseToWaypoint()    
	
    def distanceToWaypoint(self):

        delta = math.radians(self.curr_long - self.tgt_long)
        sdlong = math.sin(delta)
        cdlong = math.cos(delta)
        lat1 = math.radians(self.curr_lat)
        lat2 = math.radians(self.tgt_lat)
        slat1 = math.sin(lat1)
        clat1 = math.cos(lat1)
        slat2 = math.sin(lat2)
        clat2 = math.cos(lat2)
        delta = (clat1 * slat2) - (slat1 * clat2 * cdlong)
        delta = math.pow(delta,2)
        delta += math.pow(clat2 * sdlong,2)
        delta = math.sqrt(delta) 
        denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
        delta = math.atan2(delta, denom)
        distanceToTarget =  delta * 6372795
        '''
        # check to see if we have reached the current waypoint
        if (distanceToTarget <= self.WAYPOINT_DIST_TOLERANE) :
            print 'distance to target ', distanceToTarget
            self.getNextWayPoints()
     	'''
        return distanceToTarget

    def courseToWaypoint(self):
        
        '''
        dlon = math.radians(self.tgt_long-self.curr_long)
        cLat = math.radians(self.curr_lat)
        
        tLat = math.radians(self.tgt_lat)
        a1 = math.sin(dlon) * math.cos(tLat)
        a2 = math.sin(cLat) * math.cos(tLat) * math.cos(dlon)
        a2 = math.cos(cLat) * math.sin(tLat) - a2
        a2 = math.atan2(a1, a2)
        if (a2 < 0.0) : 
            a2 += self.TWO_PI
        
        self.targetHeading = math.degrees(a2)
        '''
        
        heading = math.atan2(self.tgt_long-self.curr_long,self.tgt_lat-self.curr_lat)
        heading *= (180 / 3.14159)
        self.targetHeading  = (450 - int(heading)) % 360 


# read compass

    def getHeading(self,data):
        if self.compassInteruppt:
            self.heading = data.data
            print "####  controller.py : getHeading : Heading Data ",self.heading 
            self.compassInteruppt = False
        
    def readCompass(self):
        
        
        '''
        self.heading = self.heading+self.DEC_ANGLE

        if self.heading < 0 :
            self.heading = self.heading + 2*math.pi

        if self.heading > 2*math.pi :
            self.heading = self.heading - 2*math.pi
        
        headingDegrees = self.heading * 180/math.pi    
        print "controller.py : readCompass : heading Degree ", headingDegrees
        
        return headingDegrees
        
        '''
        
        if self.heading < 0 :
            self.heading = 360 + self.heading
        
        print "controller.py : readCompass : heading Degree ", self.heading
        
        return self.heading

        

# calculate desired turn
    def calcDesiredTurn(self):
        # calculate where we need to turn to head to destination
        headingError = self.targetHeading - int(self.currentHeading)
        '''
        # adjust for compass wrap
        if (headingError < -180):
            headingError += 360
        
        if (headingError > 180) :
            headingError -= 360
        '''
        print 'Heading Error : ',headingError
        
        # calculate which way to turn to intercept the targetHeading
        if (abs(headingError) <= self.HEADING_TOLERANCE) :   # if within tolerance, don't turn
            self.turnDirection = "TURN_STRAIGHT" 
            self.rotationFlag = False
        elif (headingError < 0) :
            self.turnDirection = "TURN_LEFT"
            self.rotationFlag = True
        elif (headingError > 0) :
            self.turnDirection = "TURN_RIGHT"
            self.rotationFlag = True
        else :
            self.turnDirection = "TURN_STRAIGHT"
            self.rotationFlag = False

# service for sending command to aurdino through rasberry pi

    def client_srv_control_cmd(self,cmd_srv):
        print "----in client srv control cmd ------"
        rospy.wait_for_service('controller_cmd')
        try :
            controller_cmd = rospy.ServiceProxy('controller_cmd',controllerCMD)
            response_= controller_cmd(cmd_srv)
            return response_.cmd_response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


# move 
    def move(self):

        print "-----------in move function-------------"
        dataToSend = "0;S"

        if (self.turnDirection == "TURN_STRAIGHT"):
            dataToSend = "100;F"
        
        if (self.turnDirection == "TURN_LEFT"):
            dataToSend = "50;L"

        if (self.turnDirection == "TURN_RIGHT"):
            dataToSend = "50;R"
        print dataToSend

        
        serv_response = self.client_srv_control_cmd(dataToSend)        
        print 'Got response from service as ' , serv_response
	'''
	try:
            ser = serial.Serial('/dev/ttyUSB0', 9600,timeout =.1)
            ser.write(dataToSend)
	    print 'Waiting for response from AGV'
            while True:
                data = ser.readline().decode("ascii")
                if len(data)>0:
                    break
	    print 'Got response from AGV'

        except Exception as ex :
            print ("controller.py : move : can't connect to aurdino")
    
	'''
# loop for continious flow
def stopMower(data):
    print "Function Called for Stopping Mower with command " , data.data
    rospy.signal_shutdown("lawn mower command STOP being called")


def loop():

    rospy.init_node("controller")
    stopSub = rospy.Subscriber("/stopMower",String,stopMower)
    agv_controller = controller()
    
    while not rospy.is_shutdown() :
       
        agv_controller.getNextWayPoints()
        print 'Distance to waypoint ',agv_controller.distanceToTarget
        print 'Current Latitude',agv_controller.curr_lat
        print 'Current Longitude',agv_controller.curr_long

        print 'Target Latitude',agv_controller.tgt_lat
        print 'Target Longitude',agv_controller.tgt_long
        
        rospy.sleep(2)
        agv_controller.currentHeading = agv_controller.readCompass()
        #rospy.sleep(1)
        print '------ Current Heading : ',agv_controller.currentHeading,' ------------'
        print '------ Target Heading : ',agv_controller.targetHeading,' ------------'
        
        agv_controller.calcDesiredTurn()
	if agv_controller.rotationFlag == True:
		print 'AGV is Turning'
        while agv_controller.rotationFlag :
            agv_controller.move()
            agv_controller.calcDesiredTurn()
            rospy.sleep(1)
	if agv_controller.rotationFlag == False:
		print 'AGV is moving straight'
		dist = agv_controller.distanceToWaypoint()
		while dist > 0.05:
			agv_controller.move()
			rospy.sleep(1)
			dist = agv_controller.distanceToWaypoint()
	
	print 'Moving to Next Waypoint',agv_controller.WAYPOINT_INDEX
	rospy.sleep(1)

# call this function
loop()




