#*************************************************************************#
#   Description :                                                         #
#                 Script for creating waypoints which covers whole area   #
#                 in optimal manner for given parameter                   #
#   Parameters :                                                          #
#                < List of GPS points covering Parameters >               #
#                                                                         #
#**************************V - 1.0****************************************#
#   Date : 20-Sep-2018   By: Gautam  & Atul                               #
#   Initial draft of the script                                           #
#                                                                         #
#**************************V - 1.1****************************************#
#   Date : 4-Oct-2018   By: Gautam                                        #
#   changed code under function 'prepare_edges' and 'move_next'           #
#                                                                         #
#*************************************************************************#

import numpy as np
from scipy.interpolate import splprep, splev
from shapely.geometry import Polygon, Point
import pandas as pd
import datetime

class gloabl_path_coverage:
    # Parameters
    # sep : separator value , default value is 0.00001 .

    def __init__(self,sep=0.00001):

        self.longitude_upper_limit = 1 * sep
        self.latitude_upper_limit = 1 * sep
        self.limit = 1 * sep
        self.moves=[[0.,-self.limit , 'S'],[-self.limit,0. , 'W'],[0.,self.limit ,'N'],[self.limit,0. ,'E']] 

        self.points_to_follow = [] # stores the final points in orderly fashion to be followed by rower.
        self.direction = 'S' 
        self.count = 0

        self.list_all_polygons = [] # contains list of all polygons created from boundaries provided. This may include obstacles if any.
        self.list_waypoints = [] # contains all points which needs to be covered by rower , excluding obstacles if any.

        self.minx=self.miny= self.maxx=self.maxy =  self.longs =  self.lats = 0.0000

        self.current_position = None
        self.traversed_list = []

        self.thresholdPoints = 1
        '''
        self.file_name = 'obs' # not required in v 1.1       
        self.coordinates = self.load_data() # not required in v 1.1
        self.list_of_boundaries = self.list_boundaries(self.coordinates) # not required in v 1.1 
        self.list_all_polygons = self.list_all_polygons() # not required in v 1.1
        self.minx, self.miny, self.maxx, self.maxy = self.list_all_polygons[0].bounds # not required in v 1.1
        self.longs = np.arange(self.minx, self.maxx+self.longitude_upper_limit, self.limit) # not required in v 1.1
        self.lats =  np.arange(self.miny, self.maxy+self.latitude_upper_limit, self.limit) # not required in v 1.1
        self.edges = self.calculate_boundary_points(self.list_of_boundaries,self.list_all_polygons,self.longs,self.lats) # not required in v 1.1
        self.current_position = self.edges[0][:] # not required in v 1.1
        self.current_position.extend('S') # not required in v 1.1
        self.final_list_waypoint = []
        self.calculate_waypoints() # not required in v 1.1
        self.save_points() # not required in v 1.1
        '''

    '''
        Funtction which accepts GPS points for parameters and sets variable list_all_polygons

        Input : <list of ParameterPoints >

    '''       
    def set_polygons(self,listBoundaries):
                
        #temp for calculation purpose
        boundary = []

        # loop to separate points based on boundary from recieved points and store it into list of boundaries
        
        for objects in listBoundaries:
            for points in objects:
                boundary.append(points)

            points =np.array(boundary)
            self.list_all_polygons.append(Polygon(points))   

        print "pipeline.py : Info :  " , datetime.datetime.now() , " : set_polygons : polygon created for all objects , count is " , len(self.list_all_polygons)   
                 
    '''
        Function which calculates all points which need to be covered by rower in 
        lawn excluding pre-defined obstacles if any.
        This function sets value for variable list_waypoints
   
    '''
    def set_waypoints(self):

        #boundary list for lawn is a position 0.
        self.minx, self.miny, self.maxx, self.maxy = self.list_all_polygons[0].bounds 
        
        self.longs = np.arange(self.minx, self.maxx+self.longitude_upper_limit, self.limit) 
        self.lats =  np.arange(self.miny, self.maxy+self.latitude_upper_limit, self.limit) 
        flag_poly= False

        for x in range(0,len(self.longs)):
            for y in range(0,len(self.lats)):
                point = Point([self.longs[x],self.lats[y]])
                if self.list_all_polygons[0].contains(point) or self.list_all_polygons[0].boundary.contains(point):
                    if( len(self.list_all_polygons) > 1):
                        flag_poly = True
                        for obstacle_poly in self.list_all_polygons[1:]:
                            if obstacle_poly.contains(point):
                                flag_poly = False
                        if flag_poly:
                            self.list_waypoints.append([self.longs[x],self.lats[y]])

                    else:
                        self.list_waypoints.append([self.longs[x],self.lats[y]])

        print "pipeline.py : Info :  " , datetime.datetime.now() , " : set_waypoints : waypoints generated , count is " , len(self.list_waypoints) 


    '''
        Function Description
   
    '''     
    def set_pointToFollow(self,point):

        if len(self.points_to_follow)<1:
            self.points_to_follow.append(point[:2])

        if point[2] == self.direction:
            if self.count > self.thresholdPoints:
                self.count = 0
                self.points_to_follow.append(point[:2])
            else:
                self.count = self.count+1
        else:
            self.points_to_follow.append(point[:2])
            self.direction = point[2]
            self.count = 0



    '''
        Function Description
   
    '''            
    def nextMove(self):

        for move in self.moves:
            next_x = self.current_position[0] + move[0]
            next_y = self.current_position[1] + move[1]
            theta = move[2]
            
            point = Point(next_x , next_y)

            if [next_x , next_y] not in self.traversed_list:
                
                if self.list_all_polygons[0].contains(point) or self.list_all_polygons[0].boundary.contains(point):
                    
                    if( len(self.list_all_polygons) > 1):
                        flag_poly = True
                        for obstacle_poly in self.list_all_polygons[1:]:
                            if obstacle_poly.contains(point):
                                flag_poly = False
                        
                        if flag_poly:
                            return [next_x,next_y ,theta]
                    else:
                        return [next_x,next_y ,theta]
        return None
            

    '''
        Funtction which accepts GPS points for parameters and returns set of GPS points 
        to be followed by rower

        Input : <list of ParameterPoints >

        Return : <list of waypoints >
    '''
    def fetch_waypoints(self,listParameterPoints):

        self.set_polygons(listParameterPoints)

        self.set_waypoints()

        while len(self.list_waypoints)!=0 :
    
            if self.current_position == None:
                self.current_position = self.list_waypoints[0][:]
                self.current_position.extend('S')
                
            point =  self.current_position[:2]
            
            self.list_waypoints.remove(point)
            
            self.traversed_list.append(self.current_position[:2])
            
            self.set_pointToFollow(self.current_position)
            
            self.current_position = self.nextMove()

        print "pipeline.py : Info :  " , datetime.datetime.now() , " : fetch_waypoints : waypoints to follow generated , count is " , len(self.points_to_follow)
        
        return self.points_to_follow


    # not required in v1.1
    '''
    def save_points(self):
        with open('points_to_follow','w') as f:
            for data in self.points_to_follow:
                f.write(str(data[0])+';'+str(data[1]))
                f.write('\n')

    def list_all_polygons(self):
    
        # will convert points under list_of_boundaries into indiv polygon and store into list_all_polygons.

        ## its one time process
    
        list_all_polygons =[]

        for boundary in self.list_of_boundaries:
            points =np.array(boundary)
            list_all_polygons.append(Polygon(points))
            
        return list_all_polygons
       
    def load_data(self):
        # lawn_mower DATA
        df = pd.read_csv(self.file_name, sep=" ", header=None)
        #lsit of points in format [Latitude,Longitude] 
        coordinates  = df.apply(pd.to_numeric, args=('coerce',)).values
        
        return coordinates
    
    def calculate_boundary_points(self,list_of_boundaries,list_all_polygons,longs,lats):
        edges=[]
        for x in range(0,len(longs)):
            for y in range(0,len(lats)):
                point = Point([longs[x],lats[y]])
                if list_all_polygons[0].contains(point) or np.any(np.equal([longs[x],lats[y]] , list_of_boundaries[0]).all(axis=1)):
                    if( len(list_all_polygons) > 1):
                        flag_poly = True
                        for obstacle_poly in list_all_polygons[1:]:
                            if obstacle_poly.contains(point):
                                flag_poly = False
                        if flag_poly:
                            edges.append([longs[x],lats[y]])

                    else:
                        edges.append([longs[x],lats[y]])
        return edges
    
    def move_next(self,current_position):
        for i in self.move:
            next_x = current_position[0] + i[0]
            next_y = current_position[1] + i[1]
            theta = i[2]
            point = Point(next_x , next_y)
            if [next_x , next_y] not in self.traversed_list:
                if self.list_all_polygons[0].contains(point) or np.any(np.equal([next_x , next_y] , self.coordinates).all(axis=1)):
                    if( len(self.list_all_polygons) > 1):
                        flag_poly = True
                        for obstacle_poly in self.list_all_polygons[1:]:
                            if obstacle_poly.contains(point):
                                flag_poly = False
                        if flag_poly:
                            return [next_x,next_y ,theta]
                    else:
                        return [next_x,next_y ,theta]
        return None    
    '''

# a = gloabl_path_coverage()
