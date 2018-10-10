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



import json
import pipeline
from flask import Flask, jsonify, make_response
from flask import request
import subprocess
import os
import datetime


app = Flask(__name__)

#  as input and calls model pipeline to process
@app.route('/control/recordPoints', methods = ['POST'])
def analyze_aisle_image():
   
   ais_audio_file = request.form.get('data', type=str)
   data = ais_audio_file.strip('')
   coordinate = data.split('#')
   edge_points = []
   edge=[]
   for i in coordinate:
     if i == '':
       continue
     point=[]
     k = i.strip()
     print 'k',k.split(',')
     for j in k.split(','):
       point.append(float(j))
     edge.append(point)
   edge_points.append(edge)
   
   print 'Edge ',edge
   print edge_points
   pipeline_obj = pipeline.gloabl_path_coverage()
   point = pipeline_obj.fetch_waypoints(edge_points)
   print 'Done'
   with open("saved_waypoints.txt", "wb") as f:
     for i in point:  
      f.write(str(i))
      f.write('\n')
     f.close()

   #print 'Edgespoint' , edge_points
   #Process the boundary value
   # Use the boundary value to create new list of covering points
   # load the list of all point from file and store in coordinate to plot it on front end
  #  ret_str = []
  #  ret_str[0] = len(point)
  #  ret_len = 10000 if len(point)>10000 else len(point)
  #  ret_str[1] = point[:ret_len]
   resp = make_response(json.dumps(str(point))) #replace data with coordinates
   resp.headers['Access-Control-Allow-Origin'] = '*'
   return resp

@app.route('/control/startMower', methods = ['POST'])
def start_mower():
  try:
    #log_file = "/home/agv/poc_v_1/src/lawn_mower/logs/startMower_" +  datetime.datetime.now().strftime("%d_%b_%Y_%H_%M_%S") + ".log" 
    os.system("roslaunch lawn_mower outdoor.launch  &")
    
    print "pointsCollector.py : Info :  " , datetime.datetime.now() , " : startMower : command roslaunch lawn_mower outdoor.launch executed " 
    resp = make_response("Command Executed") 
    resp.headers['Access-Control-Allow-Origin'] = '*'
    return resp

  except :
    print "pointsCollector.py : Error :  " , datetime.datetime.now() , " : startMower : command roslaunch lawn_mower outdoor.launch failed " 

@app.route('/control/stopMower', methods = ['POST'])
def stop_mower():

  os.system("roslaunch lawn_mower stop_mower.launch &")
  
  print "pointsCollector.py : Info :  " , datetime.datetime.now() , " : stopMower : command roslaunch lawn_mower stop_mower.launch executed " 
  resp = make_response("Command Executed") 
  resp.headers['Access-Control-Allow-Origin'] = '*'
  return resp


if __name__ == '__main__':
  app.run(host='0.0.0.0')

