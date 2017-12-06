#!/usr/bin/env python
# -*- coding: utf-8 -*-

# filename : plot_highway.py
# date : 2017-12-06
# author : Yoshiki Kurihara <y-kurihara@ist.osaka-u.ac.jp

#import files
import pyproj
import numpy as np
import cv2
import xml.etree.ElementTree as ET

#DEFINE CONSTANT
GRAPH_SIZE = 4096
INPUT_FILENAME = './data/sumo_result.xml'
origin = [139.562110,35.816635]
max_origin = [139.919635,35.531365]
width = abs(max_origin[0] - origin[0])
height = abs(max_origin[1] - origin[1])
img = np.zeros((GRAPH_SIZE,GRAPH_SIZE,3),np.uint8)
img.fill(255)


params={}
params['+units']='m'
params['+no_defs']=True
params['+ellps']='WGS84'
params['+datum']='WGS84'
params['+proj']='utm'
params['+zone']='54'
proj = pyproj.Proj(projparams=params)
    
#DEFINE METHOD
def convertXY2LonLat(x,y):
    (x_off,y_off) = (-364313.90,-3927702.90)
    x = x - x_off
    y = y - y_off
    return proj(x, y, inverse=True)

def distance2D(lon1,lat1,lon2,lat2):
    g = pyproj.Geod(ellps='WGS84')
    result = g.inv(lon1,lat1,lon2,lat2)
    return result[2]/1000

if __name__ == '__main__' :
    print(distance2D(139.500319,35.485370,139.994436,35.915889))
    highway_tree = ET.parse('../data/highway.xml')
    highway_root = highway_tree.getroot()
    distance = 0
    for child in highway_root:
        if 'shape' in child.attrib:
            PAST_POS=[0,0]
            past_pos=[0,0]
            for pos_pair in child.attrib['shape'].split():
                (x,y) = pos_pair.split(',')
                (lon,lat) = convertXY2LonLat(float(x),float(y))
                coodinate = [int(abs(lat-origin[1])/height * GRAPH_SIZE),
                             int(abs(lon-origin[0])/width * GRAPH_SIZE)]
                if PAST_POS!=[0,0]:
                    line = distance2D(lon,lat,past_pos[0],past_pos[1])
                    cv2.line(img, (PAST_POS[0], PAST_POS[1]),
                             (coodinate[0],coodinate[1]), (255, 0, 0), 3)
                    if distance > 2 :
                        traffic_counter_lat = (lat - past_pos[1]) * (distance - 2)/line + past_pos[1]
                        traffic_counter_lon = (lon - past_pos[0]) * (distance - 2)/line + past_pos[0]
                        cv2.circle(img,
                                   (int(abs(traffic_counter_lat-origin[1])/height * GRAPH_SIZE)
                                    ,int(abs(traffic_counter_lon-origin[0])/width * GRAPH_SIZE))
                                   ,10,(0,0,255),-1)
                        distance = 0
                    distance += line
                PAST_POS = coodinate
                past_pos = (lon,lat)
    cv2.imwrite('img.jpg',img)

                
            
    
