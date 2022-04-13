#!/usr/bin/env python

'''
   ----------------------------------------------------------------------
   Copyright [2019] [George Georgiades, Anatoli A. Tziola, Savvas G. Loizou]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   ------------------------------------------------------------------------ 


 * ----------------------------------------------------------------------------------------------------
 * Cyprus University of Technology
 * Department of Mechanical Engineering and Materials Science and Engineering
 * ----------------------------------------------------------------------------------------------------
 * Project          : L4MS (Logistics for Manufacturing Systems
 * ----------------------------------------------------------------------------------------------------
 * Author(s)        : 	   George Georgiades,		Anatoli A. Tziola,	   Savvas G. Loizou
 * Contact(s)       : george.georgiades@cut.ac.cy , anatoli.tziola@cut.ac.cy , savvas.loizou@cut.ac.cy
 * ----------------------------------------------------------------------------------------------------
'''


import rospy
import time
import requests
import json
import sys, time
import numpy as np

import os.path
import math

import sensor_msgs.msg
from maptogridmap.msg import *
from std_msgs.msg import *


class ros_Graph2NodesEdges:

    def __init__(self,Stopic1="/S1_topic", Ptopic1="/P1_topic", Ptopic2="/P2_topic"):
        self.VERBOSE = True
        self.Stopic1 = Stopic1
        self.Ptopic1 = Ptopic1
        self.Ptopic2 = Ptopic2


        if self.VERBOSE: print ("ros_Graph2NodesEdges:Subscribing 2 Topic %s " % (self.Stopic1))
        if self.VERBOSE: print ("ros_Graph2NodesEdges:Publishing  2 Topic1 %s " % (self.Ptopic1))
        if self.VERBOSE: print ("ros_Graph2NodesEdges:Publishing  2 Topic2 %s " % (self.Ptopic2))


        '''Initialize ros publisher, ros subscriber'''
        # subscribed Topic
        self.subscriber_St1 = rospy.Subscriber(self.Stopic1, Graph, self.callback_St1, queue_size=1)

        # topic where we publish
        self.nodes_pub1 = rospy.Publisher(self.Ptopic1, Nodes, queue_size=1)
        self.edges_pub2 = rospy.Publisher(self.Ptopic2, Edges, queue_size=1)


    def callback_St1(self, ros_data):
	msg_nodes = Nodes()
        msg_edges = Edges()

	for i in range(0, len(ros_data.vertices)):
		msg_nodes.x.append(ros_data.vertices[i].x)
		msg_nodes.y.append(ros_data.vertices[i].y)
		msg_nodes.theta.append(ros_data.vertices[i].theta)
		msg_nodes.name.append(ros_data.vertices[i].name)
		msg_nodes.uuid.append(ros_data.vertices[i].uuid)

	for i in range(0, len(ros_data.edges)):
		msg_edges.uuid_src.append(ros_data.edges[i].uuid_src)
		msg_edges.uuid_dest.append(ros_data.edges[i].uuid_dest)
		msg_edges.name.append(ros_data.edges[i].name)
		msg_edges.uuid.append(ros_data.edges[i].uuid)		

        self.nodes_pub1.publish(msg_nodes)
        self.edges_pub2.publish(msg_edges)


if __name__ == '__main__':

    rospy.init_node('Graph2NodesEdges', anonymous=True)

    myG2NE = ros_Graph2NodesEdges(Stopic1="/map/graph", Ptopic1="/map/nodes", Ptopic2="/map/edges")

    rospy.spin()

 
