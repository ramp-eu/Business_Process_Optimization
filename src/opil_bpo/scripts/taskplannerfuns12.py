#! /usr/bin/env python

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


import sys
import rospy
from copy import copy, deepcopy
import os
import json, ast
from collections import OrderedDict

from opil_bpo.srv import *
from opil_bpo.msg import *
import maptogridmap.msg as m2g
import yaml
import numpy as np

from create_states_fun import *
from adjust_new_constraints import *
from Dijkstra import *

import requests


rb_spd = 0.25
ldt = 0.03
ult = 0.02


class ClassLettersofWord:
    def __init__(self, str):
        self.RobotPosition = str[0]
        self.ItemPosition = str[1]
        self.HumanPosition = str[2]

    def LettersofWord():
        return self

    '''
    state = 'ARB'
    temp = ClassLettersofWord(state)
    print(temp.RobotPosition, temp.ItemPosition, temp.HumanPosition) 
    '''


def get_dicval(data, *args):
    if args and data:
        element  = args[0]
        if element:
            value = data.get(element)
            return value if len(args) == 1 else get_dicval(value, *args[1:])


def printmat(A):
    print(np.matrix(A))


def request_cost_client(St, Go):
    VERBOSE=False
    rospy.wait_for_service('/pathdist')
    try:
        request_cost = rospy.ServiceProxy('/pathdist', shpath)
        if VERBOSE: print ("request_cost_client: Service call to >pathdist<  Strt-Goal= \n%s   %s")%(St,Go)
        pres = request_cost(St,Go)
        if VERBOSE: print ("request_cost_client: Service call to >pathdist<  Returned path cost= %f")%(pres.path_totalcost)
        return pres.path_totalcost
    #except (rospy.ServiceException, rospy.ROSException), co:
    except rospy.ServiceException, co:
        if VERBOSE: print("Service call  -pathdist failed: %s")%(co,)


def OneCharChangeChecking(str1, str2, n, ch, mydict):
    cnt = 0
    VERBOSE=False
    if (len(str1)!= n or len(str2)!=n):
        print('Length of states ', str1, ' and ', str2, 'is not the same')
        return -1
    else:
        for i in range(0, n):
            if (str1[i] != str2[i]):
                cnt += 1

    if (ch == cnt):
        #return cnt, 100.
        if VERBOSE: print("OneCharChangeChecking: str1-str2 = %s - %s")%(str1,str2)
        if VERBOSE: print("OneCharChangeChecking: str1[0]-str2[0] = %s - %s")%(str1[0],str2[0])
        if VERBOSE: print("OneCharChangeChecking: uuid1-uuid2 = %s - %s")%(get_dicval(mydict,str1[0],'uuid'),get_dicval(mydict,str2[0],'uuid'))
        if str1[0]==str2[0]:
            s=0.001
        else:
            if VERBOSE: print('GET VALUE =     ', get_dicval(mydict,str1[0],'name'), get_dicval(mydict,str2[0],'name'))
            s = request_cost_client(get_dicval(mydict,str1[0],'uuid'), get_dicval(mydict,str2[0],'uuid'))
            if VERBOSE: print("OneCharChangeChecking: cost calculated s=",s)
        return cnt, s
    else:
        return cnt, 0.


def OneStepChange(LX, n, ch, mydict):
    VERBOSE=False
    matr = [[0. for x in range(0, len(LX))] for y in range(0, len(LX))]
    for i in range(0, len(LX)):
        for j in range(0, len(LX)):
            if VERBOSE: print("OneStepChange:: Lxi=", LX[i], "    LXj=",LX[j])
            cnt, matr[i][j] = OneCharChangeChecking(LX[i], LX[j], n, ch, mydict)
            if VERBOSE: print("OneStepChange:: Lxi=", LX[i], "    LXj=",LX[j], "   cnt=",cnt,"   COST=",matr[i][j])
    return cnt, matr



def FiveLettersRChange(str1, str2, cnt, ch):
    str1 = ClassLettersofWord(str1)
    str2 = ClassLettersofWord(str2)
    mtrcell = 0

    if ((cnt == ch) and (str2.RobotPosition == str2.ItemPosition == str2.HumanPosition)):
        mtrcell = 1.
    else:
        mtrcell = 0.
    return mtrcell


def Zero2RChanges(LX, n, m1, mydict):
    mtr = deepcopy(m1)
    ch = 1
    cnt = 0
    
    for i in range(0, len(LX)):
        for j in range(0, len(LX)):
            LXi = ClassLettersofWord(LX[i])
            LXj = ClassLettersofWord(LX[j])

            if (LXi.ItemPosition == 'R' and LXj.ItemPosition != 'R'):
                cnt, mtr[i][j] = OneCharChangeChecking(LX[i], LX[j], n, ch,mydict)
                mtr[i][j] = FiveLettersRChange(LX[i], LX[j], cnt, ch)

            if (LXj.ItemPosition == 'R' and LXi.ItemPosition != 'R'):
                cnt, mtr[i][j] = OneCharChangeChecking(LX[j], LX[i], n, ch,mydict)
                mtr[i][j] = FiveLettersRChange(LX[j], LX[i], cnt, ch)
    return cnt, mtr



def AdjacencyMatrix(m1, m2):
    m3 = [[0. for x in range(0, len(m1))] for y in range(0, len(m1))]

    for i in range(len(m1)):
        for j in range(len(m1)):
            if (m1[i][j] == 0 or m2[i][j] == 0):
                m3[i][j] = 0
            else:
                m3[i][j] = min(m1[i][j], m2[i][j])
    return m3


#not used
def finalmatrix(m1, m2, poscon):
    mat = deepcopy(m1)

    for pc in poscon:
        for j in range(len(m2[0])):
            if (pc==j):
                for i in range(len(m2)):
                    mat[i][j]=m2[i][j]
    return mat


class AoIClass:
    def __init__(self,stopic, ANm=None, ALt=None):
        self.data = None
        self.sub = None
        self.stopic=stopic
        self.VERBOSE=False
        self.gridmap_locations=dict()
        self.vd_char=65
        self.ANm=ANm
        self.ALt=ALt

    def get_data_callback(self, msg): #NOT USED
        if self.VERBOSE: print("INSIDE Callback of  AoIClassNode")
        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)

        for i in range(len(msg.name)):
            key_letter=chr(self.vd_char)
            self.gridmap_locations[key_letter]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            self.vd_char += 1
        if self.VERBOSE: print("AoIClass: AoICallback gridmap_locations",self.gridmap_locations)
        self.data = msg
        #print self.data
        if self.VERBOSE: print("UNRegistering from topic >%s<")%( self.stopic)
        ###self.sub.unregister()

    def run(self): #NOT USED uses callback above

        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Registering to topic >%s<")%( self.stopic)
        self.sub = rospy.Subscriber(self.stopic, m2g.Nodes, self.get_data_callback)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)
        rospy.spin()

    def get_data_once(self): #msg = rospy.wait_for_message("my_topic", MyType)
        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)
        if self.VERBOSE: print("INSIDE get_data_once of  AoIClassNode")
        msg=rospy.wait_for_message(self.stopic, m2g.Nodes)

        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)
        if self.VERBOSE: print("Aoi YAML ANm=",self.ANm)
        if self.VERBOSE: print("Aoi YAML ALt=",self.ALt)

        for i in range(len(msg.name)):
            #key_letter=chr(self.vd_char)
            nmindx=self.ANm.index(msg.name[i])
            self.gridmap_locations[self.ALt[nmindx]]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            #self.vd_char += 1
        if self.VERBOSE: print("AoIClass: get_data_once gridmap_locations",self.gridmap_locations)

        self.data = msg # we have the whole message here

    def get_data_once2(self): #msg = rospy.wait_for_message("my_topic", MyType)
        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)

        if self.VERBOSE: print("INSIDE get_data_once of  AoIClassNode")
        msg=rospy.wait_for_message(self.stopic, m2g.Nodes)

        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)

        for i in range(len(msg.name)):
            key_letter=chr(self.vd_char)
            self.gridmap_locations[key_letter]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            self.vd_char += 1
        if self.VERBOSE: print("AoIClass: get_data_once gridmap_locations",self.gridmap_locations)

        self.data = msg # we have the whole message here

    def get_dict(self):
        return self.gridmap_locations

class AoIClassN:
    def __init__(self,stopic, ANm=None, ALt=None):
        self.data = None
        self.sub = None
        self.stopic=stopic
        self.VERBOSE=True
        self.gridmap_locations=dict()
        self.vd_char=65
        self.ANm=ANm
        self.ALt=ALt

    def get_data_callback(self, msg): #NOT USED
        if self.VERBOSE: print("INSIDE Callback of  AoIClassNode")
        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)

        for i in range(len(msg.name)):
            key_letter=chr(self.vd_char)
            self.gridmap_locations[key_letter]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            self.vd_char += 1
        if self.VERBOSE: print("AoIClass: AoICallback gridmap_locations",self.gridmap_locations)
        self.data = msg
        #print self.data
        if self.VERBOSE: print("UNRegistering from topic >%s<")%( self.stopic)
        ###self.sub.unregister()

    def run(self): #NOT USED uses callback above

        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Registering to topic >%s<")%( self.stopic)
        self.sub = rospy.Subscriber(self.stopic, m2g.Nodes, self.get_data_callback)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)
        rospy.spin()

    def get_data_once(self): #msg = rospy.wait_for_message("my_topic", MyType)
        if self.VERBOSE: print("Running AoIClassNode")
        #rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)
        if self.VERBOSE: print("INSIDE get_data_once of  AoIClassNode")
        msg=rospy.wait_for_message(self.stopic, m2g.Nodes)

        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)
        if self.VERBOSE: print("Aoi YAML ANm=",self.ANm)
        if self.VERBOSE: print("Aoi YAML ALt=",self.ALt)

        for i in range(len(msg.name)):
            #key_letter=chr(self.vd_char)
            nmindx=self.ANm.index(msg.name[i])
            self.gridmap_locations[self.ALt[nmindx]]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            #self.vd_char += 1
        if self.VERBOSE: print("AoIClass: get_data_once gridmap_locations",self.gridmap_locations)

        self.data = msg # we have the whole message here

    def get_data_once2(self): #msg = rospy.wait_for_message("my_topic", MyType)
        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)

        if self.VERBOSE: print("INSIDE get_data_once of  AoIClassNode")
        msg=rospy.wait_for_message(self.stopic, m2g.Nodes)

        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)

        for i in range(len(msg.name)):
            key_letter=chr(self.vd_char)
            self.gridmap_locations[key_letter]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            self.vd_char += 1
        if self.VERBOSE: print("AoIClass: get_data_once gridmap_locations",self.gridmap_locations)

        self.data = msg # we have the whole message here

    def get_dict(self):
        return self.gridmap_locations



class AoIClass3:
    def __init__(self,stopic, ANm=None, ALt=None):
        self.data = None
        self.sub = None
        self.stopic=stopic
        self.VERBOSE=False
        self.gridmap_locations=dict()
        self.vd_char=65
        self.ANm=ANm
        self.ALt=ALt

    def get_data_callback(self, msg): #NOT USED
        if self.VERBOSE: print("INSIDE Callback of  AoIClassNode")
        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)

        for i in range(len(msg.name)):
            key_letter=chr(self.vd_char)
            self.gridmap_locations[key_letter]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            self.vd_char += 1
        if self.VERBOSE: print("AoIClass: AoICallback gridmap_locations",self.gridmap_locations)
        self.data = msg
        #print self.data
        if self.VERBOSE: print("UNRegistering from topic >%s<")%( self.stopic)
        ###self.sub.unregister()

    def run(self): #NOT USED uses callback above

        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Registering to topic >%s<")%( self.stopic)
        self.sub = rospy.Subscriber(self.stopic, Nodes, self.get_data_callback)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)
        rospy.spin()

    def get_data_once(self): #msg = rospy.wait_for_message("my_topic", MyType)
        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)
        if self.VERBOSE: print("INSIDE get_data_once of  AoIClassNode")
        msg=rospy.wait_for_message(self.stopic, Nodes)

        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)
        if self.VERBOSE: print("Aoi YAML ANm=",self.ANm)
        if self.VERBOSE: print("Aoi YAML ALt=",self.ALt)

        for i in range(len(msg.name)):
            #key_letter=chr(self.vd_char)
            nmindx=self.ANm.index(msg.name[i])
            self.gridmap_locations[self.ALt[nmindx]]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            #self.vd_char += 1
        if self.VERBOSE: print("AoIClass: get_data_once gridmap_locations",self.gridmap_locations)

        self.data = msg # we have the whole message here

    def get_data_once2(self): #msg = rospy.wait_for_message("my_topic", MyType)
        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)

        if self.VERBOSE: print("INSIDE get_data_once of  AoIClassNode")
        msg=rospy.wait_for_message(self.stopic, Nodes)

        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)

        for i in range(len(msg.name)):
            key_letter=chr(self.vd_char)
            self.gridmap_locations[key_letter]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            self.vd_char += 1
        if self.VERBOSE: print("AoIClass: get_data_once gridmap_locations",self.gridmap_locations)

        self.data = msg # we have the whole message here

    def get_dict(self):
        return self.gridmap_locations

class AoIClass2:
    def __init__(self,stopic):
        self.data = None
        self.sub = None
        self.stopic=stopic
        self.VERBOSE=False
        self.gridmap_locations=dict()
        self.vd_char=65

    def get_data_callback(self, msg): #NOT USED
        if self.VERBOSE: print("INSIDE Callback of  AoIClassNode")
        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)

        for i in range(len(msg.name)):
            key_letter=chr(self.vd_char)
            self.gridmap_locations[key_letter]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            self.vd_char += 1
        if self.VERBOSE: print("AoIClass: AoICallback gridmap_locations",self.gridmap_locations)
        self.data = msg
        #print self.data
        if self.VERBOSE: print("UNRegistering from topic >%s<")%( self.stopic)
        ###self.sub.unregister()

    def run(self): #NOT USED uses callback above

        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Registering to topic >%s<")%( self.stopic)
        self.sub = rospy.Subscriber(self.stopic, Nodes, self.get_data_callback)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)
        rospy.spin()

    def get_data_once(self): #msg = rospy.wait_for_message("my_topic", MyType)
        if self.VERBOSE: print("Running AoIClassNode")
        rospy.init_node('myAoIClassNode', anonymous=True)

        if self.VERBOSE: print("Spinning topic >%s<")%( self.stopic)

        if self.VERBOSE: print("INSIDE get_data_once of  AoIClassNode")
        msg=rospy.wait_for_message(self.stopic, Nodes)

        #create a dictionary to store information received from /map/areas_of_interests topic from parcer.cpp
        self.gridmap_locations={}

        if self.VERBOSE: print("Aoi message=",msg)
        if self.VERBOSE: print("Aoi message.names=",msg.name)

        for i in range(len(msg.name)):
            key_letter=chr(self.vd_char)
            self.gridmap_locations[key_letter]={'name': msg.name[i], 'uuid': msg.uuid[i], 'x': msg.x[i], 'y':msg.y[i], 'z':msg.theta[i]}
            self.vd_char += 1
        if self.VERBOSE: print("AoIClass: get_data_once gridmap_locations",self.gridmap_locations)

        self.data = msg # we have the whole message here

    def get_dict(self):
        return self.gridmap_locations


def var2file(mvar,prc,mfile,wrt,ft):
    VERBOSE=False
    # e.g var2file(mydict,"%s" "myfilepath.yaml",'w+','yaml) #r, r+, w, w+, a, a+
    if ft=='yaml':
        with open(mfile, wrt) as file:
            documents = yaml.dump(mvar, file)

    if ft=='mat2':
        if VERBOSE: print("Writing >mat2< file:%s" % mfile)
        f=open(mfile,wrt)
        f.write("MAT2D=\n")
        f.write("[ \t")
        if VERBOSE: print("Writing >mat2< of length %d\n"% len(mvar))
        for i in range(len(mvar)):
            #if VERBOSE: print("\n Writing >mat2[%d]< = "% i)
            if VERBOSE: print("\n Writing >mat2[%d]< of length =%d "% (i,len(mvar[i])))
            if VERBOSE: print( mvar[i])
            #f.write("[ \t")
            for j in range(len(mvar[i])):
                if VERBOSE: print(mvar[i][j])
                    #f.write("This is line %d\r\n" % (i+1))
                f.write(prc % mvar[i][j])
                f.write("  ")
            if  i==(len(mvar)-1): f.write(" ]\n")
            else: f.write("\n\t")
        f.close()


def var2file2(mvar,prc,mfile,wrt,ft):
    VERBOSE=False
    # e.g var2file(mydict,"%s" "myfilepath.yaml",'w+','yaml) #r, r+, w, w+, a, a+
    if ft=='yaml':
        with open(mfile, wrt) as file:
            documents = yaml.dump(mvar, file)

    if ft=='mat2':
        if VERBOSE: print("Writing >mat2< file:%s" % mfile)
        f=open(mfile,wrt)
        f.write("MAT2D=\n")
        f.write("[ \t")
        if VERBOSE: print("Writing >mat2< of length %d\n"% len(mvar))
        for i in range(len(mvar)):
            #if VERBOSE: print("\n Writing >mat2[%d]< = "% i)
            if VERBOSE: print("\n Writing >mat2[%d]< of length =%d "% (i,len(mvar[i])))
            if VERBOSE: print( mvar[i])
            #f.write("[ \t")
            for j in range(len(mvar[i])):
                if VERBOSE: print(mvar[i][j])
                    #f.write("This is line %d\r\n" % (i+1))
                f.write(prc % mvar[i][j])
                f.write("  ")
            if  i==(len(mvar)-1): f.write(" ]\n")
            else: f.write("\n\t")
        f.close()


    if ft=='nmatxcont':
        if VERBOSE: print("Writing >mat2< file:%s" % mfile)
        #f=open(mfile,wrt)
        #f.write("MAT2D=\n")
        #f.write("[ \t")
        if VERBOSE: print("Writing >mat2< of length %d\n"% len(mvar))
        nmvar=np.matrix(mvar)
        nshp=nmvar.shape
        ndims=len(nshp)
        np.savetxt(mfile, nmvar, delimiter=' ', fmt=prc)
        #np.savetxt(mfile, nmvar)

    if ft=='nmatx':
        nmvar=np.matrix(mvar)
        nshp=nmvar.shape
        ndims=len(nshp)
        if VERBOSE: print("Writing >nmatx< file:%s" % mfile)
        if VERBOSE: print("Writing >nmatx< of shape=",nshp,"   Dims=",ndims)
        #np.savetxt(mfile, nmvar, delimiter='\t', fmt=prc)
        #np.savetxt(mfile, nmvar, delimiter='\t')
        np.savetxt(mfile, nmvar, delimiter=' ', fmt=prc)
        #np.savetxt(mfile, nmvar)

def nchnges(str1,str2,ch=1,n=3):
    cnt = 0
    opval=False

    VERBOSE=False
    if (len(str1)!= n or len(str2)!=n):
        print('Length of states ', str1, ' and ', str2, 'is not the same')
        return -1
    else:
        for i in range(0, n):
            if (str1[i] != str2[i]):
                cnt += 1
    if cnt==ch: opval=True
    return opval

def find_combinations(strin,LX, L1, L2, L3):
    VERBOSE=False
    st_op=list()
    st_cmb=list()
    ll1=list()
    ll2=list()
    ll3=list()
    if VERBOSE: print("find_combinations: strin=",strin)
    #nposns=len(strin)
    if strin[0]=='*': ll1=L1
    else: ll1.append(strin[0])
    if strin[1]=='*': ll2=L2
    else: ll2.append(strin[1])
    if strin[2]=='*': ll3=L3
    else: ll3.append(strin[2])
    if VERBOSE: print("find_combinations: st_cmb=",st_cmb)
    if VERBOSE: print("find_combinations: st_op=",st_op)
    if VERBOSE: print("find_combinations: ll1=",ll1)
    if VERBOSE: print("find_combinations: ll2=",ll2)
    if VERBOSE: print("find_combinations: ll3=",ll3)

    tot_len=len(ll1) * len(ll2) *len(ll3)
    if VERBOSE: print("find_combinations: tot_len=",tot_len)
    #for ww in range(0, tot_len):
    #mstr=""
    for i in ll1:
        for j in ll2:
            for k in ll3:
                #mstr = ""
                mstr=str(i)+str(j)+str(k)
                st_cmb.append(mstr)
    if VERBOSE: print("find_combinations: st_cmb=", st_cmb)
    for wlx in LX:
        # Filter for the combinations existing in STATE LX
        s=''
        for stc in st_cmb:
            #if VERBOSE: print("  >>find_combinations: Comparing   s.join(wlx)==stc", s.join(wlx), "==", stc)
            #if  s.join(str(wlx))==stc:
            if  s.join(wlx)==stc:
                if VERBOSE: print("  >>find_combinations: matching   s.join(wlx)==stc",  s.join(wlx),"==",stc)
                st_op.append(stc)
    if VERBOSE: print("find_combinations: st_op=", st_op)
    return st_op


def mat_updt( LX, L1, L2,L3, mat_in,mydict,frmpo='', topo='', val=0, ch=1, mmd='disable'):
    # mmd= 'enable'  , 'disable', 'value'
    VERBOSE=False
    funstr = "mat_updt >>> "
    uval=0
    if mmd=='disable': uval=0
    if mmd=='value': uval=val
    mat_op=deepcopy(mat_in)
    if VERBOSE: print(frmpo)
    fp=find_combinations(frmpo,LX, L1, L2,L3)
    if VERBOSE: print(fp)
    tp=find_combinations(topo,LX, L1, L2,L3)
    if VERBOSE: print("mat_updt: fp=",fp)
    if VERBOSE: print("mat_updt: tp=",tp)
    if VERBOSE: print("mat_updt: LX=", LX)
    if VERBOSE: print("mat_updt: len(LX)=", len(LX))
    if VERBOSE: print("mat_updt: type(LX)=", type(LX))
    if VERBOSE: print("mat_updt: type(LX[0])=", type(LX[0]))
    if VERBOSE: print('\n', funstr, 'Processing .................................. \n')

    for i in range(0, len(LX)):
        for j in range(0, len(LX)):
            for ifp in fp:
                for itp in tp:
                    si=''.join(LX[i])
                    sj=''.join(LX[j])
                    #if VERBOSE: print("mat_updt: Check- LX from->to {",  si," : ", sj,"} ")
                    #if VERBOSE: print("mat_updt: Check- LX from->to {",  si," : ", sj,"} ")
                    #if VERBOSE: print("mat_updt: Check- RX from->to {", ifp," : ",itp,"} ")
                    if (si==ifp) and (sj==itp) and nchnges(ifp,itp,ch=1,n=3):
                        if mmd == 'enable':
                            if ifp[0] != itp[0]: #and (ifp[2] == "R" or itp[2] == "R")):
                                uval=(request_cost_client(get_dicval(mydict,ifp[0],'uuid'), get_dicval(mydict,itp[0],'uuid')))/rb_spd
                            elif ifp[1] != itp[1]:
                                if VERBOSE: print(get_dicval(mydict,ifp[1],'name'), get_dicval(mydict,itp[1],'name'))
                                uval=(request_cost_client(get_dicval(mydict,ifp[1],'uuid'), get_dicval(mydict,itp[1],'uuid')))/rb_spd
                            elif ifp[2] == "R" and itp[2] != "R": #unloading
                                if itp[2]==ifp[0] and itp[2]==ifp[1] and itp[2]==itp[0] and itp[2]==itp[1]:
                                    if VERBOSE: print(ifp, itp)
                                    uval = ult
                                    if VERBOSE: print("mat_updt: MultyConstraint add: {", ifp," : ", itp,"}  c=",uval )
                            elif ifp[2] != "R" and itp[2] == "R": #loading
                                if ifp[2]==ifp[0] and ifp[2]==ifp[1] and ifp[2]==itp[0] and ifp[2]==itp[1]:
                                    uval = ldt
                                    if VERBOSE: print("mat_updt: MultyConstraint add: {", ifp," : ", itp,"}  c=",uval )
                        mat_op[i][j]=uval
                        if VERBOSE: print("mat_updt: MultyConstraint add: {", ifp," : ", itp,"}  c=",uval )

                        '''
                        if mmd == 'enable':
                            uval=request_cost_client(get_dicval(mydict,ifp[0],'uuid'), get_dicval(mydict,itp[0],'uuid'))
                        mat_op[i][j]=uval
                        if VERBOSE: print("mat_updt: MultyConstraint add: {",ifp," : ", itp,"}  c=",uval )
                        '''
    return mat_op




class IniTP:
    def __init__(self):
        self.VERBOSE2 = True
        self.VERBOSE = False
        self.G = list()
        self.dir_path=""
        self.w={}

        self.adj_matrix={}
        self.adj_matrix_modf ={}
        self.w_short = {}
        self.graph = {}
        self.AoI_dict={}
        self.L1=[]
        self.L2=[]
        self.L3=[]

        
    def set_ParamsDisabled(self, L1, L2, L3, AoI_dict, dir_path):
        # self.stopic=stopic
        self.L1=L1
        self.L2=L2
        self.L3=L3

        self.G = list()
        self.G.append(L1)
        self.G.append(L2)
        self.G.append(L3)

        self.AoI_dict=AoI_dict
        self.dir_path = dir_path

        self.w = StatesGeneration(self.G)
        n = len(self.G)
        ch = 1
              
        # Adjacency Matrix Disabled all Constraints
        self.adj_matrix_modf = [[0. for x in range(0, len(self.w))] for y in range(0, len(self.w))]
        self.w_short = StatesfromList2Graph(self.w)
        self.graph = GraphWITHAdjacency(self.w_short, self.adj_matrix_modf)
        
    def set_Params(self, L1, L2, L3, AoI_dict, dir_path):
        # self.stopic=stopic
        self.L1=L1
        self.L2=L2
        self.L3=L3

        self.G = list()
        self.G.append(L1)
        self.G.append(L2)
        self.G.append(L3)

        self.AoI_dict=AoI_dict
        self.dir_path = dir_path

        self.w = StatesGeneration(self.G)

        if self.VERBOSE2: print("IniTP: Create States output file")
        if self.VERBOSE2: print("IniTP: self.w =", self.w)
        if self.VERBOSE: var2file2(self.w, '%s', dir_path + "/op/STATES_output_file.txt", 'w',
                                   'nmatxcont')  # yaml: dictionary    mat2: matrix 2d

        # Matrix 1
        n = len(self.G)  # length of each word(state)
        if self.VERBOSE2: print("IniTP: len(self.G) =", n)
        ch = 1  # one character change is allowed per state transition
        counter1, matrix1 = OneStepChange(self.w, n, ch, AoI_dict)  #matr = [[0. for x in range(0, len(LX))] for y in range(0, len(LX))]
        if self.VERBOSE2: print("IniTP: Create  Matrix1 output file")
        if self.VERBOSE: print('IniTP: MATRIX 1')
        if self.VERBOSE: var2file2(matrix1, "%4.3f", dir_path + "/op/Matrix1_output_file.txt", 'w',
                                   'nmatx')  # yaml: dictionary    mat2: matrix 2d

	
        counter2, matrix2 = Zero2RChanges(self.w, n, matrix1, AoI_dict)
        if self.VERBOSE2: print("IniTP: Create  Matrix2 output file")
        if self.VERBOSE: print('IniTP: MATRIX 2')
        if self.VERBOSE: var2file2(matrix2, '%4.3f', dir_path + "/op/Matrix2_output_file.txt", 'w',
                                    'nmatx')  # yaml: dictionary    mat2: matrix 2d

        # Adjacency Matrix
        self.adj_matrix = AdjacencyMatrix(matrix1, matrix2)
        if self.VERBOSE2:  print('IniTP: ADJACENCY MATRIX')
        if self.VERBOSE:  print("IniTP: Create Adjacency Matrix output file")
        if self.VERBOSE:  var2file2(self.adj_matrix, '%4.3f', dir_path + "/op/ADJACENCY_output_file.txt", 'w',
                                    'nmatx')  # yaml: dictionary    mat2: matrix 2d

        # Adjacency Matrix Modified Constraints
        self.adj_matrix_modf = deepcopy(self.adj_matrix)
        self.w_short = StatesfromList2Graph(self.w)
        if self.VERBOSE2:  print('\nIniTP: STATES w_short=')
        if self.VERBOSE2:  printmat(self.w_short)

        self.graph = GraphWITHAdjacency(self.w_short, self.adj_matrix_modf)

    def madd_constraints(self,str1,str2,mmode,uval=0):
        matrix5=mat_updt(self.w, self.L1, self.L2, self.L3, self.adj_matrix_modf,self.AoI_dict,
                                 frmpo=str1, topo=str2, val=uval, ch=1 , mmd=mmode)
        self.adj_matrix_modf = deepcopy(matrix5)
        if self.VERBOSE:  var2file2(self.adj_matrix_modf, '%4.3f',
                                    self.dir_path + "/op/Multiple_NC_ADJ_output_file.txt", 'w',
                                    'nmatx')  # yaml: dictionary    mat2: matrix 2d
        self.graph = GraphWITHAdjacency(self.w_short, self.adj_matrix_modf)

    def add_constrnts(self, Cadd):
        # NewConstraints
        matrix4 = AdjustNewConstraintCosts(self.w, Cadd, self.adj_matrix_modf)
        self.adj_matrix_modf = deepcopy(matrix4)
        if self.VERBOSE:  print('ADJACENCY MATRIX : Modified Constraints')
        if self.VERBOSE:  var2file2(self.adj_matrix_modf, '%4.3f',
                                    self.dir_path + "/op/NewConstraintsADJACENCY_output_file.txt", 'w',
                                    'nmatx')  # yaml: dictionary    mat2: matrix 2d
        self.graph = GraphWITHAdjacency(self.w_short, self.adj_matrix_modf)

    def reset_constrnts(self):
        self.adj_matrix_modf = deepcopy(self.adj_matrix)
        self.graph = GraphWITHAdjacency(self.w_short, self.adj_matrix_modf)

    def solve_path(self, Strt, Goal):
        if self.VERBOSE2:  print('\n:::::::::::::::::::::::::\nRunning Strt-Goal (%s - %s)=') % (
        Strt, Goal), '.............'
        if self.VERBOSE2:  print('Start - Robot Position=',Strt[0], self.AoI_dict[Strt[0]])
        if self.VERBOSE2:  print('Start - Human Position=',Strt[1], self.AoI_dict[Strt[1]])
        if self.VERBOSE2:  print('Start - Item Position=',Strt[2], self.AoI_dict[Strt[2]])
        if self.VERBOSE2:  print(' Goal - Robot Position=',Goal[0], self.AoI_dict[Goal[0]])
        if self.VERBOSE2:  print(' Goal - Human Position=',Goal[1], self.AoI_dict[Goal[1]])
        if self.VERBOSE2:  print(' Goal - Item Position=',Goal[2], self.AoI_dict[Goal[2]])
        mreslt = dijkstra(self.graph, Strt, Goal)
        #if self.VERBOSE:  print(mreslt)
        if self.VERBOSE2:  print('\nResult for Strt-Goal (%s - %s)=') % (Strt, Goal)
        if self.VERBOSE2:  print('Shortest path Cost =' + str(mreslt[1]))
        if self.VERBOSE2:  print('The path is ' + str(mreslt[0]))

        return mreslt



class GetDatafromParcer:
    #g_d = get_dicval
    def __init__(self):
        self.VERBOSE = False
        self.dir_path=""
        #self.mfile = self.dir_path + "/" + "bpo_specif_v4.json"
        self.bpo_data = None
        self.g_d=get_dicval
        self.inTP = IniTP()

  
    def readspec(self, mfile):
        with open(mfile, "r") as f:
            #self.bpo_data = json.loads(f.read(), object_pairs_hook=OrderedDict)
            #self.bpo_data = json.loads(f.read())
            self.bpo_data = json.dumps(json.loads(f.read(), object_pairs_hook=OrderedDict))
            #self.bpo_data = json.dumps(json.loads(f.read(), object_pairs_hook=OrderedDict))
        self.bpo_data = ast.literal_eval(self.bpo_data)
        if self.VERBOSE: print("readspec >>> bpo parced dict ................ \n >>> \n", self.bpo_data, "\n <<< \n")
        return self.bpo_data

    def lt_lists_nmbr(self, bpo, wd_p):
        VERBOSE = False
        funstr = 'lt_lists_nmbr >>> '

        #tmp1 = self.g_d(bpo,"specification","Environment")
        tmp1 = self.g_d(bpo, "Environment")
        cntr = 0
        for i in tmp1:
            if not i == "locations":
                if VERBOSE: print(funstr, i)
                wd_p.update({i:cntr})
                cntr += 1
        if VERBOSE: print(funstr, wd_p)
        return wd_p

    def get_locations(self, bpo,*args):
        ANm=[]
        ALt=[]
        #PANm=self.g_d(bpo,"specification","Environment","locations", "Name")
        PANm=self.g_d(bpo, "Environment","locations", "Name")
        #PALt=self.g_d(bpo,"specification","Environment","locations", "Letter")
        PALt=self.g_d(bpo, "Environment","locations", "Letter")
        for nm in range(0, len(PANm)):
            for i in range(0,len(args)):
                if not (args[i] in PANm[nm]):
                    ANm.append(PANm[nm])
                    ALt.append(PALt[nm])
        return ANm, ALt

    def get_specs2(self, bpo, wd_p):
        VERBOSE=False
        funstr = 'get_specs2 >>> '
        ANm, ALt = self.get_locations(bpo,"on_robot")
        if VERBOSE: print("ANm =",ANm,"\n", "Alt =",ALt)
        str1 = 'L'
        str2 = str1
        str2 = {}
        nmlb = ''
        for i, j in wd_p.items():
            kkey = i
            nmlb = str(j+1)
            if VERBOSE: print(funstr, kkey, nmlb)
            for ii in bpo:
                #str2.update({str1+nmlb:get_dicval(bpo,'specification','Environment', kkey)})
                str2.update({str1+nmlb:get_dicval(bpo,'Environment', kkey)})
        return str2


    def word_constr(self, lt, pos, word_pos):        
        VERBOSE = False
        funstr = 'word_constr >>> '
        state_word = ''
        for i in range(0 , len(word_pos)):
            if VERBOSE: print(funstr, 'i = ', i, 'pos = ', pos)
            if not i == pos:
                state_word += '*'
            else:
                state_word += str(lt)
        if VERBOSE: print(funstr, state_word)
        return state_word


    def letterlist_append(self, nmlist, b, word_pos):
        list = []
        letters_dict = self.get_specs2(b, word_pos)
        for i in letters_dict:
            if i == nmlist:
                list = letters_dict[i]
        return list

    def get_constr(self, bpo, word_pos):
        VERBOSE = False
        funstr = 'get_constr >>> '
        Constr_list = []
        incom_list = []
        outcom_list = []
        #res_con = get_dicval(bpo, 'specification', 'Constraints')
        res_con = get_dicval(bpo, 'Constraints')
        for i in res_con:
            #res_con_1 = get_dicval(bpo, 'specification', 'Constraints', i)
            res_con_1 = get_dicval(bpo, 'Constraints', i)
            for ii, jj in res_con_1.items():
                input = ii
                for iii in jj:
                    output = iii
                    if VERBOSE: print(funstr, i, ': input = ', input, ' output = ', output)
                    if i in word_pos:
                        pos = word_pos[i]
                        if VERBOSE: print(funstr, 'pos = ', pos)
                        stword_input = self.word_constr(input, pos, word_pos)
                        stword_output = self.word_constr(output, pos, word_pos)
                        if VERBOSE: print(funstr, stword_input, stword_output)
                        Constr_list.append(stword_input +":"+ stword_output)
        return Constr_list


    def addconstr(self, incom, outcom, pos, word_pos):
        if not pos == word_pos['item_1']:
            #self.inTP.madd_constraints(incom, '***', 'disable')
            #self.inTP.madd_constraints('***', outcom, 'disable')
            self.inTP.madd_constraints(incom, outcom, 'value')
        elif pos == word_pos['item_1']:
            #self.inTP.madd_constraints(incom, '***', 'disable')
            #self.inTP.madd_constraints('***', outcom, 'disable')
            self.inTP.madd_constraints(incom, '*R*', 'value', ldt)
            self.inTP.madd_constraints('*R*', outcom, 'value', ult)


    def starting_word(self, bpo, word_pos):
        VERBOSE = False
        funstr = 'starting_word >>> '
        start_word = list('')
        stw = ''

        for i in word_pos:
            start_word.append('*')

        #rest = get_dicval(bpo, 'specification', 'Starting_point')
        rest = get_dicval(bpo, 'Starting_point')
        for i in rest:
            if VERBOSE: print(i, rest[i])
            entity = i
            entlt = rest[i]
            if i in word_pos:
                posent = word_pos[i]
                start_word[posent] = entlt
        for x in start_word:
            stw += x
        if VERBOSE: print(funstr, 'starting_state = ', stw)

        '''
        for j in range(0, len(word_pos)):
            if i == j:
                entity = i
                posent = word_pos[i]
                ltent = rest[i]
                if VERBOSE: print(entity, posent, ltent)
        '''
        return stw

    def startmiddlestart(self, bpo, word_pos, letters_dict):
        start1 = self.starting_word(bpo, word_pos)
        start2 = ''
        ltstart1 = start1[2]
        start2 = str(ltstart1) + str(ltstart1) + str(ltstart1)
        win1 = str(start1[0]) + '*' + ltstart1
        win2 = '*' + str(start1[1]) + ltstart1
        wout1 = ltstart1 + '*' + ltstart1
        wout2 = '*' + ltstart1 + ltstart1
        self.inTP.madd_constraints(win1, '***', 'disable')
        self.inTP.madd_constraints(win2, '***', 'disable')
        self.inTP.madd_constraints(win1, wout1, 'enable')
        self.inTP.madd_constraints(win2, wout2, 'enable')
        return start2


    def obj_word(self, bpo, word_pos):
        #function pou diavazei to "Objective" key
        #kai to metafrazei se goal state
        #eg. item_1->"A"
        #sthn thesi ths leksis tou state pou einai gia to item_1, tha mpei to gramma A

        VERBOSE = False
        funstr = 'objective_word >>> '

        #resobj = get_dicval(bpo, 'specification', 'Objective')
        resobj = get_dicval(bpo, 'Objective')
        for i in resobj:
            if i in word_pos:
                entity = i
                ltobj = resobj[i]
                posobj = word_pos[i]
                if VERBOSE: print(entity, posobj, ltobj)
                objective_word = self.word_constr(ltobj, posobj, word_pos)
                if VERBOSE: print(funstr, objective_word)
        return objective_word


    def obj_word2(self, bpo, word_pos):
        VERBOSE = False
        funstr = 'objective_word >>> '

        objective_word=''

        #resobj = get_dicval(bpo, 'specification', 'Objective')
        resobj = get_dicval(bpo, 'Objective')
        for i in resobj:
            if i in word_pos:
                entity = i
                ltobj = resobj[i]
                posobj = word_pos[i]
                if VERBOSE: print(funstr, 'entity, posobj, ltobj >>> ', entity, posobj, ltobj)

        if VERBOSE: print(ltobj)
        for i in word_pos:
            objective_word += str(ltobj)

        return objective_word


def snchk_specs(j):
    # NEED dict_keys(['id', 'dateModified', 'type', 'specification', 'dateCreated'])
    # NOT FOUND= j = ', {u'description': u'The requested entity has not been found. Check type and id', u'error': u'NotFound'})
    VERBOSE = False
    ret_val=False
    rq_type = 'BPOSpecificationInput'
    if VERBOSE: print(" snchk_specs j = ",j)
    jkeys=j.keys()
    if VERBOSE: print(" snchk_specs jkeys = ",jkeys)
    if "error" in j:
        ret_val =False
    else:
        if "type" in j:
            sp_type = j['type'] 
            if sp_type == rq_type: ret_val=True
    
    return ret_val


headers = {
	'Accept': 'application/json',
	}

def bpospecfromocb2(context_broker_ip, OCB_BPO_ID, prevtm):
    VERBOSE = True
    newtm = False
    fl = ""
    r = requests.get('http://'+ context_broker_ip + ':1026/v2/entities/' + OCB_BPO_ID + '?options=dateCreated,dateModified', headers=headers)
    j = json.loads(r.text)

    JOK=snchk_specs(j)
    if VERBOSE: print(" bpospecfromocb2 JOK = ",JOK)
    
    if JOK:    
        dir_path = os.path.dirname(os.path.realpath(__file__))
        curtm = j['dateModified']['value']
        if VERBOSE: print(" Prev/Cur Spec Times = ",prevtm,"/",curtm)  
        if curtm != prevtm:
            if VERBOSE: print("NEW Spec Found for Time=",curtm)
            newtm = True
            prevtm = curtm
            getspec = j['specification']['value']
            fl = dir_path+"/data.json"
            with open(fl, 'w') as f:
                json.dump(getspec, f, ensure_ascii=False, indent=4)
            if VERBOSE: print(" bpospecfromocb2:: Wrote data to file=", fl)
      
    return (fl,newtm,prevtm)



