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

import rospy
from opil_bpo.msg import *
from taskplannerfuns12 import *

import timeit
import numpy as np
from copy import copy, deepcopy
import os
import json, ast
from collections import OrderedDict

context_broker_ip = '127.0.0.1'
OCB_BPO_ID = 'BPO'


prevtm = " "
headers = {
	'Accept': 'application/json',
	}

maoi=AoIClassN(stopic="/map/areas_of_interests")
dir_path = ""
strttm = None

TP=IniTP()
DtPrcr=GetDatafromParcer()

pubres = rospy.Publisher('/bpo/results', Results, queue_size=1)


def bpospecfromocb():
    r = requests.get('http://'+ context_broker_ip + ':1026/v2/entities/' + OCB_BPO_ID, headers=headers)
    j = json.loads(r.text)
    getspec = j['specification']['value']
    print('getspec', getspec)
    return getspec



def bpoOCBmsg2(path, time):
    msgres = Results()
    msgres.path = path
    msgres.time = time
    pubres.publish(msgres)


def process_specifications(fl):
    VERBOSE = True
    wd_pos = OrderedDict()
    b=bpospecfromocb()
    word_pos = DtPrcr.lt_lists_nmbr(b, wd_pos)
    letters_dict = DtPrcr.get_specs2(b, word_pos)

    if VERBOSE: print("process_specifications::: OrderedDict()= wd_pos=", wd_pos)
    if VERBOSE: print("process_specifications::: DtPrcr.readspec(fl)= b=", b)
    if VERBOSE: print("process_specifications::: DtPrcr.lt_lists_nmbr(b, wd_pos)=word_pos= ", word_pos)
    if VERBOSE: print("process_specifications::: DtPrcr.get_specs2= (b, word_pos)=", (b, word_pos))

    nnm=rospy.get_name()
    nnsp=rospy.get_namespace()
    if VERBOSE: print("Node Name =%s")%(nnm)
    if VERBOSE: print("Node NameSpace =%s")%(nnsp)

    if VERBOSE: print("Getting Server Parameters..........")
    L1 = DtPrcr.letterlist_append('L1', b, word_pos)
    L2 = DtPrcr.letterlist_append('L2', b, word_pos)
    L3 = DtPrcr.letterlist_append('L3', b, word_pos)
    ANm, ALt = DtPrcr.get_locations(b, "on_robot")
    Strt=DtPrcr.starting_word(b, word_pos)
    Goal=DtPrcr.obj_word2(b, word_pos)
    if VERBOSE: print("process_specifications: Printing Parameters: \nL1=%s\nL2=%s\nL3=%s\nStrt=%s\nGoal=%s\nANm=%s\nALt=%s")%(L1,L2,L3,Strt,Goal,ANm,ALt)


    if VERBOSE: print("Getting Areas of interest from m2gmap ..........")
    maoi.ANm=ANm
    maoi.ALt=ALt
    maoi.get_data_once()
    AoI_dict=maoi.get_dict()
    if VERBOSE: print("Received AoI Data=")
    if VERBOSE: print(AoI_dict)

    if VERBOSE: print("process_specifications: Printing Parameters: \nL1=%s\nL2=%s\nL3=%s\nStrt=%s\nGoal=%s\nANm=%s\nALt=%s")%(L1,L2,L3,Strt,Goal,ANm,ALt)
    TP.set_ParamsDisabled(L1=L1,L2=L2,L3=L3,AoI_dict=AoI_dict,dir_path=dir_path)

    #Adding Multiple Constraints
    C1 = DtPrcr.get_constr(b, word_pos)
    if VERBOSE: print("process_specifications:::: DtPrcr.get_constr(b, word_pos)= C1=",C1)
   
    for i in range(len(C1)):
        print('\n Enabling Constraints ..................................[%d/%d] \n'%(i+1,len(C1)))
        inp = C1[i].split(":")[0]
        outp = C1[i].split(":")[1]
        TP.madd_constraints(inp, outp, 'enable')
    if VERBOSE: print("BPO UP TO Enabling Constraints =:", timeit.default_timer() - strttm)
    #print("After ENABLING Constrains:::::::  TP.adj_matrix_modf = \n",TP.adj_matrix_modf)

    if VERBOSE: print("process_specifications:::: DtPrcr.get_constr(b, word_pos)= C1=",C1)
    
    Start2 = DtPrcr.startmiddlestart(b, word_pos, letters_dict)

    startsres=TP.solve_path(Strt, Start2)
    if VERBOSE: print("BPO UP TO solve_path1 =:", timeit.default_timer() - strttm)
    
    # Call Function
    if VERBOSE: print("Call Function TP.solve_path..........")
    mreslt=TP.solve_path(Start2,Goal)
    if VERBOSE: print("BPO UP TO solve_path2 =:", timeit.default_timer() - strttm)

    ttlpath = startsres[0] + mreslt[0]
    if VERBOSE: print('The path is ' + str(startsres[0]) + str(mreslt[0]))
    exctime = startsres[1] + mreslt[1]
    if VERBOSE: print('Execution time is ' + str(exctime) + ' minutes')

    ttime = int(exctime)
    bpoOCBmsg2(ttlpath, ttime)

if __name__ == "__main__":
       
    rospy.init_node('myBPO4OrionNode', anonymous=True)
    dir_path = os.path.dirname(os.path.realpath(__file__))
    strttm = timeit.default_timer()
    print("In MAIN Function  Looping Specs")
     
    rate = rospy.Rate(0.3)   
    while not rospy.is_shutdown():
        fl, newtm, prevtm = bpospecfromocb2(context_broker_ip, OCB_BPO_ID, prevtm)
        if newtm:
                strttm = timeit.default_timer()
                print(" New Specification Found...")
                process_specifications(fl)
                print("BPO Calculation duration =:", timeit.default_timer() - strttm)
        rate.sleep()

