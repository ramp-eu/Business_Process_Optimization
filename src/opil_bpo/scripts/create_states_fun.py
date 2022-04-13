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

def prod(low,high,fun):
    res = 1;
    if low==high:
        return res
    else:
        for i in range(low,high):
            res = res * fun[i]
        return res


def StatesGeneration(G):
    VERBOSE=False
    m = len(G)
    ni = list()
    for i in range(0, m):
        ni.append(len(G[i]))

    totwords = 1
    for i in range(0,m):
        totwords = totwords * ni[i]

    if VERBOSE: print("\nStatesGeneration:       G=",G)
    if VERBOSE: print("\nStatesGeneration:    lenG=",m)
    if VERBOSE: print("\nStatesGeneration:      ni=",ni)
    if VERBOSE: print("\nStatesGeneration: totwords=",totwords)

    w = [None]*totwords
    for i in range(0,totwords):
        w.pop(i)
        w.insert(i,[1 for k in range(m)])

    itt=0
    for i in range(0, m):
        for r in range(0, prod(0,i,ni)):
            for j in range(0, ni[i]):
                for k in range(0,prod(i+1, m, ni)):
                    #itt=r*(j*prod(i+1, m, ni))+k
                    wtmp = w[itt]
                    w.pop(itt)
                    wtmp.pop(i)
                    wtmp.insert(i, G[i][j])
                    w.insert(itt, wtmp)
                    if (itt==(totwords-1)):
                        itt=0
                    else:
                        itt=itt+1
                        #print(itt, wtmp)
    return w

