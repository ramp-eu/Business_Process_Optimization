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


def GetCost(ConstrDiction):
    c = list()
    for i in ConstrDiction.keys():
        c.append([i])
        ccost= ConstrDiction.setdefault(i)
    return c



def AdjustNewConstraintCosts(w, NC, adjmatr):
    constr = GetCost(NC)

    s1 = (len(constr[0][0])-1)//2
    s2 = (len(constr[0][0])+1)//2

    for i in range(len(constr)):
        #print(constr[i])
        for j in range(len(constr[i])):
            state = list()
            for k in range(s1):
                state.append(constr[i][j][k])
                state_position = 0
                for ii in range(len(w)):
                    if w[ii] == state:
                        #print(w[ii], state)
                        state_position = ii
                        print('state', state, 'position', state_position)
                        constraint = list()
                        constraint_position = 0
                        for kk in range(s2, len(constr[i][j])):
                            constraint.append(constr[i][j][kk])
                            for jjj in range(len(w)):
                                if (w[jjj] == constraint):
                                    constraint_position = jjj
                                    print('constraint', constraint, 'position', constraint_position)
                                    stcost = NC.get(constr[i][j]) 
                                    adjmatr[state_position][constraint_position] = stcost 

    return adjmatr


