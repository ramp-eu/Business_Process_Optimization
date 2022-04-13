#include "ros/ros.h"
#include <stdio.h>
#include <maptogridmap/Nodes.h>

/* ----------------------------------------------------------------------
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
*/

ros::Publisher pub_AOI_nodes;
maptogridmap::Nodes opnodes;
void mapnodesCallback(const maptogridmap::Nodes::ConstPtr& msg)
{
	opnodes.name.clear();
	opnodes.uuid.clear();
	opnodes.x.clear();
	opnodes.y.clear();
	opnodes.theta.clear();
	for (int i=0;i<msg->name.size();i++)
	{
		std::size_t found = msg->name[i].find( "vertex_");
  		if (found==std::string::npos)
		{
		opnodes.name.push_back(msg->name[i]);
		opnodes.uuid.push_back(msg->uuid[i]);
		opnodes.x.push_back(msg->x[i]);
		opnodes.y.push_back(msg->y[i]);
		opnodes.theta.push_back(msg->theta[i]);
		}
	}
	opnodes.header.stamp = ros::Time::now();
    pub_AOI_nodes.publish(opnodes);
}

int main(int argc, char **argv)
{
 ros::Subscriber mapnodes_sub;

 ros::init(argc, argv, "AoI");
 ros::NodeHandle n ("AoI");

 pub_AOI_nodes = n.advertise<maptogridmap::Nodes>("/map/areas_of_interests",1); 
 mapnodes_sub = n.subscribe("/map/nodes", 1, mapnodesCallback);

 ros::spin();

 return 0;
}
