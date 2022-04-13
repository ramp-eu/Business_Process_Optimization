#include "ros/ros.h"
#include <stdio.h>
#include <maptogridmap/Nodes.h>
#include <maptogridmap/Edges.h>
#include "SPT.h"
#include "opil_bpo/shpath.h"
#include <stack>

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

int node_read=0;
int edges_read=0;
int graph_ready=0;
int maxnodes = 10;

maptogridmap::Nodes opnodes;
maptogridmap::Edges opedges;

typedef DenseGRAPH<Edge> dGraph;
//dGraph g(int(maxnodes) , false);

bool VERBOSE=false;

void mapNodesCallback(const maptogridmap::Nodes::ConstPtr& msg)
{
	opnodes.name.clear();
	opnodes.uuid.clear();
	opnodes.x.clear();
	opnodes.y.clear();
	opnodes.theta.clear();
	for (int i=0;i<msg->name.size();i++){
		opnodes.name.push_back(msg->name[i]);
		opnodes.uuid.push_back(msg->uuid[i]);
		opnodes.x.push_back(msg->x[i]);
		opnodes.y.push_back(msg->y[i]);
		opnodes.theta.push_back(msg->theta[i]);
		}
	node_read = 1;
	//ROS_INFO("PATHDIST: OPNODES READ");
}

double EuclideanDistance(double x1, double y1, double x2, double y2)
{
	double x = x1 - x2;
	double y = y1 - y2;
	double dist;

	dist = pow(x, 2) + pow(y, 2);
	dist = sqrt(dist);                  

	return dist;
}

void mapEdgesCallback(const maptogridmap::Edges::ConstPtr& msg)
{
	//typedef DenseGRAPH<Edge> dGraph;
	//dGraph g( maxnodes , false );
	
	opedges.name.clear();
	opedges.uuid.clear();
	opedges.uuid_src.clear();
	opedges.uuid_dest.clear();

	for (int i=0;i<msg->name.size();i++){
		opedges.name.push_back(msg->name[i]);
		opedges.uuid.push_back(msg->uuid[i]);
		opedges.uuid_src.push_back(msg->uuid_src[i]);
		opedges.uuid_dest.push_back(msg->uuid_dest[i]);
	}
	edges_read = 1;
	//ROS_INFO("PATHDIST: OPEDGES READ");
}


bool service_callback(opil_bpo::shpath::Request &request, opil_bpo::shpath::Response &response)
{
	int node1, node2;
	double cost;
	if (VERBOSE) ROS_INFO("definition of the graph");
	if (VERBOSE) ROS_INFO ("graph size:%d",opedges.name.size());

	dGraph g( int(opnodes.name.size()*2 ) , true);
	//dGraph g( 1000 , false);
	if (VERBOSE) ROS_INFO ("dGraph created");
	//while !(node_read && edges_read) {printf("No New Messages node_read:edges_read = %i:%i\n",node_read ,edges_read);}
	if(node_read && edges_read){
		double temp_x1, temp_x2, temp_y1, temp_y2;
		for (int j=0;j<opedges.name.size();j++){
			node1=0; node2=0;
			for (int ii=0; ii<opnodes.name.size(); ii++){
				if (opedges.uuid_src[j].compare(opnodes.uuid[ii])==0){
					node1 = ii;
					temp_x1=opnodes.x[ii];
					temp_y1=opnodes.y[ii];
				}
				if (opedges.uuid_dest[j].compare(opnodes.uuid[ii])==0){
					node2 = ii;
					temp_x2=opnodes.x[ii];
					temp_y2=opnodes.y[ii];
				}
			}
			cost = EuclideanDistance(temp_x1, temp_y1, temp_x2, temp_y2);
			//ROS_INFO ("cost:%6.4lf", cost);
			g.insert(new Edge(node1, node2, cost));
			g.insert(new Edge(node2, node1, cost));
			if (VERBOSE) ROS_INFO ("Added Node N1,N2,Cst =< %d , %d , %8.4lf>", node1, node2, cost);
			if (VERBOSE) ROS_INFO ("g.Ecnt = %d ", g.E() );
		}
		//node_read = 0;
		//edges_read = 0;
		graph_ready = 1;
		if (VERBOSE) ROS_INFO("PATHDIST: GRAPH IS READY");
	}
	else
	{
		return false;
	}
	
	if (VERBOSE) ROS_INFO("PATHDIST: ENTERING SERVICE");
	//string start = request.start_uuid;
	//string goal = request.goal_uuid; 
	if (VERBOSE) ROS_INFO("PATHDIST: DEFINE SERVICE REQUESTS");

	int source, target;
	for (int ii=0; ii<opnodes.name.size(); ii++){
		//ROS_INFO("req.start = %s, opnodes = %s ", request.start_uuid.c_str(), opnodes.uuid[ii].c_str());

		if (request.start_uuid.compare(opnodes.uuid[ii])==0){
			source = ii;
			if (VERBOSE) ROS_INFO("source = %d", source);
		}
		if (request.goal_uuid.compare(opnodes.uuid[ii])==0){
			target = ii;
			if (VERBOSE) ROS_INFO("target = %d", target);
		}
	}

	if (VERBOSE) ROS_INFO("BEFORE THE GRAPH");
	// Find shortest path between source and target
	stack<int> path;
	SPT<dGraph, Edge> sp( g, int(source) );
	
	if (VERBOSE) ROS_INFO("Printing SPT Developed");
	if (VERBOSE) ROS_INFO("\nPrinting sp weights");
	if (VERBOSE) ROS_INFO("\nPrinting SPT Edges .... TOTAL=%d", sp.spt.size());

	if (!graph_ready){
		if (VERBOSE) ROS_INFO("graph is not ready");
		return false;
	}

	if (VERBOSE) ROS_INFO("graph is ready");
	
	double path_length=0.0;
	int cntr=0;
	while ( true )
	{
		cntr++;
		if (VERBOSE) printf(" In while loop cntr=%d\n",cntr);
		Edge* e = sp.pathR( target ); // Calculate path backwards from target
		if (VERBOSE) ROS_INFO("edge e.w = %3d   e.v=%3d", e->w, e->v);

		path.push( target );  // Put each node in Path
		if (VERBOSE) ROS_INFO ("PATH SIZE:%d",path.size());
		if (VERBOSE) ROS_INFO ("target:%d",target);
		if (VERBOSE) ROS_INFO ("Adding Length e-wt:%lf",sp.dist( target));
		path_length += e->wt;  //  sp.dist( target); //e->wt;
		if (VERBOSE) ROS_INFO("path cost = %6.4lf", path_length);
		//if ( target == source ) break;
		//if ( target == source ) {printf("TARGET==SOURCE ........ I'm OUT\n"); break;}
		if (VERBOSE) ROS_INFO("path cost2 n loop = %6.4lf", path_length);
		target = e->v;   //Set target to previous node (of edge)
		if ( target == source ) {
			if (VERBOSE) printf("TARGET==SOURCE ........ I'm OUT\n"); 
			path.push( target ); 
			break;}

	}

	if (VERBOSE) ROS_INFO("ENDING 1...........\n");
	
	response.path_totalcost = path_length;
	//ROS_INFO("path cost = %6.4lf", response.path_totalcost);

	//if ( path.empty() ) printf("Path is EMPTY\n");
	//else printf("Path is NOT EMPTY\n");
	if (VERBOSE) ROS_INFO("path cost = %6.4lf", response.path_totalcost);

	if (VERBOSE) ROS_INFO("ENDING 2...........\n");

	// Reversing the path order
	int nodeid[path.size()];
	int nnds=0;
	while (path.size()>0) { //( !path.empty() ){

		if (VERBOSE) ROS_INFO("In while loop : Path number of nodes left =%d", path.size());
		//std::cout << path.top() << " ";
		//int dummy=path.top();
		int dummy=path.top();
		nodeid[nnds]=dummy;
		response.path_uuids.push_back(opnodes.uuid[dummy]);
		path.pop();
		nnds++;
	}

	for (int jj=0;jj<response.path_uuids.size();jj++) {
		if (VERBOSE) printf("Added node (id=%d) uuid = %s \n", nodeid[jj], response.path_uuids[jj].c_str());
	}
	response.total_path_points = response.path_uuids.size();
	if (VERBOSE) ROS_INFO("path size = %6.4lf", response.total_path_points);
	if (VERBOSE) ROS_INFO("PATHDIST: EXITING SERVICE");

	return true;
	
	
}


int main(int argc, char **argv)
{
	ros::Subscriber mapnodes_sub;
	ros::Subscriber mapedges_sub;
	
	ros::init(argc, argv, "service_path_finder");
	ros::NodeHandle nn1;
	
	mapnodes_sub = nn1.subscribe("/map/nodes", 1, mapNodesCallback);
	mapedges_sub = nn1.subscribe("/map/edges", 1, mapEdgesCallback);
	ros::ServiceServer ss = nn1.advertiseService("/pathdist", service_callback);
	ros::spin();
	
	return 0;
}
