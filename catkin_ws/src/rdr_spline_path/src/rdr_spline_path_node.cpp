/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: Spline Path Generator
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */

#include <ros/ros.h>

#include <iostream>
#include <cstdio>
#include <cmath>

#include "import_helpers.h"

using namespace std;



int main(int argc, char ** argv)
{
	ros::init(argc, argv, "rdr_spline_path");

	ros::NodeHandle n;
    GateLocationList gate_locations = importGateLocations(n);
    splineMaker mySplineMaker{gate_locations};
    mySplineMaker.print_gate_list();

    // Here's the order: ['Gate10', 'Gate21', 'Gate2', 'Gate13', 'Gate9', 'Gate14', 'Gate1', 'Gate22', 'Gate15', 'Gate23', 'Gate6']
    //GateList gate_list{10,21,2,13,9,14,1,22,15,23,6};
    GateList gate_list = importGateList(n);
    geometry_msgs::Pose initial_pose = get_initial_pose(n);

    mySplineMaker.SetCornerMarkersForPublishing(gate_list);

    WaypointList wplist = mySplineMaker.constructWaypointList(
      to_eigen(initial_pose.position), gate_list);

    mySplineMaker.MakeSplineFromWaypoints(wplist);

    ros::Rate loop_rate(60);
	uint64_t loop_count = 0;
	while (ros::ok())
	{		
		mySplineMaker.Run60HzLoop();
		
		if (loop_count % 12 == 0)
		{
			mySplineMaker.Run5HzLoop();
		}
		
		ros::spinOnce();
		loop_rate.sleep();
		loop_count++;
	}
	

	return 0;
}
