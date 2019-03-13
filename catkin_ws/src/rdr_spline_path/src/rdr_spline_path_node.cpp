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
#include <rdr_spline_path/rdr_spline_path.h>

using namespace std;

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "rdr_spline_path");
	
	splineMaker mySplineMaker;
	
	vector<Eigen::Vector3d> waypoints = mySplineMaker.sampleWaypointGenerator();
	
	cout << "There are " << waypoints.size() << " waypoints in the list\n";
	
	ros::spin();
	return 0;
}
