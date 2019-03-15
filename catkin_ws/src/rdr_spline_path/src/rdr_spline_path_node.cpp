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
	
	ros::Rate loop_rate(5);
	while (ros::ok())
	{
		mySplineMaker.Update();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
