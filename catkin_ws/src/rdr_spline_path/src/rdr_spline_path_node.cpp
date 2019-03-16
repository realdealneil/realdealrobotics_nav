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
