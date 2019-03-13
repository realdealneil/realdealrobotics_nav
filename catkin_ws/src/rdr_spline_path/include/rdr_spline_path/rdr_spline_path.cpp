/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: Spline Path Generator
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */

#include <rdr_spline_path/rdr_spline_path.h>

std::vector<Eigen::Vector3d>  splineMaker::sampleWaypointGenerator(void)
{
	Eigen::Vector3d point;
	std::vector<Eigen::Vector3d> points;
	
	point << 0, 0, 0;
	points.push_back(point);
	
	return points;
}


