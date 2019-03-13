/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: Spline Path Generator
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */
 
#ifndef RDR_SPLINE_PATH_H
#define RDR_SPLINE_PATH_H

#include <iostream>
#include <cmath>
#include <unsupported/Eigen/Splines>
#include <vector>

class splineMaker
{
public:
	splineMaker()
	{
		
	}
	
	/**
	 * 	Generate a sample spline path.  This is for testing the spline stuff in general:
	 */
	std::vector<Eigen::Vector3d> sampleWaypointGenerator(void);
	
private:
	double spline;
};

#endif // RDR_SPLINE_PATH_H
