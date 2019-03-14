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
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>
#include <vector>

#include <ros/ros.h>

struct gate_corners {
	Eigen::Vector3d ul;
	Eigen::Vector3d ur;
	Eigen::Vector3d lr;
	Eigen::Vector3d ll;
};

class splineMaker
{
public:
	splineMaker() 
		: _nh("~")
	{
		LoadParams();
	}
	
	void LoadParams();
	
	/**
	 * 	Generate a sample spline path.  This is for testing the spline stuff in general:
	 */
	std::vector<Eigen::Vector3d> sampleWaypointGenerator(void);
	
	std::vector<gate_corners> getGateCornerList(void);
	Eigen::Vector3d find_center(gate_corners c);
	std::vector<Eigen::Vector3d> getGateCenters(std::vector<gate_corners> corners);
	
private:
	ros::NodeHandle _nh;
	double spline;
};

#endif // RDR_SPLINE_PATH_H
