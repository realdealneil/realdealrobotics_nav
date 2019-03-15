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
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
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
	splineMaker();
	
	void LoadParams();
	
	void Update();	
	
private:

	std::vector<gate_corners> getGateCornerList(void);
	Eigen::Vector3d find_center(gate_corners c);
	std::vector<Eigen::Vector3d> getGateCenters(std::vector<gate_corners> corners);
	std::vector<Eigen::Vector3d> getGateNormals();
	void constructFrontPoints(double d, std::vector<Eigen::Vector3d>& frontPoints, std::vector<Eigen::Vector3d>& backPoints);
	
	std::vector<Eigen::Vector3d> constructWaypointList();
	
	ros::NodeHandle _nh;
	ros::Publisher _gateCornerPub;
	visualization_msgs::MarkerArray _gateCornerMarkerArray;
	void CreateCornerMarkersForPublishing();
	
	std::vector<gate_corners> _corner_vec;
	std::vector<Eigen::Vector3d> _center_vec;
	std::vector<Eigen::Vector3d> _gate_normals_vec;
	std::vector<Eigen::Vector3d> _gate_front_points;
	std::vector<Eigen::Vector3d> _gate_back_points;
	
	std::vector<Eigen::Vector3d> _primaryWaypoints;
};

#endif // RDR_SPLINE_PATH_H
