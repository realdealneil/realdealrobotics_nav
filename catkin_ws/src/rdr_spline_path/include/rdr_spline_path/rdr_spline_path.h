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
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>
#include <spline/SplineIntegration.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <rdr_spline_path/GateLocationList.h>
#include "eigen_helpers.h"
#include <ros/ros.h> 

using GateList = std::vector<size_t>; // list of gate indices
using WaypointList = std::vector<Eigen::Vector3d>;
class splineMaker
{
public:
	splineMaker(const rdr_spline_path::GateLocationList& gate_location_list);
	

	/**
	 * 	Generate a sample spline path.  This is for testing the spline stuff in general:
	 */
	
  // expects the indexes in the gate_list to be 1 based!!
  WaypointList sampleWaypointGenerator(const GateList& gate_list);
	void print_gate_list();
	
	

	void Update();

	WaypointList constructWaypointList(
    const Eigen::Vector3d& vehicle_start_position, 
    const GateList& gatelist);
	void MakeSplineFromWaypoints(const WaypointList& wplist);
	void SetCornerMarkersForPublishing(const GateList& gate_list);

private:
	ros::NodeHandle nh_;
  ros::Publisher rvizMarkerPub_;
  visualization_msgs::MarkerArray rvizMarkerArray_;
	std::vector<Eigen::Vector3d> primaryWaypoints_;
	Eigen::Integrator<Scalar> integrator_;
  // this is 0 based!!! 
	EigenGateLocationList gate_location_list_;
};

#endif // RDR_SPLINE_PATH_H
