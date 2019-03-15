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
#include <rdr_spline_path/GateLocationList.h>



class splineMaker
{
public:
	splineMaker(const rdr_spline_path::GateLocationList& gate_location_list);
	

	/**
	 * 	Generate a sample spline path.  This is for testing the spline stuff in general:
	 */
	
  // expects the indexes in the gate_list to be 1 based!!
  std::vector<Eigen::Vector3d> sampleWaypointGenerator(const std::vector<size_t>& gate_list);
	//std::vector<Eigen::Vector3d> getGateNormals();
  void print_gate_list();
	
	//Eigen::Vector3d find_center(gate_corners c);

	void Update();

private:
  // this is 0 based!!! 
	rdr_spline_path::GateLocationList gate_location_list_;
  std::vector<Eigen::Vector3d> _gate_normals_vec; 
//	ros::NodeHandle _nh;
//	ros::Publisher _gateCornerPub;
//	visualization_msgs::MarkerArray _gateCornerMarkerArray;
//	void CreateCornerMarkersForPublishing();
	
//	std::vector<gate_corners> _corner_vec;
//	std::vector<Eigen::Vector3d> _center_vec;
//	std::vector<Eigen::Vector3d> _gate_normals_vec;
	double spline;
};

#endif // RDR_SPLINE_PATH_H
