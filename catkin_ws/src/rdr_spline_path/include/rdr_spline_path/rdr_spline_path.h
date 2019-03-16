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

#include <pose_estimation/rdr_pose_estimation.h>
#include <attitude_control/AttitudeControl.h>

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
	void MakeSplineFromWaypoints(const std::vector<Eigen::Vector3d>& wplist);
	
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
	
	/// \brief This integrator is used to integrate the spline
	Eigen::Integrator<Scalar> _integrator;
	
	/// \brief Pose Estimator: gets pose from either tf or odometry:
	poseEstimation _poseEstimator;
	RdrPose _vehiclePose;
	bool _poseValid = false;
	
	/// \brief Attitude Control class
	RdrAttitudeControl _attitudeControl{};
	
};

#endif // RDR_SPLINE_PATH_H
