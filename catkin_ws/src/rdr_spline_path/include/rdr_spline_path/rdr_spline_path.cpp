/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: Spline Path Generator
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */

#include <rdr_spline_path/rdr_spline_path.h>
#include <iostream>
#include <vector>

splineMaker::splineMaker(const rdr_spline_path::GateLocationList& gate_location_list)
		: nh_("~")
		, integrator_(200)
		, _poseEstimator(nh_)
		, gate_location_list_(to_eigen(gate_location_list)) 
{
	
  rvizMarkerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("gateCornerMarkers", 10);
	
}

void splineMaker::print_gate_list()
{
  for(int gate_index = 1; gate_index <= gate_location_list_.gates.size(); gate_index++)
  {
    std::cout << "Gate " << gate_index << std::endl;

    const EigenGateLocation& egl = gate_location_list_.gates[gate_index - 1];
    std::cout << "corners:\n" << egl.corners  
      << "\n  perturbation bound: " << egl.perturbation_bound.transpose() 
      << "\n  center: " << egl.center.transpose()
      << "\n  normal: " << egl.normal.transpose()
      << "\n  front: " << egl.front_point.transpose()
      << "\n  back: " << egl.back_point.transpose() << std::endl;
  }
}

	

//! Construct waypoint list:
WaypointList splineMaker::constructWaypointList(
  const Eigen::Vector3d& vehicle_start_position, 
  const GateList& gate_list)
{
	std::vector<Eigen::Vector3d> wplist;
  
  //! First, add the vehicle's start position:
	wplist.push_back(vehicle_start_position);
	
	/** For each gate, figure out which point (front or back) is closest 
	 * 	to the previous waypoint in the list.  Then, add the points:
	 */
	const Eigen::Vector3d* prev_point = &vehicle_start_position;	//just to avoid copies...

	for (size_t i = 0; i < gate_list.size(); i++)
	{
		ROS_ASSERT(gate_list[i] <= gate_location_list_.gates.size());
		const EigenGateLocation& current_gate = gate_location_list_.gates[gate_list[i]-1];

		Eigen::Vector3d v_prev_to_fp = current_gate.front_point - *prev_point;
		double dist_prev_to_fp = v_prev_to_fp.norm();
		
		Eigen::Vector3d v_prev_to_bp = current_gate.back_point - *prev_point;
		double dist_prev_to_bp = v_prev_to_bp.norm();
		
		if (dist_prev_to_fp < dist_prev_to_bp)
		{
			wplist.push_back(current_gate.front_point);
			wplist.push_back(current_gate.center);
			wplist.push_back(current_gate.back_point);
			prev_point =  &current_gate.back_point;
		} else {
			wplist.push_back(current_gate.back_point);
			wplist.push_back(current_gate.center);
			wplist.push_back(current_gate.front_point);
			prev_point = &current_gate.front_point;
		}
	}
	
	//! Ok, we should have the waypoint list constructed now!
	std::cout << "Waypoint list is constructed!  There are " << gate_list.size() << 
		" gates and " << wplist.size() << " waypoints\n";
		
	//! Publish the waypoint path as line_strip markers in rviz:
	//! Publish points of first gate:		
	visualization_msgs::Marker waypoints;
	waypoints.header.frame_id = "world";
	waypoints.header.stamp = ros::Time::now();
	waypoints.id = 100;
	waypoints.type = visualization_msgs::Marker::LINE_STRIP;
	waypoints.action = visualization_msgs::Marker::ADD;
	waypoints.scale.x = 0.1;
	waypoints.scale.y = 0.1;
	waypoints.scale.z = 0.1;
	waypoints.color.b = 1.0;
	waypoints.color.a = 1.0;
	
	for (int i=0; i<(int)wplist.size(); i++)
	{	
		geometry_msgs::Point p;
		p.x = wplist[i](0);
		p.y = wplist[i](1);
		p.z = wplist[i](2);
		waypoints.points.push_back(p);		
	}	
	rvizMarkerArray_.markers.push_back(waypoints);
			
	return wplist;
}

void splineMaker::SetCornerMarkersForPublishing(const GateList& gate_list)
{
	//! Publish points of first gate:		
	
	visualization_msgs::Marker gate_pts;	
	
	gate_pts.header.frame_id = "world";
	gate_pts.header.stamp = ros::Time::now();	
	gate_pts.type = visualization_msgs::Marker::POINTS;
	gate_pts.action = visualization_msgs::Marker::ADD;
	gate_pts.scale.x = 0.2;
	gate_pts.scale.y = 0.2;
	gate_pts.scale.z = 0.2;
	gate_pts.color.g = 1.0;
	gate_pts.color.a = 1.0;
	
	
	
	for (size_t i = 1; i <= gate_list.size(); i++)
	{	
		ROS_INFO("processing gate list index %lu of %lu. Gate %lu", i, 
			gate_list.size(), gate_list[i-1]);
			
		ROS_ASSERT(gate_list[i-1] <= gate_location_list_.gates.size());
		const EigenGateLocation& current_gate = gate_location_list_.gates[gate_list[i-1]-1];

		visualization_msgs::Marker gate_lines;
		gate_lines.header.frame_id = "world";
		gate_lines.header.stamp = ros::Time::now();
		gate_lines.type = visualization_msgs::Marker::LINE_STRIP;
		gate_lines.action = visualization_msgs::Marker::ADD;
		gate_lines.scale.x = 0.1;
		gate_lines.scale.y = 0.1;
		gate_lines.scale.z = 0.1;
		gate_lines.color.b = 1.0;
		gate_lines.color.r = 1.0;
		gate_lines.color.a = 1.0;
		gate_lines.id = i+1000;
	
		gate_pts.id = i;
		geometry_msgs::Point p = to_ros_point(current_gate.corners.col(0));
		gate_pts.points.push_back(p);
		gate_lines.points.push_back(p);

		p = to_ros_point(current_gate.corners.col(1));
		gate_pts.points.push_back(p);
		gate_lines.points.push_back(p);
		
		p = to_ros_point(current_gate.corners.col(3));
		gate_pts.points.push_back(p);
		gate_lines.points.push_back(p);

		p = to_ros_point(current_gate.corners.col(2));
		gate_pts.points.push_back(p);
		gate_lines.points.push_back(p);

		//close the gate box
		gate_lines.points.push_back(to_ros_point(current_gate.corners.col(0)));

		// add gate center, front and back points
		gate_pts.points.push_back(to_ros_point(current_gate.center));
		gate_pts.points.push_back(to_ros_point(current_gate.front_point));
		gate_pts.points.push_back(to_ros_point(current_gate.back_point));		
		
		rvizMarkerArray_.markers.push_back(gate_pts);
		rvizMarkerArray_.markers.push_back(gate_lines);		
	}	
}

void splineMaker::MakeSplineFromWaypoints(const WaypointList& wplist)
{
	std::cout << "Making spline from waypoint list. There are " << wplist.size() << " waypoints\n";
	
	Eigen::MatrixXd wpMat(3, wplist.size());
	for (int i=0; i<(int)wplist.size(); i++)
	{
		wpMat.col(i) = wplist.at(i);
	}
	
	//typedef Eigen::Spline<double, 3> spline3d;
	
	_wpSpline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(wpMat, 3);
	_attitudeControl.SetSpline(_wpSpline); 
	//int dimension = 3;
	//Eigen::Matrix<double, 3, 1> derivatives = s.derivatives(param, 1).col(1);
	
	//spline3d s = Eigen::SplineFitting<spline3d>::InterpolateWithDerivatices(
	
	//! Get values from the spline at a variety of points along the spline and add to the rviz display:
	int num_sample_pts = 1000;
	std::vector<Eigen::Vector3d> spline_pts;
	
	for (int i=0; i<num_sample_pts+1; i++)
	{
		double u = (double)i/(double)num_sample_pts;
		
		spline_pts.emplace_back(_wpSpline(u));
	}
	
	std::cout << "First Point: \n" << spline_pts[0] << "\n";
	std::cout << "Halfway Point: \n" << spline_pts[500] << "\n";
	std::cout << "End Point: \n" << spline_pts[1000] << "\n";
	
	//! Try to draw the spline using line markers in rviz:
	//! Publish the waypoint path as line_strip markers in rviz:
	//! Publish points of first gate:		
	visualization_msgs::Marker waypoints;
	waypoints.header.frame_id = "world";
	waypoints.header.stamp = ros::Time::now();
	waypoints.id = 200;
	waypoints.type = visualization_msgs::Marker::POINTS;
	waypoints.action = visualization_msgs::Marker::ADD;
	waypoints.scale.x = 0.1;
	waypoints.scale.y = 0.1;
	waypoints.scale.z = 0.1;
	waypoints.color.r = 1.0;
	waypoints.color.a = 1.0;
	
	for (int i=0; i<(int)spline_pts.size(); i++)
	{	
		waypoints.points.push_back(to_ros_point(spline_pts[i]));		
	}	
	rvizMarkerArray_.markers.push_back(waypoints);
	
	//! Integrate the spline:
	Scalar spline_length = SplineIntegration<Eigen::Spline3d::Dimension>::Integrate(_wpSpline, integrator_, (Scalar)0.0, (Scalar)1.0);
	
	_attitudeControl.SetSplineLength(spline_length);
	
	//! Test code from AttitudeControl:
	double u=0.1;	
	double curvature = 0.0;
	double max_tangent_speed = 0.0;
	bool ret = _attitudeControl.calculateMaximumTangentSpeed(u, curvature, max_tangent_speed);
	
	ROS_INFO("Computing curvature at u = %f: k = %f, maxSpeed = %f",
		u, curvature, max_tangent_speed);
	
}

void splineMaker::Run60HzLoop()
{
	//! Get the vehicle pose:
	_poseValid = _poseEstimator.getVehiclePose(_vehiclePose);
	
	
	
}

void splineMaker::Run5HzLoop()
{	
	ROS_INFO_THROTTLE(0.5, "Vehicle Position: %f %f %f, RPY: %f %f %f",
		_vehiclePose.p(0), _vehiclePose.p(1), _vehiclePose.p(2), 
		_vehiclePose.rpy.roll*RAD2DEG, _vehiclePose.rpy.pitch*RAD2DEG, _vehiclePose.rpy.yaw*RAD2DEG); 
	
	rvizMarkerPub_.publish(rvizMarkerArray_);
	
}



