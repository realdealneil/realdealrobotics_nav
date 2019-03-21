/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: Spline Path Generator
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */

#include <rdr_spline_path/rdr_spline_path.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

splineMaker::splineMaker(const rdr_spline_path::GateLocationList& gate_location_list)
		: nh_("~")
		, integrator_(200)
		, _poseEstimator(nh_)
		, gate_location_list_(to_eigen(gate_location_list)) 
{
  startTime_ms = ms_time();
  rvizMarkerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("gateCornerMarkers", 10);
  rateThrustPub_ = nh_.advertise<mav_msgs::RateThrust>("/uav/input/rateThrust", 10);
}

void splineMaker::print_gate_list()
{
  for(int gate_index = 1; gate_index <= gate_location_list_.gates.size(); gate_index++)
  {
    cout << "Gate " << gate_index << endl;

    const EigenGateLocation& egl = gate_location_list_.gates[gate_index - 1];
    cout << "corners:\n" << egl.corners  
      << "\n  perturbation bound: " << egl.perturbation_bound.transpose() 
      << "\n  center: " << egl.center.transpose()
      << "\n  normal: " << egl.normal.transpose()
      << "\n  front: " << egl.front_point.transpose()
      << "\n  back: " << egl.back_point.transpose() << endl;
  }
}

	

//! Construct waypoint list:
WaypointList splineMaker::constructWaypointList(
  const Vector3d& vehicle_start_position, 
  const GateList& gate_list)
{
	vector<Vector3d> wplist;
	
	start_position_ = vehicle_start_position;
  
  //! First, add the vehicle's start position:
	wplist.push_back(vehicle_start_position);
	
	/** For each gate, figure out which point (front or back) is closest 
	 * 	to the previous waypoint in the list.  Then, add the points:
	 */
	const Vector3d* prev_point = &vehicle_start_position;	//just to avoid copies...

	for (size_t i = 0; i < gate_list.size(); i++)
	{
		ROS_ASSERT(gate_list[i] <= gate_location_list_.gates.size());
		const EigenGateLocation& current_gate = gate_location_list_.gates[gate_list[i]-1];

		Vector3d v_prev_to_fp = current_gate.front_point - *prev_point;
		double dist_prev_to_fp = v_prev_to_fp.norm();
		
		Vector3d v_prev_to_bp = current_gate.back_point - *prev_point;
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
	cout << "Waypoint list is constructed!  There are " << gate_list.size() << 
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
	cout << "Making spline from waypoint list. There are " << wplist.size() << " waypoints\n";
	
	MatrixXd wpMat(3, wplist.size());
	for (int i=0; i<(int)wplist.size(); i++)
	{
		wpMat.col(i) = wplist.at(i);
	}
	
	//typedef Spline<double, 3> spline3d;
	
	_wpSpline = SplineFitting<Spline3d>::Interpolate(wpMat, 3);
	_attitudeControl.setSpline(_wpSpline); 
	//int dimension = 3;
	//Matrix<double, 3, 1> derivatives = s.derivatives(param, 1).col(1);
	
	//spline3d s = SplineFitting<spline3d>::InterpolateWithDerivatices(
	
	//! Get values from the spline at a variety of points along the spline and add to the rviz display:
	int num_sample_pts = 1000;
	vector<Vector3d> spline_pts;
	
	for (int i=0; i<num_sample_pts+1; i++)
	{
		double u = (double)i/(double)num_sample_pts;
		
		spline_pts.emplace_back(_wpSpline(u));
	}
	
	cout << "First Point: \n" << spline_pts[0] << "\n";
	cout << "Halfway Point: \n" << spline_pts[500] << "\n";
	cout << "End Point: \n" << spline_pts[1000] << "\n";
	
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
	Scalar spline_length = SplineIntegration<Spline3d::Dimension>::Integrate(
		_wpSpline, integrator_, (Scalar)0.0, (Scalar)1.0);
	
	_attitudeControl.setSplineLength(spline_length);
	
	//! Test code from AttitudeControl:
	double u=0.01;	
	double curvature = 0.0;
	double max_tangent_speed = _attitudeControl.calculateMaxTangentialSpeed(u, 5.0, 0.1);
	
	ROS_INFO("Computing curvature at u = %f: k = %f, maxSpeed = %f",
		u, _attitudeControl.getCurvature(), max_tangent_speed);
		
	/** Test the function for finding the closest point on the spline to 
	 *  our current position:
	 * 
	 *  Pretend that the vehicle moves a slight distance forward, along 
	 *  the spline, but not entirely on the spline.  
	 */
	Vector3d startPosition = _wpSpline(0.0);
	Vector3d newSplinePosition = _wpSpline(u);
	Vector3d newPosition = newSplinePosition + Vector3d(0.00, 0.1, 0.0);
	
	cout << "Start Position: " << startPosition.transpose() 
		<< "\n newSplinePos: " << newSplinePosition.transpose()
		<< "\n newPosition: " << newPosition.transpose() 
		<< "\n";
		
	double u_hat = _attitudeControl.findClosestParam(newPosition);
	
	cout << "u: " << u << " u_hat: " << u_hat << "\n";
	
	//! Get the lookahead u and the corresponding point in space:
	double u_carrot = _attitudeControl.findLookAheadParam(10.0);
	
	cout << "u_carrot: " << u_carrot << " Pos: " << Vector3d(_wpSpline(u_carrot)).transpose() << "\n";
	
	
	/** We're done testing functions...reset the spline so that we start
	 * 	at the beginning again:
	 */	
	_attitudeControl.setSpline(_wpSpline);	
}

mav_msgs::RateThrust splineMaker::GetCmdsToFollowSpline(void)
{
	double closestParam = _attitudeControl.findClosestParam(_vehiclePose.p);
	
	double lookaheadParam = _attitudeControl.findLookAheadParam(1.0);
	
	double currentSpeed = _vehiclePose.v.norm();
	
	const double desiredSpeed = 5.0;
	
	Eigen::Vector3d desiredAccelVector = _attitudeControl.calculateDesiredAccelVector(
		lookaheadParam, desiredSpeed, currentSpeed);
		
	Eigen::Matrix3d desiredBodyAxes = Eigen::Matrix3d::Zero();
	Eigen::Vector3d tangentVectorAtLookahead = _attitudeControl.calculateSplineDerivatives(lookaheadParam, 1);
	desiredBodyAxes.col(0) = tangentVectorAtLookahead.normalized();
	desiredBodyAxes.col(2) = desiredAccelVector.normalized();
	desiredBodyAxes.col(1) = desiredBodyAxes.row(2).cross(desiredBodyAxes.row(0));
	
	double throttle_cmd = desiredAccelVector.norm(); // This may not be right, but it's a start
		
	//! Convert the desired body axes to a quaternion
	Eigen::Quaterniond desiredBodyQuat(desiredBodyAxes);
	
	// TODO: Create functions in a library to do the following so we do it in a consistent manner:
	//! Convert desired attitude to Euler angles for debugging:
	//! Convert the transform quaternion to an Eigen Quaternion:
	tf::Quaternion q_desired_tf;
	tf::quaternionEigenToTF(desiredBodyQuat, q_desired_tf);
	
	//! Convert the quaternion to roll pitch yaw:
	RdrRpy desiredRpy;				
	tf::Matrix3x3(q_desired_tf).getEulerYPR(
			desiredRpy.yaw, desiredRpy.pitch, desiredRpy.roll);			
	
	//! Compute error quaternion (desired composed with actual)
	Eigen::Quaterniond error_quat = calc_quaternion_error(_vehiclePose.q,
		desiredBodyQuat);
	
	//! Convert to angle-axis
	Eigen::Vector3d error_aa = to_angle_axis(error_quat);
	
	//! Generate control outputs proportional to angle-axis error
	const double kp_rate_x=1.0;
	const double kp_rate_y=1.0;
	const double kp_rate_z=1.0;
	mav_msgs::RateThrust rateThrustMsg;
	rateThrustMsg.angular_rates.x = kp_rate_x * error_aa(0);
	rateThrustMsg.angular_rates.y = kp_rate_y * error_aa(1);
	rateThrustMsg.angular_rates.z = kp_rate_z * error_aa(2);
	rateThrustMsg.thrust.z = throttle_cmd;
	
	/*
	ROS_INFO_STREAM_THROTTLE(0.5, "\n"
		"lookaheadParam: " << lookaheadParam << "\n"
		"lookahead Pos:  " << Eigen::Vector3d(_wpSpline(lookaheadParam)).transpose() << "\n"
		"tangentVectorAtLookahead.normalized(): " << tangentVectorAtLookahead.normalized().transpose() << "\n"
		"desiredAccelVector.normalized(): " << desiredAccelVector.normalized().transpose() << "\n"
	
		"desiredBodyQuat.matrix().col(0):\n" << desiredBodyQuat.matrix().col(0) << "\n"
		"tangentVectorAtLookahead: \n" << tangentVectorAtLookahead.normalized() << "\n"
		"Dot product with tangent: \n" << (desiredBodyQuat.matrix().col(0)).dot(tangentVectorAtLookahead.normalized()) << "\n"
		"Dot product with inertial:\n" << (desiredBodyQuat.matrix().col(2)).dot(desiredAccelVector.normalized()) << "\n"
		);
	*/
	/*
	ROS_INFO_STREAM_THROTTLE(0.5,  
		"Main Computations: \n"
		"  Current Speed: " << currentSpeed << "\n"
		"  Throttle Cmd: " << throttle_cmd << "\n"
		"  desiredAccelVector: " << desiredAccelVector.transpose() << "\n"
		"  tangentVectorAtLookAhead: " << tangentVectorAtLookahead.transpose() << "\n"
		"  Desired Attitude: " << desiredRpy.roll*RAD2DEG << " " 
			<< desiredRpy.pitch*RAD2DEG << " " <<  desiredRpy.yaw*RAD2DEG << "\n");
	*/
	/*
	ROS_INFO_THROTTLE(0.5, "Commands: %f %f %f Thrust: %f", 
		rateThrustMsg.angular_rates.x,
		rateThrustMsg.angular_rates.y,
		rateThrustMsg.angular_rates.z,
		rateThrustMsg.thrust.z);
	*/
	
	return rateThrustMsg;
}

mav_msgs::RateThrust splineMaker::TakeOffMode(void)
{
	
	double takeoff_alt = 1.0;
	
	Eigen::Vector3d desiredPosition = start_position_ + Eigen::Vector3d(0, 0, takeoff_alt);
	
	Eigen::Vector3d positionError = desiredPosition - _vehiclePose.p;
	
	//! Try to achieve a desired velocity:
	
	double desiredClimbRate = 1.0 * positionError(2);
	
	double actualClimbRate = _vehiclePose.v(2);
	
	double climbRateError = desiredClimbRate - actualClimbRate;
	
	//! Command acceleration proportional to position error:
	mav_msgs::RateThrust takeoffCmd;
	takeoffCmd.thrust.z = 2.0 * climbRateError + GRAVITY;
	
	//! Now, try to achieve an attitude that is straight up, facing x=0:
	Eigen::Matrix3d desiredBodyAxes = Eigen::Matrix3d::Zero();
	//Eigen::Vector3d tangentVectorAtLookahead = _attitudeControl.calculateSplineDerivatives(lookaheadParam, 1);
	//desiredBodyAxes.col(0) = Eigen::Vector3d(1.0, 0.0, 0.0).normalized();
	//desiredBodyAxes.col(2) = Eigen::Vector3d(0.0, 0.0, 1.0);
	//desiredBodyAxes.col(1) = desiredBodyAxes.row(2).cross(desiredBodyAxes.row(0));
	
	//! Convert the desired body axes to a quaternion
	//Eigen::Quaterniond desiredBodyQuat(desiredBodyAxes);
	Eigen::Quaterniond desiredBodyQuat = EulerToQuat(0.0, 0.0*DEG2RAD, 90.0*DEG2RAD);
	
	// TODO: Create functions in a library to do the following so we do it in a consistent manner:
	//! Convert desired attitude to Euler angles for debugging:
	//! Convert the transform quaternion to an Eigen Quaternion:
	tf::Quaternion q_desired_tf;
	tf::quaternionEigenToTF(desiredBodyQuat, q_desired_tf);
	
	//! Convert the quaternion to roll pitch yaw:
	RdrRpy desiredRpy;				
	tf::Matrix3x3(q_desired_tf).getEulerYPR(
			desiredRpy.yaw, desiredRpy.pitch, desiredRpy.roll);			
	
	//! Compute error quaternion (desired composed with actual)
	Eigen::Quaterniond error_quat = calc_quaternion_error(_vehiclePose.q,
		desiredBodyQuat);
	
	//! Convert to angle-axis
	Eigen::Vector3d error_aa = to_angle_axis(error_quat);
	
	//! Generate control outputs proportional to angle-axis error
	const double kp_rate_x=5.0;
	const double kp_rate_y=5.0;
	const double kp_rate_z=5.0;
	mav_msgs::RateThrust rateThrustMsg;
	takeoffCmd.angular_rates.x = kp_rate_x * error_aa(0);
	takeoffCmd.angular_rates.y = kp_rate_y * error_aa(1);
	takeoffCmd.angular_rates.z = kp_rate_z * error_aa(2);	
	
	//TODO: compute desired attitude to achieve hover:
	ROS_INFO_THROTTLE(0.25, "\n"
		"desiredClimbRate: %f, Error: %f cmd: %f\n"
		"error_aa: %f %f %f\n"
		"cmds: %f %f %f\n"
		,
		desiredClimbRate, climbRateError, takeoffCmd.thrust.z,
		error_aa(0), error_aa(1), error_aa(2),
		takeoffCmd.angular_rates.x, takeoffCmd.angular_rates.y, takeoffCmd.angular_rates.z		
		);
		
	return takeoffCmd;	
}

void splineMaker::Run60HzLoop()
{
	//! Get the vehicle pose:
	_poseValid = _poseEstimator.getVehiclePose(_vehiclePose);
	
	mav_msgs::RateThrust cmds;
	cmds.angular_rates.x = 0.0;
	cmds.angular_rates.y = 0.0;
	cmds.angular_rates.z = 0.0;
	cmds.thrust.z = GRAVITY;
	
	switch(flightMode_)
	{
		default:
		case MODE_IDLE:
			break;
		case MODE_TAKEOFF:
			cmds = TakeOffMode();
			break;
		case MODE_FOLLOW_SPLINE:
			cmds = GetCmdsToFollowSpline();
			break;
		case MODE_HOVER:
			break;
	}	
	
	rateThrustPub_.publish(cmds);
}

void splineMaker::Run5HzLoop()
{	
	ROS_INFO_THROTTLE(0.5, "Vehicle Position: %f %f %f, RPY: %f %f %f",
		_vehiclePose.p(0), _vehiclePose.p(1), _vehiclePose.p(2), 
		_vehiclePose.rpy.roll*RAD2DEG, _vehiclePose.rpy.pitch*RAD2DEG, _vehiclePose.rpy.yaw*RAD2DEG); 
	
	rvizMarkerPub_.publish(rvizMarkerArray_);
	
}



