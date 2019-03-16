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

splineMaker::splineMaker()
		: _nh("~")
		, _integrator(200)
{
	//LoadParams();
	
	_gateCornerPub = _nh.advertise<visualization_msgs::MarkerArray>("gateCornerMarkers", 10);
	
	//! Get the list of gate corners:
	_corner_vec = getGateCornerList();
	_center_vec = getGateCenters(_corner_vec);
	_gate_normals_vec = getGateNormals();
	constructFrontPoints(3.0, _gate_front_points, _gate_back_points);
	_primaryWaypoints = constructWaypointList();
	
	//! Make a spline!
	MakeSplineFromWaypoints(_primaryWaypoints);
	
	CreateCornerMarkersForPublishing();
}

vector<gate_corners> splineMaker::getGateCornerList(void)
{
	// Here's the order: ['Gate10', 'Gate21', 'Gate2', 'Gate13', 'Gate9', 'Gate14', 'Gate1', 'Gate22', 'Gate15', 'Gate23', 'Gate6']
	gate_corners gc;
	vector<gate_corners> corner_vec;
	
	// Gate 10: [[18.15111, 3.631447, 7.229498], [16.35111, 3.63155, 7.229498], 
	// [18.15111, 3.631447, 5.383497], [16.35111, 3.63155, 5.383497]]
	gc.ul = Eigen::Vector3d(18.15111, 3.631447, 7.229498);
	gc.ur = Eigen::Vector3d(18.15111, 3.631447, 5.383497);
	gc.lr = Eigen::Vector3d(16.35111, 3.63155, 5.383497);
	gc.ll = Eigen::Vector3d(16.35111, 3.63155, 7.229498);
	corner_vec.push_back(gc);
	
	// gate 21: [[15.15353, 37.63426, 8.376249], [18.43008, 41.25023, 8.376249], [18.43008, 41.25023, 5.14875], [15.15353, 37.63426, 5.14875]]
	gc.ul = Eigen::Vector3d(15.15353, 37.63426, 8.376249);
	gc.ur = Eigen::Vector3d(18.43008, 41.25023, 8.376249);
	gc.lr = Eigen::Vector3d(18.43008, 41.25023, 5.14875);
	gc.ll = Eigen::Vector3d(15.15353, 37.63426, 5.14875);
	corner_vec.push_back(gc);
	
	// gate 2: [[-0.2551794, 27.86797, 4.16025], [4.433571, 27.86797, 4.16025], [4.433571, 27.86797, 0.9327497], [-0.2551794, 27.86797, 0.9327497]]
	gc.ul = Eigen::Vector3d(-0.2551794, 27.86797, 4.16025);
	gc.ur = Eigen::Vector3d(4.433571, 27.86797, 4.16025);
	gc.lr = Eigen::Vector3d(4.433571, 27.86797, 0.9327497);
	gc.ll = Eigen::Vector3d(-0.2551794, 27.86797, 0.9327497);
	corner_vec.push_back(gc);
	
	// gate 13: [[1.237332, 9.001728, 2.9625], [3.162332, 9.001728, 2.9625], [3.162332, 9.001728, 1.025], [1.237332, 9.001728, 1.025]]
	gc.ul = Eigen::Vector3d(1.237332, 9.001728, 2.9625);
	gc.ur = Eigen::Vector3d(3.162332, 9.001728, 2.9625);
	gc.lr = Eigen::Vector3d(3.162332, 9.001728, 1.025);
	gc.ll = Eigen::Vector3d(1.237332, 9.001728, 1.025);
	corner_vec.push_back(gc);

	// gate 9: [[-6.40867, -12.13678, 4.152941], [-8.208672, -12.13678, 4.152941], [-6.408669, -12.13678, 2.306941], [-8.208672, -12.13678, 2.306941]]
	gc.ul = Eigen::Vector3d(-6.40867, -12.13678, 4.152941);
	gc.ur = Eigen::Vector3d(-8.208672, -12.13678, 4.152941);
	gc.lr = Eigen::Vector3d(-8.208672, -12.13678, 2.306941);
	gc.ll = Eigen::Vector3d(-6.408669, -12.13678, 2.306941);
	corner_vec.push_back(gc);

	// gate 14: [[-10.4501, -31.31195, 4.25625], [-6.545984, -27.40784, 4.25625], [-6.545984, -27.40784, 1.02875], [-10.4501, -31.31195, 1.02875]]
	gc.ul = Eigen::Vector3d(-10.4501, -31.31195, 4.25625);
	gc.ur = Eigen::Vector3d(-6.545984, -27.40784, 4.25625);
	gc.lr = Eigen::Vector3d(-6.545984, -27.40784, 1.02875);
	gc.ll = Eigen::Vector3d(-10.4501, -31.31195, 1.02875);
	corner_vec.push_back(gc);

	// gate 1: [[-0.009000421, -32.9505, 3.071861], [-0.009000659, -34.8755, 3.071861], [-0.009000659, -34.8755, 1.134362], [-0.009000421, -32.9505, 1.134362]]
	gc.ul = Eigen::Vector3d(-0.009000421, -32.9505, 3.071861);
	gc.ur = Eigen::Vector3d(-0.009000659, -34.8755, 3.071861);
	gc.lr = Eigen::Vector3d(-0.009000659, -34.8755, 1.134362);
	gc.ll = Eigen::Vector3d(-0.009000421, -32.9505, 1.134362);
	corner_vec.push_back(gc);

	// gate 22: [[4.798321, -27.37053, 4.18125], [8.70332, -31.27553, 4.18125], [8.70332, -31.27553, 0.9537499], [4.798321, -27.37053, 0.9537499]]
	gc.ul = Eigen::Vector3d(4.798321, -27.37053, 4.18125);
	gc.ur = Eigen::Vector3d(8.70332, -31.27553, 4.18125);
	gc.lr = Eigen::Vector3d(8.70332, -31.27553, 0.9537499);
	gc.ll = Eigen::Vector3d(4.798321, -27.37053, 0.9537499);
	corner_vec.push_back(gc);

	// gate 15: [[5.744822, -11.79203, 4.16025], [8.75482, -11.79203, 4.16025], [8.75482, -11.79203, 0.9327497], [5.744822, -11.79203, 0.9327497]]
	gc.ul = Eigen::Vector3d(5.744822, -11.79203, 4.16025);
	gc.ur = Eigen::Vector3d(8.75482, -11.79203, 4.16025);
	gc.lr = Eigen::Vector3d(8.75482, -11.79203, 0.9327497);
	gc.ll = Eigen::Vector3d(5.744822, -11.79203, 0.9327497);
	corner_vec.push_back(gc);

	// gate 23: [[-9.328671, 7.773174, 2.942941], [-11.12867, 7.773277, 2.942941], [-9.328669, 7.773174, 1.096941], [-11.12867, 7.773277, 1.096941]]
	gc.ul = Eigen::Vector3d(-9.328671, 7.773174, 2.942941);
	gc.ur = Eigen::Vector3d(-11.12867, 7.773277, 2.942941);
	gc.lr = Eigen::Vector3d(-11.12867, 7.773277, 1.096941);
	gc.ll = Eigen::Vector3d(-9.328669, 7.773174, 1.096941);
	corner_vec.push_back(gc);

	// gate 6: [[-9.14867, 30.62316, 3.820941], [-10.94867, 30.62329, 3.820941], [-9.148668, 30.62316, 1.974941], [-10.94867, 30.62329, 1.974941]]
	gc.ul = Eigen::Vector3d(-9.14867, 30.62316, 3.820941);
	gc.ur = Eigen::Vector3d(-10.94867, 30.62329, 3.820941);
	gc.lr = Eigen::Vector3d(-10.94867, 30.62329, 1.974941);
	gc.ll = Eigen::Vector3d(-9.148668, 30.62316, 1.974941);
	corner_vec.push_back(gc);
	
	return corner_vec;
}

Eigen::Vector3d splineMaker::find_center(gate_corners c)
{
	double avg_x = (c.ul(0) + c.ur(0) + c.lr(0) + c.ll(0))/4.0;
	double avg_y = (c.ul(1) + c.ur(1) + c.lr(1) + c.ll(1))/4.0;
	double avg_z = (c.ul(2) + c.ur(2) + c.lr(2) + c.ll(2))/4.0;
	return Eigen::Vector3d(avg_x, avg_y, avg_z);
}	

vector<Eigen::Vector3d> splineMaker::getGateCenters(std::vector<gate_corners> corners)
{
	vector<Eigen::Vector3d> centers;
	
	for (int i=0; i<(int)corners.size(); i++)
	{
		Eigen::Vector3d center = find_center(corners.at(i));
		centers.push_back(center);
	}
	return centers;
}

vector<Eigen::Vector3d> splineMaker::getGateNormals()
{
	//! For each gate, use the center and two corner points to figure out the normal to the plane:	
	vector<Eigen::Vector3d> normals;
	for (int i=0; i<(int)_center_vec.size(); i++)
	{
		Eigen::Vector3d normal;
		Eigen::Vector3d Vec1 = _corner_vec[i].ul - _center_vec[i];
		Eigen::Vector3d Vec2 = _corner_vec[i].ur - _center_vec[i];
		normal = Vec1.cross(Vec2);	
		normal.normalize();	
		normals.push_back(normal);
	}
	return normals;
}

void splineMaker::constructFrontPoints(double d, vector<Eigen::Vector3d>& frontPoints, vector<Eigen::Vector3d>& backPoints)
{
	//! Make the front points: 
	for (int i=0; i<(int)_center_vec.size(); i++)
	{
		Eigen::Vector3d fp = _center_vec[i] + d * _gate_normals_vec[i];
		Eigen::Vector3d bp = _center_vec[i] - d * _gate_normals_vec[i];
		frontPoints.push_back(fp);
		backPoints.push_back(bp);
	}
}

//! Construct waypoint list:
std::vector<Eigen::Vector3d> splineMaker::constructWaypointList()
{
	//! First, add the vehicle's start position:
	std::vector<Eigen::Vector3d> wplist;
	
	//! Vehicle start location:
	Eigen::Vector3d vehicle_start_point(0.0, 0, 0);	//! From challenge_hard.yaml file
	wplist.push_back(vehicle_start_point);
	
	/** For each gate, figure out which point (front or back) is closest 
	 * 	to the previous waypoint in the list.  Then, add the points:
	 */
	Eigen::Vector3d prev_point = vehicle_start_point;	
	for (int i=0; i<(int)_center_vec.size(); i++)
	{
		Eigen::Vector3d v_prev_to_fp = _gate_front_points[i] - prev_point;
		double dist_prev_to_fp = v_prev_to_fp.norm();
		
		Eigen::Vector3d v_prev_to_bp = _gate_back_points[i] - prev_point;
		double dist_prev_to_bp = v_prev_to_bp.norm();
		
		if (dist_prev_to_fp < dist_prev_to_bp)
		{
			wplist.push_back(_gate_front_points[i]);
			wplist.push_back(_center_vec[i]);
			wplist.push_back(_gate_back_points[i]);
			prev_point = _gate_back_points[i];
		} else {
			wplist.push_back(_gate_back_points[i]);
			wplist.push_back(_center_vec[i]);
			wplist.push_back(_gate_front_points[i]);
			prev_point = _gate_front_points[i];
		}
	}
	
	//! Ok, we should have the waypoint list constructed now!
	cout << "Waypoint list is constructed!  There are " << _center_vec.size() << 
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
	_gateCornerMarkerArray.markers.push_back(waypoints);
	
	return wplist;
}

void splineMaker::CreateCornerMarkersForPublishing()
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
	
	
	
	for (int i=0; i<(int)_corner_vec.size(); i++)
	{		
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
		geometry_msgs::Point p;
		p.x = _corner_vec[i].ul(0);
		p.y = _corner_vec[i].ul(1);
		p.z = _corner_vec[i].ul(2);
		gate_pts.points.push_back(p);
		gate_lines.points.push_back(p);
		p.x = _corner_vec[i].ur(0);
		p.y = _corner_vec[i].ur(1);
		p.z = _corner_vec[i].ur(2);
		gate_pts.points.push_back(p);
		gate_lines.points.push_back(p);
		p.x = _corner_vec[i].lr(0);
		p.y = _corner_vec[i].lr(1);
		p.z = _corner_vec[i].lr(2);
		gate_pts.points.push_back(p);
		gate_lines.points.push_back(p);
		p.x = _corner_vec[i].ll(0);
		p.y = _corner_vec[i].ll(1);
		p.z = _corner_vec[i].ll(2);	
		gate_pts.points.push_back(p);
		gate_lines.points.push_back(p);
		p.x = _corner_vec[i].ul(0);
		p.y = _corner_vec[i].ul(1);
		p.z = _corner_vec[i].ul(2);
		gate_lines.points.push_back(p);
		// Center point:
		p.x = _center_vec[i](0);
		p.y = _center_vec[i](1);
		p.z = _center_vec[i](2);
		gate_pts.points.push_back(p);		
		// Front point:
		p.x = _gate_front_points[i](0);
		p.y = _gate_front_points[i](1);
		p.z = _gate_front_points[i](2);
		gate_pts.points.push_back(p);
		// back point:
		p.x = _gate_back_points[i](0);
		p.y = _gate_back_points[i](1);
		p.z = _gate_back_points[i](2);
		gate_pts.points.push_back(p);		
		
		_gateCornerMarkerArray.markers.push_back(gate_pts);
		_gateCornerMarkerArray.markers.push_back(gate_lines);		
	}	
}

void splineMaker::MakeSplineFromWaypoints(const std::vector<Eigen::Vector3d>& wplist)
{
	cout << "Making spline from waypoint list. There are " << wplist.size() << " waypoints\n";
	
	Eigen::MatrixXd wpMat(3, wplist.size());
	for (int i=0; i<(int)wplist.size(); i++)
	{
		wpMat.col(i) = wplist.at(i);
	}
	
	//typedef Eigen::Spline<double, 3> spline3d;
	
	Eigen::Spline3d s = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(wpMat, 3);
	//int dimension = 3;
	//Eigen::Matrix<double, 3, 1> derivatives = s.derivatives(param, 1).col(1);
	
	//spline3d s = Eigen::SplineFitting<spline3d>::InterpolateWithDerivatices(
	
	//! Get values from the spline at a variety of points along the spline and add to the rviz display:
	int num_sample_pts = 1000;
	vector<Eigen::Vector3d> spline_pts;
	
	for (int i=0; i<num_sample_pts+1; i++)
	{
		double u = (double)i/(double)num_sample_pts;
		
		spline_pts.emplace_back(s(u));
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
	waypoints.type = visualization_msgs::Marker::LINE_STRIP;
	waypoints.action = visualization_msgs::Marker::ADD;
	waypoints.scale.x = 0.1;
	waypoints.scale.y = 0.1;
	waypoints.scale.z = 0.1;
	waypoints.color.r = 1.0;
	waypoints.color.a = 1.0;
	
	for (int i=0; i<(int)spline_pts.size(); i++)
	{	
		geometry_msgs::Point p;
		p.x = spline_pts[i](0);
		p.y = spline_pts[i](1);
		p.z = spline_pts[i](2);
		waypoints.points.push_back(p);		
	}	
	_gateCornerMarkerArray.markers.push_back(waypoints);
	
	//! Integrate the spline:
	Scalar spline_length = SplineIntegration<Eigen::Spline3d::Dimension>::Integrate(s, _integrator, (Scalar)0.0, (Scalar)1.0);
	
	cout << "Spline length is: " << spline_length << "\n";
	
}

void splineMaker::Update()
{
	_gateCornerPub.publish(_gateCornerMarkerArray);
	
}

void splineMaker::LoadParams()
{
	std::cout << "LoadParams\n";
	std::vector<double> temp12(12);
	
	_nh.getParam("/rdr_spline_path/Gate1/nominal_location", temp12);
	
	for (int i=0; i<12; i++)
	{
		std::cout << temp12[i] << "\n";
	}
}


