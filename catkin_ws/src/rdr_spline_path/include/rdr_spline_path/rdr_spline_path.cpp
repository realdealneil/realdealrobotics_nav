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
{
	//LoadParams();
	
	_gateCornerPub = _nh.advertise<visualization_msgs::MarkerArray>("gateCornerMarkers", 10);
	
	//! Get the list of gate corners:
	_corner_vec = getGateCornerList();
	_center_vec = getGateCenters(_corner_vec);
	_gate_normals_vec = getGateNormals();
	constructFrontPoints(3.0, _gate_front_points, _gate_back_points);
	
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
	gc.ur = Eigen::Vector3d(16.35111, 3.63155, 7.229498);
	gc.lr = Eigen::Vector3d(18.15111, 3.631447, 5.383497);
	gc.ll = Eigen::Vector3d(16.35111, 3.63155, 5.383497);
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

void splineMaker::CreateCornerMarkersForPublishing()
{
	//! Publish points of first gate:		
	for (int i=0; i<(int)_corner_vec.size(); i++)
	{
		visualization_msgs::Marker gate;
		gate.header.frame_id = "world";
		gate.header.stamp = ros::Time::now();
		gate.id = i;
		gate.type = visualization_msgs::Marker::POINTS;
		gate.action = visualization_msgs::Marker::ADD;
		gate.scale.x = 0.5;
		gate.scale.y = 0.5;
		gate.scale.z = 0.5;
		gate.color.g = 1.0;
		gate.color.a = 1.0;
		geometry_msgs::Point p;
		p.x = _corner_vec[i].ul(0);
		p.y = _corner_vec[i].ul(1);
		p.z = _corner_vec[i].ul(2);
		gate.points.push_back(p);
		p.x = _corner_vec[i].ur(0);
		p.y = _corner_vec[i].ur(1);
		p.z = _corner_vec[i].ur(2);
		gate.points.push_back(p);
		p.x = _corner_vec[i].lr(0);
		p.y = _corner_vec[i].lr(1);
		p.z = _corner_vec[i].lr(2);
		gate.points.push_back(p);
		p.x = _corner_vec[i].ll(0);
		p.y = _corner_vec[i].ll(1);
		p.z = _corner_vec[i].ll(2);	
		gate.points.push_back(p);
		// Center point:
		p.x = _center_vec[i](0);
		p.y = _center_vec[i](1);
		p.z = _center_vec[i](2);
		gate.points.push_back(p);		
		// Front point:
		p.x = _gate_front_points[i](0);
		p.y = _gate_front_points[i](1);
		p.z = _gate_front_points[i](2);
		gate.points.push_back(p);
		// back point:
		p.x = _gate_back_points[i](0);
		p.y = _gate_back_points[i](1);
		p.z = _gate_back_points[i](2);
		gate.points.push_back(p);		
		
		_gateCornerMarkerArray.markers.push_back(gate);
	}	
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


