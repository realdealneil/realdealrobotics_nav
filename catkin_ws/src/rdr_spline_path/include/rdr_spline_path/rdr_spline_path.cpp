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
		: gate_location_list_(gate_location_list) 
{
	
//	_gateCornerPub = _nh.advertise<visualization_msgs::MarkerArray>("gateCornerMarkers", 10);
	
	//! Get the list of gate corners:

//	CreateCornerMarkersForPublishing();
  //_gate_normals_vec = getGateNormals();
}

std::vector<Eigen::Vector3d>  splineMaker::sampleWaypointGenerator(const std::vector<size_t>& gate_list)
{
  std::cout << "TODO: make spline with these gates:" << std::endl;

  for (auto& gate_index : gate_list)
  {
    if (gate_index - 1 > gate_location_list_.gates.size() )
    {
      std::cout << "Invalid waypoint index " << gate_index << std::endl;
      return {};
    }
    const rdr_spline_path::GateLocation& gl = gate_location_list_.gates[gate_index - 1];

    std::cout << "Gate " << gate_index 
      << ", center: [" << gl.center.x << ", " << gl.center.y << ", " 
      << gl.center.z << "]" << std::endl;
  }


	Eigen::Vector3d point;
	std::vector<Eigen::Vector3d> points;
	
	point << 0, 0, 0;
	points.push_back(point);
	
	return points;
  
}

void splineMaker::print_gate_list()
{
  for(int gate_index = 0; gate_index < gate_location_list_.gates.size(); gate_index++)
  {
    std::cout << "Gate " << gate_index + 1 << std::endl;

    const rdr_spline_path::GateLocation& gl = gate_location_list_.gates[gate_index];
    for (int corner_index = 0; corner_index < gl.corners.size(); corner_index++)
    {
      std::cout << "  [" << gl.corners[corner_index].x << ", "
        << gl.corners[corner_index].y << ", "
        << gl.corners[corner_index].z << "]" << std::endl;
    }
    std::cout << "  center: [" << gl.center.x 
      << ", " << gl.center.y
      << ", " << gl.center.z << "]" << std::endl; 

    std::cout << "  perturbation bound: " << gl.perturbation_bound.x 
      << ", " << gl.perturbation_bound.y
      << ", " << gl.perturbation_bound.z << "]" << std::endl;
  }
}


// vector<Eigen::Vector3d> splineMaker::getGateNormals()
// {
// 	//! For each gate, use the center and two corner points to figure out the normal to the plane:	
// 	vector<Eigen::Vector3d> normals;
// 	for (int i=0; i<(int)_center_vec.size(); i++)
// 	{
// 		Eigen::Vector3d normal;
// 		Eigen::Vector3d Vec1 = _corner_vec[i].ul - _center_vec[i];
// 		Eigen::Vector3d Vec2 = _corner_vec[i].ur - _center_vec[i];
// 		normal = Vec1.cross(Vec2);		
// 		normals.push_back(normal);
// 	}
// 	return normals;
// }

// void splineMaker::CreateCornerMarkersForPublishing()
// {
// 	//! Publish points of first gate:		
// 	for (int i=0; i<(int)_corner_vec.size(); i++)
// 	{
// 		visualization_msgs::Marker gate;
// 		gate.header.frame_id = "world";
// 		gate.header.stamp = ros::Time::now();
// 		gate.id = i;
// 		gate.type = visualization_msgs::Marker::POINTS;
// 		gate.action = visualization_msgs::Marker::ADD;
// 		gate.scale.x = 0.5;
// 		gate.scale.y = 0.5;
// 		gate.scale.z = 0.5;
// 		gate.color.g = 1.0;
// 		gate.color.a = 1.0;
// 		geometry_msgs::Point p;
// 		p.x = _corner_vec[i].ul(0);
// 		p.y = _corner_vec[i].ul(1);
// 		p.z = _corner_vec[i].ul(2);
// 		gate.points.push_back(p);
// 		p.x = _corner_vec[i].ur(0);
// 		p.y = _corner_vec[i].ur(1);
// 		p.z = _corner_vec[i].ur(2);
// 		gate.points.push_back(p);
// 		p.x = _corner_vec[i].lr(0);
// 		p.y = _corner_vec[i].lr(1);
// 		p.z = _corner_vec[i].lr(2);
// 		gate.points.push_back(p);
// 		p.x = _corner_vec[i].ll(0);
// 		p.y = _corner_vec[i].ll(1);
// 		p.z = _corner_vec[i].ll(2);	
// 		gate.points.push_back(p);
// 		// Center point:
// 		p.x = _center_vec[i](0);
// 		p.y = _center_vec[i](1);
// 		p.z = _center_vec[i](2);
// 		gate.points.push_back(p);
		
// 		_gateCornerMarkerArray.markers.push_back(gate);
// 	}
	
// }

// void splineMaker::Update()
// {
// 	_gateCornerPub.publish(_gateCornerMarkerArray);
// }



