/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: Spline Path Generator
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */

#include <ros/ros.h>

#include <iostream>
#include <cstdio>
#include <cmath>
#include <rdr_spline_path/rdr_spline_path.h>
#include <rdr_spline_path/GateLocationList.h>

using namespace rdr_spline_path;
using namespace std;

GateLocationList importGateLocations(ros::NodeHandle &n) 
{
  XmlRpc::XmlRpcValue cornerlist;
  XmlRpc::XmlRpcValue perturb;

  GateLocationList gll;

  const std::string& ns{"rdr_spline_path"}; 
  const std::string& gate_location_key_name {"nominal_location"};
  const std::string& perturbation_key_name {"perturbation_bound"};

  unsigned gate_index = 0;
  while(1)
  {
    gate_index++;
    std::string location_param_name = std::string(ns + "/Gate" + 
      std::to_string(gate_index) + "/" + gate_location_key_name);
    std::string perturbation_param_name = std::string(ns + "/Gate" + 
      std::to_string(gate_index) + "/" + perturbation_key_name);

    ROS_INFO("Looking for param name: %s", location_param_name.c_str());
    if (!n.hasParam(location_param_name.c_str()))
    {
      ROS_INFO("Param: %s not found. Not looking for more gates.", 
        location_param_name.c_str());
      fflush(stdout);  
      break;
    }

    GateLocation gl;
    gl.center.x = 0;
    gl.center.y = 0;
    gl.center.z = 0;
    n.getParam(location_param_name.c_str(), cornerlist);
    for (int corner_index = 0; corner_index < cornerlist.size(); corner_index++)
    {
      ROS_ASSERT(cornerlist[corner_index].size() == 3);
      geometry_msgs::Point point;  
      point.x = cornerlist[corner_index][0];
      point.y = cornerlist[corner_index][1];
      point.z = cornerlist[corner_index][2];  
      gl.corners.emplace_back(point);
      ROS_INFO("adding corner %d: [%.6f, %.6f, %.6f]", corner_index, 
        point.x, point.y, point.z);
      gl.center.x += point.x;
      gl.center.y += point.y;
      gl.center.z += point.z;
    }
    gl.center.x /= cornerlist.size();
    gl.center.y /= cornerlist.size();
    gl.center.z /= cornerlist.size();
    ROS_INFO("Gate %d center is [%.3f, %.3f, %.3f]", gate_index, 
      gl.center.x, gl.center.y, gl.center.z);

    if (n.hasParam(perturbation_param_name.c_str()))
    {
      n.getParam(perturbation_param_name.c_str(), perturb);
      gl.perturbation_bound.x = perturb[0];
      gl.perturbation_bound.y = perturb[1];
      gl.perturbation_bound.z = perturb[2];
      ROS_INFO("add perturbation bound: [%.3f, %.3f, %.3f]",
        gl.perturbation_bound.x, gl.perturbation_bound.y, 
        gl.perturbation_bound.z);  
    }
    gll.gates.emplace_back(gl);
  }
  return gll;
}



int main(int argc, char ** argv)
{
	ros::init(argc, argv, "rdr_spline_path");
	ros::NodeHandle n;

  GateLocationList gate_locations = importGateLocations(n);
	splineMaker mySplineMaker{gate_locations};

  // Here's the order: ['Gate10', 'Gate21', 'Gate2', 'Gate13', 'Gate9', 'Gate14', 'Gate1', 'Gate22', 'Gate15', 'Gate23', 'Gate6']
  std::vector<size_t> gate_list{10,21,2,13,9,14,1,22,15,23,6};
  
  mySplineMaker.sampleWaypointGenerator(gate_list);

	ros::Rate loop_rate(5);
	while (ros::ok())
	{
		//mySplineMaker.Update();
		ros::spinOnce();
		loop_rate.sleep();
	}
	

	return 0;
}
