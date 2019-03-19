#include "import_helpers.h"

const std::string ns{"/uav"};

GateLocationList importGateLocations(ros::NodeHandle &n) 
{
  XmlRpc::XmlRpcValue cornerlist;
  XmlRpc::XmlRpcValue perturb;

  GateLocationList gll;

  const std::string gate_location_key_name{"nominal_location"};
  const std::string perturbation_key_name{"perturbation_bound"};

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
    }

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

GateList importGateList(ros::NodeHandle& n)
{
  GateList gl;

  std::string gate_list_param{ns + "/gate_names"};

  if (!n.hasParam(gate_list_param.c_str()))
  {
    return {};
  }
  std::vector<std::string> gate_names;
  n.getParam(gate_list_param.c_str(), gate_names);

  ROS_INFO("Importing %lu gate names", gate_names.size());

  for(auto& gate_name : gate_names)
  {
    //erase first 4 chars and then convert to unsigned long
    size_t gate_num = std::stoul(gate_name.erase(0,4));
    gl.push_back(gate_num);
    ROS_INFO("Added gate %lu", gate_num);
  }
  return gl;
}

geometry_msgs::Pose get_initial_pose(ros::NodeHandle& n)
{
  std::string init_pose_param{ns + "/flightgoggles_uav_dynamics/init_pose"};

  if (!n.hasParam(init_pose_param.c_str()))
  {
    ROS_ERROR("Initial pose param: %s not found", init_pose_param.c_str());
    return {};
  }
  std::vector<double> init_pose_vals;
  n.getParam(init_pose_param.c_str(), init_pose_vals);

  ROS_ASSERT(init_pose_vals.size() == 7);

  geometry_msgs::Pose pose;
  pose.position.x = init_pose_vals[0];
  pose.position.y = init_pose_vals[1];
  pose.position.z = init_pose_vals[2];
  pose.orientation.x = init_pose_vals[3];
  pose.orientation.y = init_pose_vals[4];
  pose.orientation.z = init_pose_vals[5];
  pose.orientation.w = init_pose_vals[6];

  return pose;
}
