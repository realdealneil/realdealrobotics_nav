#include <ros/ros.h>
#include <rdr_spline_path/rdr_spline_path.h>
#include <rdr_spline_path/GateLocationList.h>

using namespace rdr_spline_path;

GateLocationList importGateLocations(ros::NodeHandle &n);

GateList importGateList(ros::NodeHandle& n);

geometry_msgs::Pose get_initial_pose(ros::NodeHandle& n);
