#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rdr_spline_path/GateLocationList.h>


struct EigenGateLocation
{
  Eigen::Matrix<double,3,4> corners;
  Eigen::Vector3d perturbation_bound;
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  Eigen::Vector3d front_point;
  Eigen::Vector3d back_point;
  // TODO: Add Normal, and front and back points.
};

struct EigenGateLocationList
{
  std::vector<EigenGateLocation> gates;
};

Eigen::Vector3d to_eigen(const geometry_msgs::Point&);

EigenGateLocation to_eigen(const rdr_spline_path::GateLocation&);

EigenGateLocationList to_eigen(const rdr_spline_path::GateLocationList&);

geometry_msgs::Point to_ros_point(const Eigen::Vector3d&);

//geometry_msgs::Vector3 to_ros_vector(const Eigen::Vector3d&);

Eigen::Vector3d calc_gate_center(const Eigen::Matrix<double,3,4>& corners);

Eigen::Vector3d calc_gate_normal(const Eigen::Matrix<double,3,4>& corners,
  const Eigen::Vector3d& center);

Eigen::Vector3d calc_gate_center_offset(double dist, 
  const Eigen::Vector3d& gate_center, const Eigen::Vector3d& gate_normal);

Eigen::Quaterniond calc_quaternion_error(const Eigen::Quaterniond& act,
  const Eigen::Quaterniond& des);

Eigen::Vector3d to_angle_axis(const Eigen::Quaterniond& q);