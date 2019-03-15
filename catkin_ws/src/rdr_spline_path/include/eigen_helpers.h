#pragma once

#include <Eigen/Dense>
#include <rdr_spline_path/GateLocationList.h>


struct EigenGateLocation
{
  Eigen::Matrix<3,4,double> corners;
  Eigen::Vector3d center;
  Eigen::Vector3d perturbation_bound;
  // TODO: Add Normal, and front and back points.
}

struct EigenGateLocationList
{
  std::vector<EigenGateLocation> gates;
}

Eigen::Vector3d to_eigen(const geometry_msgs::Point&);

EigenGateLocation to_eigen(const rdr_spline_path::GateLocation&);

EigenGateLocationList to_eigen(const rdr_spline_path::GateLocationList&);

rdr_spline_path::GateLocation to_ros(const Eigen::Vector3d&);
