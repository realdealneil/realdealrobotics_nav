#include "eigen_helpers.h"

Eigen::Vector3d to_eigen(const geometry_msgs::Point& point)
{
  Eigen::Vector3d eigen_point;
  eigen_point << point.x, point.y, point.z;
  return eigen_point;
}

EigenGateLocation to_eigen(const rdr_spline_path::GateLocation& gl)
{
  EigenGateLocation egl;
  for(size_t i = 0; i < 4; ++i)
  {
    egl.corners.col(i) = to_eigen(gl.corners[i]);
  }
  egl.center = to_eigen(gl.center);
  egl.perturbation_bound = to_eigen(gl.perturbation_bound);
  return egl;
}

EigenGateLocationList to_eigen(const rdr_spline_path::GateLocationList& gll)
{
  EigenGateLocationList egll;
  for(auto& gl : gll)
  {
    egll.gates.emplace_back(to_eigen(gl));
  }
}

rdr_spline_path::GateLocation to_ros(const Eigen::Vector3d&)
{

}