#include "eigen_helpers.h"

Eigen::Vector3d to_eigen(const geometry_msgs::Point& point)
{
  Eigen::Vector3d eigen_point;
  eigen_point << point.x, point.y, point.z;
  return eigen_point;
}

Eigen::Vector3d to_eigen(const geometry_msgs::Vector3& point)
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
  egl.perturbation_bound = to_eigen(gl.perturbation_bound);
  egl.center = calc_gate_center(egl.corners);
  egl.normal = calc_gate_normal(egl.corners, egl.center);
  egl.front_point = calc_gate_center_offset(3.0, egl.center, egl.normal);
  egl.back_point = calc_gate_center_offset(-3.0, egl.center, egl.normal);

  return egl;
}

EigenGateLocationList to_eigen(const rdr_spline_path::GateLocationList& gll)
{
  EigenGateLocationList egll;
  for(const rdr_spline_path::GateLocation& gl : gll.gates)
  {
    egll.gates.push_back(to_eigen(gl));
  }
  return egll;
}

geometry_msgs::Point to_ros_point(const Eigen::Vector3d& ev)
{
  geometry_msgs::Point p;
  p.x = ev[0];
  p.y = ev[1];
  p.z = ev[2];
  return p;
}

Eigen::Vector3d calc_gate_center(const Eigen::Matrix<double, 3,4>& corners)
{
  return corners.rowwise().mean();
}

Eigen::Vector3d calc_gate_normal(const Eigen::Matrix<double, 3, 4>& corners, 
  const Eigen::Vector3d& center)
{
  return ((corners.col(0) - center).cross(corners.col(1) - center)).normalized();
}

Eigen::Vector3d calc_gate_center_offset(double dist, 
  const Eigen::Vector3d& gate_center, const Eigen::Vector3d& gate_normal)
{
  return dist * gate_normal + gate_center;
}

