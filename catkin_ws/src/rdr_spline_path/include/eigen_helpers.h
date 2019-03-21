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

/*
 * @brief Normalize the given quaternion to unit quaternion.
 */
inline void quaternionNormalize(Eigen::Vector4d& q) {
  double norm = q.norm();
  q = q / norm;
  return;
}

inline Eigen::Quaterniond EulerToQuat(double phi, double theta, double psi)
{
	Eigen::Quaterniond qphi;
	Eigen::Quaterniond qtheta;
	Eigen::Quaterniond qpsi;
	qphi.w() = cos(phi/2.0);
	qphi.vec() = Eigen::Vector3d(sin(phi/2.0), 0.0, 0.0);
	
	qtheta.w() = cos(theta/2.0);
	qtheta.vec() = Eigen::Vector3d(0.0, sin(theta/2.0), 0.0);
	
	qpsi.w() = cos(psi/2.0);
	qpsi.vec() = Eigen::Vector3d(0.0, 0.0, sin(psi/2.0));
	
	Eigen::Quaterniond result = qpsi * qtheta * qphi;
	return result;	
}

/*
 * @brief Convert a rotation matrix to a quaternion.
 * @note Pay attention to the convention used. The function follows the
 *    conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for Quaternion Algebra", Equation (78).
 *
 *    The output quaternion will be in the form
 *      [q1, q2, q3, q4(scalar)]^T
 */
/*
inline Eigen::Quaterniond rotationToQuaternion(
    const Eigen::Matrix3d& R) 
{
  Eigen::Vector4d score;
  score(0) = R(0, 0);
  score(1) = R(1, 1);
  score(2) = R(2, 2);
  score(3) = R.trace();

  int max_row = 0, max_col = 0;
  score.maxCoeff(&max_row, &max_col);

  Eigen::Vector4d q = Eigen::Vector4d::Zero();
  if (max_row == 0) {
    q(0) = std::sqrt(1+2*R(0, 0)-R.trace()) / 2.0;
    q(1) = (R(0, 1)+R(1, 0)) / (4*q(0));
    q(2) = (R(0, 2)+R(2, 0)) / (4*q(0));
    q(3) = (R(1, 2)-R(2, 1)) / (4*q(0));
  } else if (max_row == 1) {
    q(1) = std::sqrt(1+2*R(1, 1)-R.trace()) / 2.0;
    q(0) = (R(0, 1)+R(1, 0)) / (4*q(1));
    q(2) = (R(1, 2)+R(2, 1)) / (4*q(1));
    q(3) = (R(2, 0)-R(0, 2)) / (4*q(1));
  } else if (max_row == 2) {
    q(2) = std::sqrt(1+2*R(2, 2)-R.trace()) / 2.0;
    q(0) = (R(0, 2)+R(2, 0)) / (4*q(2));
    q(1) = (R(1, 2)+R(2, 1)) / (4*q(2));
    q(3) = (R(0, 1)-R(1, 0)) / (4*q(2));
  } else {
    q(3) = std::sqrt(1+R.trace()) / 2.0;
    q(0) = (R(1, 2)-R(2, 1)) / (4*q(3));
    q(1) = (R(2, 0)-R(0, 2)) / (4*q(3));
    q(2) = (R(0, 1)-R(1, 0)) / (4*q(3));
  }

  if (q(3) < 0) q = -q;
  quaternionNormalize(q);
  
  //! Convert to eigen quaterniond:
  Eigen::Quaterniond e;
  e.w() = q(3);
  e.vec() = q.block(0,0,3,1);
  
  return e;
}
* */
