/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: AttitudeControl.h
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */
 
#ifndef RDR_ATTITUDE_CONTROL_H
#define RDR_ATTITUDE_CONTROL_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>
#include <rdr_spline_path/rdr_utilities.h>

class RdrAttitudeControl
{
public:
	RdrAttitudeControl() {}
	
	/// \brief Find closest spot on spline nearby previous position.  
	/**	\param spline: the spline
	 * 	\param prevU: previous nearest u (we assume that we're making forward progress on the spline)
	 * 	\param currentPosition: position of vehicle for which we want to find the u parameter
	 * 	\param currentU: output 
	 * 	\param splinePosition: Corresponding 3d location on the spline 
	 */
	bool findNearestPointOnSpline(const Eigen::Spline3d& spline, 
		const double& prevU, const Eigen::Vector3d currentPosition, 
		double& currentU, Eigen::Vector3d& splinePosition)
	{
		 
	}
	
	/// \brief get curvature and max tangent speed from a spline
	bool calculate_maximum_tangent_speed(const Eigen::Spline3d& spline, const double& parameter_u, 
		double& curvature, double& max_tangent_speed)
	{
		const int derivative_order = 3;
		Eigen::MatrixXd derivatives = spline.derivatives<3>(parameter_u);

		_tangent_unit_vector = derivatives.col(1);
		_tangent_normal_vector = derivatives.col(2);

		// Ensure perpendicularity to tangent vector.
		_centripetal_acceleration = _tangent_unit_vector.cross(_tangent_normal_vector.cross(_tangent_unit_vector));
		
		max_tangent_speed = _max_speed;

		if (_centripetal_acceleration.norm() > 0)
		{
			_centripetal_acceleration.normalize();

			// k = |y' x y"| / ||y'^3||
			curvature = _tangent_unit_vector.cross(_tangent_normal_vector).norm()
							   / pow(_tangent_unit_vector.norm(), 3.0);

			// a = v^2 / r   =>   v^2 * k   =>  v = sqrt(a / k)
			max_tangent_speed = sqrt(_max_accel * curvature);
			return true;
		}
		assert(false);
		return false;
	}
	
	/*
	double AttitudeControl::calculate_tangential_accel_magnitude(const double& parameter_u)
	{

		double accel_magnitude = 0.;

		return accel_magnitude;
	}
	
	Vector3d AttitudeControl::calculate_inertial_accel(const double& parameter_u)
	{
		calculate_maximum_tangent_speed(parameter_u);
		calculate_tangential_accel_magnitude(parameter_u);
		
		_inertial_accel_vector = _centripetal_accel_vector + _inertial_accel_vector + _gravity_vector;

		if (inertial_accel_vector.norm() == 0.)
		{
			inertial_accel_vector = Vector3d(0., 0., 0.0000001);
		}

		return inertial_accel_vector;
	}*/
	
private:
	Eigen::Vector3d _gravity_vector{Eigen::Vector3d(0., 0., -Gravity)};

	Eigen::Vector3d _centripetal_acceleration{Eigen::Vector3d::Zero()};
	Eigen::Vector3d _inertial_accel_vector{Eigen::Vector3d::Zero()};
	Eigen::Vector3d _tangent_accel_vector{Eigen::Vector3d::Zero()};
	Eigen::Vector3d _tangent_unit_vector{Eigen::Vector3d::Zero()};
	Eigen::Vector3d _tangent_normal_vector{Eigen::Vector3d::Zero()};
	
	double _max_speed{10.}; // m/s
	double _min_speed{0.};  // m/s
};

#endif // RDR_ATTITUDE_CONTROL_H
