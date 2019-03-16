/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: AttitudeControl.h
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */
 
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>
#include <rdr_spline_path/rdr_utilities.h>
#include <iostream>

static constexpr int SPLINE_DEGREE{3};          // Spline degree is the polynomial order N + 1.

static constexpr double GRAVITY{9.80665};       // m/s
static constexpr double MAX_ACCEL{5 * GRAVITY}; // 5 G's

static constexpr double MAX_SPEED{10.};         // m/s
static constexpr double MIN_SPEED{0.};          // m/s

class AttitudeControl
{
public:
	AttitudeControl() {}
	
	void SetSpline(const Eigen::Spline3d& s) 
	{
		spline_ = s;
	}
	
	void SetSplineLength(double l) 
	{ 
		spline_length_ = l; 
		std::cout << "Spline length is: " << spline_length_ << "\n";
	}	

	/**
     * \brief Find closest spot on spline nearby previous position.  
	 * \param spline: the spline
	 * \param prevU: previous nearest u (we assume that we're making forward progress on the spline)
	 * \param currentPosition: position of vehicle for which we want to find the u parameter
	 * \param currentU: output 
	 * \param splinePosition: Corresponding 3d location on the spline 
	 */
	bool findNearestPointOnSpline(const Eigen::Spline3d& spline, 
		                          const double& prevU,
                                  const Eigen::Vector3d currentPosition, 
		                          double& currentU,
                                  Eigen::Vector3d& splinePosition)
	{
		 //! Find the position at the previous U.  That's where we'll start searching:
		 Eigen::Vector3d prevSplinePosition = spline(prevU);
		 
		 //! Start with a rough search along the spline.  Look 1/5 of the spline ahead, with an increment close to 1 m:
		 double meter_increment = 1.0;
		 double meter_search_dist = spline_length_/5.0;
		 double u_increment = meter_increment/spline_length_;
		 double u_search_dist = meter_search_dist/spline_length_;
	}	

    void calculateSplineDerivatives(const double& parameter_u)
    {
        Eigen::MatrixXd derivatives = spline_.derivatives<SPLINE_DEGREE>(parameter_u);

        _tangent_unit_vector = derivatives.col(1);
        _tangent_normal_vector = derivatives.col(2);
    }

	/**
     * \brief get curvature and max tangent speed from a spline.
     */
	bool calculateMaximumTangentSpeed(const double& parameter_u,
		                              double& curvature,
                                      double& max_tangent_speed)
	{
        calculateSplineDerivatives(parameter_u);

		// Ensure perpendicularity to tangent vector.
		_centripetal_accel_vector = _tangent_unit_vector.cross(_tangent_normal_vector.cross(_tangent_unit_vector));
		
		max_tangent_speed = MAX_SPEED;

		if (_centripetal_accel_vector.norm() > 0)
		{
			_centripetal_accel_vector.normalize();

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

    void calculateTangentialAccelVector(const double& parameter_u)
    {
        double accel_magnitude = 0.;

        _tangenial_accel_vector = 
    }

    Eigen::Vector3d calculateInertialAccel(const double& parameter_u)
    {
        Vector3d max_velocity = calculate_maximum_tangent_speed(parameter_u);
        calculate_tangential_accel_vector(parameter_u);
        
        _inertial_accel_vector = _centripetal_accel_vector + _inertial_accel_vector + _gravity_vector;

        if (inertial_accel_vector.norm() == 0.)
        {
            inertial_accel_vector = Vector3d(0., 0., 0.0000001);
        }

        return inertial_accel_vector;
    }
    */

private:
	Eigen::Spline3d spline_;
    Eigen::Vector3d _centripetal_accel_vector{Eigen::Vector3d::Zero()};
    Eigen::Vector3d _inertial_accel_vector{Eigen::Vector3d::Zero()};
    Eigen::Vector3d _tangent_accel_vector{Eigen::Vector3d::Zero()};
    Eigen::Vector3d _tangent_unit_vector{Eigen::Vector3d::Zero()};
    Eigen::Vector3d _tangent_normal_vector{Eigen::Vector3d::Zero()};
    
    Eigen::Vector3d _gravity_vector{Eigen::Vector3d(0., 0., -GRAVITY)};
    
    double spline_length_ = 25.0;	//! Set this to minimize search speeds. 
};
