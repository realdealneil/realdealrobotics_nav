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
#include <ros/ros.h>

static constexpr int SPLINE_DEGREE{3};          // Spline degree is the polynomial order N + 1.

static constexpr double GRAVITY{9.80665};       // m/s
static constexpr double MAX_ACCEL{5. * GRAVITY}; // 5 G's

static constexpr double MAX_SPEED{10.};         // m/s
static constexpr double MIN_SPEED{0.};          // m/s

class AttitudeControl
{
public:
    AttitudeControl() {}
    
    void setSpline(const Eigen::Spline3d& newSpline) 
    {
        spline = newSpline;
        previousClosestParam = 0.0;
    }
    
    void setSplineLength(const double& length) 
    { 
        splineLength = length; 
        std::cout << "Spline length is: " << splineLength << "\n";
    }   
    
    double getCurvature(void) { return curvature; }

    /**
     * \brief Find closest spot on spline nearby previous position.
     * @param currentPosition: position of vehicle for which we want to find the u parameter
     * @return Returns the current closest parameter on the spline to the vehicle.
     */
    double findClosestParam(const Eigen::Vector3d currentPosition)
    {
		constexpr size_t discretePointsToCheck = 1000;
		constexpr double searchThreshold = 1e-4;
		/// Brute force version:
		        
        double searchDistMeters=10.0;
        double searchIncrement = (searchDistMeters/splineLength) / static_cast<double>(discretePointsToCheck);
        

        double distance = 0.;
        double startParam = previousClosestParam;
        
        double minimumDistance = (currentPosition - Eigen::Vector3d(spline(startParam))).norm();
        double closestParam = startParam;

        // Search the entire spline for closest spline parameter at given resolution to find the global minimum.
        for (unsigned int i = 1; i < discretePointsToCheck; ++i)
        {
            startParam += searchIncrement;
            if (startParam > 1.0)
			{	
				return 1.0;
			}	
            distance = (currentPosition - Eigen::Vector3d(spline(startParam))).norm();
            
            if (distance < minimumDistance)
            {
                minimumDistance = distance;
                closestParam = startParam;
            }
        }
        
		previousClosestParam = closestParam;
        
        return closestParam;
    }
    
    /**
     * \brief Find closest spot on spline nearby previous position.
     * @param currentPosition: position of vehicle for which we want to find the u parameter
     * @return Returns the current closest parameter on the spline to the vehicle.
     */
    double findClosestParamGradientDescent(const Eigen::Vector3d currentPosition)
    {
		constexpr size_t discretePointsToCheck = 1000;
		constexpr double searchThreshold = 1e-4;
        /// Gradient search
        
        /**( I tried getting this to work, but didn't have much luck.  So, 
         * 	I'm using the brute force approach above.  NGJ
         */
        Eigen::MatrixXd derivatives          = Eigen::Vector3d::Zero();
        Eigen::Vector3d vectorToParam        = Eigen::Vector3d::Zero();
        Eigen::Vector3d tangentVectorAtParam = Eigen::Vector3d::Zero();
        Eigen::Vector3d curvatureAtParam     = Eigen::Vector3d::Zero();
        double closestParam = previousClosestParam;
        size_t counter = 0;

        // Gradient descent method to search for the local minimum..
        while (counter < discretePointsToCheck)
        {
            derivatives = spline.derivatives<SPLINE_DEGREE>(closestParam);

            vectorToParam        = (derivatives.col(0).matrix() - currentPosition).normalized();
            tangentVectorAtParam = derivatives.col(1).matrix();
            curvatureAtParam     = derivatives.col(2).matrix();

            double searchGradient  = tangentVectorAtParam.normalized().dot(vectorToParam);
            double searchIncrement = std::abs(curvatureAtParam.dot(vectorToParam) + pow(tangentVectorAtParam.norm(), 2.));

/*
			std::cout << "Iteration " << counter << "\n"
				"     Prev pos on spline: " << spline(closestParam).matrix().transpose() << "\n"
				"      Zeroth derivative: " << derivatives.col(0).matrix().transpose() << "\n"
				"Tangent Vector at Param: " << tangentVectorAtParam.transpose() << "\n"
				"          vectorToParam: " << vectorToParam.transpose() << "\n"
				"                   grad: " << searchGradient << "\n"
				"                    inc: " << searchIncrement << "\n"
				"            delta param: " << searchGradient / searchIncrement << "\n";
*/

            closestParam = previousClosestParam - searchGradient / searchIncrement;
            
            //std::cout << "  closestParam: " << closestParam << "\n";           

            // Constrain within the bounds of the spline, (1,0), C++ 17 provides std::clamp() to accomplish this.
            closestParam = std::max(std::min(closestParam, 1.), 0.);

            // End the search if we are within our search threshold.
            double criterion = ((closestParam - previousClosestParam) * tangentVectorAtParam).norm();
            if (criterion < searchThreshold)
            {
                //std::cout << "Found closest param after " << counter << " iterations, " << criterion << "\n";
                return closestParam;
            }            
            previousClosestParam = closestParam;

            counter++;
            
        }

        return -1;
    }
    
    double findLookAheadParam(double lookaheadDist_meters=1.0)
    {
		//! Get the param at a point roughly the lookahead distance along the spline:
		double lookaheadDist_u = lookaheadDist_meters/splineLength;        
        return previousClosestParam + lookaheadDist_u;
    }

    Eigen::Vector3d calculateSplineDerivatives(const double& parameterU, const int derivativeDegree)
    {

        if (derivativeDegree > SPLINE_DEGREE)
        {
            return Eigen::Vector3d::Zero();
        }
    
        Eigen::MatrixXd derivatives = spline.derivatives<SPLINE_DEGREE>(parameterU);
        return derivatives.col(derivativeDegree);
    }

    /**
     * \brief Compute the maximum tangential speed at one point on the spline.
     * \param parameterU The point of interest on the spline.
     * \param curvature The computed curvature at that point on the spline.
     */
    double calculateMaxTangentialSpeed(const double& parameterU, 
		const double desiredSpeed, const double currentSpeed)
    {
        tangentVector = calculateSplineDerivatives(parameterU, 1);
        tangentNormalVector = calculateSplineDerivatives(parameterU, 2);

        // Ensure perpendicularity to tangent vector.
        tangentNormalVector = tangentVector.cross(tangentNormalVector.cross(tangentVector));
        
        double maxTangentSpeed = 0.0;

        if (tangentNormalVector.norm() > 0)
        {
            // k = |y' x y"| / ||y'^3||
            curvature = tangentVector.cross(tangentNormalVector).norm()
                               / pow(tangentVector.norm(), 3.0);

            // a = v^2 / r   =>   v^2 * k   =>  v = sqrt(a / k)
            maxTangentSpeed = std::min(sqrt(MAX_ACCEL / curvature), MAX_SPEED);
            
            centripetalAccelVector = desiredSpeed * desiredSpeed * curvature * tangentNormalVector.normalized();
            
            ROS_INFO_THROTTLE(0.25, "curvature: %f, centripetalAccelVector: %f %f %f",
				curvature, centripetalAccelVector(0), centripetalAccelVector(1), centripetalAccelVector(2));
        }

        return maxTangentSpeed;
    }

    Eigen::Vector3d calculateDesiredAccelVector(const double& parameterU,
												const double desiredSpeed,
                                                const double currentSpeed)
    {
		double maxTangentialSpeed = calculateMaxTangentialSpeed(parameterU, desiredSpeed, currentSpeed);
		/*
        double speedError = maxTangentialSpeed - currentSpeed;

        double inertialAccelMagnitude = (centripetalAccelVector + gravityVector).norm();
        double maxTangentialAccelMagnitude = MAX_ACCEL - inertialAccelMagnitude;
        double tangentialAccelEffort = std::min((tangentAccelerationGain * speedError), maxTangentialAccelMagnitude);

        tangentialAccelVector = tangentVector.normalized() * tangentialAccelEffort;
        inertialAccelVector = centripetalAccelVector + tangentialAccelVector - gravityVector;

        if (inertialAccelVector.norm() == 0.)
        {
            inertialAccelVector = Eigen::Vector3d(0., 0., -1.e-8); // A zero vector would result in an undefined orientation.
        }*/
        
        
			
		//! We want to go in the direction of the tangent vector.  
		double speedError = desiredSpeed - currentSpeed;
		double tangentialAccelEffort  = tangentAccelerationGain * speedError;
		
		tangentialAccelVector = tangentVector.normalized() * tangentialAccelEffort;
		inertialAccelVector = centripetalAccelVector.normalized() + tangentialAccelVector - gravityVector;
		
		
		
        ROS_INFO_STREAM_THROTTLE(0.5, "calculateDesiredAccelVector\n"
			"                 speedError: " << speedError << "\n"
			//"         maxTangentialSpeed: " << maxTangentialSpeed << "\n"
			//"     inertialAccelMagnitude: " << inertialAccelMagnitude << "\n"
			//"     centripetalAccelVector: " << centripetalAccelVector.transpose() << "\n"
			//"maxTangentialAccelMagnitude: " << maxTangentialAccelMagnitude << "\n"
			"      tangentialAccelEffort: " << tangentialAccelEffort << "\n"
			"      tangentialAccelVector: " << tangentialAccelVector.transpose() << "\n"
			"        inertialAccelVector: " << inertialAccelVector.transpose() << "\n"
			);		

        return inertialAccelVector;
    }

private:
    Eigen::Spline3d spline;
    Eigen::Vector3d centripetalAccelVector{Eigen::Vector3d::Zero()};
    Eigen::Vector3d inertialAccelVector{Eigen::Vector3d::Zero()};
    Eigen::Vector3d tangentialAccelVector{Eigen::Vector3d::Zero()};
    Eigen::Vector3d tangentVector{Eigen::Vector3d::Zero()};
    Eigen::Vector3d tangentNormalVector{Eigen::Vector3d::Zero()};
    
    Eigen::Vector3d gravityVector{Eigen::Vector3d(0., 0., -GRAVITY)};
    
    double tangentAccelerationGain{1.}; // Proportional gain to tilt the trust vector.
    double previousClosestParam{0.};    // Previous closest spline parameter.
    double curvature{0.};
    
    double splineLength{25.0};          //! Set this to minimize search speeds.
    
    double startTime;
};
