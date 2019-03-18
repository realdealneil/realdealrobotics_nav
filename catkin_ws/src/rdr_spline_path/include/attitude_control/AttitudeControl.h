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
    
    void SetSpline(const Eigen::Spline3d& newSpline) 
    {
        spline = newSpline;
    }
    
    void SetSplineLength(const double& length) 
    { 
        splineLength = length; 
        std::cout << "Spline length is: " << splineLength << "\n";
    }   
    
    double getCurvature(void) { return curvature; }

    /**
     * \brief Find closest spot on spline nearby previous position.
     * \param currentPosition: position of vehicle for which we want to find the u parameter
     * \return Returns the current closest parameter on the spline to the vehicle.
     */
    double findClosestParam(const Eigen::Vector3d currentPosition)
    {
        // Perform a rough search for closest spline parameter at given resolution to avoid local minimum.
        constexpr size_t discretePointsToCheck = 1000;
        double searchDistMeters = 10.0;        
        double searchIncrement = (searchDistMeters/splineLength) / static_cast<double>(discretePointsToCheck);
        constexpr double searchThreshold = 1e-4;

        double distance = 0.;
        double startParam = previousClosestParam;
        
        double minimumDistance = (currentPosition - Eigen::Vector3d(spline(startParam))).norm();
        double u_closest = startParam;

        for (unsigned int i = 1; i < discretePointsToCheck; ++i)
        {
            startParam += searchIncrement;
            if (startParam > 1.0)
			{	
				return -1.0;
			}	
            distance = (currentPosition - Eigen::Vector3d(spline(startParam))).norm();
            
            if (distance < minimumDistance)
            {
                minimumDistance = distance;
                u_closest = startParam;
            }
        }
        
        return u_closest;

        // Gradient search
        /*
        double closestParam = previousClosestParam;
        size_t counter = 0;
        
        

        while (counter < discretePointsToCheck)
        {
            Eigen::MatrixXd derivatives = spline.derivatives<SPLINE_DEGREE>(closestParam);

            Eigen::Vector3d vectorToSpline = spline(closestParam).matrix() - currentPosition;
            Eigen::Vector3d localTangentVector = derivatives.col(1);
            //derivatives.col(0).matrix() - currentPosition;

            double numerator = localTangentVector.dot(vectorToSpline);
            double denominator = (derivatives.col(2).matrix().dot(vectorToSpline) + std::pow(localTangentVector.norm(), 2.));

            closestParam = std::max(std::min(previousClosestParam - numerator / denominator, 1.), 0.);

            if (((closestParam - previousClosestParam) * tangentVector).norm() < searchThreshold)
            {
                previousClosestParam = closestParam;
                std::cout << "Found closest param after " << counter << " iterations\n";
                return closestParam;
            }

            counter++;
        }*/

        //return -1;
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
    double calculateMaxTangentialSpeed(const double& parameterU)
    {
        tangentVector = calculateSplineDerivatives(parameterU, 1);
        tangentNormalVector = calculateSplineDerivatives(parameterU, 2);

        // Ensure perpendicularity to tangent vector.
        centripetalAccelVector = tangentVector.cross(tangentNormalVector.cross(tangentVector));
        
        double maxTangentSpeed = MAX_SPEED;

        if (centripetalAccelVector.norm() > 0)
        {
            centripetalAccelVector.normalize();

            // k = |y' x y"| / ||y'^3||
            curvature = tangentVector.cross(tangentNormalVector).norm()
                               / pow(tangentVector.norm(), 3.0);

            // a = v^2 / r   =>   v^2 * k   =>  v = sqrt(a / k)
            maxTangentSpeed = std::min(sqrt(MAX_ACCEL * curvature), MAX_SPEED);
        }

        return maxTangentSpeed;
    }

    Eigen::Vector3d calculateDesiredAccelVector(const double& parameterU,
                                                const double currentSpeed)
    {
        double speedError = calculateMaxTangentialSpeed(parameterU) - currentSpeed;

        double centripetalAccelMagnitude = (centripetalAccelVector + gravityVector).norm();
        double maxTangentialAccelMagnitude = MAX_ACCEL - centripetalAccelMagnitude;
        double tangentialAccelEffort = std::min((tangentAccelerationGain * speedError), maxTangentialAccelMagnitude);

        tangentialAccelVector *= tangentialAccelEffort;
        inertialAccelVector = centripetalAccelVector + tangentialAccelVector - gravityVector;

        if (inertialAccelVector.norm() == 0.)
        {
            inertialAccelVector = Eigen::Vector3d(0., 0., -1.e-8); // A zero vector would result in an undefined orientation.
        }

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
};
