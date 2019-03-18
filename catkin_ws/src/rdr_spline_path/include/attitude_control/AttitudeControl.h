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

    /**
     * \brief Sets the spline.
     * \newSpline The new spline to be set.
     */
    void setSpline(const Eigen::Spline3d& newSpline) 
    {
        spline = newSpline;
    }

    /**
     * \brief Sets the length of the current spline.
     * \length The length to be set.
     */
    void setSplineLength(const double& length) 
    { 
        splineLength = length; 
        std::cout << "Spline length is: " << splineLength << "\n";
    }   

    /**
     * \brief Sets the tangential acceleration gain value.
     * \gain The new gain to be set.
     */
    void setTangentAccelerationGain(const double& gain)
    {
        tangentAccelerationGain = gain;
    }

    /**
     * \brief Find closest spot on spline nearby previous position.
     * \param currentPosition: position of vehicle for which we want to find the u parameter
     * \return Returns the current closest parameter on the spline to the vehicle.
     */
    double findClosestParam(const Eigen::Vector3d currentPosition)
    {
        constexpr size_t discretePointsToCheck = 1000;
        constexpr double searchIncrement = 1. / static_cast<double>(discretePointsToCheck);
        constexpr double searchThreshold = 1e-4;

        double distance = 0.;
        double startParam = previousClosestParam;

        double minimumDistance = (currentPosition - Eigen::Vector3d(spline(startParam))).norm();

        // Search the entire spline for closest spline parameter at given resolution to find the global minimum.
        for (unsigned int i = 1; i < discretePointsToCheck; ++i)
        {
            startParam += searchIncrement;
            distance = (currentPosition - Eigen::Vector3d(spline(startParam))).norm();
            
            if (distance < minimumDistance)
            {
                minimumDistance = distance;
            }
        }

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

            vectorToParam        = derivatives.col(0).matrix() - currentPosition;
            tangentVectorAtParam = derivatives.col(1).matrix();
            curvatureAtParam     = derivatives.col(2).matrix();

            tangentVectorAtParam.normalize();
            curvatureAtParam.normalize();

            double searchGradient  = tangentVectorAtParam.dot(vectorToParam);
            double searchIncrement = curvatureAtParam.dot(vectorToParam) + std::pow(tangentVectorAtParam.norm(), 2.);

            closestParam -= searchGradient / searchIncrement;

            // Constrain within the bounds of the spline, (1,0), C++ 17 provides std::clamp() to accomplish this.
            closestParam = std::max(std::min(closestParam, 1.), 0.);

            // End the search if we are within our search threshold.
            if (((closestParam - previousClosestParam) * tangentVectorAtParam).norm() < searchThreshold)
            {
                return closestParam;
            }

            counter++;
            previousClosestParam = closestParam;
        }

        return -1;
    }

    Eigen::Vector3d calculateSplineDerivative(const double& parameterU, const int derivativeDegree)
    {

        if (derivativeDegree > SPLINE_DEGREE)
        {
            return Eigen::Vector3d::Zero();
        }
    
        Eigen::MatrixXd derivatives = spline.derivatives<SPLINE_DEGREE>(parameterU);
        return derivatives.col(derivativeDegree);
    }

    /**
     * \brief Returns the curvature on the spline at parameterU
     * \parameterU The parameter of interest along the spline.
     */
    double calculateCurvature(const double& parameterU)
    {
        Eigen::Vector3d tangentVector = calculateSplineDerivative(parameterU, 1);
        Eigen::Vector3d tangentNormalVector = calculateSplineDerivative(parameterU, 2);

        // k = |y' x y"| / ||y'^3||
        double curvature = tangentVector.cross(tangentNormalVector).norm() / pow(tangentVector.norm(), 3.0);
        return curvature;
    }

    Eigen::Vector3d calculateCentripetalAccelVector(const double& parameterU)
    {
        Eigen::Vector3d tangentVector = calculateSplineDerivative(parameterU, 1);
        Eigen::Vector3d tangentNormalVector = calculateSplineDerivative(parameterU, 2);

        // Ensure perpendicularity to tangent vector.
        Eigen::Vector3d centripetalAccelVector = tangentVector.cross(tangentNormalVector.cross(tangentVector));
        return centripetalAccelVector;
    }

    /**
     * \brief Compute the maximum tangential speed at one point on the spline.
     * \param parameterU The point of interest on the spline.
     * \param curvature The computed curvature at that point on the spline.
     */
    double calculateMaxTangentialSpeed(const double& parameterU)
    {
        Eigen::Vector3d centripetalAccelVector = calculateCentripetalAccelVector(parameterU);

        double maxTangentSpeed = MAX_SPEED;

        if (centripetalAccelVector.norm() > 0)
        {
            centripetalAccelVector.normalize();

            double curvature = calculateCurvature(parameterU);

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
            inertialAccelVector = Eigen::Vector3d(0., 0., 1.e-8); // A zero vector would result in an undefined orientation.
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
    
    Eigen::Vector3d gravityVector{GRAVITY* Vector3d::UnitZ()};
    
    double tangentAccelerationGain{1.}; // Proportional gain to tilt the trust vector.
    double previousClosestParam{0.};    // Previous closest spline parameter.
    double curvature{0.};
    
    double splineLength{25.0};          //! Set this to minimize search speeds.
};
