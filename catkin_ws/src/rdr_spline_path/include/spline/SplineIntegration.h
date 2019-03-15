/**
 * @file SplineMath.h
 * @copyright Grey Point Corporation
 */

#pragma once

#include "SplineFitting.h"

#include <unsupported/Eigen/Splines>
#include <unsupported/Eigen/NumericalIntegration>


/**
 * The integrand which is evaluated in the calculation of the arc length of a curve.
 */
template <int dimension>
class SplineFunctor
{
public:
    SplineFunctor(const Eigen::Spline<Scalar, dimension>& spline)
        : spline(spline)
    {
    }

    /**
     * @param param The parameter at which to evaluate the integrand.
     * @returns the value of the integrand at @a param.
     */
    Scalar operator()(const Scalar param) const
    {
        Eigen::Matrix<Scalar, dimension, 1> derivatives = spline.derivatives(param, 1).col(1);
        return  sqrt(derivatives.squaredNorm());
    }

private:
    /**
     * @brief The spline to evaluate the arc length of.
     */
    const Eigen::Spline<Scalar, dimension> spline;

};


template <int dimension>
class SplineMath
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
    static Scalar arcLength(const Eigen::Spline<Scalar, dimension>& spline,
                            Eigen::Integrator<Scalar>& integrator);
 
    static Scalar arcLength(const Eigen::Spline<Scalar, dimension>& spline,
                            Scalar lowerLimit,
                            Scalar upperLimit,
                            Eigen::Integrator<Scalar>& integrator);

private:
    static Scalar arcLengthIntegrand(const Eigen::Spline<Scalar, dimension>& spline,
                                     const Scalar& u);

    static Eigen::Spline<Scalar, dimension> spline;
};

template <int dimension>
inline Scalar SplineMath<dimension>::arcLength(const Eigen::Spline<Scalar, dimension>& spline,
                                               Eigen::Integrator<Scalar>& integrator)
{
    return SplineMath<dimension>::arcLength(spline, 0., 1., integrator);
}

template <int dimension>
Scalar SplineMath<dimension>::arcLength(const Eigen::Spline<Scalar, dimension>& spline, 
                                        Scalar lowerLimit,
                                        Scalar upperLimit, 
                                        Eigen::Integrator<Scalar>& integrator)
{
    Scalar absolute_error = 1.e-3;
    Scalar relative_error = 1.e-3;

    SplineFunctor<dimension> splineFunctor(spline);

    Scalar integral = integrator.quadratureAdaptive(splineFunctor,
                                                    lowerLimit,
                                                    upperLimit,
                                                    absolute_error,
                                                    relative_error,
                                                    Eigen::Integrator<Scalar>::GaussKronrod15);
    
    return integral;
}
