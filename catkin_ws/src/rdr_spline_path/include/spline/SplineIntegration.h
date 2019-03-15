/**
 * @file SplineIntegration.h
 * @copyright Mark Sauder
 */

#pragma once

#include "SplineFitting.h"

#include <unsupported/Eigen/Splines>
#include <unsupported/Eigen/NumericalIntegration>

/**
 * Define the scalar typedef: e.g. float, double, int, etc.
 */
typedef float Scalar;


/**
 * @class SplineIntegration Helper class to simplify the usage of the
 *        Eigen::Numerical Integration class with splines.
 *
 * Example usage:
 *    Eigen::Spline3d spline {};
 *    Scalar spline_length = SplineIntegration<PathType::Dimension>::Integrate(spline, 0, 1, integrator);
 */
template <int dimension>
class SplineIntegration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static Scalar Integrate(const Eigen::Spline<Scalar, dimension>& spline,
                            const Scalar lower_limit = static_cast<Scalar>(0),
                            const Scalar upper_limit = static_cast<Scalar>(1),
                            Eigen::Integrator<Scalar>& integrator);

    Scalar operator()(const Scalar param) const
    {
        // Return the normalized derivatives.
        return sqrt(spline.derivatives(param, 1).col(1).squaredNorm());
    }

private:

    Scalar _absolute_error{1.e-3};
    Scalar _relative_rrror{1.e-3};

};

template <int dimension>
Scalar SplineIntegration<dimension>::Integrate(const Eigen::Spline<Scalar, dimension>& spline, 
                                               const Scalar lower_limit = static_cast<Scalar>(0),
                                               const Scalar upper_limit = static_cast<Scalar>(1),
                                               Eigen::Integrator<Scalar>& integrator)
{
    Scalar integral = integrator.quadratureAdaptive(spline,
                                                    lower_limit,
                                                    upper_limit,
                                                    _absolute_error,
                                                    _relative_rrror,
                                                    Eigen::Integrator<Scalar>::GaussKronrod15);
    
    return integral;
}
