/**
 * @file SplineIntegration.h
 * @copyright Mark Sauder
 */

#pragma once

//#include "SplineFitting.h"

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
							Eigen::Integrator<Scalar>& integrator,
                            const Scalar lower_limit = static_cast<Scalar>(0),
                            const Scalar upper_limit = static_cast<Scalar>(1)
                            );    
};

template <int dimension>
Scalar SplineIntegration<dimension>::Integrate(const Eigen::Spline<Scalar, dimension>& spline, 
											   Eigen::Integrator<Scalar>& integrator,
                                               const Scalar lower_limit,
                                               const Scalar upper_limit
                                               )
{
	Scalar absolute_error{1.e-3};
    Scalar relative_error{1.e-3};

    Scalar integral = integrator.quadratureAdaptive(spline,
                                                    lower_limit,
                                                    upper_limit,
                                                    absolute_error,
                                                    relative_error,
                                                    Eigen::Integrator<Scalar>::GaussKronrod15);
    
    return integral;
}
