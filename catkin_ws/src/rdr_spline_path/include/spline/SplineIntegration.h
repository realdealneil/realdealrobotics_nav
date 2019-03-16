/**
 * @file SplineMath.h
 * @copyright Grey Point Corporation
 */

#pragma once

typedef double Scalar;

#include <unsupported/Eigen/Splines>
#include <unsupported/Eigen/NumericalIntegration>


/**
 * @class SplineFunction A spline function, S(u), to be integrated.
 */
template <int dimension>
class SplineFunction
{
public:
    SplineFunction(const Eigen::Spline<Scalar, dimension>& spline)
        : spline(spline)
    {
    }

    /**
     * @param parameter_u The parameter at which to evaluate the integrand.
     * @returns the value of the integrand at @a parameter_u.
     */
    Scalar operator()(const Scalar parameter_u) const
    {
        Eigen::Matrix<Scalar, dimension, 1> derivatives = spline.derivatives(parameter_u, 1).col(1);
        return sqrt(derivatives.squaredNorm());
    }

private:
    /**
     * A spline member variable to allow for const correctness in the NumericalIntegration class.
     */
    const Eigen::Spline<Scalar, dimension> spline;

};


template <int dimension>
class SplineIntegration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static Scalar Integrate(const Eigen::Spline<Scalar, dimension>& spline,
                            Eigen::Integrator<Scalar>& integrator,
                            const Scalar lower_limit = 0.,
                            const Scalar upper_limit = 1.);

private:
    static Eigen::Spline<Scalar, dimension> spline;
};

template <int dimension>
Scalar SplineIntegration<dimension>::Integrate(const Eigen::Spline<Scalar, dimension>& spline, 
                                        Eigen::Integrator<Scalar>& integrator, 
                                        const Scalar lower_limit,
                                        const Scalar upper_limit)
{
    Scalar absolute_error = static_cast<Scalar>(1.e-3);
    Scalar relative_error = static_cast<Scalar>(1.e-3);

    SplineFunction<dimension> splineFunction(spline);
    return integrator.quadratureAdaptive(splineFunction,
                                         lower_limit,
                                         upper_limit,
                                         absolute_error,
                                         relative_error,
                                         Eigen::Integrator<Scalar>::GaussKronrod15);
}
