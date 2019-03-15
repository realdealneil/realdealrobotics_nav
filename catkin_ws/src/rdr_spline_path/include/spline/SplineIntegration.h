/**
 * @file SplineMath.h
 * @copyright Grey Point Corporation
 */

#pragma once

typedef double Scalar;

#include <unsupported/Eigen/Splines>
#include <unsupported/Eigen/NumericalIntegration>


/**
 * The spline function is evaluated in the calculation of the arc length of a curve.
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
     * @param param The parameter at which to evaluate the integrand.
     * @returns the value of the integrand at @a param.
     */
    Scalar operator()(const Scalar param) const
    {
        Eigen::Matrix<Scalar, dimension, 1> derivatives = spline.derivatives(param, 1).col(1);

        Scalar sumOfSquaredDerivatives = derivatives.squaredNorm();

        Scalar integrand = sqrt(sumOfSquaredDerivatives);

        return integrand;
    }

private:
    /**
     * The spline to evaluate the arc length of.
     */
    const Eigen::Spline<Scalar, dimension> spline;

};


template <int dimension>
class SplineIntegration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static Scalar Integrate(const Eigen::Spline<Scalar, dimension>& spline,
                            Eigen::Integrator<Scalar>& integrator);

    static Scalar Integrate(const Eigen::Spline<Scalar, dimension>& spline,
                            Scalar lowerLimit,
                            Scalar upperLimit,
                            Eigen::Integrator<Scalar>& integrator);

private:
    static Eigen::Spline<Scalar, dimension> spline;
};

template <int dimension>
inline Scalar SplineIntegration<dimension>::Integrate(const Eigen::Spline<Scalar, dimension>& spline,
                                               Eigen::Integrator<Scalar>& integrator)
{
    return SplineIntegration<dimension>::Integrate(spline, 0., 1., integrator);
}

template <int dimension>
Scalar SplineIntegration<dimension>::Integrate(const Eigen::Spline<Scalar, dimension>& spline, 
                                        Scalar lowerLimit,
                                        Scalar upperLimit, 
                                        Eigen::Integrator<Scalar>& integrator)
{
    Scalar absolute_error = 1.e-3;
    Scalar relative_error = 1.e-3;

    SplineFunction<dimension> splineFunction(spline);

    Scalar integral = integrator.quadratureAdaptive(splineFunction,
                                                    lowerLimit,
                                                    upperLimit,
                                                    absolute_error,
                                                    relative_error,
                                                    Eigen::Integrator<Scalar>::GaussKronrod15);
    
    return integral;
}
