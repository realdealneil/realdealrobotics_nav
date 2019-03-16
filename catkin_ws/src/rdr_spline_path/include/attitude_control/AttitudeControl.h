



static constexpr double Gravity{9.80665};   // m/s
static constexpr double _max_accel{5 * Gravity};   // 5 G's

Eigen::Vector3d _gravity_vector{Eigen::Vector3d(0., 0., Gravity)};

Eigen::Vector3d _centripetal_accel_vector{Eigen::Vector3d::Zero()};
Eigen::Vector3d _inertial_accel_vector{Eigen::Vector3d::Zero()};
Eigen::Vector3d _tangent_accel_vector{Eigen::Vector3d::Zero()};
Eigen::Vector3d _tangent_unit_vector{Eigen::Vector3d::Zero()};
Eigen::Vector3d _tangent_normal_vector{Eigen::Vector3d::Zero()};

double max_speed{50.}; // m/s
double min_speed{0.};  // m/s


Vector3d AttitudeControl::calculate_maximum_tangent_speed(const double& parameter_u)
{
    Array<double, Eigen::Spline3d::Dimension, 3> derivatives = _spline.derivatives<derivative_order>(parameter_u);

    _tangent_unit_vector = derivatives.col(1);
    _tangent_normal_vector = derivatives.col(2);

    // Ensure perpendicularity to tangent vector.
    _centripetal_acceleration = _tangent_unit_vector.cross(_tangent_normal_vector.cross(_tangent_unit_vector));
    
    double max_tangent_speed = max_speed;

    if (_centripetal_acceleration.norm() > 0)
    {
        _centripetal_acceleration.normalize();

        // k = |y' x y"| / ||y'^3||
        double curvature = _tangent_unit_vector.cross(_tangent_normal_vector).norm()
                           / pow(_tangent_unit_vector.norm(), 3.0);

        // a = v^2 / r   =>   v^2 * k   =>  v = sqrt(a / k)
        max_tangent_speed = sqrt(_max_accel * curvature);
    }

    return max_tangent_speed;
}

double AttitudeControl::calculate_tangential_accel_magnitude(const double& parameter_u)
{

    double accel_magnitude = 0.;


}



Vector3d Path::calculate_inertial_accel(const double& parameter_u)
{
    calculate_maximum_tangent_speed(parameter_u);
    calculate_tangential_accel_magnitude(parameter_u);
    
    _inertial_accel_vector = _centripetal_accel_vector + _inertial_accel_vector + _gravity_vector;

    if (inertial_accel_vector.norm() == 0.)
    {
        inertial_accel_vector = Vector3d(0., 0., 0.0000001);
    }

    return inertial_accel_vector;
}
