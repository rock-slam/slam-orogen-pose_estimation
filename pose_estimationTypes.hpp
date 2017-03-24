#ifndef pose_estimation_TYPES_HPP
#define pose_estimation_TYPES_HPP

#include <vector>
#include <string>
#include <base/Eigen.hpp>
#include <base/Float.hpp>
#include <base/Time.hpp>

namespace pose_estimation
{

struct ProcessNoise
{
    base::Matrix3d position_noise;
    base::Matrix3d orientation_noise;
    base::Matrix3d velocity_noise;
    base::Matrix3d angular_velocity_noise;

    ProcessNoise() : position_noise(base::Matrix3d::Constant(base::unknown<double>())),
                     orientation_noise(base::Matrix3d::Constant(base::unknown<double>())),
                     velocity_noise(base::Matrix3d::Constant(base::unknown<double>())),
                     angular_velocity_noise(base::Matrix3d::Constant(base::unknown<double>())) {}
};

struct OrientationUKFSecondaryStates
{
    base::Time time;
    base::Vector3d bias_gyro;
    base::Matrix3d cov_bias_gyro;
    base::Vector3d bias_acc;
    base::Matrix3d cov_bias_acc;
    double gravity;
    double var_gravity;
    double latitude;
    double var_latitude;
};

namespace deprecated
{
    /** @deprecated
     */
    enum FilterType
    {
        UKF
    };
}

}

#endif

