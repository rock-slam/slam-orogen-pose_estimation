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
    base::Vector3d velocity_bias_instability;

    ProcessNoise() : position_noise(base::Matrix3d::Constant(base::unknown<double>())),
                     orientation_noise(base::Matrix3d::Constant(base::unknown<double>())),
                     velocity_noise(base::Matrix3d::Constant(base::unknown<double>())),
                     angular_velocity_noise(base::Matrix3d::Constant(base::unknown<double>())),
                     velocity_bias_instability(base::Vector3d::Zero()) {}
};

struct PoseUKFSecondaryStates
{
    base::Time time;
    base::Vector3d velocity_bias;
    base::Vector3d cov_velocity_bias;
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

