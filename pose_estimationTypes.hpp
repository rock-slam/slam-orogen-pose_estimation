#ifndef pose_estimation_TYPES_HPP
#define pose_estimation_TYPES_HPP

#include <vector>
#include <string>
#include <pose_estimation/Measurement.hpp>
#include <base/Eigen.hpp>
#include <base/Float.hpp>

namespace pose_estimation
{
    
struct MeasurementConfig
{
    base::VectorXd measurement_mask;

    MeasurementConfig() : measurement_mask(MEASUREMENT_SIZE, 1) {measurement_mask.setZero();}
};

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

}

#endif

