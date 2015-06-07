#ifndef pose_estimation_TYPES_HPP
#define pose_estimation_TYPES_HPP

#include <vector>
#include <string>
#include <base/Time.hpp>
#include <pose_estimation/Measurement.hpp>
#include <base/Eigen.hpp>

namespace pose_estimation
{
    
struct MeasurementConfig
{
    base::VectorXd measurement_mask;

    MeasurementConfig() : measurement_mask(BODY_STATE_SIZE, 1) {measurement_mask.setZero();}
};

struct OperationalArea
{
    base::Vector2d from;
    base::Vector2d to;
    OperationalArea()
    {
	from.setZero();
	to.setZero();
    }
};

}

#endif

