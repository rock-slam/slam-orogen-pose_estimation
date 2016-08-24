/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "VehiclePoseEstimator.hpp"

using namespace pose_estimation;

VehiclePoseEstimator::VehiclePoseEstimator(std::string const& name)
    : VehiclePoseEstimatorBase(name)
{
}

VehiclePoseEstimator::VehiclePoseEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : VehiclePoseEstimatorBase(name, engine)
{
}

VehiclePoseEstimator::~VehiclePoseEstimator()
{
}

void VehiclePoseEstimator::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_imu2body, ts, sensorInBody))
        return;

    if(orientation_samples_sample.hasValidOrientation() && orientation_samples_sample.hasValidOrientationCovariance())
    {
        predictionStep(ts);
        try
        {
            Eigen::Quaterniond transformed_orientation((orientation_samples_sample.orientation * sensorInBody.inverse()).linear());

            PoseUKF::OrientationMeasurement measurement;
            measurement.mu = MTK::SO3<double>::log(transformed_orientation);
            measurement.cov = sensorInBody.rotation() * orientation_samples_sample.cov_orientation * sensorInBody.rotation().transpose();
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add orientation measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Orientation measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void VehiclePoseEstimator::position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &position_samples_sample)
{
    if(position_samples_sample.hasValidPosition() && position_samples_sample.hasValidPositionCovariance())
    {
        predictionStep(ts);
        try
        {
            PoseUKF::PositionMeasurement measurement;
            measurement.mu = position_samples_sample.position;
            measurement.cov = position_samples_sample.cov_position;
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add position measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Position measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void VehiclePoseEstimator::velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample)
{
    if(!base::isNaN(velocity_samples_sample.velocity.x()) && !base::isNaN(velocity_samples_sample.cov_velocity(0,0)) &&
       !base::isNaN(velocity_samples_sample.angular_velocity.z()) && !base::isNaN(velocity_samples_sample.cov_angular_velocity(2,2)))
    {
        predictionStep(ts);
        try
        {
            PoseUKF::XVelYawVelMeasurement measurement;
            measurement.mu = PoseUKF::XYVelocityMeasurement::Mu(velocity_samples_sample.velocity.x(), velocity_samples_sample.angular_velocity.z());
            measurement.cov << velocity_samples_sample.cov_velocity(0,0), 0.0,
                               0.0, velocity_samples_sample.cov_angular_velocity(2,2);
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add velocity measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Velocity measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See VehiclePoseEstimator.hpp for more detailed
// documentation about them.

bool VehiclePoseEstimator::configureHook()
{
    if (! VehiclePoseEstimatorBase::configureHook())
        return false;
    
    source_frame = _body_frame.get();
    
    return true;
}
bool VehiclePoseEstimator::startHook()
{
    if (! VehiclePoseEstimatorBase::startHook())
        return false;
    return true;
}
void VehiclePoseEstimator::updateHook()
{
    VehiclePoseEstimatorBase::updateHook();
    
    // verify stream aligner status
    verifyStreamAlignerStatus(_transformer);
    
    // update and write new state
    writeCurrentState();
}
void VehiclePoseEstimator::errorHook()
{
    VehiclePoseEstimatorBase::errorHook();
}
void VehiclePoseEstimator::stopHook()
{
    VehiclePoseEstimatorBase::stopHook();
}
void VehiclePoseEstimator::cleanupHook()
{
    VehiclePoseEstimatorBase::cleanupHook();
}
