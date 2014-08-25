/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AvalonPoseEstimator.hpp"

using namespace pose_estimation;

AvalonPoseEstimator::AvalonPoseEstimator(std::string const& name)
    : AvalonPoseEstimatorBase(name)
{
}

AvalonPoseEstimator::AvalonPoseEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : AvalonPoseEstimatorBase(name, engine)
{
}

AvalonPoseEstimator::~AvalonPoseEstimator()
{
}

void AvalonPoseEstimator::depth_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberZ] = 1;
    handleMeasurement(ts, depth_samples_sample, config, _pressure_sensor2body);
}

void AvalonPoseEstimator::model_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &model_velocity_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberVx] = 1;
    config.measurement_mask[BodyStateMemberVy] = 1;
    handleMeasurement(ts, model_velocity_samples_sample, config);
}

void AvalonPoseEstimator::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberRoll] = 1;
    config.measurement_mask[BodyStateMemberPitch] = 1;
    config.measurement_mask[BodyStateMemberYaw] = 1;
    config.measurement_mask[BodyStateMemberVroll] = 1;
    config.measurement_mask[BodyStateMemberVpitch] = 1;
    config.measurement_mask[BodyStateMemberVyaw] = 1;
    handleMeasurement(ts, orientation_samples_sample, config, _imu2body);
}

void AvalonPoseEstimator::xy_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberX] = 1;
    config.measurement_mask[BodyStateMemberY] = 1;
    handleMeasurement(ts, xy_position_samples_sample, config);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AvalonPoseEstimator.hpp for more detailed
// documentation about them.

bool AvalonPoseEstimator::configureHook()
{
    if (! AvalonPoseEstimatorBase::configureHook())
        return false;
    
    source_frame = _body_frame.get();
    
    return true;
}
bool AvalonPoseEstimator::startHook()
{
    if (! AvalonPoseEstimatorBase::startHook())
        return false;
    return true;
}
void AvalonPoseEstimator::updateHook()
{
    AvalonPoseEstimatorBase::updateHook();
    
    // update and write new state
    updateState();
}
void AvalonPoseEstimator::errorHook()
{
    AvalonPoseEstimatorBase::errorHook();
}
void AvalonPoseEstimator::stopHook()
{
    AvalonPoseEstimatorBase::stopHook();
}
void AvalonPoseEstimator::cleanupHook()
{
    AvalonPoseEstimatorBase::cleanupHook();
}
