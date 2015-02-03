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
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberRoll] = 1;
    config.measurement_mask[BodyStateMemberPitch] = 1;
    config.measurement_mask[BodyStateMemberYaw] = 1;
    handleMeasurement(ts, orientation_samples_sample, config, _imu2body);
}

void VehiclePoseEstimator::position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &position_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberX] = 1;
    config.measurement_mask[BodyStateMemberY] = 1;
    config.measurement_mask[BodyStateMemberZ] = 1;
    handleMeasurement(ts, position_samples_sample, config);
}

void VehiclePoseEstimator::velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample)
{
    MeasurementConfig config;
    config.measurement_mask[BodyStateMemberVx] = 1;
    config.measurement_mask[BodyStateMemberVyaw] = 1;
    handleMeasurement(ts, velocity_samples_sample, config);
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
    updateState();
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
