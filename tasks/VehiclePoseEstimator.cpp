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
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberRoll] = 1;
    measurement_mask[BodyStateMemberPitch] = 1;
    measurement_mask[BodyStateMemberYaw] = 1;
    handleMeasurement(ts, orientation_samples_sample, measurement_mask, _imu2body);
}

void VehiclePoseEstimator::position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &position_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberX] = 1;
    measurement_mask[BodyStateMemberY] = 1;
    measurement_mask[BodyStateMemberZ] = 1;
    handleMeasurement(ts, position_samples_sample, measurement_mask);
}

void VehiclePoseEstimator::velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberVx] = 1;
    measurement_mask[BodyStateMemberVyaw] = 1;
    handleMeasurement(ts, velocity_samples_sample, measurement_mask);
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
