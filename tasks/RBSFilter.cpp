/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RBSFilter.hpp"
#include <pose_estimation/pose_with_velocity/BodyStateMeasurement.hpp>

using namespace pose_estimation;

RBSFilter::RBSFilter(std::string const& name)
    : RBSFilterBase(name)
{
}

RBSFilter::RBSFilter(std::string const& name, RTT::ExecutionEngine* engine)
    : RBSFilterBase(name, engine)
{
}

RBSFilter::~RBSFilter()
{
}

void RBSFilter::predictionStep(const base::Time& sample_time)
{
    try
    {
        pose_estimator->predictionStepFromSampleTime(sample_time);
    }
    catch(const std::runtime_error& e)
    {
        RTT::log(RTT::Error) << "Failed to execute prediction step: " << e.what() << RTT::endlog();
        RTT::log(RTT::Error) << "Skipping prediction step." << RTT::endlog();
    }
}

void RBSFilter::writeCurrentState()
{
    // write estimated body state
    PoseUKF::State filter_state;
    PoseUKF::Covariance filter_state_cov;
    base::Time current_sample_time = base::Time::fromMicroseconds(pose_estimator->getLastMeasurementTime());
    if(current_sample_time > last_sample_time && pose_estimator->getCurrentState(filter_state, filter_state_cov))
    {
        base::samples::RigidBodyState body_state;
        BodyStateMeasurement::toRigidBodyState(filter_state, filter_state_cov, body_state);
        body_state.time = current_sample_time;
        body_state.targetFrame = _target_frame.get();
        body_state.sourceFrame = source_frame;
        _pose_samples.write(body_state);
        last_sample_time = current_sample_time;
    }
    
    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}

bool RBSFilter::setupFilter()
{
    // setup initial state
    base::samples::RigidBodyState init_rbs(false);
    init_rbs.initUnknown();
    init_rbs.cov_position = 1.0 * base::Matrix3d::Identity();
    init_rbs.cov_orientation = 1.0 * base::Matrix3d::Identity();
    init_rbs.cov_velocity = 0.1 * base::Matrix3d::Identity();
    init_rbs.cov_angular_velocity = 0.05 * base::Matrix3d::Identity();

    base::samples::RigidBodyState override_init_state = _initial_state.value();
    if(override_init_state.hasValidPosition())
        init_rbs.position = override_init_state.position;
    if(override_init_state.hasValidOrientation())
        init_rbs.orientation = override_init_state.orientation;
    if(override_init_state.hasValidVelocity())
        init_rbs.velocity = override_init_state.velocity;
    if(override_init_state.hasValidAngularVelocity())
        init_rbs.angular_velocity = override_init_state.angular_velocity;

    if(override_init_state.hasValidPositionCovariance())
        init_rbs.cov_position = override_init_state.cov_position;
    if(override_init_state.hasValidOrientationCovariance())
        init_rbs.cov_orientation = override_init_state.cov_orientation;
    if(override_init_state.hasValidVelocityCovariance())
        init_rbs.cov_velocity = override_init_state.cov_velocity;
    if(override_init_state.hasValidAngularVelocityCovariance())
        init_rbs.cov_angular_velocity = override_init_state.cov_angular_velocity;

    PoseUKF::State init_state;
    PoseUKF::Covariance init_state_cov;
    BodyStateMeasurement::fromRigidBodyState(init_rbs, init_state, init_state_cov);
    pose_estimator.reset(new PoseUKF(init_state, init_state_cov));
    
    // setup process noise
    const ProcessNoise& process_noise = _process_noise.value();
    PoseUKF::Covariance process_noise_cov = PoseUKF::Covariance::Zero();

    if(base::isnotnan(process_noise.position_noise))
        process_noise_cov.block(0,0,3,3) = process_noise.position_noise;
    else
        process_noise_cov.block(0,0,3,3) = 0.01 * base::Matrix3d::Identity();
    if(base::isnotnan(process_noise.orientation_noise))
        process_noise_cov.block(3,3,3,3) = process_noise.orientation_noise;
    else
        process_noise_cov.block(3,3,3,3) = 0.001 * base::Matrix3d::Identity();
    if(base::isnotnan(process_noise.velocity_noise))
        process_noise_cov.block(6,6,3,3) = process_noise.velocity_noise;
    else
        process_noise_cov.block(6,6,3,3) = 0.001 * base::Matrix3d::Identity();
    if(base::isnotnan(process_noise.angular_velocity_noise))
        process_noise_cov.block(9,9,3,3) = process_noise.angular_velocity_noise;
    else
        process_noise_cov.block(9,9,3,3) = 0.0001 * base::Matrix3d::Identity();

    pose_estimator->setProcessNoiseCovariance(process_noise_cov);
    
    pose_estimator->setMaxTimeDelta(_max_time_delta.get());

    return true;
}

bool RBSFilter::resetState()
{
    return setupFilter();
}

void RBSFilter::verifyStreamAlignerStatus(const aggregator::StreamAlignerStatus& status, double verification_interval, 
                                          double drop_rate_warning, double drop_rate_critical)
{
    verifier->setVerificationInterval(verification_interval);
    verifier->setDropRateWarningThreshold(drop_rate_warning);
    verifier->setDropRateCriticalThreshold(drop_rate_critical);
    verifier->verifyStreamAlignerStatus(status, aligner_stream_failures, critical_aligner_stream_failures);
    
    if(aligner_stream_failures > 0)
	 new_state = TRANSFORMATION_ALIGNMENT_FAILURES;
    if(critical_aligner_stream_failures > 0)
        exception(CRITICAL_ALIGNMENT_FAILURE);
}

bool RBSFilter::getSensorInBodyPose(const transformer::Transformation& sensor2body_transformer, const base::Time& ts, Eigen::Affine3d& sensorInBody)
{
    // receive sensor to body transformation
    if (!sensor2body_transformer.get(ts, sensorInBody))
    {
        RTT::log(RTT::Error) << "skip, have no " << sensor2body_transformer.getSourceFrame() << "In" << sensor2body_transformer.getTargetFrame() << " transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return false;
    }
    return true;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RBSFilter.hpp for more detailed
// documentation about them.

bool RBSFilter::configureHook()
{
    if (! RBSFilterBase::configureHook())
        return false;
    
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    // the source frame should be overwritten in the derived task
    source_frame = "body";
    last_sample_time = base::Time();
    
    // stream aligner verification
    verifier.reset(new StreamAlignmentVerifier());
    aligner_stream_failures = 0;
    critical_aligner_stream_failures = 0;
    
    if(!setupFilter())
        return false;
    
    return true;
}
bool RBSFilter::startHook()
{
    if (! RBSFilterBase::startHook())
        return false;
    return true;
}
void RBSFilter::updateHook()
{
    new_state = RUNNING;
    RBSFilterBase::updateHook();
}
void RBSFilter::errorHook()
{
    RBSFilterBase::errorHook();
}
void RBSFilter::stopHook()
{
    RBSFilterBase::stopHook();
}
void RBSFilter::cleanupHook()
{
    RBSFilterBase::cleanupHook();
}
