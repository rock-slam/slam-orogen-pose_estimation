/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BaseTask.hpp"
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/pose_with_velocity/PoseUKF.hpp>
#include <pose_estimation/pose_with_velocity/BodyStateMeasurement.hpp>

using namespace pose_estimation;

BaseTask::BaseTask(std::string const& name)
    : BaseTaskBase(name)
{
}

BaseTask::BaseTask(std::string const& name, RTT::ExecutionEngine* engine)
    : BaseTaskBase(name, engine)
{
}

BaseTask::~BaseTask()
{
}

void BaseTask::handleMeasurement(const base::Time& ts, const base::samples::RigidBodyState& measurement, const MemberMask& member_mask, const transformer::Transformation& sensor2body_transformer)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensor2body;
    if (!sensor2body_transformer.get(ts, sensor2body))
    {
        RTT::log(RTT::Error) << "skip, have no " << sensor2body_transformer.getSourceFrame() << "2" << sensor2body_transformer.getTargetFrame() << " transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    base::samples::RigidBodyState transformed_rbs = measurement;

    // set possible NaN's to zero to allow transformation
    for(unsigned i = 0; i < 3; i++)
    {
        if(member_mask[i] == 0)
            transformed_rbs.position[i] = 0.0;
    }
    for(unsigned i = 0; i < 3; i++)
    {
        if(member_mask[BodyStateMemberVx+i] == 0)
            transformed_rbs.velocity[i] = 0.0;
    }

    // transform measurement in body frame
    if(BodyStateMeasurement::hasPositionMeasurement(member_mask) && BodyStateMeasurement::hasOrientationMeasurement(member_mask))
    {
        transformed_rbs.setTransform(transformed_rbs.getTransform() * sensor2body.inverse());
    }
    else if(BodyStateMeasurement::hasPositionMeasurement(member_mask))
    {
        transformed_rbs.orientation = base::Orientation::Identity();
        transformed_rbs.setTransform(transformed_rbs.getTransform() * sensor2body.inverse());
    }
    else if(BodyStateMeasurement::hasOrientationMeasurement(member_mask))
    {
        transformed_rbs.position = base::Position::Zero();
        transformed_rbs.setTransform(transformed_rbs.getTransform() * sensor2body.inverse());
    }
    
    transformed_rbs.velocity = sensor2body.rotation() * transformed_rbs.velocity;
    if(current_body_state.hasValidAngularVelocity() && !current_body_state.angular_velocity.isZero())
    {
        base::Vector3d euler_angle_velocity = base::getEuler(base::Orientation(Eigen::AngleAxisd(current_body_state.angular_velocity.norm(), current_body_state.angular_velocity.normalized())));
        transformed_rbs.velocity -= Eigen::Vector3d(euler_angle_velocity.z(), euler_angle_velocity.y(), euler_angle_velocity.x()).cross(sensor2body.translation());
    }
    transformed_rbs.angular_velocity = sensor2body.rotation() * transformed_rbs.angular_velocity;
    
    handleMeasurement(ts, transformed_rbs, member_mask);
}

void BaseTask::handleMeasurement(const base::Time& ts, const base::samples::RigidBodyState& rbs, const MemberMask& member_mask)
{
    Measurement measurement;
    BodyStateMeasurement::fromBodyStateToMeasurement(rbs, member_mask, measurement);

    // enqueue new measurement
    if(!pose_estimator->enqueueMeasurement(measurement))
	RTT::log(RTT::Error) << "Failed to add measurement from " << rbs.sourceFrame << "." << RTT::endlog();
}

void BaseTask::handleMeasurement(const base::Time& ts, const base::samples::RigidBodyAcceleration& rba)
{
    Measurement measurement;
    measurement.time = ts;
    measurement.mu = rba.acceleration;
    measurement.cov = rba.cov_acceleration;
    measurement.integration = pose_estimation::UserDefined;
    measurement.measurement_name = "acceleration";

    // enqueue new measurement
    if(!pose_estimator->enqueueMeasurement(measurement))
        RTT::log(RTT::Error) << "Failed to add from acceleration measurement." << RTT::endlog();
}

void BaseTask::updateState()
{
    // integrate measurements
    try
    {
	pose_estimator->integrateMeasurements();
    }
    catch (std::runtime_error e)
    {
	RTT::log(RTT::Error) << "Failed to integrate measurements: " << e.what() << RTT::endlog();
    }
    
    // write estimated body state
    StateAndCovariance current_state;
    if(pose_estimator->getEstimatedState(current_state))
    {
        base::samples::RigidBodyState body_state;
        BodyStateMeasurement::toRigidBodyState(current_state.mu, current_state.cov, body_state);
        current_body_state = body_state;
	body_state.targetFrame = _target_frame.get();
	body_state.sourceFrame = source_frame;
	_pose_samples.write(body_state);
    }
    
    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}

bool BaseTask::setupFilter()
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

    StateAndCovariance init_state;
    BodyStateMeasurement::fromRigidBodyState(init_rbs, init_state.mu, init_state.cov);
    boost::shared_ptr<PoseUKF> ukf(new PoseUKF(init_state));
    pose_estimator.reset(new PoseEstimator(ukf));
    
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

    pose_estimator->setProcessNoise(process_noise_cov);
    
    pose_estimator->setMaxTimeDelta(_max_time_delta.get());

    return true;
}

bool BaseTask::resetState()
{
    return setupFilter();
}

void BaseTask::verifyStreamAlignerStatus(const aggregator::StreamAlignerStatus& status, double verification_interval)
{
    verifier->verifyStreamAlignerStatus(status, aligner_stream_failures);
    
    if(aligner_stream_failures > 0)
	 new_state = TRANSFORMATION_ALIGNMENT_FAILURES;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseTask.hpp for more detailed
// documentation about them.

bool BaseTask::configureHook()
{
    if (! BaseTaskBase::configureHook())
        return false;
    
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    // the source frame should be overwritten in the derived task
    source_frame = "body";
    
    // stream aligner verification
    verifier.reset(new StreamAlignmentVerifier());
    verifier->setVerificationInterval(2.0);
    verifier->setDropRateThreshold(0.5);
    aligner_stream_failures = 0;

    current_body_state.invalidate();
    
    if(!setupFilter())
        return false;
    
    return true;
}
bool BaseTask::startHook()
{
    if (! BaseTaskBase::startHook())
        return false;
    return true;
}
void BaseTask::updateHook()
{
    new_state = RUNNING;
    BaseTaskBase::updateHook();
}
void BaseTask::errorHook()
{
    BaseTaskBase::errorHook();
}
void BaseTask::stopHook()
{
    BaseTaskBase::stopHook();
}
void BaseTask::cleanupHook()
{
    BaseTaskBase::cleanupHook();
}
