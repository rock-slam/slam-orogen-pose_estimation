/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BaseTask.hpp"
#include <pose_estimation/Measurement.hpp>

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

void BaseTask::handleMeasurement(const base::Time& ts, const base::samples::RigidBodyState& measurement, const MeasurementConfig& config, const transformer::Transformation& sensor2body_transformer)
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
        if(config.measurement_mask[i] == 0)
            transformed_rbs.position[i] = 0.0;
    }
    for(unsigned i = 6; i < 9; i++)
    {
        if(config.measurement_mask[i] == 0)
            transformed_rbs.velocity[i] = 0.0;
    }

    // transform measurement in body frame
    Measurement m;
    m.member_mask = config.measurement_mask.cast<unsigned>();
    if(m.hasPositionMeasurement() && m.hasOrientationMeasurement())
    {
        transformed_rbs.setTransform(transformed_rbs.getTransform() * sensor2body.inverse());
    }
    else if(m.hasPositionMeasurement())
    {
        transformed_rbs.orientation = base::Orientation::Identity();
        transformed_rbs.setTransform(transformed_rbs.getTransform() * sensor2body.inverse());
    }
    else if(m.hasOrientationMeasurement())
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
    
    handleMeasurement(ts, transformed_rbs, config);
}

void BaseTask::handleMeasurement(const base::Time& ts, const base::samples::RigidBodyState& measurement, const MeasurementConfig& config)
{
    // enqueue new measurement
    if(!pose_estimator->enqueueMeasurement(measurement, config.measurement_mask.cast<unsigned>()))
	RTT::log(RTT::Error) << "Failed to add measurement from " << measurement.sourceFrame << "." << RTT::endlog();
}

void BaseTask::handleMeasurement(const base::Time& ts, const base::samples::RigidBodyState& measurement, const base::samples::RigidBodyAcceleration& measurement_acc, const MeasurementConfig& config)
{
    // enqueue new measurement
    if(!pose_estimator->enqueueMeasurement(ts, measurement, measurement_acc, config.measurement_mask.cast<unsigned>()))
        RTT::log(RTT::Error) << "Failed to add measurement from " << measurement.sourceFrame << "." << RTT::endlog();
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
    base::samples::RigidBodyState body_state;
    if(pose_estimator->getEstimatedState(body_state))
    {
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
    pose_estimator.reset(new PoseEstimator(_filter_type.get()));
    
    // setup initial state
    base::samples::RigidBodyState init_state(false);
    init_state.initUnknown();
    init_state.cov_position = 1.0 * base::Matrix3d::Identity();
    init_state.cov_orientation = 1.0 * base::Matrix3d::Identity();
    init_state.cov_velocity = 0.1 * base::Matrix3d::Identity();
    init_state.cov_angular_velocity = 0.05 * base::Matrix3d::Identity();

    base::samples::RigidBodyState override_init_state = _initial_state.value();
    if(override_init_state.hasValidPosition())
        init_state.position = override_init_state.position;
    if(override_init_state.hasValidOrientation())
        init_state.orientation = override_init_state.orientation;
    if(override_init_state.hasValidVelocity())
        init_state.velocity = override_init_state.velocity;    
    if(override_init_state.hasValidAngularVelocity())
        init_state.angular_velocity = override_init_state.angular_velocity;

    if(override_init_state.hasValidPositionCovariance())
        init_state.cov_position = override_init_state.cov_position;
    if(override_init_state.hasValidOrientationCovariance())
        init_state.cov_orientation = override_init_state.cov_orientation;
    if(override_init_state.hasValidVelocityCovariance())
        init_state.cov_velocity = override_init_state.cov_velocity;    
    if(override_init_state.hasValidAngularVelocityCovariance())
        init_state.cov_angular_velocity = override_init_state.cov_angular_velocity;

    pose_estimator->setInitialState(init_state);
    
    // setup process noise
    const ProcessNoise& process_noise = _process_noise.value();
    Covariance process_noise_cov = Covariance::Zero();

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
    if((status.time - aligner_last_verified).toSeconds() > verification_interval)
    {
	aligner_stream_failures = 0;
	for(std::vector<aggregator::StreamStatus>::const_iterator it = status.streams.begin(); 
	    it != status.streams.end(); it++)
	{
	    // no samples received
	    if(aligner_samples_received[it->name] == 0)
	    {
		aligner_samples_received[it->name] = it->samples_received;
		continue;
	    }
	    
	    size_t new_samples_received = it->samples_received - aligner_samples_received[it->name];
	    size_t samples_dropped = it->samples_dropped_buffer_full + it->samples_dropped_late_arriving + it->samples_backward_in_time;
	    size_t new_samples_dropped = samples_dropped - aligner_samples_dropped[it->name];
	    
	    // check if more than 50% of samples are dropped
	    double drop_rate = (double)new_samples_dropped / (double)new_samples_received;
	    if(drop_rate > 0.5)
	    {
		aligner_stream_failures++;
		RTT::log(RTT::Error) << "Transformation alignment failure in stream " << it->name <<
					". " << drop_rate * 100.0 << "% of all samples were dropped in the last " << 
					verification_interval << " seconds." << RTT::endlog();
	    }
	    
	    aligner_samples_received[it->name] = it->samples_received;
	    aligner_samples_dropped[it->name] = samples_dropped;
	}
	aligner_last_verified = status.time;
    }
    
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
    aligner_last_verified.microseconds = 0;
    aligner_stream_failures = 0;
    aligner_samples_received.clear();
    aligner_samples_dropped.clear();
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
