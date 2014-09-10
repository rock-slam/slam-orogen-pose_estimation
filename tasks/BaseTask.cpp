/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BaseTask.hpp"

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
    transformed_rbs.position = sensor2body * transformed_rbs.position;
    transformed_rbs.orientation = sensor2body.rotation() * transformed_rbs.orientation;
    transformed_rbs.velocity = sensor2body.rotation() * transformed_rbs.velocity;
    transformed_rbs.angular_velocity = sensor2body.rotation() * transformed_rbs.angular_velocity;
    
    handleMeasurement(ts, transformed_rbs, config);
}

void BaseTask::handleMeasurement(const base::Time& ts, const base::samples::RigidBodyState& measurement, const MeasurementConfig& config)
{
    // enqueue new measurement
    if(!pose_estimator->enqueueMeasurement(measurement, config.measurement_mask.cast<unsigned>()))
	RTT::log(RTT::Error) << "Failed to add measurement from " << measurement.sourceFrame << "." << RTT::endlog();
}

void BaseTask::updateState()
{
    // integrate measurements
    pose_estimator->integrateMeasurements();
    
    // write estimated body state
    base::samples::RigidBodyState body_state;
    if(pose_estimator->getEstimatedState(body_state))
    {
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
    
    pose_estimator.reset(new PoseEstimator(_filter_type.get()));
    
    base::samples::RigidBodyState init_state(false);
    init_state.initUnknown();
    init_state.cov_position = 1.0 * base::Matrix3d::Identity();
    init_state.cov_orientation = 1.0 * base::Matrix3d::Identity();
    init_state.cov_velocity = 0.1 * base::Matrix3d::Identity();
    init_state.cov_angular_velocity = 0.05 * base::Matrix3d::Identity();
    pose_estimator->setInitialState(init_state);
    
    Covariance process_noise = Covariance::Zero();
    process_noise.block(0,0,3,3) = 0.01 * base::Matrix3d::Identity();
    process_noise.block(3,3,3,3) = 0.001 * base::Matrix3d::Identity();
    process_noise.block(6,6,3,3) = 0.001 * base::Matrix3d::Identity();
    process_noise.block(9,9,3,3) = 0.0001 * base::Matrix3d::Identity();
    pose_estimator->setProcessNoise(process_noise);
    
    pose_estimator->setMaxTimeDelta(_max_time_delta.get());
    
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
