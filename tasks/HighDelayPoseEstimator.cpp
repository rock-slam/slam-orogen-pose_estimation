/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "HighDelayPoseEstimator.hpp"
#include <pose_estimation/pose_with_velocity/BodyStateMeasurement.hpp>

using namespace pose_estimation;

HighDelayPoseEstimator::HighDelayPoseEstimator(std::string const& name)
    : HighDelayPoseEstimatorBase(name)
{
}

HighDelayPoseEstimator::HighDelayPoseEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : HighDelayPoseEstimatorBase(name, engine)
{
}

HighDelayPoseEstimator::~HighDelayPoseEstimator()
{
}

void HighDelayPoseEstimator::pose_samples_slowTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_slow_sample)
{
    if(pose_samples_slow_sample.hasValidPosition() && pose_samples_slow_sample.hasValidPositionCovariance() &&
       pose_samples_slow_sample.hasValidOrientation() && pose_samples_slow_sample.hasValidOrientationCovariance() &&
       pose_samples_slow_sample.hasValidVelocity() && pose_samples_slow_sample.hasValidVelocityCovariance())
    {
        // initialize filter
        if(!pose_estimator->isInitialized())
        {
            PoseUKF::State last_state;
            PoseUKF::Covariance last_state_cov;
            BodyStateMeasurement::fromRigidBodyState(pose_samples_slow_sample, last_state, last_state_cov);
            pose_estimator->initializeFilter(last_state, last_state_cov);
            aligned_slow_pose_sample = pose_samples_slow_sample.getTransform();
            return;
        }

        predictionStep(ts);
        try
        {
            PoseUKF::ZMeasurement z_position_measurement;
            z_position_measurement.mu = pose_samples_slow_sample.position.block(2,0,1,1);
            z_position_measurement.cov = pose_samples_slow_sample.cov_position.block(2,2,1,1);
            pose_estimator->integrateMeasurement(z_position_measurement);

            PoseUKF::XYVelocityMeasurement xy_vel_measurement;
            xy_vel_measurement.mu = pose_samples_slow_sample.velocity.block(0,0,2,1);
            xy_vel_measurement.cov = pose_samples_slow_sample.cov_velocity.block(0,0,2,2);
            pose_estimator->integrateMeasurement(xy_vel_measurement);

            PoseUKF::OrientationMeasurement orientation_measurement;
            orientation_measurement.mu = MTK::SO3<double>::log(Eigen::Quaterniond(pose_samples_slow_sample.orientation));
            orientation_measurement.cov = pose_samples_slow_sample.cov_orientation;
            pose_estimator->integrateMeasurement(orientation_measurement);

            aligned_slow_pose_sample = pose_samples_slow_sample.getTransform();
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add reference pose measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Reference pose measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void HighDelayPoseEstimator::xy_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample)
{
    if(!pose_estimator->isInitialized())
    {
        RTT::log(RTT::Error) << "Waiting for pose samples, filter has not jet been initialized" << RTT::endlog();
        return;
    }

    // receive sensor to body transformation
    Eigen::Affine3d sensor_map2target_map;
    if (!getSensorInBodyPose(_xy_map2target_map, ts, sensor_map2target_map))
        return;

    if(xy_position_samples_sample.position.block(0,0,2,1).allFinite() && xy_position_samples_sample.cov_position.block(0,0,2,2).allFinite())
    {
        predictionStep(ts);
        try
        {
            // this transforms a position measurement expressed in a different map frame to a position expressed in the target map frame
            Eigen::Vector3d transformed_position_sample = sensor_map2target_map * Eigen::Vector3d(xy_position_samples_sample.position.x(), xy_position_samples_sample.position.y(), 0.0);

            PoseUKF::XYMeasurement measurement;
            measurement.mu = transformed_position_sample.block(0,0,2,1);
            measurement.cov = xy_position_samples_sample.cov_position.block(0,0,2,2);
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add delayed position measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Delayed position measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See HighDelayPoseEstimator.hpp for more detailed
// documentation about them.

bool HighDelayPoseEstimator::configureHook()
{
    if (! HighDelayPoseEstimatorBase::configureHook())
        return false;
    
    aligned_slow_pose_sample.matrix() = base::NaN<double>() * Eigen::Matrix<double,4,4>::Ones();
    
    return true;
}
bool HighDelayPoseEstimator::startHook()
{
    if (! HighDelayPoseEstimatorBase::startHook())
        return false;
    return true;
}
void HighDelayPoseEstimator::updateHook()
{
    HighDelayPoseEstimatorBase::updateHook();

    // verify stream aligner status
    verifyStreamAlignerStatus(_transformer);

    // update and write new state
    base::samples::RigidBodyState pose_sample;
    while(_pose_samples_fast.read(pose_sample) == RTT::NewData)
    {
        pose_sample.targetFrame = _target_frame.get();
        pose_sample.sourceFrame = source_frame;
        PoseUKF::State state;
        PoseUKF::Covariance state_cov;
        if(pose_estimator->getCurrentState(state, state_cov) && base::isnotnan(aligned_slow_pose_sample.matrix()))
        {
            base::samples::RigidBodyState filter_state_rbs;
            BodyStateMeasurement::toRigidBodyState(state, state_cov, filter_state_rbs);
            base::samples::RigidBodyState new_state = pose_sample;
	    new_state.cov_position = filter_state_rbs.cov_position;
            new_state.setTransform(filter_state_rbs.getTransform() * (aligned_slow_pose_sample.inverse() * pose_sample.getTransform()));
            _pose_samples.write(new_state);
        }
        else
        {
            _pose_samples.write(pose_sample);
        }
    }

    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        RBSFilterBase::state(new_state);
    }
}
void HighDelayPoseEstimator::errorHook()
{
    HighDelayPoseEstimatorBase::errorHook();
}
void HighDelayPoseEstimator::stopHook()
{
    HighDelayPoseEstimatorBase::stopHook();
}
void HighDelayPoseEstimator::cleanupHook()
{
    HighDelayPoseEstimatorBase::cleanupHook();
}
