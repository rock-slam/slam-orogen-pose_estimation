/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "HighDelayPoseEstimator.hpp"

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
    DelayedMeasurement measurement;
    measurement.measurement = pose_samples_slow_sample;
    measurement.ts = ts;
    measurement.config.measurement_mask[BodyStateMemberX] = 0;
    measurement.config.measurement_mask[BodyStateMemberY] = 0;
    measurement.config.measurement_mask[BodyStateMemberZ] = 1;
    measurement.config.measurement_mask[BodyStateMemberVx] = 1;
    measurement.config.measurement_mask[BodyStateMemberVy] = 1;
    measurement.config.measurement_mask[BodyStateMemberVz] = 1;
    measurement.config.measurement_mask[BodyStateMemberRoll] = 1;
    measurement.config.measurement_mask[BodyStateMemberPitch] = 1;
    measurement.config.measurement_mask[BodyStateMemberYaw] = 1;
    measurement.config.measurement_mask[BodyStateMemberVroll] = 1;
    measurement.config.measurement_mask[BodyStateMemberVpitch] = 1;
    measurement.config.measurement_mask[BodyStateMemberVyaw] = 1;
    // transform velocity to source frame, since the input is expected to be this way
    measurement.measurement.velocity = pose_samples_slow_sample.orientation.inverse() * pose_samples_slow_sample.velocity;
    delayed_measurements.push_back(measurement);
}

void HighDelayPoseEstimator::xy_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample)
{
    handleDelayedMeasurements(ts);
    
    // receive sensor to body transformation
    Eigen::Affine3d sensor_map2target_map;
    if (!_xy_map2target_map.get(ts, sensor_map2target_map))
    {
        RTT::log(RTT::Error) << "skip, have no " << _xy_map2target_map.getSourceFrame() << "2" << _xy_map2target_map.getTargetFrame() << std::endl;
        new_state = BaseTask::MISSING_TRANSFORMATION;
        return;
    }

    base::samples::RigidBodyState transformed_position_sample = xy_position_samples_sample;
    transformed_position_sample.position = sensor_map2target_map * Eigen::Vector3d(xy_position_samples_sample.position.x(), xy_position_samples_sample.position.y(), 0.0);

	if(use_operational_area)
	{
		if(operational_area.contains(Eigen::Vector2d(transformed_position_sample.position.x(), transformed_position_sample.position.y())))
		{
		    MeasurementConfig config;
		    config.measurement_mask[BodyStateMemberX] = 1;
		    config.measurement_mask[BodyStateMemberY] = 1;
		    handleMeasurement(ts, transformed_position_sample, config);
		}
		else
		{
			RTT::log(RTT::Error) << "Measurement position " << transformed_position_sample.position.transpose() << " is outside of the operational area! Measurement will be skiped." << std::endl;
		}
	}
	else
	{
	    MeasurementConfig config;
	    config.measurement_mask[BodyStateMemberX] = 1;
	    config.measurement_mask[BodyStateMemberY] = 1;
	    handleMeasurement(ts, transformed_position_sample, config);
	}
}

void HighDelayPoseEstimator::handleDelayedMeasurements(const base::Time& ts)
{
    std::list<DelayedMeasurement>::iterator it = delayed_measurements.begin();
    while(it != delayed_measurements.end() && it->ts <= ts)
    {
        aligned_slow_pose_sample = it->measurement.getTransform();
        handleMeasurement(it->ts, it->measurement, it->config);
        it = delayed_measurements.erase(it);
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See HighDelayPoseEstimator.hpp for more detailed
// documentation about them.

bool HighDelayPoseEstimator::configureHook()
{
    if (! HighDelayPoseEstimatorBase::configureHook())
        return false;
    
    aligned_slow_pose_sample.matrix() = base::NaN<double>() * Eigen::Matrix<double,4,4>::Ones();

	operational_area.setEmpty();
	if(_operational_area.value().from != _operational_area.value().to)
	{
		use_operational_area = true;
		operational_area.extend(_operational_area.value().from);
		operational_area.extend(_operational_area.value().to);
	}
	else
		use_operational_area = false;
	
    
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
    
    // integrate measurements
    try
    {
        pose_estimator->integrateMeasurements();
    }
    catch (std::runtime_error e)
    {
        RTT::log(RTT::Error) << "Failed to integrate measurements: " << e.what() << RTT::endlog();
    }

    // update and write new state
    base::samples::RigidBodyState pose_sample;
    while(_pose_samples_fast.read(pose_sample) == RTT::NewData)
    {
        pose_sample.targetFrame = _target_frame.get();
        pose_sample.sourceFrame = source_frame;
        base::samples::RigidBodyState filter_state;
        if(pose_estimator->getEstimatedState(filter_state) && base::isnotnan(aligned_slow_pose_sample.matrix()))
        {
            base::samples::RigidBodyState new_state = pose_sample;
	    new_state.cov_position = filter_state.cov_position;
            new_state.setTransform(filter_state.getTransform() * (aligned_slow_pose_sample.inverse() * pose_sample.getTransform()));
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
        BaseTaskBase::state(new_state);
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
