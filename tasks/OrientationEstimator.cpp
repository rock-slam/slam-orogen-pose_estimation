/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OrientationEstimator.hpp"

using namespace pose_estimation;

OrientationEstimator::OrientationEstimator(std::string const& name)
    : OrientationEstimatorBase(name)
{
}

OrientationEstimator::OrientationEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : OrientationEstimatorBase(name, engine)
{
}

OrientationEstimator::~OrientationEstimator()
{
}

void OrientationEstimator::imu_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sensor_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d imuInBody;
    if (!_imu2body.get(ts, imuInBody))
    {
        RTT::log(RTT::Error) << "skip, couldn't receive a valid imu-in-body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    // do prediction step
    predictionStep(ts);

    if(imu_sensor_samples_sample.gyro.allFinite())
    {
        try
        {
            OrientationUKF::RotationRate measurement;
            current_angular_velocity = imuInBody.rotation() * imu_sensor_samples_sample.gyro;
            measurement.mu = current_angular_velocity;
            orientation_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to integrate gyro measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Angular velocity measurement contains NaN's, it will be skipped!" << RTT::endlog();

    if(imu_sensor_samples_sample.acc.allFinite())
    {
        try
        {
            OrientationUKF::Acceleration measurement;
            measurement.mu = imuInBody.rotation() * imu_sensor_samples_sample.acc;
            orientation_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to integrate acceleration measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Acceleration measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void OrientationEstimator::velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d velocityProviderInBody;
    if (!_velocity_provider2body.get(ts, velocityProviderInBody))
    {
        RTT::log(RTT::Error) << "skip, couldn't receive a valid velocity_provider-in-body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    // receive sensor to body transformation
    Eigen::Affine3d imuInBody;
    if (!_imu2body.get(ts, imuInBody))
    {
        RTT::log(RTT::Error) << "skip, couldn't receive a valid imu-in-body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }

    if(velocity_samples_sample.hasValidVelocity() && velocity_samples_sample.hasValidVelocityCovariance())
    {
        predictionStep(ts);
        try
        {
            // transform velocity to the imu frame
            base::Vector3d velocity = velocityProviderInBody.rotation() * velocity_samples_sample.velocity;
            if(base::isnotnan(current_angular_velocity))
            {
                velocity -= current_angular_velocity.cross(velocityProviderInBody.translation() - imuInBody.translation());
            }

            // apply new velocity measurement
            OrientationUKF::VelocityMeasurement measurement;
            measurement.mu = velocity;
            measurement.cov = velocityProviderInBody.rotation().matrix() * velocity_samples_sample.cov_velocity * velocityProviderInBody.rotation().matrix().transpose();
            orientation_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add DVL measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Info) << "Velocity measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

bool OrientationEstimator::resetHeading(double heading)
{
    // TODO impl
    return false;
}

void OrientationEstimator::predictionStep(const base::Time& sample_time)
{
    try
    {
        orientation_estimator->predictionStepFromSampleTime(sample_time);
    }
    catch(const std::runtime_error& e)
    {
        RTT::log(RTT::Error) << "Failed to execute prediction step: " << e.what() << RTT::endlog();
        RTT::log(RTT::Error) << "Skipping prediction step." << RTT::endlog();
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OrientationEstimator.hpp for more detailed
// documentation about them.

bool OrientationEstimator::configureHook()
{
    if (! OrientationEstimatorBase::configureHook())
        return false;

    // setup initial state
    OrientationUKF::State init_state;
    OrientationUKF::Covariance init_state_cov = OrientationUKF::Covariance::Zero();
    // set initial orientation
    Eigen::Vector3d euler = _initial_orientation.value();
    Eigen::Quaterniond orientation = Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
    init_state.orientation = MTK::SO3<double>(orientation);
    init_state.velocity = VelocityType(Eigen::Vector3d::Zero());
    init_state.bias_gyro = BiasType(Eigen::Vector3d::Zero());
    init_state.bias_acc = BiasType(Eigen::Vector3d::Zero());

    // set initial covariance
    init_state_cov.block(0,0,3,3) = pow(0.02, 2.0) * Eigen::Matrix3d::Identity();
    init_state_cov.block(3,3,3,3) = pow(0.1, 2.0) * Eigen::Matrix3d::Identity();
    init_state_cov.block(6,6,3,3) = pow(_filter_config.value().gyro_bias_std, 2.0) * Eigen::Matrix3d::Identity();
    init_state_cov.block(9,9,3,3) = pow(_filter_config.value().acc_bias_std, 2.0) * Eigen::Matrix3d::Identity();

    // instantiate filter
    orientation_estimator.reset(new pose_estimation::OrientationUKF(init_state, init_state_cov, _filter_config.value()));
    orientation_estimator->setMaxTimeDelta(_max_time_delta.get());

    // setup stream alignment verifier
    verifier.reset(new pose_estimation::StreamAlignmentVerifier());
    verifier->setVerificationInterval(2.0);
    verifier->setDropRateWarningThreshold(0.5);
    verifier->setDropRateCriticalThreshold(1.0);

    current_angular_velocity = Eigen::Vector3d::Zero();

    // Task states
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    return true;
}
bool OrientationEstimator::startHook()
{
    if (! OrientationEstimatorBase::startHook())
        return false;
    return true;
}
void OrientationEstimator::updateHook()
{
    OrientationEstimatorBase::updateHook();

    // check stream alignment status
    unsigned streams_with_alignment_failures = 0;
    unsigned streams_with_critical_alignment_failures = 0;
    verifier->verifyStreamAlignerStatus(_transformer.getStreamAlignerStatus(), streams_with_alignment_failures, streams_with_critical_alignment_failures);
    if(streams_with_alignment_failures > 0)
        new_state = TRANSFORMATION_ALIGNMENT_FAILURES;
    if(streams_with_critical_alignment_failures > 0)
        error(CRITICAL_ALIGNMENT_FAILURE);

    // write out estimated orientation
    OrientationUKF::State current_state;
    OrientationUKF::Covariance current_state_cov;
    if(orientation_estimator->getCurrentState(current_state, current_state_cov))
    {
        base::samples::RigidBodyState orientation_sample;
        orientation_sample.orientation = current_state.orientation;
        orientation_sample.velocity = current_state.velocity;
        orientation_sample.cov_orientation = current_state_cov.block(0,0,3,3);
        orientation_sample.cov_velocity = current_state_cov.block(3,3,3,3);
        orientation_sample.time = orientation_estimator->getLastMeasurementTime();
        orientation_sample.targetFrame = _target_frame.value();
        orientation_sample.sourceFrame = _body_frame.value();
        _orientation_samples.write(orientation_sample);

        _bias_gyro.write(current_state.bias_gyro);
        _bias_acc.write(current_state.bias_acc);
    }

    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void OrientationEstimator::errorHook()
{
    OrientationEstimatorBase::errorHook();
}
void OrientationEstimator::stopHook()
{
    OrientationEstimatorBase::stopHook();
}
void OrientationEstimator::cleanupHook()
{
    OrientationEstimatorBase::cleanupHook();
}
