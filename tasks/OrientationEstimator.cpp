/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OrientationEstimator.hpp"
#include <pose_estimation/orientation_estimator/OrientationUKF.hpp>
#include <pose_estimation/EulerConversion.hpp>

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

    if(base::isnotnan(imu_sensor_samples_sample.gyro))
    {
        // enqueue new gyro measurement
        pose_estimation::Measurement gyro_measurement;
        gyro_measurement.time = ts;
        gyro_measurement.integration = pose_estimation::UserDefined;
        gyro_measurement.measurement_name = OrientationUKF::rotation_rate_measurement;
        current_angular_velocity = imuInBody.rotation() * imu_sensor_samples_sample.gyro;
        gyro_measurement.mu = current_angular_velocity;
        if(!pose_estimator->enqueueMeasurement(gyro_measurement))
            RTT::log(RTT::Error) << "Failed to add angular velocity measurement." << RTT::endlog();
    }
    else
        RTT::log(RTT::Error) << "Angular velocity measurement contains NaN's, it will be skipped!" << RTT::endlog();

    if(base::isnotnan(imu_sensor_samples_sample.acc))
    {
        // enqueue new acceleration measurement
        pose_estimation::Measurement acc_measurement;
        acc_measurement.time = ts;
        acc_measurement.integration = pose_estimation::UserDefined;
        acc_measurement.measurement_name = OrientationUKF::acceleration_measurement;
        acc_measurement.mu = imuInBody.rotation() * imu_sensor_samples_sample.acc;
        if(!pose_estimator->enqueueMeasurement(acc_measurement))
            RTT::log(RTT::Error) << "Failed to add acceleration measurement." << RTT::endlog();
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
        // transform velocity to the imu frame
        base::Vector3d velocity = velocityProviderInBody.rotation() * velocity_samples_sample.velocity;
        if(base::isnotnan(current_angular_velocity) && !current_angular_velocity.isZero())
        {
            base::Vector3d euler_angle_velocity = base::getEuler(base::Orientation(Eigen::AngleAxisd(current_angular_velocity.norm(), current_angular_velocity.normalized())));
            velocity -= Eigen::Vector3d(euler_angle_velocity.z(), euler_angle_velocity.y(), euler_angle_velocity.x()).cross(velocityProviderInBody.translation() - imuInBody.translation());
        }

        // add velocity measurement
        pose_estimation::Measurement measurement;
        measurement.time = ts;
        measurement.measurement_name = OrientationUKF::velocity_measurement;
        measurement.integration = pose_estimation::UserDefined;
        measurement.mu = velocity;
        measurement.cov = velocityProviderInBody.rotation().matrix() * velocity_samples_sample.cov_velocity * velocityProviderInBody.rotation().matrix().transpose();
        if(!pose_estimator->enqueueMeasurement(measurement))
            RTT::log(RTT::Error) << "Failed to add velocity measurement." << RTT::endlog();
    }
    else
        RTT::log(RTT::Error) << "Velocity measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

bool OrientationEstimator::resetHeading(double heading)
{
    // TODO impl
    return false;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OrientationEstimator.hpp for more detailed
// documentation about them.

bool OrientationEstimator::configureHook()
{
    if (! OrientationEstimatorBase::configureHook())
        return false;

    // setup initial state
    OrientationUKF::FilterState init_state;
    init_state.mu = OrientationUKF::Mu::Zero();
    init_state.cov = OrientationUKF::Covariance::Zero();
    // set initial orientation
    init_state.mu.block(0,0,3,1) = _initial_orientation.value();

    // set initial covariance
    init_state.cov.block(0,0,3,3) = pow(0.02, 2.0) * Eigen::Matrix3d::Identity();
    init_state.cov.block(3,3,3,3) = pow(0.1, 2.0) * Eigen::Matrix3d::Identity();
    init_state.cov.block(6,6,3,3) = pow(_filter_config.value().gyro_bias_std, 2.0) * Eigen::Matrix3d::Identity();
    init_state.cov.block(9,9,3,3) = pow(_filter_config.value().acc_bias_std, 2.0) * Eigen::Matrix3d::Identity();

    // instantiate filter
    boost::shared_ptr<OrientationUKF> filter(new OrientationUKF(init_state, _filter_config.value()));
    pose_estimator.reset(new pose_estimation::PoseEstimator(filter));
    pose_estimator->setMaxTimeDelta(_max_time_delta.get());

    // setup stream alignment verifier
    verifier.reset(new pose_estimation::StreamAlignmentVerifier());
    verifier->setVerificationInterval(2.0);
    verifier->setDropRateThreshold(0.5);

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

    // integrate measurements
    try
    {
        pose_estimator->integrateMeasurements();
    }
    catch (std::runtime_error e)
    {
        RTT::log(RTT::Error) << "Failed to integrate measurements: " << e.what() << RTT::endlog();
    }

    // check stream alignment status
    unsigned streams_with_alignment_failures = 0;
    verifier->verifyStreamAlignerStatus(_transformer.getStreamAlignerStatus(), streams_with_alignment_failures);
    if(streams_with_alignment_failures > 0)
        new_state = TRANSFORMATION_ALIGNMENT_FAILURES;

    // write out estimated orientation
    OrientationUKF::FilterState current_state;
    if(pose_estimator->getEstimatedState(current_state))
    {
        base::samples::RigidBodyState orientation_sample;
        EulerConversion::eulerToQuad(current_state.mu.block(0,0,3,1), orientation_sample.orientation);
        orientation_sample.velocity = current_state.mu.block(3,0,3,1);
        orientation_sample.cov_orientation = current_state.cov.block(0,0,3,3);
        orientation_sample.cov_velocity = current_state.cov.block(3,3,3,3);
        orientation_sample.time = pose_estimator->getLastMeasurementTime();
        orientation_sample.targetFrame = _target_frame.value();
        orientation_sample.sourceFrame = _body_frame.value();
        _orientation_samples.write(orientation_sample);

        _bias_gyro.write(base::Vector3d(current_state.mu.block(6,0,3,1)));
        _bias_acc.write(base::Vector3d(current_state.mu.block(9,0,3,1)));
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
