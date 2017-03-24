/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OrientationEstimator.hpp"
#include "pose_estimationTypes.hpp"
#include <pose_estimation/GravityModel.hpp>

using namespace pose_estimation;

typedef OrientationUKF::WState FilterState;

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
    // do prediction step
    predictionStep(ts);

    try
    {
        OrientationUKF::RotationRate measurement;
        measurement.mu = imu_sensor_samples_sample.gyro;
        measurement.cov = cov_angular_velocity;
        orientation_estimator->integrateMeasurement(measurement);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate gyro measurement: " << e.what();
    }

    try
    {
        OrientationUKF::Acceleration measurement;
        measurement.mu = imu_sensor_samples_sample.acc;
        measurement.cov = cov_acceleration;
        orientation_estimator->integrateMeasurement(measurement);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate acceleration measurement: " << e.what();
    }

}

void OrientationEstimator::velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d velocityProviderInBody;
    if (!_velocity_provider2body.get(ts, velocityProviderInBody))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid velocity_provider-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    Eigen::Affine3d velocityProviderInIMU = imu_in_body.inverse() * velocityProviderInBody;

    if(velocity_samples_sample.hasValidVelocity() && velocity_samples_sample.hasValidVelocityCovariance())
    {
        try
        {
            // transform velocity to the imu frame
            base::Vector3d velocity = velocityProviderInIMU.rotation() * velocity_samples_sample.velocity;
            velocity -= orientation_estimator->getRotationRate().cross(velocityProviderInIMU.translation());

            // apply new velocity measurement
            OrientationUKF::VelocityMeasurement measurement;
            measurement.mu = velocity;
            measurement.cov = velocityProviderInIMU.rotation().matrix() * velocity_samples_sample.cov_velocity * velocityProviderInIMU.rotation().matrix().transpose();
            orientation_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            LOG_ERROR_S << "Failed to add DVL measurement: " << e.what();
        }
    }
    else
        LOG_ERROR_S << "Velocity measurement contains NaN's, it will be skipped!";
}

bool OrientationEstimator::addHeadingOffset(double heading_offset, double variance)
{
    OrientationUKF::State current_state;
    OrientationUKF::Covariance current_state_cov;
    if(!orientation_estimator->getCurrentState(current_state, current_state_cov))
    {
        LOG_ERROR_S << "Failed to get current state from filter!";
        return false;
    }

    Eigen::Vector3d euler = current_state.orientation.matrix().eulerAngles(2,1,0);
    Eigen::Quaterniond orientation = Eigen::AngleAxisd(euler[0] + heading_offset, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d orientation_cov = Eigen::Matrix3d::Zero();
    orientation_cov.topLeftCorner<2,2>() = current_state_cov.topLeftCorner<2,2>();
    orientation_cov(2,2) = variance;

    return initializeFilter(orientation, orientation_cov, _filter_config.value());
}

void OrientationEstimator::predictionStep(const base::Time& sample_time)
{
    try
    {
        orientation_estimator->predictionStepFromSampleTime(sample_time);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to execute prediction step: " << e.what();
        LOG_ERROR_S << "Skipping prediction step.";
    }
}

bool OrientationEstimator::initializeFilter(const Eigen::Quaterniond& orientation, const Eigen::Matrix3d& orientation_cov, const OrientationUKFConfig& filter_config)
{
    if(!orientation.matrix().allFinite() || !orientation_cov.allFinite())
    {
        LOG_ERROR_S << "Undefined values in initial orientation!";
        return false;
    }

    FilterState initial_state;
    initial_state.orientation = RotationType(MTK::SO3<double>(orientation));
    initial_state.velocity = VelocityType(Eigen::Vector3d::Zero());
    initial_state.bias_gyro = BiasType(filter_config.rotation_rate.bias_offset);
    initial_state.bias_acc = BiasType(filter_config.acceleration.bias_offset);
    Eigen::Matrix<double, 1, 1> gravity;
    gravity(0) = pose_estimation::GravityModel(filter_config.location.latitude, filter_config.location.altitude);
    initial_state.gravity = GravityType(gravity);
    Eigen::Matrix<double, 1, 1> latitude;
    latitude(0) = filter_config.location.latitude;
    initial_state.latitude = LatitudeType(latitude);

    OrientationUKF::Covariance initial_state_cov = OrientationUKF::Covariance::Zero();
    MTK::subblock(initial_state_cov, &FilterState::orientation) = orientation_cov;
    MTK::subblock(initial_state_cov, &FilterState::velocity) = Eigen::Matrix3d::Identity(); // velocity is unknown at the start
    MTK::subblock(initial_state_cov, &FilterState::bias_gyro) = filter_config.rotation_rate.bias_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(initial_state_cov, &FilterState::bias_acc) = filter_config.acceleration.bias_instability.cwiseAbs2().asDiagonal();
    Eigen::Matrix<double, 1, 1> gravity_var;
    gravity_var << pow(0.05, 2.); // give the gravity model a sigma of 5 cm/s^2 at the start
    MTK::subblock(initial_state_cov, &FilterState::gravity) = gravity_var;
    Eigen::Matrix<double, 1, 1> latitude_var;
    latitude_var << pow(atan2(50.0, pose_estimation::EQUATORIAL_RADIUS), 2.); // sigma of 50 m on earth surface
    MTK::subblock(initial_state_cov, &FilterState::latitude) = latitude_var;

    orientation_estimator.reset(new OrientationUKF(initial_state, initial_state_cov,
                                  filter_config.rotation_rate.bias_tau, filter_config.acceleration.bias_tau));
    return true;
}

bool OrientationEstimator::setProcessNoise(const OrientationUKFConfig& filter_config, double sensor_delta_t)
{
    OrientationUKF::Covariance process_noise_cov = OrientationUKF::Covariance::Zero();
    MTK::subblock(process_noise_cov, &FilterState::orientation) = filter_config.rotation_rate.randomwalk.cwiseAbs2().asDiagonal();
    MTK::subblock(process_noise_cov, &FilterState::velocity) = filter_config.acceleration.randomwalk.cwiseAbs2().asDiagonal();
    MTK::subblock(process_noise_cov, &FilterState::bias_gyro) = (2. / (filter_config.rotation_rate.bias_tau * sensor_delta_t)) *
                                        filter_config.rotation_rate.bias_instability.cwiseAbs2().asDiagonal();
    MTK::subblock(process_noise_cov, &FilterState::bias_acc) = (2. / (filter_config.acceleration.bias_tau * sensor_delta_t)) *
                                        filter_config.acceleration.bias_instability.cwiseAbs2().asDiagonal();
    Eigen::Matrix<double, 1, 1> gravity_noise;
    gravity_noise << 1.e-12; // add a tiny bit of noise only for numeric stability
    MTK::subblock(process_noise_cov, &FilterState::gravity) = gravity_noise;
    Eigen::Matrix<double, 1, 1> latitude_noise;
    latitude_noise << pow(atan2(1.0, pose_estimation::EQUATORIAL_RADIUS), 2.); // 1m/s on earth surface
    MTK::subblock(process_noise_cov, &FilterState::latitude) = latitude_noise;
    orientation_estimator->setProcessNoiseCovariance(process_noise_cov);

    return true;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OrientationEstimator.hpp for more detailed
// documentation about them.

bool OrientationEstimator::configureHook()
{
    if (! OrientationEstimatorBase::configureHook())
        return false;

    // get IMU to body transformation
    if(!_imu2body.get(base::Time(), imu_in_body))
    {
        LOG_ERROR_S << "Failed to get IMU pose in body frame. Note that this has to be a static transformation!";
        return false;
    }
    if(!imu_in_body.linear().isApprox(Eigen::Matrix3d::Identity()))
    {
        LOG_ERROR_S << "The IMU frame can't be rotated with respect to the body frame!";
        LOG_ERROR_S << "This is currently not supported by the filter.";
        return false;
    }

    // set initial orientation
    Eigen::Vector3d euler = _initial_orientation.value();
    Eigen::Quaterniond orientation = Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());

    // initialize filter
    if(!initializeFilter(orientation, _initial_orientation_cov_diag.value().asDiagonal(), _filter_config.value()))
        return false;

    // set process noise
    if(!setProcessNoise(_filter_config.value(), _imu_sensor_samples_period.value()))
        return false;

    orientation_estimator->setMaxTimeDelta(_max_time_delta.get());

    // compute measurement covariances
    double sqrt_delta_t = sqrt(_imu_sensor_samples_period.value());
    Eigen::Vector3d rotation_rate_std = (1./sqrt_delta_t) * _filter_config.value().rotation_rate.randomwalk;
    Eigen::Vector3d acceleration_std = (1./sqrt_delta_t) * _filter_config.value().acceleration.randomwalk;
    cov_angular_velocity = rotation_rate_std.cwiseAbs2().asDiagonal();
    cov_acceleration = acceleration_std.cwiseAbs2().asDiagonal();

    // setup stream alignment verifier
    verifier.reset(new pose_estimation::StreamAlignmentVerifier());
    verifier->setVerificationInterval(2.0);
    verifier->setDropRateWarningThreshold(0.5);
    verifier->setDropRateCriticalThreshold(1.0);

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
    new_state = RUNNING;
    OrientationEstimatorBase::updateHook();

    // check stream alignment status
    unsigned streams_with_alignment_failures = 0;
    unsigned streams_with_critical_alignment_failures = 0;
    verifier->verifyStreamAlignerStatus(_transformer.getStreamAlignerStatus(), streams_with_alignment_failures, streams_with_critical_alignment_failures);
    if(streams_with_alignment_failures > 0)
        new_state = TRANSFORMATION_ALIGNMENT_FAILURES;
    if(streams_with_critical_alignment_failures > 0)
        exception(CRITICAL_ALIGNMENT_FAILURE);

    // write out estimated orientation
    OrientationUKF::State current_state;
    OrientationUKF::Covariance current_state_cov;
    if(orientation_estimator->getCurrentState(current_state, current_state_cov))
    {
        base::samples::RigidBodyState orientation_sample;
        orientation_sample.orientation = current_state.orientation;
        orientation_sample.velocity = current_state.velocity;
        orientation_sample.angular_velocity = orientation_estimator->getRotationRate();
        orientation_sample.cov_orientation = MTK::subblock(current_state_cov, &FilterState::orientation);
        orientation_sample.cov_velocity = MTK::subblock(current_state_cov, &FilterState::velocity);
        orientation_sample.cov_angular_velocity = cov_angular_velocity;
        orientation_sample.time = orientation_estimator->getLastMeasurementTime();
        orientation_sample.targetFrame = _target_frame.value();
        orientation_sample.sourceFrame = _imu_frame.value();
        _orientation_samples.write(orientation_sample);

        OrientationUKFSecondaryStates secondary_states;
        secondary_states.time = orientation_estimator->getLastMeasurementTime();
        secondary_states.bias_gyro = current_state.bias_gyro;
        secondary_states.bias_acc = current_state.bias_acc;
        secondary_states.gravity = current_state.gravity(0);
        secondary_states.latitude = current_state.latitude(0);
        secondary_states.cov_bias_gyro = MTK::subblock(current_state_cov, &FilterState::bias_gyro);
        secondary_states.cov_bias_acc = MTK::subblock(current_state_cov, &FilterState::bias_acc);
        secondary_states.var_gravity = MTK::subblock(current_state_cov, &FilterState::gravity)(0);
        secondary_states.var_latitude = MTK::subblock(current_state_cov, &FilterState::latitude)(0);
        _secondary_states.write(secondary_states);
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
