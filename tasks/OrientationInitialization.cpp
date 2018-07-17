/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OrientationInitialization.hpp"
#include "pose_estimationTypes.hpp"
#include <pose_estimation/GravitationalModel.hpp>

using namespace pose_estimation;

typedef OrientationUKF::WState FilterState;

OrientationInitialization::OrientationInitialization(std::string const& name)
    : OrientationInitializationBase(name)
{
}

OrientationInitialization::OrientationInitialization(std::string const& name, RTT::ExecutionEngine* engine)
    : OrientationInitializationBase(name, engine)
{
}

OrientationInitialization::~OrientationInitialization()
{
}


void OrientationInitialization::imu_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sensor_samples_sample)
{
    // do prediction step
    predictionStep(ts);

    try
    {
        OrientationUKF::RotationRate measurement;
        measurement.mu = imu_in_body_rotation * imu_sensor_samples_sample.gyro;
        measurement.cov = cov_angular_velocity;
        for(unsigned i = 0; i < estimator_count; i++)
            orientation_estimators[i]->integrateMeasurement(measurement);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate gyro measurement: " << e.what();
    }

    try
    {
        OrientationUKF::Acceleration measurement;
        measurement.mu = imu_in_body_rotation * imu_sensor_samples_sample.acc;
        measurement.cov = cov_acceleration;
        for(unsigned i = 0; i < estimator_count; i++)
            orientation_estimators[i]->integrateMeasurement(measurement);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to integrate acceleration measurement: " << e.what();
    }

    // integrate zero velocity measurement with a sigma of max velocity in order to constrain the velocity
    if(velocity_unknown)
    {
        try
        {
            OrientationUKF::VelocityMeasurement measurement;
            measurement.mu = Eigen::Vector3d::Zero();
            measurement.cov = cov_velocity_unknown;

            for(unsigned i = 0; i < estimator_count; i++)
                orientation_estimators[i]->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            LOG_ERROR_S << "Failed to integrate zero velocity measurement: " << e.what();
        }
    }
    else if((ts - last_velocity_sample_time).toSeconds() > (_velocity_samples_period.value() * 3.0))
        velocity_unknown = true;
}

void OrientationInitialization::velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d velocityProviderInBody;
    if (!_velocity_provider2body.get(ts, velocityProviderInBody))
    {
        LOG_ERROR_S << "skip, couldn't receive a valid velocity_provider-in-body transformation sample!";
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    Eigen::Affine3d velocityProviderInIMU = imu_in_body_translation.inverse() * velocityProviderInBody;

    if(velocity_samples_sample.hasValidVelocity() && velocity_samples_sample.hasValidVelocityCovariance())
    {
        try
        {
            // if this is not set, zero velocity updates in the IMU sample callback are integrated
            velocity_unknown = false;
            last_velocity_sample_time = ts;

            OrientationUKF::VelocityMeasurement measurement;
            // transform velocity to the imu frame
            measurement.cov = velocityProviderInIMU.rotation().matrix() * velocity_samples_sample.cov_velocity * velocityProviderInIMU.rotation().matrix().transpose();
            Eigen::Vector3d velocity = velocityProviderInIMU.rotation() * velocity_samples_sample.velocity;

            for(unsigned i = 0; i < estimator_count; i++)
            {
                measurement.mu = velocity - orientation_estimators[i]->getRotationRate().cross(velocityProviderInIMU.translation());

                // apply new velocity measurement
                orientation_estimators[i]->integrateMeasurement(measurement);
            }
        }
        catch(const std::runtime_error& e)
        {
            LOG_ERROR_S << "Failed to add DVL measurement: " << e.what();
        }
    }
    else
        LOG_ERROR_S << "Velocity measurement contains NaN's, it will be skipped!";
}

void OrientationInitialization::predictionStep(const base::Time& sample_time)
{
    try
    {
        for(unsigned i = 0; i < estimator_count; i++)
            orientation_estimators[i]->predictionStepFromSampleTime(sample_time);
    }
    catch(const std::runtime_error& e)
    {
        LOG_ERROR_S << "Failed to execute prediction step: " << e.what();
        LOG_ERROR_S << "Skipping prediction step.";
    }
}

bool OrientationInitialization::initializeFilter(boost::shared_ptr<pose_estimation::OrientationUKF>& filter, const Eigen::Quaterniond& orientation, const Eigen::Matrix3d& orientation_cov, const OrientationUKFConfig& filter_config)
{
    if(!orientation.matrix().allFinite() || !orientation_cov.allFinite())
    {
        LOG_ERROR_S << "Undefined values in initial orientation!";
        return false;
    }

    FilterState initial_state;
    initial_state.orientation = RotationType(MTK::SO3<double>(orientation));
    initial_state.velocity = VelocityType(Eigen::Vector3d::Zero());
    initial_state.bias_gyro = BiasType(imu_in_body_rotation * filter_config.rotation_rate.bias_offset);
    initial_state.bias_acc = BiasType(imu_in_body_rotation * filter_config.acceleration.bias_offset);
    Eigen::Matrix<double, 1, 1> gravity;
    gravity(0) = pose_estimation::GravitationalModel::WGS_84(filter_config.location.latitude, filter_config.location.altitude);
    initial_state.gravity = GravityType(gravity);

    OrientationUKF::Covariance initial_state_cov = OrientationUKF::Covariance::Zero();
    MTK::subblock(initial_state_cov, &FilterState::orientation) = orientation_cov;
    MTK::subblock(initial_state_cov, &FilterState::velocity) = Eigen::Matrix3d::Identity(); // velocity is unknown at the start
    MTK::subblock(initial_state_cov, &FilterState::bias_gyro) = imu_in_body_rotation.matrix() * filter_config.rotation_rate.bias_instability.cwiseAbs2().asDiagonal() * imu_in_body_rotation.matrix().transpose();
    MTK::subblock(initial_state_cov, &FilterState::bias_acc) = imu_in_body_rotation.matrix() * filter_config.acceleration.bias_instability.cwiseAbs2().asDiagonal() * imu_in_body_rotation.matrix().transpose();
    Eigen::Matrix<double, 1, 1> gravity_var;
    gravity_var << pow(0.05, 2.); // give the gravity model a sigma of 5 cm/s^2 at the start
    MTK::subblock(initial_state_cov, &FilterState::gravity) = gravity_var;

    filter.reset(new OrientationUKF(initial_state, initial_state_cov,
                                  filter_config.rotation_rate.bias_tau, filter_config.acceleration.bias_tau,
                                  filter_config.location));
    return true;
}

bool OrientationInitialization::setProcessNoise(boost::shared_ptr<pose_estimation::OrientationUKF>& filter, const OrientationUKFConfig& filter_config, double sensor_delta_t)
{
    OrientationUKF::Covariance process_noise_cov = OrientationUKF::Covariance::Zero();
    MTK::subblock(process_noise_cov, &FilterState::orientation) = imu_in_body_rotation.matrix() * filter_config.rotation_rate.randomwalk.cwiseAbs2().asDiagonal() * imu_in_body_rotation.matrix().transpose();
    MTK::subblock(process_noise_cov, &FilterState::velocity) = imu_in_body_rotation.matrix() * filter_config.acceleration.randomwalk.cwiseAbs2().asDiagonal() * imu_in_body_rotation.matrix().transpose();
    MTK::subblock(process_noise_cov, &FilterState::bias_gyro) = (2. / (filter_config.rotation_rate.bias_tau * sensor_delta_t)) *
                                        imu_in_body_rotation.matrix() * filter_config.rotation_rate.bias_instability.cwiseAbs2().asDiagonal() * imu_in_body_rotation.matrix().transpose();
    MTK::subblock(process_noise_cov, &FilterState::bias_acc) = (2. / (filter_config.acceleration.bias_tau * sensor_delta_t)) *
                                        imu_in_body_rotation.matrix() * filter_config.acceleration.bias_instability.cwiseAbs2().asDiagonal() * imu_in_body_rotation.matrix().transpose();
    Eigen::Matrix<double, 1, 1> gravity_noise;
    gravity_noise << 1.e-12; // add a tiny bit of noise only for numeric stability
    MTK::subblock(process_noise_cov, &FilterState::gravity) = gravity_noise;
    filter->setProcessNoiseCovariance(process_noise_cov);

    return true;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OrientationInitialization.hpp for more detailed
// documentation about them.

bool OrientationInitialization::configureHook()
{
    if (! OrientationInitializationBase::configureHook())
        return false;
    return true;
}
bool OrientationInitialization::startHook()
{
    if (! OrientationInitializationBase::startHook())
        return false;

    // get IMU to body transformation
    Eigen::Affine3d imu_in_body;
    if(!_imu2body.get(base::Time(), imu_in_body))
    {
        LOG_ERROR_S << "Failed to get IMU pose in body frame. Note that this has to be a static transformation!";
        return false;
    }
    imu_in_body_translation = Eigen::Affine3d::Identity();
    imu_in_body_translation.translation() = imu_in_body.translation();
    imu_in_body_rotation = imu_in_body.rotation();

    // set initial orientation
    Eigen::Vector3d euler = _initial_orientation.value();
    Eigen::Quaterniond orientations[estimator_count];
    base::Angle step_size = base::Angle::fromRad(2. * sqrt(_initial_orientation_cov_diag.value().z()) / (double)(estimator_count-1));
    base::Angle start_angle = base::Angle::fromRad(euler.z() - sqrt(_initial_orientation_cov_diag.value().z()));
    for(unsigned i = 0; i < estimator_count; i++)
    {
        orientations[i] = Eigen::AngleAxisd((start_angle + (double)i * step_size).getRad(), Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
    }

    for(unsigned i = 0; i < estimator_count; i++)
    {
        // initialize filter
        if(!initializeFilter(orientation_estimators[i], orientations[i], _initial_orientation_cov_diag.value().asDiagonal(), _filter_config.value()))
            return false;

        // set process noise
        if(!setProcessNoise(orientation_estimators[i], _filter_config.value(), _imu_sensor_samples_period.value()))
            return false;

        orientation_estimators[i]->setMaxTimeDelta(_max_time_delta.value());
    }

    // compute measurement covariances
    double sqrt_delta_t = sqrt(_imu_sensor_samples_period.value());
    Eigen::Vector3d rotation_rate_std = (1./sqrt_delta_t) * _filter_config.value().rotation_rate.randomwalk;
    Eigen::Vector3d acceleration_std = (1./sqrt_delta_t) * _filter_config.value().acceleration.randomwalk;
    cov_angular_velocity = imu_in_body_rotation.matrix() * rotation_rate_std.cwiseAbs2().asDiagonal() * imu_in_body_rotation.matrix().transpose();
    cov_acceleration = imu_in_body_rotation.matrix() * acceleration_std.cwiseAbs2().asDiagonal() * imu_in_body_rotation.matrix().transpose();
    cov_velocity_unknown = (1./_imu_sensor_samples_period.value()) * (_filter_config.value().max_velocity.cwiseAbs2()).asDiagonal();

    // setup stream alignment verifier
    verifier.reset(new pose_estimation::StreamAlignmentVerifier());
    verifier->setVerificationInterval(2.0);
    verifier->setDropRateWarningThreshold(0.5);
    verifier->setDropRateCriticalThreshold(1.0);

    // reset state machine related members
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    velocity_unknown = false;
    last_velocity_sample_time = base::Time();

    return true;
}
void OrientationInitialization::updateHook()
{
    // check convergence criteria
    bool initialization_complete = true;
    for(unsigned i = 0; i < estimator_count; i++)
    {
        OrientationUKF::State current_state_i;
        orientation_estimators[i]->getCurrentState(current_state_i);
        base::Angle yaw_i = base::Angle::fromRad(base::getYaw(current_state_i.orientation));
        for(unsigned j = i+1; j < estimator_count; j++)
        {
            OrientationUKF::State current_state_j;
            orientation_estimators[j]->getCurrentState(current_state_j);
            base::Angle yaw_j = base::Angle::fromRad(base::getYaw(current_state_j.orientation));
            if(std::abs((yaw_i - yaw_j).getRad()) > _convergence_threshold.value())
            {
                initialization_complete = false;
            }
        }
    }

    if(initialization_complete)
    {
        new_state = INITIALIZATION_COMPLETE;

        // write out average yaw of current orientations
        double cos_angle = 0.;
        double sin_angle = 0.;
        OrientationUKF::State current_state;
        for(unsigned i = 0; i < estimator_count; i++)
        {
            orientation_estimators[i]->getCurrentState(current_state);
            double yaw = base::getYaw(current_state.orientation);
            cos_angle += cos(yaw);
            sin_angle += sin(yaw);
        }
        double mean_yaw = atan2(sin_angle, cos_angle);
        Eigen::Vector3d euler = base::getEuler(current_state.orientation);
        Eigen::Quaterniond average_orientation = Eigen::AngleAxisd(mean_yaw, Eigen::Vector3d::UnitZ()) *
                                                Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                                Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitX());

        base::samples::RigidBodyState orientation_sample;
        orientation_sample.orientation = average_orientation;
        orientation_sample.cov_orientation = std::pow(2.0 * _convergence_threshold.value(), 2.0) * Eigen::Matrix3d::Identity();
        orientation_sample.time = orientation_estimators[0]->getLastMeasurementTime();
        orientation_sample.targetFrame = _target_frame.value();
        orientation_sample.sourceFrame = _imu_frame.value();
        _orientation_samples.write(orientation_sample);
    }
    else
        new_state = INITIALIZATION;

    // integrate new measurements
    OrientationInitializationBase::updateHook();

    // check stream alignment status
    unsigned streams_with_alignment_failures = 0;
    unsigned streams_with_critical_alignment_failures = 0;
    verifier->verifyStreamAlignerStatus(_transformer.getStreamAlignerStatus(), streams_with_alignment_failures, streams_with_critical_alignment_failures);
    if(streams_with_alignment_failures > 0)
        new_state = TRANSFORMATION_ALIGNMENT_FAILURES;
    if(streams_with_critical_alignment_failures > 0)
        exception(CRITICAL_ALIGNMENT_FAILURE);

    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void OrientationInitialization::errorHook()
{
    OrientationInitializationBase::errorHook();
}
void OrientationInitialization::stopHook()
{
    OrientationInitializationBase::stopHook();
}
void OrientationInitialization::cleanupHook()
{
    OrientationInitializationBase::cleanupHook();
}
