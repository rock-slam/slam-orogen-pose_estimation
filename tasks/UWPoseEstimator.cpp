/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "UWPoseEstimator.hpp"

using namespace pose_estimation;

UWPoseEstimator::UWPoseEstimator(std::string const& name)
    : UWPoseEstimatorBase(name)
{
}

UWPoseEstimator::UWPoseEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : UWPoseEstimatorBase(name, engine)
{
}

UWPoseEstimator::~UWPoseEstimator()
{
}

void UWPoseEstimator::acceleration_samplesTransformerCallback(const base::Time& ts, const base::samples::RigidBodyAcceleration& acceleration_samples_sample)
{
    if(acceleration_samples_sample.acceleration.allFinite() && acceleration_samples_sample.cov_acceleration.allFinite())
    {
        predictionStep(ts);
        try
        {
            PoseUKF::AccelerationMeasurement measurement;
            measurement.mu = acceleration_samples_sample.acceleration;
            measurement.cov = acceleration_samples_sample.cov_acceleration;
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add acceleration measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Acceleration measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void UWPoseEstimator::depth_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_pressure_sensor2body, ts, sensorInBody))
        return;

    if(!base::isNaN(depth_samples_sample.position.z()) && !base::isNaN(depth_samples_sample.cov_position(2,2)))
    {
        predictionStep(ts);
        try
        {
            // apply sensorInBody transformation to measurement
            Eigen::Matrix<double, 1, 1> depth;
            depth << depth_samples_sample.position.z() - sensorInBody.translation().z();

            PoseUKF::ZMeasurement measurement;
            measurement.mu = depth;
            measurement.cov(0,0) = depth_samples_sample.cov_position(2,2);
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add depth measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Depth measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void UWPoseEstimator::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_imu2body, ts, sensorInBody))
        return;

    if(orientation_samples_sample.hasValidOrientation() && orientation_samples_sample.hasValidOrientationCovariance())
    {
        predictionStep(ts);
        try
        {
            // apply sensorInBody transformation to measurement
            Eigen::Quaterniond transformed_orientation((orientation_samples_sample.orientation * sensorInBody.inverse()).linear());

            PoseUKF::OrientationMeasurement measurement;
            measurement.mu = MTK::SO3<double>::log(transformed_orientation);
            measurement.cov = sensorInBody.rotation() * orientation_samples_sample.cov_orientation * sensorInBody.rotation().transpose();
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add orientation measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Orientation measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void UWPoseEstimator::lbl_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &lbl_position_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_lbl2body, ts, sensorInBody))
        return;

    PoseUKF::State current_state;
    if(lbl_position_samples_sample.hasValidPosition() &&
        lbl_position_samples_sample.hasValidPositionCovariance() &&
        pose_estimator->getCurrentState(current_state)
    )
    {
        predictionStep(ts);
        try
        {
            // apply sensorInBody transformation to measurement
            Eigen::Vector3d bodyInWorld = lbl_position_samples_sample.position - current_state.orientation * sensorInBody.translation();

            PoseUKF::PositionMeasurement measurement;
            measurement.mu = bodyInWorld;
            measurement.cov = lbl_position_samples_sample.cov_position;
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add LBL/USBL measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "LBL/USBL measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void UWPoseEstimator::xy_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample)
{
    if(xy_position_samples_sample.position.block(0,0,2,1).allFinite() && xy_position_samples_sample.cov_position.block(0,0,2,2).allFinite())
    {
        predictionStep(ts);
        try
        {
            PoseUKF::XYMeasurement measurement;
            measurement.mu = xy_position_samples_sample.position.block(0,0,2,1);
            measurement.cov = xy_position_samples_sample.cov_position.block(0,0,2,2);
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add XY position measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "XY position measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void UWPoseEstimator::dvl_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_dvl2body, ts, sensorInBody))
        return;

    if(dvl_velocity_samples_sample.hasValidVelocity() && dvl_velocity_samples_sample.hasValidVelocityCovariance())
    {
        predictionStep(ts);
        try
        {
            // apply sensorInBody transformation to measurement, assuming that the velocity is expressed in the DVL device frame
            base::Vector3d velocity = sensorInBody.rotation() * dvl_velocity_samples_sample.velocity;
            PoseUKF::State current_state;
            if(pose_estimator->getCurrentState(current_state))
            {
                velocity -= current_state.angular_velocity.cross(sensorInBody.translation());
            }

            // apply new velocity measurement
            PoseUKF::VelocityMeasurement measurement;
            measurement.mu = velocity;
            measurement.cov = sensorInBody.rotation() * dvl_velocity_samples_sample.cov_velocity * sensorInBody.rotation().transpose();
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add DVL measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Info) << "DVL measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void UWPoseEstimator::model_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &model_velocity_samples_sample)
{
    if(model_velocity_samples_sample.velocity.block(0,0,2,1).allFinite() && model_velocity_samples_sample.cov_velocity.block(0,0,2,2).allFinite())
    {
        predictionStep(ts);
        try
        {
            PoseUKF::XYVelocityMeasurement measurement;
            measurement.mu = model_velocity_samples_sample.velocity.block(0,0,2,1);
            measurement.cov = model_velocity_samples_sample.cov_velocity.block(0,0,2,2);
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add model velocity measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "Model position measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void UWPoseEstimator::gps_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_position_samples_sample)
{
    // receive sensor to body transformation
    Eigen::Affine3d sensorInBody;
    if (!getSensorInBodyPose(_gps2body, ts, sensorInBody))
        return;

    PoseUKF::State current_state;
    if(gps_position_samples_sample.position.block(0,0,2,1).allFinite() &&
        gps_position_samples_sample.cov_position.block(0,0,2,2).allFinite() &&
        pose_estimator->getCurrentState(current_state))
    {
        predictionStep(ts);
        try
        {
            // apply sensorInBody transformation to measurement
            Eigen::Vector3d bodyInWorld = gps_position_samples_sample.position - current_state.orientation * sensorInBody.translation();

            PoseUKF::XYMeasurement measurement;
            measurement.mu = bodyInWorld.block(0,0,2,1);
            measurement.cov = gps_position_samples_sample.cov_position.block(0,0,2,2);
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add GPS measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "GPS position measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

void UWPoseEstimator::xyz_position_samplesTransformerCallback( const base::Time &ts, const ::base::samples::RigidBodyState &xyz_position_samples_sample)
{
    if(xyz_position_samples_sample.hasValidPosition() && xyz_position_samples_sample.hasValidPositionCovariance())
    {
        predictionStep(ts);
        try
        {
            PoseUKF::PositionMeasurement measurement;
            measurement.mu = xyz_position_samples_sample.position;
            measurement.cov = xyz_position_samples_sample.cov_position;
            pose_estimator->integrateMeasurement(measurement);
        }
        catch(const std::runtime_error& e)
        {
            RTT::log(RTT::Error) << "Failed to add XYZ position measurement: " << e.what() << RTT::endlog();
        }
    }
    else
        RTT::log(RTT::Error) << "XYZ position measurement contains NaN's, it will be skipped!" << RTT::endlog();
}

bool UWPoseEstimator::setTransformer_max_latency(double value)
{
    if (pose_estimation::UWPoseEstimatorBase::setTransformer_max_latency(value))
    {
        _transformer.setTimeout( base::Time::fromSeconds(value) );
        return true;
    }
    else
    {
        return false;
    }
}
  
/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See UWPoseEstimator.hpp for more detailed
// documentation about them.

bool UWPoseEstimator::configureHook()
{
    if (! UWPoseEstimatorBase::configureHook())
        return false;
    
    source_frame = _body_frame.get();
    
    return true;
}
bool UWPoseEstimator::startHook()
{
    if (! UWPoseEstimatorBase::startHook())
        return false;
    return true;
}
void UWPoseEstimator::updateHook()
{
    UWPoseEstimatorBase::updateHook();
    
    // verify stream aligner status
    verifyStreamAlignerStatus(_transformer, 60.0);
    
    // update and write new state
    writeCurrentState();
}
void UWPoseEstimator::errorHook()
{
    UWPoseEstimatorBase::errorHook();
}
void UWPoseEstimator::stopHook()
{
    UWPoseEstimatorBase::stopHook();
}
void UWPoseEstimator::cleanupHook()
{
    UWPoseEstimatorBase::cleanupHook();
}
