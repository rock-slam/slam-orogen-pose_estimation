/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef POSE_ESTIMATION_ORIENTATIONINITIALIZATION_TASK_HPP
#define POSE_ESTIMATION_ORIENTATIONINITIALIZATION_TASK_HPP

#include "pose_estimation/OrientationInitializationBase.hpp"
#include <boost/shared_ptr.hpp>
#include <pose_estimation/orientation_estimator/OrientationUKF.hpp>
#include <pose_estimation/orientation_estimator/OrientationUKFConfig.hpp>
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/StreamAlignmentVerifier.hpp>

namespace pose_estimation{

    /*! \class OrientationInitialization
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','pose_estimation::OrientationInitialization')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class OrientationInitialization : public OrientationInitializationBase
    {
        friend class OrientationInitializationBase;
        static const unsigned estimator_count = 3;
    protected:
        boost::shared_ptr<pose_estimation::OrientationUKF> orientation_estimators[estimator_count];
        boost::shared_ptr<pose_estimation::StreamAlignmentVerifier> verifier;
        Eigen::Affine3d imu_in_body_translation;
        Eigen::Quaterniond imu_in_body_rotation;
        Eigen::Matrix3d cov_angular_velocity;
        Eigen::Matrix3d cov_acceleration;
        Eigen::Matrix3d cov_velocity_unknown;
        base::Time last_velocity_sample_time;
        bool velocity_unknown;
        States last_state;
        States new_state;

        virtual void imu_sensor_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_sensor_samples_sample);

        virtual void velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample);

        /** Applies a prediction step of the filter with a given current sample time.
         * The delta time step is the difference between the last sample time and the current one.
         */
        virtual void predictionStep(const base::Time& sample_time);

        bool initializeFilter(boost::shared_ptr<pose_estimation::OrientationUKF>& filter, const Eigen::Quaterniond& orientation, const Eigen::Matrix3d& orientation_cov, const OrientationUKFConfig& filter_config);

        bool setProcessNoise(boost::shared_ptr<pose_estimation::OrientationUKF>& filter, const OrientationUKFConfig& filter_config, double sensor_delta_t);

    public:
        /** TaskContext constructor for OrientationInitialization
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        OrientationInitialization(std::string const& name = "pose_estimation::OrientationInitialization");

        /** TaskContext constructor for OrientationInitialization
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         * 
         */
        OrientationInitialization(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of OrientationInitialization
         */
	~OrientationInitialization();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

