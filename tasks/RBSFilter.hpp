/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef POSE_ESTIMATION_RBSFilter_TASK_HPP
#define POSE_ESTIMATION_RBSFilter_TASK_HPP

#include "pose_estimation/RBSFilterBase.hpp"

#include <boost/shared_ptr.hpp>
#include <pose_estimation/pose_with_velocity/PoseUKF.hpp>
#include <transformer/Transformer.hpp>
#include <pose_estimationTypes.hpp>
#include <pose_estimation/StreamAlignmentVerifier.hpp>
#include <base/samples/RigidBodyAcceleration.hpp>

namespace pose_estimation {

    /*! \class RBSFilter
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','pose_estimation::RBSFilter')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */

    class RBSFilter : public RBSFilterBase
    {
	friend class RBSFilterBase;
    protected:
	States last_state;
	States new_state;
	std::string source_frame;
	boost::shared_ptr<PoseUKF> pose_estimator;
	boost::shared_ptr<StreamAlignmentVerifier> verifier;
        bool filter_state_changed;
	unsigned aligner_stream_failures;
        unsigned critical_aligner_stream_failures;

        /** Initializes the filter with a valid state and covariance (Given by the _initial_state property).
         *  Also sets the process noise (_process_noise property).
         */
        bool setupFilter();

        /** Applies a prediction step of the filter with a given current sample time.
         * The delta time step is the difference between the last sample time and the current one.
         */
        void predictionStep(const base::Time& sample_time);
	
	/** Writes out the current robot pose and task state.
	 * The seperation in this method ensures that this is done at the end of the update hook
	 * of a derivated task. 
	 */
	void writeCurrentState();
	
        /** Resets the filter to the initial state by calling setupFilter()
         */
        virtual bool resetState();
	
        /** Checks the current stream aligner sample drop rates of the aligned input ports.
         * Drops might happen due to missaligned system times, high delays or sensor failuers.
         *
         * When the drop_rate_warning is reached on at least one stream the task will switch to the runtime state
         * TRANSFORMATION_ALIGNMENT_FAILURES. The task will continue to work.
         * When the drop_rate_critical is reached on at least one stream the task will switch to the error state
         * CRITICAL_ALIGNMENT_FAILURE. The task will be in an error state.
         */
	void verifyStreamAlignerStatus(transformer::Transformer &trans, double verification_interval = 2.0, 
                                       double drop_rate_warning = 0.5, double drop_rate_critical = 1.0)
				    {verifyStreamAlignerStatus(trans.getStreamAlignerStatus(), verification_interval, drop_rate_warning, drop_rate_critical);}
	void verifyStreamAlignerStatus(const aggregator::StreamAlignerStatus &status, double verification_interval = 2.0, 
                                       double drop_rate_warning = 0.5, double drop_rate_critical = 1.0);

        /** Tries to get a transformation from the given transformer at time ts
         * @returns true if successful
         */
        bool getSensorInBodyPose(const transformer::Transformation& sensor2body_transformer, const base::Time &ts, Eigen::Affine3d& sensorInBody);

    public:
        /** TaskContext constructor for RBSFilter
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        RBSFilter(std::string const& name = "pose_estimation::RBSFilter");

        /** TaskContext constructor for RBSFilter
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        RBSFilter(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of RBSFilter
         */
	~RBSFilter();

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

