/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef POSE_ESTIMATION_RBSFilter_TASK_HPP
#define POSE_ESTIMATION_RBSFilter_TASK_HPP

#include "pose_estimation/RBSFilterBase.hpp"

#include <boost/shared_ptr.hpp>
#include <pose_estimation/PoseEstimator.hpp>
#include <pose_estimation/Measurement.hpp>
#include <pose_estimation/pose_with_velocity/BodyStateMeasurement.hpp>
#include <transformer/Transformer.hpp>
#include <pose_estimationTypes.hpp>
#include <pose_estimation/StreamAlignmentVerifier.hpp>

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
	boost::shared_ptr<PoseEstimator> pose_estimator;
	boost::shared_ptr<StreamAlignmentVerifier> verifier;
	unsigned aligner_stream_failures;
        base::samples::RigidBodyState current_body_state;
	
	void handleMeasurement(const base::Time &ts, const base::samples::RigidBodyState &rbs, const MemberMask& member_mask, const transformer::Transformation& sensor2body_transformer);
	void handleMeasurement(const base::Time &ts, const base::samples::RigidBodyState &rbs, const MemberMask& member_mask);
        void handleMeasurement(const base::Time &ts, const base::samples::RigidBodyAcceleration &rba);

        bool setupFilter();
	
	/** Updates and writes the current robot pose and task state.
	 * The seperation in this method ensures that this is done at the end of the update hook
	 * of a derivated task. 
	 */
	void updateState();
	
        virtual bool resetState();
	
	void verifyStreamAlignerStatus(transformer::Transformer &trans, double verification_interval = 2.0) 
				    {verifyStreamAlignerStatus(trans.getStreamAlignerStatus(), verification_interval);}
	void verifyStreamAlignerStatus(const aggregator::StreamAlignerStatus &status, double verification_interval = 2.0);

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

