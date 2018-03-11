#include <vigir_step_control/step_controller_plugin.h>



namespace vigir_step_control
{
std::string toString(const StepControllerState& state)
{
  switch (state)
  {
    case NOT_READY:   return "NOT_READY";
    case READY:       return "READY";
    case ACTIVE:      return "ACTIVE";
    case PAUSED:      return "PAUSED";
    case FINISHED:    return "FINISHED";
    case FAILED:      return "FAILED";
    default:          return "UNKNOWN";
  }
}

StepControllerPlugin::StepControllerPlugin()
  : vigir_pluginlib::Plugin("step_controller")
  , state_(NOT_READY)
{
  step_queue_.reset(new StepQueue());

  reset();
}

StepControllerPlugin::~StepControllerPlugin()
{
}

void StepControllerPlugin::setStepPlanMsgPlugin(vigir_footstep_planning::StepPlanMsgPlugin::Ptr plugin)
{
  UniqueLock lock(plugin_mutex_);

  if (plugin)
    step_plan_msg_plugin_ = plugin;
  else
    ROS_ERROR("[StepControllerPlugin] Null pointer to StepPlanMsgPlugin rejected! Fix it immediately!");
}

StepControllerState StepControllerPlugin::getState() const
{
  SharedLock lock(plugin_mutex_);
  return state_;
}

int StepControllerPlugin::getNextStepIndexNeeded() const
{
  SharedLock lock(plugin_mutex_);
  return next_step_index_needed_;
}

int StepControllerPlugin::getLastStepIndexSent() const
{
  SharedLock lock(plugin_mutex_);
  return last_step_index_sent_;
}

const msgs::ExecuteStepPlanFeedback& StepControllerPlugin::getFeedbackState() const
{
  SharedLock lock(plugin_mutex_);
  return feedback_state_;
}

void StepControllerPlugin::reset()
{
  step_queue_->reset();

  msgs::ExecuteStepPlanFeedback feedback;
  feedback.last_performed_step_index = -1;
  feedback.currently_executing_step_index = -1;
  feedback.first_changeable_step_index = -1;
  setFeedbackState(feedback);

  setNextStepIndexNeeded(-1);
  setLastStepIndexSent(-1);

  setState(READY);
}

void StepControllerPlugin::setState(StepControllerState state)
{
  UniqueLock lock(plugin_mutex_);
  ROS_INFO("[StepControllerPlugin] Switching state from '%s' to '%s'.", toString(this->state_).c_str(), toString(state).c_str());
  this->state_ = state;
  feedback_state_.controller_state = state;
}

void StepControllerPlugin::setNextStepIndexNeeded(int index)
{
  UniqueLock lock(plugin_mutex_);
  next_step_index_needed_ = index;
}

void StepControllerPlugin::setLastStepIndexSent(int index)
{
  UniqueLock lock(plugin_mutex_);
  last_step_index_sent_ = index;
}

void StepControllerPlugin::setFeedbackState(const msgs::ExecuteStepPlanFeedback& feedback)
{
  UniqueLock lock(plugin_mutex_);
  this->feedback_state_ = feedback;
}

void StepControllerPlugin::updateQueueFeedback()
{
  UniqueLock lock(plugin_mutex_);
  feedback_state_.queue_size = static_cast<int>(step_queue_->size());
  feedback_state_.first_queued_step_index = step_queue_->firstStepIndex();
  feedback_state_.last_queued_step_index = step_queue_->lastStepIndex();
}

bool StepControllerPlugin::updateStepPlan(const msgs::StepPlan& step_plan)
{
  if (step_plan.steps.empty())
    return true;

  // Reset controller if previous execution was finished or has failed
  StepControllerState state = getState();
  if (state == FINISHED || state == FAILED)
    reset();

  // Allow step plan updates only in READY and ACTIVE state
  state = getState();
  if (state == READY || state == ACTIVE)
  {
    msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();

    if (step_queue_->updateStepPlan(step_plan, feedback.first_changeable_step_index))
    {
      // resets last_step_index_sent counter to trigger (re)executing steps in process()
      if (state == ACTIVE)
        setLastStepIndexSent(feedback.first_changeable_step_index-1);

      updateQueueFeedback();

      ROS_INFO("[StepControllerPlugin] Updated step queue from [%i; %i]. Current queue has steps in range [%i; %i].", step_plan.steps.front().step_index, step_plan.steps.back().step_index, step_queue_->firstStepIndex(), step_queue_->lastStepIndex());

      return true;
    }
  }

  setState(FAILED);
  return false;
}

void StepControllerPlugin::preProcess(const ros::TimerEvent& /*event*/)
{
  // check if new walking request has been done
  if (getState() == READY && !step_queue_->empty())
  {
    // check consisty
    if (step_queue_->firstStepIndex() != 0)
    {
      ROS_ERROR("[StepControllerPlugin] Step plan doesn't start with initial step (step_index = 0). Execution aborted!");
      setState(FAILED);
    }
    else
    {
      initWalk();
    }
  }
}

void StepControllerPlugin::process(const ros::TimerEvent& /*event*/)
{
  // execute steps
  if (getState() == ACTIVE)
  {
    // spool all requested steps
    while (getLastStepIndexSent() < getNextStepIndexNeeded())
    {
      // check if queue isn't empty
      if (step_queue_->empty())
      {
        ROS_ERROR("[StepControllerPlugin] Step %i required but queue is empty. Execution aborted!", getNextStepIndexNeeded());
        setState(FAILED);
        return;
      }

      // determine next step index
      int next_step_index = getLastStepIndexSent()+1;
      msgs::Step step;

      // retrieve next step
      if (!step_queue_->getStep(step, next_step_index))
      {
        ROS_ERROR("[StepControllerPlugin] Missing step %i in queue. Execution aborted!", next_step_index);
        setState(FAILED);
        return;
      }

      ROS_ASSERT(next_step_index == step.step_index);

      // send step to walking engine
      if (!executeStep(step))
      {
        ROS_ERROR("[StepControllerPlugin] Error while execution request of step %i. Execution aborted!", next_step_index);
        setState(FAILED);
        return;
      }

      // increment last_step_index_sent
      setLastStepIndexSent(next_step_index);

      msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();

      // garbage collection: remove already executed steps
      if (step_queue_->firstStepIndex() <= feedback.last_performed_step_index)
        step_queue_->removeSteps(0, feedback.last_performed_step_index);

      // update feedback
      updateQueueFeedback();
    }
  }
}

void StepControllerPlugin::stop()
{
  UniqueLock lock(plugin_mutex_);

  ROS_INFO("[StepControllerPlugin] Stop requested. Resetting walk controller.");
  reset();
}
} // namespace
