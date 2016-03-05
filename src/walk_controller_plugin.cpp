#include <vigir_walk_control/walk_controller_plugin.h>



namespace vigir_walk_control
{
std::string toString(const WalkControllerState& state)
{
  switch (state)
  {
    case IDLE: return "IDLE";
    case ACTIVE: return "ACTIVE";
    case PAUSED: return "PAUSED";
    case FINISHED: return "FINISHED";
    case FAILED: return "FAILED";
    default: return "UNKNOWN";
  }
}

WalkControllerPlugin::WalkControllerPlugin()
  : vigir_pluginlib::Plugin("walk_controller")
  , state_(IDLE)
{
  walk_controller_queue_.reset(new WalkControllerQueue());

  reset();
}

WalkControllerPlugin::~WalkControllerPlugin()
{
}

void WalkControllerPlugin::reset()
{
  setState(IDLE);

  walk_controller_queue_->reset();

  msgs::ExecuteStepPlanFeedback feedback;
  feedback.last_performed_step_index = -1;
  feedback.currently_executing_step_index = -1;
  feedback.first_changeable_step_index = -1;
  setFeedback(feedback);

  setNextStepIndexNeeded(-1);
  setLastStepIndexSent(-1);
}

WalkControllerState WalkControllerPlugin::getState() const
{
  boost::shared_lock<boost::shared_mutex> lock(plugin_mutex_);
  return state_;
}

int WalkControllerPlugin::getNextStepIndexNeeded() const
{
  boost::shared_lock<boost::shared_mutex> lock(plugin_mutex_);
  return next_step_index_needed_;
}

int WalkControllerPlugin::getLastStepIndexSent() const
{
  boost::shared_lock<boost::shared_mutex> lock(plugin_mutex_);
  return last_step_index_sent_;
}

const msgs::ExecuteStepPlanFeedback& WalkControllerPlugin::getFeedback() const
{
  boost::shared_lock<boost::shared_mutex> lock(plugin_mutex_);
  return feedback_;
}

void WalkControllerPlugin::setState(WalkControllerState state)
{
  boost::unique_lock<boost::shared_mutex> lock(plugin_mutex_);
  ROS_INFO("[WalkControllerPlugin] Switching state from '%s' to '%s'.", toString(this->state_).c_str(), toString(state).c_str());
  this->state_ = state;
  feedback_.controller_state = state;
}

void WalkControllerPlugin::setNextStepIndexNeeded(int index)
{
  boost::unique_lock<boost::shared_mutex> lock(plugin_mutex_);
  next_step_index_needed_ = index;
}

void WalkControllerPlugin::setLastStepIndexSent(int index)
{
  boost::unique_lock<boost::shared_mutex> lock(plugin_mutex_);
  last_step_index_sent_ = index;
}

void WalkControllerPlugin::setFeedback(const msgs::ExecuteStepPlanFeedback& feedback)
{
  boost::unique_lock<boost::shared_mutex> lock(plugin_mutex_);
  this->feedback_ = feedback;
}

void WalkControllerPlugin::updateQueueFeedback()
{
  boost::unique_lock<boost::shared_mutex> lock(plugin_mutex_);
  feedback_.queue_size = static_cast<int>(walk_controller_queue_->size());
  feedback_.first_queued_step_index = walk_controller_queue_->firstStepIndex();
  feedback_.last_queued_step_index = walk_controller_queue_->lastStepIndex();
}

void WalkControllerPlugin::updateStepPlan(const msgs::StepPlan& step_plan)
{
  if (step_plan.steps.empty())
    return;

  // Reset controller if previous execution was finished or has failed
  if (state_ == FINISHED || state_ == FAILED)
    reset();

  // Allow step plan updates only in IDLE and ACTIVE state
  WalkControllerState state = getState();
  if (state == IDLE || state == ACTIVE)
  {
    msgs::ExecuteStepPlanFeedback feedback = getFeedback();

    if (walk_controller_queue_->updateStepPlan(step_plan, feedback.first_changeable_step_index))
    {
      if (state == ACTIVE)
        setLastStepIndexSent(feedback.first_changeable_step_index-1);

      updateQueueFeedback();

      ROS_INFO("[WalkControllerPlugin] Updated step queue. Current queue has steps in range [%i; %i].", walk_controller_queue_->firstStepIndex(), walk_controller_queue_->lastStepIndex());
    }
  }
}

void WalkControllerPlugin::preProcess(const ros::TimerEvent& /*event*/)
{
  // check if new walking request has been done
  if (getState() == IDLE && !walk_controller_queue_->empty())
  {
    // check consisty
    if (walk_controller_queue_->firstStepIndex() != 0)
    {
      ROS_ERROR("[WalkControllerTestPlugin] Step plan doesn't start with initial step (step_index = 0). Execution aborted!");
      setState(FAILED);
    }
    else
    {
      initWalk();
      setState(ACTIVE);
    }
  }
}

void WalkControllerPlugin::process(const ros::TimerEvent& /*event*/)
{
  // execute steps
  if (getState() == ACTIVE)
  {
    // spool all requested steps
    while (getLastStepIndexSent() < getNextStepIndexNeeded())
    {
      // check if queue isn't empty
      if (walk_controller_queue_->empty())
      {
        ROS_ERROR("[WalkControllerTestPlugin] Step %i required but not in queue. Execution aborted!", getNextStepIndexNeeded());
        setState(FAILED);
        return;
      }

      // determine next step index
      int next_step_index = getLastStepIndexSent()+1;
      msgs::Step step;

      // retrieve next step
      if (!walk_controller_queue_->getStep(step, next_step_index))
      {
        ROS_ERROR("[WalkControllerTestPlugin] Missing step %i in queue. Execution aborted!", next_step_index);
        setState(FAILED);
        return;
      }

      // sent step to walking engine
      if (!executeStep(step))
      {
        ROS_ERROR("[WalkControllerTestPlugin] Error while execution request of step %i. Execution aborted!", next_step_index);
        setState(FAILED);
        return;
      }

      // increment last_step_index_sent
      setLastStepIndexSent(next_step_index);

      msgs::ExecuteStepPlanFeedback feedback = getFeedback();

      // garbage collection: remove already executed steps
      if (feedback.last_performed_step_index >= 0)
        walk_controller_queue_->removeSteps(0, feedback.last_performed_step_index);

      // update feedback
      updateQueueFeedback();
    }
  }
}

void WalkControllerPlugin::stop()
{
  ROS_INFO("[WalkControllerTestPlugin] Stop requested. Resetting walk controller.");
  reset();
}
}
