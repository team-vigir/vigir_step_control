#include <vigir_walk_control/walk_controller.h>

#include <vigir_generic_params/parameter_manager.h>



namespace vigir_walk_control
{
WalkController::WalkController(ros::NodeHandle& nh, bool auto_spin)
{
  vigir_pluginlib::PluginManager::addPluginClassLoader<vigir_footstep_planning::StepPlanMsgPlugin>("vigir_footstep_planning_plugins", "vigir_footstep_planning::StepPlanMsgPlugin");
  vigir_pluginlib::PluginManager::addPluginClassLoader<WalkControllerPlugin>("vigir_walk_control", "vigir_walk_control::WalkControllerPlugin");

  // init step plan msg plugin
  loadPlugin(nh.param("step_plan_msg_plugin", std::string("step_plan_msg_plugin")), step_plan_msg_plugin_);

  // init walk controller plugin
  loadPlugin(nh.param("walk_controller_plugin", std::string("walk_controller_test_plugin")), walk_controller_plugin_);
  if (walk_controller_plugin_)
    walk_controller_plugin_->setStepPlanMsgPlugin(step_plan_msg_plugin_);

  // subscribe topics
  load_step_plan_msg_plugin_sub_ = nh.subscribe("load_step_plan_msg_plugin", 1, &WalkController::loadStepPlanMsgPlugin, this);
  load_walk_controller_plugin_sub_ = nh.subscribe("load_walk_controller_plugin", 1, &WalkController::loadWalkControllerPlugin, this);
  execute_step_plan_sub_ = nh.subscribe("execute_step_plan", 1, &WalkController::executeStepPlan, this);

  // publish topics
  planning_feedback_pub_ = nh.advertise<msgs::ExecuteStepPlanFeedback>("execute_feedback", 1, true);

  // init action servers
  execute_step_plan_as_.reset(new ExecuteStepPlanActionServer(nh, "execute_step_plan", false));
  execute_step_plan_as_->registerGoalCallback(boost::bind(&WalkController::executeStepPlanAction, this, boost::ref(execute_step_plan_as_)));
  execute_step_plan_as_->registerPreemptCallback(boost::bind(&WalkController::executePreemptionAction, this, boost::ref(execute_step_plan_as_)));

  // start action servers
  execute_step_plan_as_->start();

  // schedule main update loop
  if (auto_spin)
    update_timer_ = nh.createTimer(nh.param("rate", 10.0), &WalkController::update, this);
}

WalkController::~WalkController()
{
}

void WalkController::executeStepPlan(const msgs::StepPlan& step_plan)
{
  boost::unique_lock<boost::shared_mutex> lock(controller_mutex_);

  // An empty step plan will always trigger a soft stop
  if (step_plan.steps.empty())
    walk_controller_plugin_->stop();
  else
    walk_controller_plugin_->updateStepPlan(step_plan);
}

void WalkController::update(const ros::TimerEvent& event)
{
  boost::unique_lock<boost::shared_mutex> lock(controller_mutex_);

  if (!walk_controller_plugin_)
    return;

  // Save current state to be able to handle action server correctly;
  // We must not send setSucceeded/setAborted state while sending the
  // final feedback message in the same update cycle!
  WalkControllerState state = walk_controller_plugin_->getState();

  // pre process
  walk_controller_plugin_->preProcess(event);

  // process
  walk_controller_plugin_->process(event);

  // publish feedback
  publishFeedback();

  // update action server
  switch (state)
  {
    case FINISHED:
      if (execute_step_plan_as_->isActive())
        execute_step_plan_as_->setSucceeded(msgs::ExecuteStepPlanResult());
      break;

    case FAILED:
      if (execute_step_plan_as_->isActive())
        execute_step_plan_as_->setAborted(msgs::ExecuteStepPlanResult());
      break;

    default:
      break;
  }

  // post process
  walk_controller_plugin_->postProcess(event);
}

void WalkController::publishFeedback() const
{
  if (walk_controller_plugin_->getState() != IDLE)
  {
    const msgs::ExecuteStepPlanFeedback& feedback = walk_controller_plugin_->getFeedback();

    // publish feedback
    planning_feedback_pub_.publish(feedback);

    if (execute_step_plan_as_->isActive())
      execute_step_plan_as_->publishFeedback(feedback);
  }
}

// --- Subscriber calls ---

void WalkController::loadStepPlanMsgPlugin(const std_msgs::StringConstPtr& plugin_name)
{
  loadPlugin(plugin_name->data, step_plan_msg_plugin_);

  if (walk_controller_plugin_)
    walk_controller_plugin_->setStepPlanMsgPlugin(step_plan_msg_plugin_);
}

void WalkController::loadWalkControllerPlugin(const std_msgs::StringConstPtr& plugin_name)
{
  loadPlugin(plugin_name->data, walk_controller_plugin_);

  if (walk_controller_plugin_)
    walk_controller_plugin_->setStepPlanMsgPlugin(step_plan_msg_plugin_);
}

void WalkController::executeStepPlan(const msgs::StepPlanConstPtr& step_plan)
{
  executeStepPlan(*step_plan);
}

//--- action server calls ---

void WalkController::executeStepPlanAction(ExecuteStepPlanActionServerPtr& as)
{
  const msgs::ExecuteStepPlanGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  executeStepPlan(goal->step_plan);
}

void WalkController::executePreemptionAction(ExecuteStepPlanActionServerPtr& as)
{
  if (as->isActive())
    as->setPreempted();

  //walk_controller_plugin->stop();
}
} // namespace
