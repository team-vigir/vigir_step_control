#include <vigir_walk_control/walk_controller_test_plugin.h>

#include <pluginlib/class_list_macros.h>



namespace vigir_walk_control
{
WalkControllerTestPlugin::WalkControllerTestPlugin()
  : WalkControllerPlugin()
{
}

WalkControllerTestPlugin::~WalkControllerTestPlugin()
{
}

void WalkControllerTestPlugin::initWalk()
{
  WalkControllerPlugin::initWalk();

  // init feedback
  msgs::ExecuteStepPlanFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.last_performed_step_index = -2;
  feedback.currently_executing_step_index =-1;
  feedback.first_changeable_step_index = 0;
  setFeedback(feedback);

  next_step_needed_time_ = ros::Time::now();

  ROS_INFO("[WalkControllerTestPlugin] Start fake execution.");
}

void WalkControllerTestPlugin::preProcess(const ros::TimerEvent& event)
{
  WalkControllerPlugin::preProcess(event);

  if (getState() != ACTIVE)
    return;

  // fake succesful execution of single step
  if (next_step_needed_time_ <= ros::Time::now())
  {
    msgs::ExecuteStepPlanFeedback feedback = getFeedback();

    feedback.header.stamp = ros::Time::now();
    feedback.last_performed_step_index++;

    // check for succesful execution of queue
    if (walk_controller_queue_->lastStepIndex() == feedback.last_performed_step_index)
    {
      ROS_INFO("[WalkControllerTestPlugin] Fake execution finished.");

      feedback.currently_executing_step_index = -1;
      feedback.first_changeable_step_index = -1;
      setFeedback(feedback);

      walk_controller_queue_->reset();
      updateQueueFeedback();

      setState(FINISHED);
    }
    // otherwise trigger fake execution of next step
    else
    {
      feedback.currently_executing_step_index++;
      feedback.first_changeable_step_index++;
      setFeedback(feedback);

      setNextStepIndexNeeded(feedback.currently_executing_step_index);
    }
  }
}

bool WalkControllerTestPlugin::executeStep(const msgs::Step& step)
{
  next_step_needed_time_ = ros::Time::now() + ros::Duration(1.0 + step.step_duration);
  ROS_INFO("[WalkControllerTestPlugin] Fake execution of step %i", step.step_index);
  return true;
}
}

PLUGINLIB_EXPORT_CLASS(vigir_walk_control::WalkControllerTestPlugin, vigir_walk_control::WalkControllerPlugin)
