#include <vigir_step_control/step_controller_test_plugin.h>



namespace vigir_step_control
{
StepControllerTestPlugin::StepControllerTestPlugin()
  : StepControllerPlugin()
{
}

StepControllerTestPlugin::~StepControllerTestPlugin()
{
}

void StepControllerTestPlugin::initWalk()
{
  // init feedback states
  msgs::ExecuteStepPlanFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.last_performed_step_index = -2;
  feedback.currently_executing_step_index =-1;
  feedback.first_changeable_step_index = 0;
  setFeedbackState(feedback);

  next_step_needed_time_ = ros::Time::now();

  setState(ACTIVE);

  ROS_INFO("[StepControllerTestPlugin] Start fake execution.");
}

void StepControllerTestPlugin::preProcess(const ros::TimerEvent& event)
{
  StepControllerPlugin::preProcess(event);

  if (getState() != ACTIVE)
    return;

  // fake succesful execution of single step
  if (next_step_needed_time_ <= ros::Time::now())
  {
    msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();

    feedback.header.stamp = ros::Time::now();
    feedback.last_performed_step_index++;

    // check for successful execution of queue
    if (step_queue_->lastStepIndex() == feedback.last_performed_step_index)
    {
      ROS_INFO("[StepControllerTestPlugin] Fake execution finished.");

      feedback.currently_executing_step_index = -1;
      feedback.first_changeable_step_index = -1;
      setFeedbackState(feedback);

      step_queue_->reset();
      updateQueueFeedback();

      setState(FINISHED);
    }
    // otherwise trigger fake execution of next step
    else
    {
      feedback.currently_executing_step_index++;
      feedback.first_changeable_step_index++;
      setFeedbackState(feedback);

      setNextStepIndexNeeded(feedback.currently_executing_step_index);
    }
  }
}

bool StepControllerTestPlugin::executeStep(const msgs::Step& step)
{
  next_step_needed_time_ = ros::Time::now() + ros::Duration(1.0 + step.step_duration);
  ROS_INFO("[StepControllerTestPlugin] Fake execution of step %i", step.step_index);
  return true;
}
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vigir_step_control::StepControllerTestPlugin, vigir_step_control::StepControllerPlugin)
