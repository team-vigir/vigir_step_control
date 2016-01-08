#include <vigir_walk_control/walk_controller_queue.h>

namespace vigir_walk_control
{
WalkControllerQueue::WalkControllerQueue()
{
}

WalkControllerQueue::~WalkControllerQueue()
{
}

void WalkControllerQueue::reset()
{
  boost::unique_lock<boost::shared_mutex> lock(queue_mutex);
  step_plan.clear();
}

bool WalkControllerQueue::empty() const
{
  boost::unique_lock<boost::shared_mutex> lock(queue_mutex);
  return step_plan.empty();
}

bool WalkControllerQueue::updateStepPlan(const msgs::StepPlan& step_plan, int min_step_index)
{
  if (step_plan.steps.empty())
    return true;

  /// check for consistency
  msgs::ErrorStatus status = isConsistent(step_plan);
  if (!isOk(status))
  {
    ROS_ERROR("[WalkControllerQueue] updateStepPlan: Consistency check failed!");
    ROS_ERROR("%s", toString(status).c_str());
    return false;
  }

  unsigned int step_plan_start_index = std::max(min_step_index, step_plan.steps.front().step_index);

  boost::unique_lock<boost::shared_mutex> lock(queue_mutex);

  // step index has to start at 0, when step queue is empty
  if (this->step_plan.empty())
  {
    if (step_plan_start_index != 0)
    {
      ROS_ERROR("[WalkControllerQueue] updateStepPlan: Current step queue is empty. Expected step plan starting with index 0!");
      return false;
    }
  }
  else
  {
    msgs::Step old_step;
    msgs::Step new_step;

    // check if queue and given step plan has overlapping steps
    if (!this->step_plan.getStep(old_step, step_plan_start_index))
    {
      ROS_ERROR("[WalkControllerQueue] updateStepPlan: Can't merge plan due to non-overlapping step indices of current step plan (max queued index: %u, needed index: %u)!", this->step_plan.getLastStepIndex(), step_plan_start_index);
      return false;
    }
    // check if input step plan has needed overlapping steps
    else if (!StepPlan::getStep(new_step, step_plan, step_plan_start_index))
    {
      ROS_ERROR("[WalkControllerQueue] updateStepPlan: Can't merge plan due to non-overlapping step indices of new step plan (max index: %u, needed index: %u)!", step_plan.steps.back().step_index, step_plan_start_index);
      return false;
    }
    // check if overlapping indeces have the same foot index
    else if (old_step.foot.foot_index != new_step.foot.foot_index)
    {
      ROS_ERROR("[WalkControllerQueue] updateStepPlan: Step %u has wrong foot index!", step_plan_start_index);
      return false;
    }
    // check if start foot position is equal
    else
    {
      geometry_msgs::Pose p_old = old_step.foot.pose;
      geometry_msgs::Pose p_new = new_step.foot.pose;
      if (p_old.position.x != p_new.position.x || p_old.position.y != p_new.position.y || p_old.position.z != p_new.position.z)
      {
        ROS_WARN("[WalkControllerQueue] updateStepPlan: Overlapping step differs in position!");
      }
      if (p_old.orientation.x != p_new.orientation.x || p_old.orientation.y != p_new.orientation.y || p_old.orientation.z != p_new.orientation.z || p_old.orientation.w != p_new.orientation.w)
      {
        ROS_WARN("[WalkControllerQueue] updateStepPlan: Overlapping step differs in orientation!");
      }
    }
  }

  /// merge step plan
  this->step_plan.stitchStepPlan(step_plan, step_plan_start_index);
  this->step_plan.removeSteps(step_plan.steps.rbegin()->step_index+1);

  return true;
}

bool WalkControllerQueue::getStep(msgs::Step& step, unsigned int step_index)
{
  boost::shared_lock<boost::shared_mutex> lock(queue_mutex);
  return step_plan.getStep(step, step_index);
}

bool WalkControllerQueue::getStepAt(msgs::Step& step, unsigned int position)
{
  boost::shared_lock<boost::shared_mutex> lock(queue_mutex);
  return step_plan.getStepAt(step, position);
}

std::vector<msgs::Step> WalkControllerQueue::getSteps(unsigned int start_index, unsigned int end_index) const
{
  boost::shared_lock<boost::shared_mutex> lock(queue_mutex);

  std::vector<msgs::Step> steps;

  for (unsigned int i = start_index; i <= end_index; i++)
  {
    msgs::Step step;
    if (step_plan.getStep(step, i))
      steps.push_back(step);
  }

  return steps;
}

void WalkControllerQueue::removeStep(unsigned int step_index)
{
  boost::unique_lock<boost::shared_mutex> lock(queue_mutex);
  step_plan.removeStep(step_index);
}

void WalkControllerQueue::removeSteps(unsigned int from_step_index, int to_step_index)
{
  boost::unique_lock<boost::shared_mutex> lock(queue_mutex);
  step_plan.removeSteps(from_step_index, to_step_index);
}

bool WalkControllerQueue::popStep(msgs::Step& step)
{
  boost::unique_lock<boost::shared_mutex> lock(queue_mutex);
  return step_plan.popStep(step);
}

bool WalkControllerQueue::popStep()
{
  msgs::Step step;
  return popStep(step);
}

int WalkControllerQueue::firstStepIndex() const
{
  boost::shared_lock<boost::shared_mutex> lock(queue_mutex);

  msgs::Step step;
  if (step_plan.getfirstStep(step))
    return step.step_index;
  else
    return -1;
}

int WalkControllerQueue::lastStepIndex() const
{
  boost::shared_lock<boost::shared_mutex> lock(queue_mutex);

  msgs::Step step;
  if (step_plan.getLastStep(step))
    return step.step_index;
  else
    return -1;
}
}
