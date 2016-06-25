//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef WALK_CONTROLLER_PLUGIN_H__
#define WALK_CONTROLLER_PLUGIN_H__

#include <ros/ros.h>

#include <vigir_pluginlib/plugin.h>

#include <vigir_footstep_planning_plugins/step_plan_msg_plugin.h>

#include <vigir_walk_control/step_queue.h>



namespace vigir_walk_control
{
using namespace vigir_footstep_planning_msgs;

enum WalkControllerState
{
  NOT_READY = msgs::ExecuteStepPlanFeedback::NOT_READY,
  READY     = msgs::ExecuteStepPlanFeedback::READY,
  ACTIVE    = msgs::ExecuteStepPlanFeedback::ACTIVE,
  PAUSED    = msgs::ExecuteStepPlanFeedback::PAUSED,
  FINISHED  = msgs::ExecuteStepPlanFeedback::FINISHED,
  FAILED    = msgs::ExecuteStepPlanFeedback::FAILED
};

std::string toString(const WalkControllerState& state);

class WalkControllerPlugin
  : public vigir_pluginlib::Plugin
{
public:
  // typedefs
  typedef boost::shared_ptr<WalkControllerPlugin> Ptr;
  typedef boost::shared_ptr<const WalkControllerPlugin> ConstPtr;

  WalkControllerPlugin();
  virtual ~WalkControllerPlugin();

  /**
   * @brief Sets the StepPlanMsgPlugin to be used.
   * @param plugin Plugin of type StepPlanMsgPlugin
   */
  virtual void setStepPlanMsgPlugin(vigir_footstep_planning::StepPlanMsgPlugin::Ptr plugin);

  /**
   * @brief Get current state of execution.
   * @return WalkControllerState
   */
  WalkControllerState getState() const;

  /**
   * @brief Returns next step index needed by the walking engine
   * @return step index
   */
  int getNextStepIndexNeeded() const;

  /**
   * @brief Returns last step index which has been sent to walking engine.
   * @return step index
   */
  int getLastStepIndexSent() const;

  /**
   * @brief Returns current feedback information provided by the plugin
   * @return current feedback
   */
  const msgs::ExecuteStepPlanFeedback& getFeedbackState() const;

  /**
   * @brief Updates feedback information with internal state data.
   */
  void updateQueueFeedback();

  /**
   * @brief Merges given step plan to the current step queue of steps. Hereby, two cases have to considered:
   * 1. In case of an empty step queue (robot is standing) the step plan has to begin with step index 0.
   * 2. In case of an non-empty step queue (robot is walking) the first step of the step plan has to be
   * identical with the corresponding step (=same step index) in the step queue. Be aware that already performed
   * steps have been popped from step queue and therefore are not exisiting anymore.
   * The default implementation resets the plugin previously when in FINISHED or FAILED state.
   * @param step_plan Step plan to be merged into step queue.
   */
  virtual void updateStepPlan(const msgs::StepPlan& step_plan);

  /**
   * @brief This method is called when new step plan has been enqueued and previously the walk controller state was READY.
   * This method must be override and set state to ACTIVE when everything has been set up successfully.
   */
  virtual void initWalk() = 0;

  /**
   * @brief PreProcess Method is called before processing walk controller, e.g. for precompute/update data or check walking engine status.
   * For keeping the default behavior running, following variables has to be properly updated here:
   * - feedback.first_changeable_step_index (or update it in postProcess(...) or executeStep(...) alternatively)
   * - feedback.last_performed_step_index (or update it in postProcess(...) or executeStep(...) alternatively)
   * - next_step_index_needed
   * -- Note: Don't forget to update header data of feedback, when updating one of the mentioned values!
   * - setState(FINISEHD) when execution of all steps were successfully completed
   * - setState(FAILED) when an error has occured
   */
  virtual void preProcess(const ros::TimerEvent& event);

  /**
   * @brief Overwrite to handle robot specific behavior. The default behavior behaves as followed:
   * - For each step s in queue with index in (last_step_index_sent_; next_step_index_needed_] executeStep(s)
   *   will be called. Hereby, last_step_index_sent_ will be automatically updated.
   * - Each step in [0; feedback.last_performed_step_index] will be removed from step queue
   */
  virtual void process(const ros::TimerEvent& event);

  /**
   * @brief PostProcess Method is called after processing step, in purpose of sum up current status and cleanups for instance.
   */
  virtual void postProcess(const ros::TimerEvent& /*event*/) {}

  /**
   * @brief This method will be called when the next step should be added to execution pipeline. The call of this function should
   * be triggered by the process(...) method when next_step_index_needed_ has been changed.
   * @param step Step to be executed now
   */
  virtual bool executeStep(const msgs::Step& step) = 0;

  /**
   * @brief Will be called when (soft) stop is requested and resets plugin.
   */
  virtual void stop();

protected:
  /**
   * @brief Resets the plugin (called during construction and by stop()).
   * The default implementation raises the READY flag. Overwrite this method if another behavior is desired.
   */
  virtual void reset();

  void setState(WalkControllerState state);

  void setNextStepIndexNeeded(int index);

  void setLastStepIndexSent(int index);

  void setFeedbackState(const msgs::ExecuteStepPlanFeedback& feedback);

  StepQueue::Ptr step_queue_;

  vigir_footstep_planning::StepPlanMsgPlugin::Ptr step_plan_msg_plugin_;

  // mutex to ensure thread safeness
  mutable boost::shared_mutex plugin_mutex_;

private:
  // current state of walk controller
  WalkControllerState state_;

  // next step index needed by walk engine
  int next_step_index_needed_;

  // last step index sent to walk engine
  int last_step_index_sent_;

  // contains current feedback state; should be updated in each cycle
  msgs::ExecuteStepPlanFeedback feedback_state_;
};
}

#endif
