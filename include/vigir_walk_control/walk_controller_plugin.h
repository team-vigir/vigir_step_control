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

#include <vigir_walk_control/walk_controller_queue.h>



namespace vigir_walk_control
{
using namespace vigir_footstep_planning_msgs;

enum WalkControllerState {IDLE, ACTIVE, PAUSED, FINISHED, FAILED};

std::string toString(const WalkControllerState& state);

class WalkControllerPlugin
  : public vigir_pluginlib::Plugin
{
public:
  WalkControllerPlugin();
  virtual ~WalkControllerPlugin();

  /**
   * @brief Resets the plugin. When an execution is active, stop() will be called.
   */
  virtual void reset();

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
  const msgs::ExecuteStepPlanFeedback& getFeedback() const;

  /**
   * @brief Merges given step plan to the current execution queue of steps. Hereby, two cases have to considered:
   * 1. In case of an empty execution queue (robot is standing) the step plan has to begin with step index 0.
   * 2. In case of an non-empty execution queue (robot is walking) the first step of the step plan has to be
   * identical with the corresponding step (=same step index) in the execution queue. Be aware that already executed
   * steps have been popped from execution queue and therefore are not exisiting anymore.
   * @param step_plan Step plan to be merged into execution queue.
   */
  void updateStepPlan(const msgs::StepPlan& step_plan);

  /**
   * @brief This method is called when new step plan has been enqueued and previously the walk controller state was IDLE.
   */
  virtual void initWalk() {}

  /**
   * @brief PreProcess Method is called before processing walk controller, e.g. precompute/update data, check walking engine status.
   * For keeping the default behavior running, following variables has to be updated here:
   * - feedback.first_changeable_step_index
   * - next_step_index_needed
   * - setState(FINISEHD) when execution of all steps were successfully completed
   * - setState(FAILED) when error has occured
   */
  virtual void preProcess(const ros::TimerEvent& event);

  /**
   * @brief Process Overwrite to handle robot specific behavior. The default behavior behaves followed:
   * - Each step in [0; feedback.first_changeable_step_index) will be removed from execution row
   * - For each step s in queue with index in (last_step_index_sent; next_step_index_needed] executeStep(s)
   *   will be called. Hereby, lastStepIndexSent will be automatically updated.
   */
  virtual void process(const ros::TimerEvent& event);

  /**
   * @brief PostProcess Method is called after processing step, e.g. sum up current status and cleanups.
   * The default implementation resets the plugin when execution has been finshed or has failed.
   */
  virtual void postProcess(const ros::TimerEvent& event);

  /**
   * @brief This method will be called when a new step should be executed. The call of this function should be triggered
   * by the process(...) method when nextStepIndexNeeded has been changed.
   * @param step
   */
  virtual bool executeStep(const msgs::Step& step) = 0;

  /**
   * @brief Will be called when (soft) stop is requested.
   */
  virtual void stop();

  typedef boost::shared_ptr<WalkControllerPlugin> Ptr;
  typedef boost::shared_ptr<const WalkControllerPlugin> ConstPtr;

protected:
  void setState(WalkControllerState state);

  void setNextStepIndexNeeded(int index);

  void setLastStepIndexSent(int index);

  void setFeedback(const msgs::ExecuteStepPlanFeedback& feedback);

  WalkControllerQueue::Ptr walk_controller_queue;

  // mutex to ensure thread safeness
  mutable boost::shared_mutex plugin_mutex;

private:
  // current state of walk controller
  WalkControllerState state;

  // next step index needed by walk engine
  int next_step_index_needed;

  // last step index sent to walk engine
  int last_step_index_sent;

  // contains current feedback state; should be updated in each cycle
  msgs::ExecuteStepPlanFeedback feedback;
};
}

#endif
