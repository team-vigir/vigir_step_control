//=================================================================================================
// Copyright (c) 2018, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_STEP_QUEUE_H__
#define VIGIR_STEP_QUEUE_H__

#include <ros/ros.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_msgs/step_plan.h>



namespace vigir_step_control
{
using namespace vigir_footstep_planning;

typedef boost::shared_mutex Mutex;
typedef boost::shared_lock<Mutex> SharedLock;
typedef boost::unique_lock<Mutex> UniqueLock;
typedef boost::upgrade_lock<Mutex> UpgradeLock;
typedef boost::upgrade_to_unique_lock<Mutex> UpgradeToUniqueLock;



class StepQueue
{
public:
  // typedefs
  typedef boost::shared_ptr<StepQueue> Ptr;
  typedef boost::shared_ptr<const StepQueue> ConstPtr;

  StepQueue();
  virtual ~StepQueue();

  void reset();

  /**
   * @brief Checks if there are steps in queue.
   * @return True if any steps has been enqueued.
   */
  bool empty() const;

  /**
   * @brief Returns size of step queue
   * @return Size of step queue
   */
  size_t size() const;

  /**
   * @brief Returns if step queue has been changed
   * @return true if recent changes have been appleid
   */
  bool isDirty() const;

  /**
   * @brief Resets dirty flag
   */
  void clearDirtyFlag();

  /**
   * @brief Merges given step plan to the current execution queue of steps. Hereby, two cases have to considered:
   * 1. In case of an empty execution queue (robot is standing) the step plan has to begin with step index 0.
   * 2. In case of an non-empty execution queue (robot is walking)
   * TODO: Stitching rules, new total step plan length
   * @param step_plan Step plan to be merged into execution queue.
   * @param min_step_index Only steps with index >= min_step_index are considered for merge
   * @return True if step plan could be merged.
   */
  bool updateStepPlan(const msgs::StepPlan& step_plan, int min_step_index = 0);

  /**
   * @brief Retrieves step of execution queue.
   * @param step Outgoing variable for retrieved step.
   * @param index Position of step in queue to be returned. 0 points to the next step to be executed next.
   * @return True If step could been returned.
   */
  bool getStep(msgs::Step& step, unsigned int step_index = 0u);

  /**
   * @brief Retrieves step of execution queue.
   * @param step Outgoing variable for retrieved step.
   * @param position Position of step in queue to be returned. 0 points to the next step to be executed next.
   * @return True If step could been returned.
   */
  bool getStepAt(msgs::Step& step, unsigned int position = 0u);

  /**
   * @brief getSteps Retrieves all steps with index in range of [start_index; end_index]
   * @param start_index Starting index
   * @param end_index Ending index
   * @return List of all found steps within given range of [start_index; end_index]
   */
  std::vector<msgs::Step> getSteps(unsigned int start_index, unsigned int end_index) const;

  /**
   * @brief Remove steps with specific index from queue.
   * @param step_index Step to be removed
   */
  void removeStep(unsigned int step_index);

  /**
   * @brief Remove steps in the given range [from_step_index, to_step_index]
   * @param from_step_index start index
   * @param to_step_index end index; to_step_index < 0 is equal to removing all steps with index >= from_step_index
   */
  void removeSteps(unsigned int from_step_index, int to_step_index = -1);

  /**
   * @brief Dequeues and returns next step from execution queue.
   * @param step Outgoing variable for dequeued step.
   * @return True if step could been dequeued.
   */
  bool popStep(msgs::Step& step);
  bool popStep();

  /**
   * @brief Returns next step index enqueued for execution.
   * @return Step index of next step in queue. If queue is empty -1 will be returned.
   */
  int firstStepIndex() const;

  /**
   * @brief Returns last step index enqueued for execution.
   * @return Step index of last step in queue. If queue is empty -1 will be returned.
   */
  int lastStepIndex() const;

protected:
  StepPlan step_plan_;

  bool is_dirty_;

  // mutex to ensure thread safeness
  mutable Mutex queue_mutex_;
};
}

#endif
