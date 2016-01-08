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

#ifndef WALK_CONTROLLER_H__
#define WALK_CONTROLLER_H__

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <vigir_walk_control/walk_controller_plugin.h>



namespace vigir_walk_control
{
using namespace vigir_footstep_planning_msgs;

typedef actionlib::SimpleActionServer<msgs::ExecuteStepPlanAction> ExecuteStepPlanActionServer;
typedef boost::shared_ptr<ExecuteStepPlanActionServer> ExecuteStepPlanActionServerPtr;

class WalkController
{
public:
  /**
   * @brief WalkController
   * @param nh Nodehandle living in correct namespace for all services
   * @param spin When true, the controller sets up it's own ros timer for calling update(...) continously.
   */
  WalkController(ros::NodeHandle& nh, bool auto_spin = true);
  virtual ~WalkController();

  /**
   * @brief Loads plugin with specific name to be used by the controller. The name should be configured in
   * the plugin config file and loaded to the rosparam server. The call can only succeed when currently no
   * exection is runnning.
   * @param plugin_name Name of plugin
   */
  void loadPluginByName(const std::string& plugin_name);

  /**
   * @brief Instruct the controller to execute the given step plan. If execution is already in progress,
   * the step plan will be merged into current execution queue.
   * @param Step plan to be executed
   */
  void executeStepPlan(const msgs::StepPlan& step_plan);

  /**
   * @brief Publishes feedback messages of current state of execution.
   */
  void publishFeedback() const;

  /**
   * @brief Main update loop to be called in regular intervals.
   */
  void update(const ros::TimerEvent& event = ros::TimerEvent());

  // typedefs
  typedef boost::shared_ptr<WalkController> Ptr;
  typedef boost::shared_ptr<const WalkController> ConstPtr;

protected:
  WalkControllerPlugin::Ptr walk_controller_plugin;

  /// ROS API

  // subscriber
  void loadPluginByName(const std_msgs::StringConstPtr& plugin_name);
  void executeStepPlan(const msgs::StepPlanConstPtr& step_plan);

  // action server calls
  void executeStepPlanAction(ExecuteStepPlanActionServerPtr& as);
  void executePreemptionAction(ExecuteStepPlanActionServerPtr& as);

  // subscriber
  ros::Subscriber load_plugin_by_name_sub;
  ros::Subscriber execute_step_plan_sub;

  // publisher
  ros::Publisher planning_feedback_pub;

  // action servers
  boost::shared_ptr<ExecuteStepPlanActionServer> execute_step_plan_as;

  ros::Timer update_timer;
};
}

#endif
