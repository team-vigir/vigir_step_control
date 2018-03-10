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

#ifndef STEP_CONTROLLER_H__
#define STEP_CONTROLLER_H__

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <vigir_pluginlib/plugin_manager.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_plugins/plugins/step_plan_msg_plugin.h>

#include <vigir_step_control/step_controller_plugin.h>



namespace vigir_step_control
{
using namespace vigir_footstep_planning_msgs;

typedef actionlib::SimpleActionServer<msgs::ExecuteStepPlanAction> ExecuteStepPlanActionServer;
typedef boost::shared_ptr<ExecuteStepPlanActionServer> ExecuteStepPlanActionServerPtr;

class StepController
{
public:
  // typedefs
  typedef boost::shared_ptr<StepController> Ptr;
  typedef boost::shared_ptr<const StepController> ConstPtr;

  /**
   * @brief StepController
   * @param nh Nodehandle living in correct namespace for all services
   */
  StepController(ros::NodeHandle& nh);
  virtual ~StepController();

  /**
   * @brief Initializes step controller
   * @param nh Nodehandle living in correct namespace for all parameters
   * @param auto_spin If true, then the controller sets up it's own ros timer for calling update(...) continously.
   * @return true, if initialization was successful
   */
  bool initialize(ros::NodeHandle& nh, bool auto_spin = true);

  /**
   * @brief Initializes step controller
   * @param nh Nodehandle living in correct namespace for all parameters
   * @param step_controller_plugin Plugin interfacing the H/W layer of the robot
   * @param step_plan_msg_plugin Plugin decoding the robot specific parameters from step msg
   * @param auto_spin If true, then the controller sets up it's own ros timer for calling update(...) continously.
   * @return true, if initialization was successful
   */
  bool initialize(ros::NodeHandle& nh, StepControllerPlugin::Ptr step_controller_plugin, vigir_footstep_planning::StepPlanMsgPlugin::Ptr step_plan_msg_plugin, bool auto_spin = true);

  /**
   * @brief Loads plugin with specific name to be used by the controller. The name should be configured
   * in the plugin config file and loaded to the rosparam server. The call can only succeed when currentl
   * no exection is runnning.
   * @param plugin_name Name of plugin
   */
  template<typename T>
  bool loadPlugin(const std::string& plugin_name, boost::shared_ptr<T>& plugin)
  {
    boost::unique_lock<boost::shared_mutex> lock(controller_mutex_);

    if (step_controller_plugin_ && step_controller_plugin_->getState() == ACTIVE)
    {
      ROS_ERROR("[StepController] Cannot replace plugin due to active footstep execution!");
      return false;
    }

    if (!vigir_pluginlib::PluginManager::addPluginByName(plugin_name))
    {
      ROS_ERROR("[StepController] Could not load plugin '%s'!", plugin_name.c_str());
      return false;
    }
    else if (!vigir_pluginlib::PluginManager::getPlugin(plugin))
    {
      ROS_ERROR("[StepController] Could not obtain plugin '%s' from plugin manager!", plugin_name.c_str());
      return false;
    }
    else
      ROS_INFO("[StepController] Loaded plugin '%s'.", plugin_name.c_str());

    return true;
  }

  /**
   * @brief Instruct the controller to execute the given step plan. If execution is already in progress,
   * the step plan will be merged into current execution queue.
   * @param Step plan to be executed
   */
  void executeStepPlan(const msgs::StepPlan& step_plan);

  /**
   * @brief Main update loop to be called in regular intervals.
   */
  void update(const ros::TimerEvent& event = ros::TimerEvent());

protected:
  /**
   * @brief Publishes feedback messages of current state of execution.
   */
  void publishFeedback() const;

  StepControllerPlugin::Ptr step_controller_plugin_;
  vigir_footstep_planning::StepPlanMsgPlugin::Ptr step_plan_msg_plugin_;

  // mutex to ensure thread safeness
  boost::shared_mutex controller_mutex_;

  /// ROS API

  // subscriber
  void loadStepPlanMsgPlugin(const std_msgs::StringConstPtr& plugin_name);
  void loadStepControllerPlugin(const std_msgs::StringConstPtr& plugin_name);
  void executeStepPlan(const msgs::StepPlanConstPtr& step_plan);

  // action server calls
  void executeStepPlanAction(ExecuteStepPlanActionServerPtr as);
  void executePreemptionAction(ExecuteStepPlanActionServerPtr as);

  // subscriber
  ros::Subscriber load_step_plan_msg_plugin_sub_;
  ros::Subscriber load_step_controller_plugin_sub_;
  ros::Subscriber execute_step_plan_sub_;

  // publisher
  ros::Publisher feedback_pub_;

  // action servers
  boost::shared_ptr<ExecuteStepPlanActionServer> execute_step_plan_as_;

  // timer for updating periodically
  ros::Timer update_timer_;
};
}

#endif
