#include <vigir_walk_control/walk_controller_node.h>

#include <vigir_generic_params/parameter_manager.h>
#include <vigir_pluginlib/plugin_manager.h>



namespace vigir_walk_control
{
WalkControllerNode::WalkControllerNode(ros::NodeHandle& nh)
{
  walk_controller_.reset(new WalkController(nh, nh.param("auto_spin", true)));
}

WalkControllerNode::~WalkControllerNode()
{
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vigir_walk_controller");

  ros::NodeHandle nh;

  // ensure that node's services are set up in proper namespace
  if (nh.getNamespace().size() <= 1)
    nh = ros::NodeHandle("~");

  // init parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(nh);
  vigir_pluginlib::PluginManager::initialize(nh);

  vigir_walk_control::WalkControllerNode walk_controller_node(nh);

  ros::spin();

  return 0;
}
