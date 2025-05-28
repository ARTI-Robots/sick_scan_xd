#include "sick_scansegment_lifecycle_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/logging.hpp>

ScansegmentLifecycleNode::ScansegmentLifecycleNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sick_scansegment_lifecycle_node", options)
{
  RCLCPP_INFO(get_logger(), "ScansegmentLifecycleNode constructed");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ScansegmentLifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{

  RCLCPP_INFO(get_logger(), "Configuring node...");
  config_.node = this->shared_from_this();
  if (!config_.Init(config_.node)) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize config.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  config_.PrintConfig();

  msgpack_threads_ = std::make_unique<sick_scansegment_xd::MsgPackThreads>();

  RCLCPP_INFO(get_logger(), "Configuration complete.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ScansegmentLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating node...");

  if (!msgpack_threads_ || !msgpack_threads_->start(config_)) {
    RCLCPP_ERROR(get_logger(), "Failed to start MsgPackThreads.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Node activated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ScansegmentLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating node...");
  if (msgpack_threads_) {
    msgpack_threads_->stop(true);  // join thread
  }
  RCLCPP_INFO(get_logger(), "Node deactivated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ScansegmentLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up node...");
  msgpack_threads_.reset();
  RCLCPP_INFO(get_logger(), "Cleanup complete.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ScansegmentLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down node...");
  msgpack_threads_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Register with component system
RCLCPP_COMPONENTS_REGISTER_NODE(ScansegmentLifecycleNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScansegmentLifecycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}