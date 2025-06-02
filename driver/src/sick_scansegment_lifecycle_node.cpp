#include "sick_scansegment_lifecycle_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/logging.hpp>
#include <sick_scan_xd/srv/cola_msg_srv.hpp> 

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
  cola_client_ = this->create_client<sick_scan_xd::srv::ColaMsgSrv>("/ColaMsg");
  msgpack_threads_ = std::make_unique<sick_scansegment_xd::MsgPackThreads>();

  RCLCPP_INFO(get_logger(), "Configuration complete.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ScansegmentLifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating node...");


  send_sopas_command("sMN LMCstartmeas");

  
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

  
  if (send_sopas_command("sMN LMCstopmeas")) {
  msgpack_threads_->stop(true);  
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

bool ScansegmentLifecycleNode::send_sopas_command(const std::string &command)
{
  using ColaMsgSrv = sick_scan_xd::srv::ColaMsgSrv;
  if (!cola_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "SOPAS /ColaMsg service not available.");
    return false;
  }

  auto request = std::make_shared<ColaMsgSrv::Request>();
  request->request = command;

  auto future = cola_client_->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5));

  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "SOPAS command '%s' timed out.", command.c_str());
    return false;
  }

  auto response = future.get();
  RCLCPP_INFO(get_logger(), "SOPAS response: %s", response->response.c_str());

  return !response->response.empty() &&
         response->response.find("sAN") != std::string::npos;
}
// Register with component system
RCLCPP_COMPONENTS_REGISTER_NODE(ScansegmentLifecycleNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScansegmentLifecycleNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}