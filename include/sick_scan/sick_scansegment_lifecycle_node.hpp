#ifndef SICK_SCANSEGMENT_LIFECYCLE_NODE_HPP
#define SICK_SCANSEGMENT_LIFECYCLE_NODE_HPP

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "sick_scansegment_xd/config.h"
#include "sick_scansegment_xd/scansegment_threads.h"

class ScansegmentLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ScansegmentLifecycleNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ScansegmentLifecycleNode() override = default;

  // Lifecycle transition callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  sick_scansegment_xd::Config config_;
  std::unique_ptr<sick_scansegment_xd::MsgPackThreads> msgpack_threads_;
};

#endif // SICK_SCANSEGMENT_LIFECYCLE_NODE_HPP
