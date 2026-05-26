#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/tree_node.h"

#include "behavior_tree_cpp/bt_nodes.hpp"

using behavior_tree_cpp_pkg::SharedData;
using behavior_tree_cpp_pkg::CondCriticalOK;
using behavior_tree_cpp_pkg::EmergencyStop;
using behavior_tree_cpp_pkg::CartographerRestartNode;
using behavior_tree_cpp_pkg::CheckObstacleNode;
using behavior_tree_cpp_pkg::SelectPathNode;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_main");
  auto shared = std::make_shared<SharedData>();

  BT::BehaviorTreeFactory factory;

  factory.registerBuilder<CondCriticalOK>(
    "CondCriticalOK",
    [node, shared](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<CondCriticalOK>(name, config, node, shared);
    });

  factory.registerBuilder<EmergencyStop>(
    "EmergencyStop",
    [node](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<EmergencyStop>(name, config, node);
    });

  factory.registerBuilder<CartographerRestartNode>(
    "CartographerRestart",
    [node](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<CartographerRestartNode>(name, config, node);
    });

  factory.registerBuilder<CheckObstacleNode>(
    "CheckObstacle",
    [node, shared](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<CheckObstacleNode>(name, config, node, shared);
    });

  factory.registerBuilder<SelectPathNode>(
    "SelectPath",
    [node, shared](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SelectPathNode>(name, config, node, shared);
    });

  // Python 트리 구조 그대로:
  // Root = Sequence( CriticalGuard, Main )
  // CriticalGuard = Selector( CondCriticalOK, EmergencyStop )
  // Main = Selector( CheckObstacle, SelectPath )
  //
  // CheckObstacle는 FAILURE를 반환 -> Selector가 SelectPath로 넘어감 (python 동일)
  // BehaviorTree.CPP 에서는 Selector라고 하지 않고 Fallback 이라고 한다!
  const char* xml_text = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <Sequence name="Root">
        <CartographerRestart name="CartographerRestart"/>

        <Fallback name="CriticalGuard">
          <CondCriticalOK name="CondCriticalOK"/>
          <EmergencyStop name="E-Stop[Guard]"/>
        </Fallback>

        <Fallback name="Main">
          <CheckObstacle name="CheckObstacle"
                         dynamic_obstacle="{dynamic_obstacle}"
                         static_obstacle="{static_obstacle}"
                         dynamic_distance="{dynamic_distance}"
                         static_distance="{static_distance}"
                         prioritize_dynamic_flag="{prioritize_dynamic_flag}"
                         obstacle_mode="{obstacle_mode}"/>
          <SelectPath name="SelectPath"
                      dynamic_distance="{dynamic_distance}"
                      static_distance="{static_distance}"
                      overtake_flag="{overtake_flag}"/>
        </Fallback>
      </Sequence>
    </BehaviorTree>
  </root>
  )";

  BT::Tree tree = factory.createTreeFromText(xml_text);

  RCLCPP_INFO(node->get_logger(), "BT started.");

  rclcpp::WallRate rate(40.0);  // python: spin_once timeout_sec=0.025
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    tree.rootNode()->executeTick();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
