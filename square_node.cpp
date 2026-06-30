#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  // 1. Setup Node
  auto const node = std::make_shared<rclcpp::Node>(
    "square_move_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("square_move_node");

  // 2. Initialize MoveGroup (Ensure group name is "arm")
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "arm");

  // Setting planning parameters for better success rates
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setGoalPositionTolerance(0.01); 

  // 3. Define the Square Corners (X, Y, Z)
  // Ensure these points are physically reachable by your 3-DOF arm!
  std::vector<std::vector<double>> square_corners = {
    {0.25, -0.1, 0.2}, // Corner 1
    {0.25,  0.1, 0.2}, // Corner 2
    {0.35,  0.1, 0.2}, // Corner 3
    {0.35, -0.1, 0.2}  // Corner 4
  };

  RCLCPP_INFO(logger, "Starting Square Pattern movement...");

  // 4. Loop through the corners
  while (rclcpp::ok()) {
    for (size_t i = 0; i < square_corners.size(); ++i) {
      double x = square_corners[i][0];
      double y = square_corners[i][1];
      double z = square_corners[i][2];

      RCLCPP_INFO(logger, "Moving to Corner %zu: [x: %.2f, y: %.2f, z: %.2f]", i + 1, x, y, z);

      // Set the position target
      move_group.setPositionTarget(x, y, z);

      // Plan and Execute
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success) {
        move_group.execute(my_plan);
        RCLCPP_INFO(logger, "Reached Corner %zu.", i + 1);
      } else {
        RCLCPP_ERROR(logger, "Failed to plan to Corner %zu!", i + 1);
      }

      // Small pause between corners
      rclcpp::sleep_for(1s);
    }
    
    RCLCPP_INFO(logger, "Square complete. Repeating...");
  }

  rclcpp::shutdown();
  return 0;
}
