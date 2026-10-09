/** @file
 * @brief Executable entry point for scene selection and serialized ROS callbacks.
 */
#include <iostream>
#include <memory>
#include <stdexcept>

#include "distance_controller/distance_controller.hpp"

/**
 * @brief Select a scene and run its controller until ROS shutdown.
 *
 * @par Process lifecycle
 * Initialize ROS, remove ROS-specific arguments, and accept optional scene 1 or 2.
 * Construct the node and spin callbacks serially; report exceptions to stderr.
 *
 * @param[in] argc Argument count supplied by the process runtime; read by rclcpp::init()
 * and rclcpp::remove_ros_arguments() to interpret argv. No result is written back.
 * @param[in] argv Argument strings supplied by the process runtime; ROS consumes its
 * options and main() reads the remaining scene argument for DistanceController().
 * This program does not modify the strings or return results through them.
 * @return Zero after normal spin completion, or one after a caught standard exception.
 * @note Initializes and shuts down the default ROS context. Scene 2 validates distance and heading parameters before creating motion interfaces.
 */
int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);

    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() > 2 || (args.size() == 2 && args[1] != "1" && args[1] != "2")) {
      throw std::invalid_argument("Usage: distance_controller [1|2] [--ros-args ...]");
    }
    const int scene_number = args.size() == 2 && args[1] == "2" ? 2 : 1;
    auto node = std::make_shared<DistanceController>(scene_number);

    rclcpp::spin(node);
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "distance_controller: " << error.what() << std::endl;
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
