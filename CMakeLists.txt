cmake_minimum_required(VERSION 3.8)
project(distance_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/ExecuteStep.srv")
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(distance_controller_node
  src/main.cpp
  src/distance_controller.cpp
  src/initial_centering.cpp
  src/precise_start.cpp
  src/precise_start_correction.cpp
  src/right_wall_heading.cpp
  src/frozen_wall_alignment.cpp
  src/route.cpp
  src/route_execution.cpp
  src/preparation_guard.cpp
  src/scan_recovery.cpp
  src/step_configuration.cpp
  src/manual_steps.cpp
  src/step_feedback.cpp
  src/route_history.cpp
  src/history_control.cpp)
set_target_properties(distance_controller_node PROPERTIES OUTPUT_NAME distance_controller)
target_include_directories(distance_controller_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(
  distance_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  sensor_msgs
)

rosidl_get_typesupport_target(step_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(distance_controller_node "${step_typesupport}")
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/step.py DESTINATION lib/${PROJECT_NAME} RENAME step)
install(PROGRAMS tools/survey_return.py DESTINATION lib/${PROJECT_NAME} RENAME survey_return)
install(FILES tools/survey_distances.py tools/survey_return.py tools/action_plan.py
  tools/odom_path.py tools/action_runtime.py tools/wall_geometry.py tools/wall_policy.py
  tools/front_settling.py tools/front_adjustment.py tools/turn_escape.py tools/turn_adjustment.py tools/turn_recovery.py tools/wall_runtime.py tools/wall_session.py tools/session_checkpoint.py tools/session_audit.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/action_session.py DESTINATION lib/${PROJECT_NAME} RENAME action_session)

install(
  TARGETS distance_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_route_session
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_route_session*.py)
  set_tests_properties(test_route_session PROPERTIES TIMEOUT 30)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_route test/test_route.cpp test/test_history.cpp src/route.cpp src/route_history.cpp)
  target_include_directories(test_route PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_robust_wall test/test_robust_wall.cpp)
  target_include_directories(test_robust_wall PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_start_verification test/test_start_verification.cpp)
  target_include_directories(test_start_verification PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(PROGRAMS tools/odom_trial.py DESTINATION lib/${PROJECT_NAME} RENAME odom_trial)

install(FILES tools/motion_model.py tools/teaching_path.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/motion_console.py DESTINATION lib/${PROJECT_NAME} RENAME motion_console)
install(PROGRAMS tools/task6_current_turn.py DESTINATION lib/${PROJECT_NAME} RENAME task6_current_turn)
install(FILES tools/task6_terminal.py tools/motion_console.py tools/task6_route_model.py tools/route_session_state.py tools/route_session_runtime.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/task6_route_runner.py DESTINATION lib/${PROJECT_NAME} RENAME task6_route_runner)
install(FILES tools/route_laser_guard.py tools/route_laser_runtime.py DESTINATION lib/${PROJECT_NAME})

install(FILES tools/route_return_plan.py tools/route_return_calibration.py tools/task6_route_runner.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/route_return_runner.py DESTINATION lib/${PROJECT_NAME} RENAME route_return_runner)
if(BUILD_TESTING)
  add_test(NAME test_route_return
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_route_return.py)
  set_tests_properties(test_route_return PROPERTIES TIMEOUT 30)
endif()

install(FILES tools/route_prefix_return_plan.py tools/route_prefix_return_runtime.py
  tools/route_return_runner.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/route_prefix_return_runner.py DESTINATION lib/${PROJECT_NAME}
  RENAME route_prefix_return_runner)

install(FILES tools/current_frame_step_runtime.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/current_frame_step.py DESTINATION lib/${PROJECT_NAME} RENAME current_frame_step)

install(FILES tools/current_frame_session_runtime.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/current_frame_session.py DESTINATION lib/${PROJECT_NAME} RENAME current_frame_session)
