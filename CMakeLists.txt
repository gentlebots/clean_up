cmake_minimum_required(VERSION 3.5)
project(clean_up)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
#find_package(ros2_knowledge_graph_plugins REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    plansys2_executor
    ament_index_cpp
    ros2_knowledge_graph
    #ros2_knowledge_graph_plugins
    rclcpp_cascade_lifecycle
)

include_directories(include)

add_executable(cleanup_executor_node src/hfsm/CleanUpHFSM.cpp src/hfsm/CleanUp_executor.cpp)
ament_target_dependencies(cleanup_executor_node ${dependencies})

add_executable(vision_sim_node src/vision_sim/vision_sim.cpp)
ament_target_dependencies(vision_sim_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  cleanup_executor_node
  vision_sim_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()