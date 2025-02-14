cmake_minimum_required(VERSION 3.8)
project(jbd_bms_status)
add_compile_options(-std=c++17)

find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(message_generation REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(jbd_bms_msg REQUIRED)

include_directories(include)
add_executable(${PROJECT_NAME}_node src/jbd_bms_status_node.cpp src/jbd_bms_status.cpp)
#ament_target_dependencies(rclcpp serial std_msgs message_genereation jbd_msg_msg)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp serial std_msgs sensor_msgs diagnostic_updater jbd_bms_msg)
install(TARGETS
  jbd_bms_status_node
  DESTINATION lib/${PROJECT_NAME}
  )
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
