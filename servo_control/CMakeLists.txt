cmake_minimum_required(VERSION 2.8.3)
project(servo_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  tf
)

add_service_files(
  DIRECTORY
  srv
  FILES
  CommandServo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES servo_control
  CATKIN_DEPENDS roscpp std_msgs
# DEPENDS system_lib
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

link_directories(/usr/local/lib/sFoundation)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp src/ServoProxy.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} sFoundation20)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})  
                    
add_executable(command_servo_server src/command_servo_server.cpp)
target_link_libraries(command_servo_server ${catkin_LIBRARIES})
add_dependencies(command_servo_server ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

