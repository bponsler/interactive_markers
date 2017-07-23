cmake_minimum_required(VERSION 3.5)
project(interactive_markers)

# Add support for C++11
if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros2_console REQUIRED)
find_package(ros2_time REQUIRED)

ament_python_install_package(${PROJECT_NAME})

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${visualization_msgs_INCLUDE_DIRS}
  ${ros2_console_INCLUDE_DIRS} ${ros2_time_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS}
  ${rclcpp_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${tf2_LIBRARY_DIRS} ${visualization_msgs_LIBRARY_DIRS}
  ${ros2_console_LIBRARY_DIRS} ${ros2_time_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES}
  ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES}
  ${tf2_LIBRARIES} ${visualization_msgs_LIBRARIES}
  ${ros2_console_LIBRARIES} ${ros2_time_LIBRARIES})

add_library(${PROJECT_NAME} SHARED src/interactive_marker_server.cpp src/tools.cpp
  src/menu_handler.cpp src/interactive_marker_client.cpp src/single_client.cpp
  src/message_context.cpp)

target_link_libraries(${PROJECT_NAME} ${LIBS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib)
install(DIRECTORY include/interactive_markers/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h")

# C++ Unit Tests
#
if(GTEST_FOUND)

  add_executable(server_test EXCLUDE_FROM_ALL src/test/server_test.cpp)
  target_link_libraries(server_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
  add_dependencies(tests server_test)
  add_rostest(test/cpp_server.test)

  add_executable(client_test EXCLUDE_FROM_ALL src/test/client_test.cpp)
  target_link_libraries(client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
  add_dependencies(tests client_test)
  add_rostest(test/cpp_client.test)

  add_executable(server_client_test
    EXCLUDE_FROM_ALL src/test/server_client_test.cpp)
  target_link_libraries(server_client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
  add_dependencies(tests server_client_test)
  add_rostest(test/cpp_server_client.test)

  # Test program to simulate Interactive Marker with missing tf information
  add_executable(bursty_tf EXCLUDE_FROM_ALL src/test/bursty_tf.cpp)
  target_link_libraries(bursty_tf ${PROJECT_NAME})
  add_dependencies(tests bursty_tf)

  # Test program to simulate Interactive Marker with wrong tf information
  add_executable(missing_tf EXCLUDE_FROM_ALL src/test/missing_tf.cpp)
  target_link_libraries(missing_tf ${PROJECT_NAME})
  add_dependencies(tests missing_tf)
endif()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(ros2_console)
ament_export_dependencies(ros2_time)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
