cmake_minimum_required(VERSION 3.8)
project(urg_node2_nl)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(URG_LIBRARY_DIR urg_library/current)
set(URG_LIBRARY_C_INCLUDE_DIR ${URG_LIBRARY_DIR}/include/c)
set(URG_LIBRARY_SRC_DIR ${URG_LIBRARY_DIR}/src)

include_directories(include ${URG_LIBRARY_C_INCLUDE_DIR})

add_library(urg_c SHARED
  ${URG_LIBRARY_SRC_DIR}/urg_sensor.c
  ${URG_LIBRARY_SRC_DIR}/urg_utils.c
  ${URG_LIBRARY_SRC_DIR}/urg_debug.c
  ${URG_LIBRARY_SRC_DIR}/urg_connection.c
  ${URG_LIBRARY_SRC_DIR}/urg_ring_buffer.c
  ${URG_LIBRARY_SRC_DIR}/urg_serial.c
  ${URG_LIBRARY_SRC_DIR}/urg_serial_utils.c
  ${URG_LIBRARY_SRC_DIR}/urg_tcpclient.c
)

add_library(urg_node2_nl SHARED
	src/urg_node2_nl.cpp)

target_include_directories(urg_node2_nl PUBLIC include/${PROJECT_NAME})

ament_target_dependencies(urg_node2_nl
	rclcpp
	rclcpp_components
	sensor_msgs
)

rclcpp_components_register_node(urg_node2_nl
	PLUGIN "urg_node2_nl::UrgNode2NL"
	EXECUTABLE urg_node2_nl_node
)
target_link_libraries(urg_node2_nl urg_c)

install(TARGETS
	urg_node2_nl urg_c
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(
	DIRECTORY launch config
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
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
