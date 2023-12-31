cmake_minimum_required(VERSION 3.8)
project(seahawk)

# Default to C++20
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 20)
#   set(CMAKE_CXX_STANDARD_REQUIRED ON)
# endif()

# if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)

ament_auto_find_build_dependencies()

# add_executable(cpp_node path/to/node.cpp)
# ament_target_dependencies(cpp_node rclcpp ...)
# target_include_directories(cpp_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(cpp_node PUBLIC c_std_99 cxx_std_20)

# install(
#   TARGETS
#     cpp_node
#     ...
#   DESTINATION lib/${PROJECT_NAME}
# )

# Copy files
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Copy directories
install(
  DIRECTORY rviz launch resources
  DESTINATION share/${PROJECT_NAME}
)

# Install Python Modules
ament_python_install_package(seahawk_deck)
ament_python_install_package(seahawk_rov)

# Install Python Executables
install(PROGRAMS
  seahawk_deck/thrust.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME thrust
)

install(PROGRAMS
  seahawk_deck/input_xbox_one.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME input_xbox_one
)

install(PROGRAMS
  seahawk_deck/rviz_markers.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rviz_markers
)

install(PROGRAMS
  seahawk_rov/__main__.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME seahawk_rov
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
