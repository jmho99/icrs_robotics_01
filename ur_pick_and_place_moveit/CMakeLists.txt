cmake_minimum_required(VERSION 3.8)
project(ur_pick_and_place_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(teamproject_01 src/teamproject_01.cpp)

target_include_directories(teamproject_01 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teamproject_01 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  teamproject_01
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS teamproject_01
  DESTINATION lib/${PROJECT_NAME}/
)

add_executable(teamproject_02 src/teamproject_02.cpp)

target_include_directories(teamproject_02 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teamproject_02 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  teamproject_02
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS teamproject_02
  DESTINATION lib/${PROJECT_NAME}/
)


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
 set(ament_cmake_copyright_FOUND TRUE)
 set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
