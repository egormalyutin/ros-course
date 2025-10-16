cmake_minimum_required(VERSION 3.8)
project(turtle_multi_target)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

find_package(turtle_multi_target_interfaces REQUIRED)

add_executable(carrot src/carrot.cpp)
ament_target_dependencies(carrot
    rclcpp
    std_msgs
    tf2_ros
)

add_executable(turtle_tf src/turtle_tf.cpp)
ament_target_dependencies(turtle_tf
    rclcpp
    std_msgs
    tf2_ros
    turtlesim
)

add_executable(spawn src/spawn.cpp)
ament_target_dependencies(spawn
    rclcpp
    turtlesim
)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller
    rclcpp
    tf2_ros
    turtlesim
    geometry_msgs
    std_srvs
    turtle_multi_target_interfaces
)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop
    rclcpp
    std_srvs
)


install(TARGETS
    carrot turtle_tf spawn controller teleop
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/)


if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
