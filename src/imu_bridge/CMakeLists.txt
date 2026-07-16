cmake_minimum_required(VERSION 3.8)
project(imu_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

add_executable(orientation_pubsub src/orientation_pubsub.cpp)

ament_target_dependencies(orientation_pubsub
  rclcpp
  geometry_msgs
)

target_include_directories(orientation_pubsub PRIVATE ${LIBSERIAL_INCLUDE_DIRS})
target_link_libraries(orientation_pubsub ${LIBSERIAL_LIBRARIES})
target_compile_options(orientation_pubsub PRIVATE ${LIBSERIAL_CFLAGS_OTHER})

install(TARGETS
  orientation_pubsub
  DESTINATION lib/${PROJECT_NAME})

ament_package()