cmake_minimum_required(VERSION 3.5)
project(mobictl_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

# add_compile_definitions(MIN_DEBUG_PRINTING)
add_compile_definitions(MOBICTL_DEBUG_PRINTING)

include_directories(include)

file(GLOB my_sources src/*)

add_executable(mobictl ${my_sources})
ament_target_dependencies(mobictl rclcpp std_msgs sensor_msgs)

target_include_directories(mobictl PRIVATE ${SERIAL_INCLUDE_DIRS})
target_link_libraries(mobictl ${SERIAL_LDFLAGS})


install(TARGETS
  mobictl
  DESTINATION lib/${PROJECT_NAME})

ament_package()