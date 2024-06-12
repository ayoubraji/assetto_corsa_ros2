cmake_minimum_required(VERSION 3.5)
project(deep_orange_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BrakeTempReport.msg"
  "msg/Coordinates.msg"
  "msg/CtReport.msg"
  "msg/MiscReport.msg"
  "msg/PtReport.msg"
  "msg/RcToCt.msg"
  "msg/TireReport.msg"
  "msg/JoystickCommand.msg"
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
