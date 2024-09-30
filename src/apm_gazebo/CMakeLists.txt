cmake_minimum_required(VERSION 3.8)
project(apm_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(GimbalControllerPlugin SHARED src/GimbalControllerPlugin.cc)
target_link_libraries(GimbalControllerPlugin ${GAZEBO_LIBRARIES})
target_include_directories(GimbalControllerPlugin PRIVATE include)

add_library(ArduPilotPlugin SHARED src/ArduPilotPlugin.cc)
target_link_libraries(ArduPilotPlugin ${GAZEBO_LIBRARIES})
target_include_directories(ArduPilotPlugin PRIVATE include)

install(TARGETS
GimbalControllerPlugin
ArduPilotPlugin
  DESTINATION ${PROJECT_SOURCE_DIR}/bin
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  bin
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
