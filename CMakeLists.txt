cmake_minimum_required(VERSION 3.0.2)
project(indooruav_dji_driver)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  roscpp
  std_msgs
)

add_service_files(
  FILES
    PSDK_CameraShootPhoto.srv
    PSDK_GimbalPitchAngleInDeg.srv
    PSDK_Landing.srv
    PSDK_TakeOff.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include psdk_lib/include
  CATKIN_DEPENDS geometry_msgs message_filters message_runtime nav_msgs roscpp std_msgs
)

set(PAYLOAD_SDK_LIBRARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/psdk_lib/lib")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(PAYLOAD_SDK_STATIC_LIB
      "${PAYLOAD_SDK_LIBRARY_DIR}/x86_64-linux-gnu-gcc/libpayloadsdk.a")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(PAYLOAD_SDK_STATIC_LIB
      "${PAYLOAD_SDK_LIBRARY_DIR}/aarch64-linux-gnu-gcc/libpayloadsdk.a")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(armv7l|armv7|arm)$")
  set(PAYLOAD_SDK_STATIC_LIB
      "${PAYLOAD_SDK_LIBRARY_DIR}/arm-linux-gnueabihf-gcc/libpayloadsdk.a")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT EXISTS "${PAYLOAD_SDK_STATIC_LIB}")
  message(FATAL_ERROR "Payload SDK library not found: ${PAYLOAD_SDK_STATIC_LIB}")
endif()

include_directories(
  include
  include/dependences
  psdk_lib/include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_platform
  src/dependences/application.cpp
  src/dependences/hal_uart.c
  src/dependences/osal.c
  src/dependences/osal_fs.c
  src/dependences/osal_socket.c
)
add_dependencies(${PROJECT_NAME}_platform
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_platform
  PUBLIC
    ${PAYLOAD_SDK_STATIC_LIB}
    ${catkin_LIBRARIES}
    dl
    pthread
    rt
)

add_library(${PROJECT_NAME}_driver
  src/indooruav_dji_driver/mavic_3t_camera_controller.cpp
  src/indooruav_dji_driver/mavic_3t_data_subscription_manager.cpp
  src/indooruav_dji_driver/mavic_3t_flight_controller.cpp
  src/indooruav_dji_driver/mavic_3t_gimbal_controller.cpp
)
add_dependencies(${PROJECT_NAME}_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_driver
  PUBLIC
    ${PROJECT_NAME}_platform
    ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_driver
  PRIVATE
    -Wall
    -Wextra
)

add_executable(mavic_3t_driver_node
  node/mavic_3t_driver.cpp
)
add_dependencies(mavic_3t_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mavic_3t_driver_node
  PRIVATE
    ${PROJECT_NAME}_driver
)
target_compile_options(mavic_3t_driver_node
  PRIVATE
    -Wall
    -Wextra
)

add_executable(mavic_3t_driver_test_node
  node/mavic_3t_driver_test.cpp
)
add_dependencies(mavic_3t_driver_test_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mavic_3t_driver_test_node
  PRIVATE
    ${catkin_LIBRARIES}
)
target_compile_options(mavic_3t_driver_test_node
  PRIVATE
    -Wall
    -Wextra
)

set_source_files_properties(
  src/indooruav_dji_driver/mavic_3t_data_subscription_manager.cpp
  PROPERTIES
    COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-unused-variable;-Wno-unused-value"
)

install(TARGETS
  ${PROJECT_NAME}_platform
  ${PROJECT_NAME}_driver
  mavic_3t_driver_node
  mavic_3t_driver_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
