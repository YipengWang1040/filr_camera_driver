cmake_minimum_required(VERSION 2.8.3)
project(flir_camera_driver)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_BUILD_TYPE Release)

###
# camera Acquisition specific 
###
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" CACHE PATH "Project Include Directory")
set(CUDA_USE_STATIC_CUDA_RUNTIME  OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  cv_bridge
  image_transport
  sensor_msgs
)



###
# Acquisition specific
###
# Find Packages
find_package(Spinnaker REQUIRED)
message("spinnaker lib :   " ${Spinnaker_LIBRARIES})
find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED)
if(Boost_FOUND)
  find_package ( Boost COMPONENTS filesystem system serialization REQUIRED )
  set(Boost_GENERAL ${Boost_LIBRARIES})
elseif(NOT Boost_FOUND)
  message("Boost not found!")
endif()


add_message_files(
    FILES
    ImageAddition.msg
)


generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs message_runtime
    DEPENDS OpenCV
)


include_directories(
    include
    ${PROJECT_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    SYSTEM ${Spinnaker_INCLUDE_DIRS}
    SYSTEM ${OpenCV_INCLUDE_DIRS}
    SYSTEM ${Boost_INCLUDE_DIR}
)

link_directories( ${Spinnaker_LIBRARIES} )

set (LIBS ${Spinnaker_LIBRARIES} ${OpenCV_LIBS} ${Boost_GENERAL})

add_executable (flir_camera_driver src/driver.cpp src/config.cpp src/device.cpp)
add_dependencies(flir_camera_driver ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries (flir_camera_driver ${LIBS} ${catkin_LIBRARIES})

