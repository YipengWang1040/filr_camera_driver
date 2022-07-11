cmake_minimum_required(VERSION 2.8.3)
project(spinnaker_sdk_camera_driver)

add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

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
#find_package(PCL REQUIRED)

###
# Acquisition specific
###
# Find Packages
find_package(Spinnaker REQUIRED)
message("spinnaker lib :   " ${Spinnaker_LIBRARIES})
find_package(OpenCV REQUIRED)
# use LibUnwind only for x86_64 or x86_32 architecture
# do not use LibUnwind for arm architecture
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
  message("uses LibUnwind for x86_64 or x86_32 architecture")
  find_package(LibUnwind REQUIRED)
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64 OR x86_32)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES aarch64 OR arm)
  message("Detected ARM architecture")
endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES aarch64 OR arm)

find_package(Boost REQUIRED)
if(Boost_FOUND)
  find_package ( Boost COMPONENTS filesystem system serialization REQUIRED )
  set(Boost_GENERAL ${Boost_LIBRARIES})
elseif(NOT Boost_FOUND)
  message("Boost not found!")
endif()

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/include/spinnaker_sdk_camera_driver/spinnaker_configure.h.in"
  "${PROJECT_SOURCE_DIR}/include/spinnaker_sdk_camera_driver/spinnaker_configure.h"
)

add_executable (driver src/driver.cpp)
add_dependencies(driver ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries (driver ${LIBS} ${catkin_LIBRARIES})

