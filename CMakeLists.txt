cmake_minimum_required(VERSION 2.8)
project(mavis_ros)

add_definitions(-ftemplate-backtrace-limit=0 -Wno-unused-parameter)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 4.2 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  message_filters
  roscpp
  rospy
  tf
  tf2
  message_generation
)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

add_service_files(
  FILES
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package (
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/mavis
  ${PROJECT_SOURCE_DIR}/mavis/include
  ${PROJECT_SOURCE_DIR}/mavis/include/CameraModels
  ${PROJECT_SOURCE_DIR}/mavis/Thirdparty
  ${PROJECT_SOURCE_DIR}/mavis/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ORT_DIR}/include
)


include(${PROJECT_SOURCE_DIR}/mavis/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/mavis/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/mavis/lib)

add_library(${PROJECT_NAME} SHARED
  mavis/src/System.cc
  mavis/src/Tracking.cc
  mavis/src/LocalMapping.cc
  mavis/src/LoopClosing.cc
  mavis/src/ORBextractor.cc
  mavis/src/ORBmatcher.cc
  mavis/src/FrameDrawer.cc
  mavis/src/Converter.cc
  mavis/src/MapPoint.cc
  mavis/src/KeyFrame.cc
  mavis/src/Atlas.cc
  mavis/src/Map.cc
  mavis/src/MapDrawer.cc
  mavis/src/Optimizer.cc
  mavis/src/Frame.cc
  mavis/src/KeyFrameDatabase.cc
  mavis/src/Sim3Solver.cc
  mavis/src/Viewer.cc
  mavis/src/ImuTypes.cc
  mavis/src/G2oTypes.cc
  mavis/src/CameraModels/Pinhole.cpp
  mavis/src/CameraModels/KannalaBrandt8.cpp
  mavis/src/OptimizableTypes.cpp
  mavis/src/MLPnPsolver.cpp
  mavis/src/GeometricTools.cc
  mavis/src/TwoViewReconstruction.cc
  mavis/src/Config.cc
  mavis/src/Settings.cc
  mavis/include/System.h
  mavis/include/Tracking.h
  mavis/include/LocalMapping.h
  mavis/include/LoopClosing.h
  mavis/include/ORBextractor.h
  mavis/include/ORBmatcher.h
  mavis/include/FrameDrawer.h
  mavis/include/Converter.h
  mavis/include/MapPoint.h
  mavis/include/KeyFrame.h
  mavis/include/Atlas.h
  mavis/include/Map.h
  mavis/include/MapDrawer.h
  mavis/include/Optimizer.h
  mavis/include/Frame.h
  mavis/include/KeyFrameDatabase.h
  mavis/include/Sim3Solver.h
  mavis/include/Viewer.h
  mavis/include/ImuTypes.h
  mavis/include/G2oTypes.h
  mavis/include/CameraModels/GeometricCamera.h
  mavis/include/CameraModels/Pinhole.h
  mavis/include/CameraModels/KannalaBrandt8.h
  mavis/include/OptimizableTypes.h
  mavis/include/MLPnPsolver.h
  mavis/include/GeometricTools.h
  mavis/include/TwoViewReconstruction.h
  mavis/include/SerializationUtils.h
  mavis/include/Config.h
  mavis/include/Settings.h
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/mavis/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/mavis/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)

## ROS node
add_executable(ros_multi
  src/ros_multi.cc
  src/common.cc
)
target_link_libraries(ros_multi
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
