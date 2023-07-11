cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(GeographicLib REQUIRED)
find_package(Ceres REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
)

# include directories
include_directories(
	include
  include/ScanContext
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Utilities library
add_library(${PROJECT_NAME}_utilities STATIC
  src/ScanContext/Scancontext.cpp
  src/CommonDefine.cpp
  src/CommonUtility.cpp
  src/CommonNode.cpp
  src/CoreUtility.cpp
  src/NodeUtility.cpp
  src/ImuOdometry.cpp
  src/FeatureCalculate.cpp
  src/LaserScanAdapt.cpp
  src/MapBuilding.cpp
  src/ImuOdometryPredictor.cpp
  src/LaserCloudExtractor.cpp
  src/LaserCloudRegister.cpp
  src/LaserLoopDetector.cpp
  src/MapPoseOptimizer.cpp
  src/MapCloudBuilder.cpp
)
target_link_libraries(${PROJECT_NAME}_utilities ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# Map Cloud Builder Node
add_executable(${PROJECT_NAME}_mapCloudBuilderNode src/MapCloudBuilderNode.cpp)
add_dependencies(${PROJECT_NAME}_mapCloudBuilderNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapCloudBuilderNode PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapCloudBuilderNode ${PROJECT_NAME}_utilities Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${CERES_LIBRARIES} gtsam)

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${PROJECT_NAME}_utilities ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Range Image Projection Node
add_executable(${PROJECT_NAME}_imageProjectionNode src/ImageProjectionNode.cpp)
add_dependencies(${PROJECT_NAME}_imageProjectionNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjectionNode ${PROJECT_NAME}_utilities ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${PROJECT_NAME}_utilities ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association Node
add_executable(${PROJECT_NAME}_featureExtractionNode src/featureExtractionNode.cpp)
add_dependencies(${PROJECT_NAME}_featureExtractionNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtractionNode ${PROJECT_NAME}_utilities ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${PROJECT_NAME}_utilities Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam)

# Mapping Optimization Node
add_executable(${PROJECT_NAME}_mapOptmizationNode src/MapOptimizationNode.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmizationNode ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmizationNode PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmizationNode ${PROJECT_NAME}_utilities Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${PROJECT_NAME}_utilities Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# IMU Preintegration Node
add_executable(${PROJECT_NAME}_imuPreintegrationNode src/ImuPreintegrationNode.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegrationNode ${PROJECT_NAME}_utilities Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# Transform Fusion
add_executable(${PROJECT_NAME}_transformFusion src/transformFusion.cpp)
target_link_libraries(${PROJECT_NAME}_transformFusion ${PROJECT_NAME}_utilities Boost::timer ${catkin_LIBRARIES})

# Transform Fusion Node
add_executable(${PROJECT_NAME}_transformFusionNode src/TransformFusionNode.cpp)
target_link_libraries(${PROJECT_NAME}_transformFusionNode ${PROJECT_NAME}_utilities Boost::timer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

