cmake_minimum_required(VERSION 3.14)
project(loam_feature_localization)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common io registration)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/loam_feature_localization/loam_feature_localization.hpp
  include/loam_feature_localization/image_projection.hpp
  include/loam_feature_localization/feature_extraction.hpp
  include/loam_feature_localization/utils.hpp
  src/loam_feature_localization.cpp
  src/image_projection.cpp
  src/feature_extraction.cpp
  src/utils.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(loam_feature_localization
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "LoamFeatureLocalization"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

#if(BUILD_TESTING)
#
#endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
