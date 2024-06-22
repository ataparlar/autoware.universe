cmake_minimum_required(VERSION 3.14)
project(loam_feature_localization)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common io registration)
include_directories(${PCL_INCLUDE_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/loam_feature_localization.cpp
  src/image_projection.cpp
  src/feature_extraction.cpp
  src/utils.cpp
  include/loam_feature_localization/loam_feature_localization.hpp
  include/loam_feature_localization/image_projection.hpp
  include/loam_feature_localization/feature_extraction.hpp
  include/loam_feature_localization/utils.hpp
)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(loam_feature_localization ${PCL_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "LoamFeatureLocalization"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

#if(BUILD_TESTING)
#
#endif()

#ament_auto_package(
#  INSTALL_TO_SHARE
#  launch
#  config
#)
