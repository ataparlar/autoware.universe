// Copyright 2024 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LOAM_FEATURE_LOCALIZATION__IMAGE_PROJECTION_HPP_
#define LOAM_FEATURE_LOCALIZATION__IMAGE_PROJECTION_HPP_

#include "utils.hpp"

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>

#include <deque>
#include <memory>
#include <string>

namespace loam_feature_localization
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;




class ImageProjection
{
public:
  using SharedPtr = std::shared_ptr<ImageProjection>;
  using ConstSharedPtr = const SharedPtr;

  explicit ImageProjection();

  pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
  pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
  pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr extractedCloud;

  Utils::CloudInfo cloudInfo;

  int imuPointerCur;
  bool firstPointFlag;
  bool odomDeskewFlag;

  cv::Mat rangeMat;


  Eigen::Affine3f transStartInverse;


  int ringFlag = 0;
  int deskewFlag;

  float odomIncreX;
  float odomIncreY;
  float odomIncreZ;

  double timeScanCur;
  double timeScanEnd;
  std_msgs::msg::Header cloudHeader;

  std::vector<int> columnIdnCountVec;


private:


};
}







#endif  // LOAM_FEATURE_LOCALIZATION__IMAGE_PROJECTION_HPP_
