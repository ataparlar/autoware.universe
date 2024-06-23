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

#ifndef LOAM_FEATURE_LOCALIZATION__LOAM_FEATURE_LOCALIZATION_HPP_
#define LOAM_FEATURE_LOCALIZATION__LOAM_FEATURE_LOCALIZATION_HPP_



#include "utils.hpp"
#include "image_projection.hpp"
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
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
const int queueLength = 2000;


class LoamFeatureLocalization : public rclcpp::Node
//class LoamFeatureLocalization
{
public:
  using SharedPtr = std::shared_ptr<LoamFeatureLocalization>;
  using ConstSharedPtr = const std::shared_ptr<LoamFeatureLocalization>;

  explicit LoamFeatureLocalization(const rclcpp::NodeOptions & options);

private:

  // parameters
  std::string point_cloud_topic_;
  std::string imu_topic_;
  std::string odom_topic_;

  ImageProjection::ImageProjection::SharedPtr image_projection;

  std::mutex imuLock;
  std::mutex odoLock;

  double *imuTime = new double[queueLength];
  double *imuRotX = new double[queueLength];
  double *imuRotY = new double[queueLength];
  double *imuRotZ = new double[queueLength];


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
  rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubRangeImage;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
  //  rclcpp::Publisher<Utils::CloudInfo>::SharedPtr pubLaserCloudInfo;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
  rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
  std::deque<sensor_msgs::msg::Imu> imuQueue;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
  rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
  std::deque<nav_msgs::msg::Odometry> odomQueue;

  std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
  sensor_msgs::msg::PointCloud2 currentCloudMsg;



  void allocateMemory();
  void resetParameters();
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg);
  void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg);
  void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
  bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& laserCloudMsg);
  bool deskewInfo();
  void imuDeskewInfo();
  void odomDeskewInfo();

  int N_SCAN = 1800;
  int Horizon_SCAN = 16;


};
}




#endif  // LOAM_FEATURE_LOCALIZATION__LOAM_FEATURE_LOCALIZATION_HPP_
