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

#include "loam_feature_localization/loam_feature_localization.hpp"

#include "cv_bridge/cv_bridge.h"
#include "loam_feature_localization/utils.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

namespace loam_feature_localization
{
LoamFeatureLocalization::LoamFeatureLocalization(const rclcpp::NodeOptions & options)
: Node("loam_feature_localization", options)
{
  this->declare_parameter("imu_topic", "");
  this->declare_parameter("odom_topic", "");
  this->declare_parameter("point_cloud_topic", "");
  this->declare_parameter("lidar_min_range", 1.0);
  this->declare_parameter("lidar_max_range", 120.0);
  this->declare_parameter("N_SCAN", 120.0);
  this->declare_parameter("Horizon_SCAN", 120.0);
  this->declare_parameter("odometry_surface_leaf_size", 120.0);
  this->declare_parameter("edge_threshold", 120.0);
  this->declare_parameter("surface_threshold", 120.0);

  imu_topic_ = this->get_parameter("imu_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  point_cloud_topic_ = this->get_parameter("point_cloud_topic").as_string();
  lidar_min_range_ = this->get_parameter("lidar_min_range").as_double();
  lidar_max_range_ = this->get_parameter("lidar_max_range").as_double();
  N_SCAN_ = this->get_parameter("N_SCAN").as_double();
  Horizon_SCAN_ = this->get_parameter("Horizon_SCAN").as_double();
  odometry_surface_leaf_size_ = this->get_parameter("odometry_surface_leaf_size").as_double();
  edge_threshold_ = this->get_parameter("edge_threshold").as_double();
  surface_threshold_ = this->get_parameter("surface_threshold").as_double();

  //  image_projection = std::make_shared<loam_feature_localization::ImageProjection>(2.0, 120.0,
  //  32, 2000);
  image_projection = std::make_shared<loam_feature_localization::ImageProjection>(
    lidar_min_range_, lidar_max_range_, N_SCAN_, Horizon_SCAN_);
  image_projection->allocateMemory();

  feature_extraction = std::make_shared<loam_feature_localization::FeatureExtraction>(
    N_SCAN_, Horizon_SCAN_, odometry_surface_leaf_size_, edge_threshold_, surface_threshold_);

  callbackGroupLidar = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callbackGroupImu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callbackGroupOdom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto lidarOpt = rclcpp::SubscriptionOptions();
  lidarOpt.callback_group = callbackGroupLidar;
  auto imuOpt = rclcpp::SubscriptionOptions();
  imuOpt.callback_group = callbackGroupImu;
  auto odomOpt = rclcpp::SubscriptionOptions();
  odomOpt.callback_group = callbackGroupOdom;

  subImu = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LoamFeatureLocalization::imuHandler, this, std::placeholders::_1), imuOpt);
  subOdom = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LoamFeatureLocalization::odometryHandler, this, std::placeholders::_1), odomOpt);
  subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LoamFeatureLocalization::cloudHandler, this, std::placeholders::_1), lidarOpt);

  pubRangeImage =
    create_publisher<sensor_msgs::msg::Image>("/range_image", rclcpp::SensorDataQoS());
  pubCornerCloud =
    create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_corner", rclcpp::SensorDataQoS());
  pubSurfaceCloud =
    create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_surface", rclcpp::SensorDataQoS());
}

void LoamFeatureLocalization::imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
{
  sensor_msgs::msg::Imu thisImu = image_projection->utils->imuConverter(*imuMsg);

  std::lock_guard<std::mutex> lock1(imuLock);
  imuQueue.push_back(thisImu);
}

void LoamFeatureLocalization::odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
{
  std::lock_guard<std::mutex> lock2(odoLock);
  odomQueue.push_back(*odometryMsg);
}

void LoamFeatureLocalization::cloudHandler(
  const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
  if (!cachePointCloud(laserCloudMsg)) return;

  if (!deskewInfo()) return;

  image_projection->projectPointCloud();  // will be in the image_projection

  publishImage();

  image_projection->cloudExtraction();

  feature_extraction->laserCloudInfoHandler(image_projection->cloudInfo, image_projection->cloudHeader, image_projection->laserCloudIn);
  std::cout << "test msg " << std::endl;
  publishClouds();

  image_projection->resetParameters();

  feature_extraction->freeCloudInfoMemory();
}

void LoamFeatureLocalization::publishImage()
{
  cv::Mat BGR;
  cv::cvtColor(image_projection->HSV, BGR, cv::COLOR_HSV2BGR);

  cv::Mat bgr_resized;
  cv::resize(BGR, bgr_resized, cv::Size(), 1.0, 20.0);

  cv_bridge::CvImage cv_image;
  cv_image.header.frame_id = "map";
  cv_image.header.stamp = this->get_clock()->now();
  cv_image.encoding = "bgr8";
  cv_image.image = bgr_resized;

  sensor_msgs::msg::Image image;
  cv_image.toImageMsg(image);

  pubRangeImage->publish(image);
}

void LoamFeatureLocalization::publishClouds() {

  sensor_msgs::msg::PointCloud2 cornerCloud_;
  sensor_msgs::msg::PointCloud2 surfaceCloud_;

  pcl::toROSMsg(*feature_extraction->cornerCloud, cornerCloud_);
  cornerCloud_.header.stamp = this->get_clock()->now();
  cornerCloud_.header.frame_id = "POS_REF";

  pcl::toROSMsg(*feature_extraction->surfaceCloud, surfaceCloud_);
  surfaceCloud_.header.stamp = this->get_clock()->now();
  surfaceCloud_.header.frame_id = "POS_REF";

  pubCornerCloud->publish(cornerCloud_);
  pubSurfaceCloud->publish(surfaceCloud_);
}

bool LoamFeatureLocalization::cachePointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & laserCloudMsg)
{
  // cache point cloud
  cloudQueue.push_back(*laserCloudMsg);
  if (cloudQueue.size() <= 2) return false;

  // convert cloud
  currentCloudMsg = std::move(cloudQueue.front());
  cloudQueue.pop_front();

  pcl::moveFromROSMsg(currentCloudMsg, *image_projection->laserCloudIn);

  //  if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
  //  {
  //    pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
  //  }
  //  else if (sensor == SensorType::OUSTER)
  //  {
  //    // Convert to Velodyne format
  //    pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
  //    laserCloudIn->points.resize(tmpOusterCloudIn->size());
  //    laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
  //    for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
  //    {
  //      auto &src = tmpOusterCloudIn->points[i];
  //      auto &dst = laserCloudIn->points[i];
  //      dst.x = src.x;
  //      dst.y = src.y;
  //      dst.z = src.z;
  //      dst.intensity = src.intensity;
  //      dst.ring = src.ring;
  //      dst.time = src.t * 1e-9f;
  //    }
  //  }
  //  else
  //  {
  //    RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor type: " << int(sensor));
  //    rclcpp::shutdown();
  //  }

  // get timestamp
  image_projection->cloudHeader = currentCloudMsg.header;
  image_projection->timeScanCur =
    image_projection->utils->stamp2Sec(image_projection->cloudHeader.stamp);
  image_projection->timeScanEnd =
    image_projection->timeScanCur + image_projection->laserCloudIn->points.back().time;

  // remove Nan
  //  std::vector<int> indices;
  //  pcl::removeNaNFromPointCloud(
  //    *image_projection->laserCloudIn, *image_projection->laserCloudIn, indices);

  // check dense flag
  if (!image_projection->laserCloudIn->is_dense) {
    RCLCPP_ERROR(
      get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
    rclcpp::shutdown();
  }

  // check ring channel
  // we will skip the ring check in case of velodyne - as we calculate the ring value downstream
  // (line 572)
  if (image_projection->ringFlag == 0) {
    image_projection->ringFlag = -1;
    for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i) {
      if (currentCloudMsg.fields[i].name == "ring") {
        image_projection->ringFlag = 1;
        break;
      }
    }
    if (image_projection->ringFlag == -1) {
      image_projection->ringFlag = 2;
      //      if (sensor == SensorType::VELODYNE) {
      //        ringFlag = 2;
      //      } else {
      //        RCLCPP_ERROR(get_logger(), "Point cloud ring channel not available, please configure
      //        your point cloud data!"); rclcpp::shutdown();
      //      }
    }
  }

  // check point time
  if (image_projection->deskewFlag == 0) {
    image_projection->deskewFlag = -1;
    for (auto & field : currentCloudMsg.fields) {
      if (field.name == "timestamp" || field.name == "t") {
        image_projection->deskewFlag = 1;
        break;
      }
    }
    if (image_projection->deskewFlag == -1)
      RCLCPP_WARN(
        get_logger(),
        "Point cloud timestamp not available, deskew function disabled, system will drift "
        "significantly!");
  }

  return true;
}

bool LoamFeatureLocalization::deskewInfo()
{
  std::lock_guard<std::mutex> lock1(imuLock);
  std::lock_guard<std::mutex> lock2(odoLock);

  // make sure IMU data available for the scan
  if (
    imuQueue.empty() ||
    image_projection->utils->stamp2Sec(imuQueue.front().header.stamp) >
      image_projection->timeScanCur ||
    image_projection->utils->stamp2Sec(imuQueue.back().header.stamp) <
      image_projection->timeScanEnd) {
    RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");
    return false;
  }

  imuDeskewInfo();

  //  odomDeskewInfo();

  return true;
}

void LoamFeatureLocalization::imuDeskewInfo()
{
  image_projection->cloudInfo.imu_available = false;

  while (!imuQueue.empty()) {
    if (
      image_projection->utils->stamp2Sec(imuQueue.front().header.stamp) <
      image_projection->timeScanCur - 0.01)
      imuQueue.pop_front();
    else
      break;
  }

  if (imuQueue.empty()) return;

  image_projection->imuPointerCur = 0;

  for (int i = 0; i < (int)imuQueue.size(); ++i) {
    sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
    double currentImuTime = image_projection->utils->stamp2Sec(thisImuMsg.header.stamp);

    // get roll, pitch, and yaw estimation for this scan
    if (currentImuTime <= image_projection->timeScanCur)
      image_projection->utils->imuRPY2rosRPY(
        &thisImuMsg, &image_projection->cloudInfo.imu_roll_init,
        &image_projection->cloudInfo.imu_pitch_init, &image_projection->cloudInfo.imu_yaw_init);
    if (currentImuTime > image_projection->timeScanEnd + 0.01) break;

    if (image_projection->imuPointerCur == 0) {
      image_projection->imuRotX[0] = 0;
      image_projection->imuRotY[0] = 0;
      image_projection->imuRotZ[0] = 0;
      image_projection->imuTime[0] = currentImuTime;
      ++image_projection->imuPointerCur;
      continue;
    }

    // get angular velocity
    double angular_x, angular_y, angular_z;
    image_projection->utils->imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

    // integrate rotation
    double timeDiff =
      currentImuTime - image_projection->imuTime[image_projection->imuPointerCur - 1];
    image_projection->imuRotX[image_projection->imuPointerCur] =
      image_projection->imuRotX[image_projection->imuPointerCur - 1] + angular_x * timeDiff;
    image_projection->imuRotY[image_projection->imuPointerCur] =
      image_projection->imuRotY[image_projection->imuPointerCur - 1] + angular_y * timeDiff;
    image_projection->imuRotZ[image_projection->imuPointerCur] =
      image_projection->imuRotZ[image_projection->imuPointerCur - 1] + angular_z * timeDiff;
    image_projection->imuTime[image_projection->imuPointerCur] = currentImuTime;
    ++image_projection->imuPointerCur;
  }

  --image_projection->imuPointerCur;

  if (image_projection->imuPointerCur <= 0) return;

  image_projection->cloudInfo.imu_available = true;
}

void LoamFeatureLocalization::odomDeskewInfo()
{
  image_projection->cloudInfo.odom_available = false;

  while (!odomQueue.empty()) {
    if (
      image_projection->utils->stamp2Sec(odomQueue.front().header.stamp) <
      image_projection->timeScanCur - 0.01)
      odomQueue.pop_front();
    else
      break;
  }

  if (odomQueue.empty()) return;

  if (
    image_projection->utils->stamp2Sec(odomQueue.front().header.stamp) >
    image_projection->timeScanCur)
    return;

  // get start odometry at the beinning of the scan
  nav_msgs::msg::Odometry startOdomMsg;

  for (int i = 0; i < (int)odomQueue.size(); ++i) {
    startOdomMsg = odomQueue[i];

    if (
      image_projection->utils->stamp2Sec(startOdomMsg.header.stamp) < image_projection->timeScanCur)
      continue;
    else
      break;
  }

  tf2::Quaternion orientation;
  tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  // Initial guess used in mapOptimization
  image_projection->cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
  image_projection->cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
  image_projection->cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
  image_projection->cloudInfo.initial_guess_roll = roll;
  image_projection->cloudInfo.initial_guess_pitch = pitch;
  image_projection->cloudInfo.initial_guess_yaw = yaw;

  image_projection->cloudInfo.odom_available = true;

  // get end odometry at the end of the scan
  image_projection->odomDeskewFlag = false;

  if (
    image_projection->utils->stamp2Sec(odomQueue.back().header.stamp) <
    image_projection->timeScanEnd)
    return;

  nav_msgs::msg::Odometry endOdomMsg;

  for (int i = 0; i < (int)odomQueue.size(); ++i) {
    endOdomMsg = odomQueue[i];

    if (image_projection->utils->stamp2Sec(endOdomMsg.header.stamp) < image_projection->timeScanEnd)
      continue;
    else
      break;
  }

  if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
    return;

  Eigen::Affine3f transBegin = pcl::getTransformation(
    startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y,
    startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

  tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  Eigen::Affine3f transEnd = pcl::getTransformation(
    endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y,
    endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

  Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

  float rollIncre, pitchIncre, yawIncre;
  pcl::getTranslationAndEulerAngles(
    transBt, image_projection->odomIncreX, image_projection->odomIncreY,
    image_projection->odomIncreZ, rollIncre, pitchIncre, yawIncre);

  image_projection->odomDeskewFlag = true;
}

}  // namespace loam_feature_localization


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(loam_feature_localization::LoamFeatureLocalization)
