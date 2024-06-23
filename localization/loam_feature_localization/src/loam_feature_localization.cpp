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

#include "loam_feature_localization/utils.hpp"

#include <pcl/common/impl/eigen.hpp>

#include <pcl/filters/filter.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

namespace loam_feature_localization
{
LoamFeatureLocalization::LoamFeatureLocalization(const rclcpp::NodeOptions & options)
: Node("loam_feature_localization", options)
{
  this->declare_parameter("imu_topic", "");
  this->declare_parameter("odom_topic", "");
  this->declare_parameter("point_cloud_topic", "");

  imu_topic_ = this->get_parameter("imu_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  point_cloud_topic_ = this->get_parameter("point_cloud_topic").as_string();

  image_projection = std::make_shared<loam_feature_localization::ImageProjection>();

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
    odom_topic_ + "_incremental", rclcpp::SensorDataQoS(),
    std::bind(&LoamFeatureLocalization::odometryHandler, this, std::placeholders::_1), odomOpt);
  subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LoamFeatureLocalization::cloudHandler, this, std::placeholders::_1), lidarOpt);
}

void LoamFeatureLocalization::allocateMemory()
{
  image_projection->laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
  image_projection->tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
  image_projection->fullCloud.reset(new pcl::PointCloud<PointType>());
  image_projection->extractedCloud.reset(new pcl::PointCloud<PointType>());

  image_projection->fullCloud->points.resize(N_SCAN * Horizon_SCAN);

  image_projection->cloudInfo.start_ring_index.assign(N_SCAN, 0);
  image_projection->cloudInfo.end_ring_index.assign(N_SCAN, 0);

  image_projection->cloudInfo.point_col_index.assign(N_SCAN * Horizon_SCAN, 0);
  image_projection->cloudInfo.point_range.assign(N_SCAN * Horizon_SCAN, 0);

  resetParameters();
}

void LoamFeatureLocalization::resetParameters()
{
  image_projection->laserCloudIn->clear();
  image_projection->extractedCloud->clear();
  // reset range matrix for range image projection
  image_projection->rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

  image_projection->imuPointerCur = 0;
  image_projection->firstPointFlag = true;
  image_projection->odomDeskewFlag = false;

  for (int i = 0; i < queueLength; ++i) {
    imuTime[i] = 0;
    imuRotX[i] = 0;
    imuRotY[i] = 0;
    imuRotZ[i] = 0;
  }
}

void LoamFeatureLocalization::imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
{
  sensor_msgs::msg::Imu thisImu = Utils::imuConverter(*imuMsg);

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

  //  projectPointCloud();  // will be in the image_projection

  //  cloudExtraction();

  //  publishClouds();

  resetParameters();
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
  image_projection->timeScanCur = Utils::stamp2Sec(image_projection->cloudHeader.stamp);
  image_projection->timeScanEnd =
    image_projection->timeScanCur + image_projection->laserCloudIn->points.back().time;

  // remove Nan
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(
    *image_projection->laserCloudIn, *image_projection->laserCloudIn, indices);

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
    Utils::stamp2Sec(imuQueue.front().header.stamp) > image_projection->timeScanCur ||
    Utils::stamp2Sec(imuQueue.back().header.stamp) < image_projection->timeScanEnd) {
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
    if (Utils::stamp2Sec(imuQueue.front().header.stamp) < image_projection->timeScanCur - 0.01)
      imuQueue.pop_front();
    else
      break;
  }

  if (imuQueue.empty()) return;

  image_projection->imuPointerCur = 0;

  for (int i = 0; i < (int)imuQueue.size(); ++i) {
    sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
    double currentImuTime = Utils::stamp2Sec(thisImuMsg.header.stamp);

    // get roll, pitch, and yaw estimation for this scan
    if (currentImuTime <= image_projection->timeScanCur)
      Utils::imuRPY2rosRPY(
        &thisImuMsg, &image_projection->cloudInfo.imu_roll_init,
        &image_projection->cloudInfo.imu_pitch_init, &image_projection->cloudInfo.imu_yaw_init);
    if (currentImuTime > image_projection->timeScanEnd + 0.01) break;

    if (image_projection->imuPointerCur == 0) {
      imuRotX[0] = 0;
      imuRotY[0] = 0;
      imuRotZ[0] = 0;
      imuTime[0] = currentImuTime;
      ++image_projection->imuPointerCur;
      continue;
    }

    // get angular velocity
    double angular_x, angular_y, angular_z;
    Utils::imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

    // integrate rotation
    double timeDiff = currentImuTime - imuTime[image_projection->imuPointerCur - 1];
    imuRotX[image_projection->imuPointerCur] =
      imuRotX[image_projection->imuPointerCur - 1] + angular_x * timeDiff;
    imuRotY[image_projection->imuPointerCur] =
      imuRotY[image_projection->imuPointerCur - 1] + angular_y * timeDiff;
    imuRotZ[image_projection->imuPointerCur] =
      imuRotZ[image_projection->imuPointerCur - 1] + angular_z * timeDiff;
    imuTime[image_projection->imuPointerCur] = currentImuTime;
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
    if (Utils::stamp2Sec(odomQueue.front().header.stamp) < image_projection->timeScanCur - 0.01)
      odomQueue.pop_front();
    else
      break;
  }

  if (odomQueue.empty()) return;

  if (Utils::stamp2Sec(odomQueue.front().header.stamp) > image_projection->timeScanCur) return;

  // get start odometry at the beinning of the scan
  nav_msgs::msg::Odometry startOdomMsg;

  for (int i = 0; i < (int)odomQueue.size(); ++i) {
    startOdomMsg = odomQueue[i];

    if (Utils::stamp2Sec(startOdomMsg.header.stamp) < image_projection->timeScanCur)
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

  if (Utils::stamp2Sec(odomQueue.back().header.stamp) < image_projection->timeScanEnd) return;

  nav_msgs::msg::Odometry endOdomMsg;

  for (int i = 0; i < (int)odomQueue.size(); ++i) {
    endOdomMsg = odomQueue[i];

    if (Utils::stamp2Sec(endOdomMsg.header.stamp) < image_projection->timeScanEnd)
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
