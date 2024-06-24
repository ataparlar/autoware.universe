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

#include "loam_feature_localization/image_projection.hpp"

#include <pcl/common/transforms.h>

namespace loam_feature_localization
{

ImageProjection::ImageProjection()
{

  utils = std::make_shared<loam_feature_localization::Utils>();


  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}


void ImageProjection::allocateMemory()
{
  laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
  tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
  fullCloud.reset(new pcl::PointCloud<PointType>());
  extractedCloud.reset(new pcl::PointCloud<PointType>());

  fullCloud->points.resize(N_SCAN * Horizon_SCAN);

  cloudInfo.start_ring_index.assign(N_SCAN, 0);
  cloudInfo.end_ring_index.assign(N_SCAN, 0);

  cloudInfo.point_col_index.assign(N_SCAN * Horizon_SCAN, 0);
  cloudInfo.point_range.assign(N_SCAN * Horizon_SCAN, 0);

  resetParameters();
}

void ImageProjection::resetParameters()
{
  laserCloudIn->clear();
  extractedCloud->clear();
  // reset range matrix for range image projection
//  rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8SC1, cv::Scalar::all(FLT_MAX));
  rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

  imuPointerCur = 0;
  firstPointFlag = true;
  odomDeskewFlag = false;

  for (int i = 0; i < queueLength; ++i) {
    imuTime[i] = 0;
    imuRotX[i] = 0;
    imuRotY[i] = 0;
    imuRotZ[i] = 0;
  }
}


void ImageProjection::findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
  *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

  int imuPointerFront = 0;
  while (imuPointerFront < imuPointerCur)
  {
    if (pointTime < imuTime[imuPointerFront])
      break;
    ++imuPointerFront;
  }

  if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
  {
    *rotXCur = imuRotX[imuPointerFront];
    *rotYCur = imuRotY[imuPointerFront];
    *rotZCur = imuRotZ[imuPointerFront];
  } else {
    int imuPointerBack = imuPointerFront - 1;
    double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
    double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
    *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
    *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
    *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
  }
}

void ImageProjection::findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
{
  *posXCur = 0; *posYCur = 0; *posZCur = 0;

  // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

   if (cloudInfo.odom_available == false || odomDeskewFlag == false)
       return;

   float ratio = relTime / (timeScanEnd - timeScanCur);

   *posXCur = ratio * odomIncreX;
   *posYCur = ratio * odomIncreY;
   *posZCur = ratio * odomIncreZ;
}

PointType ImageProjection::deskewPoint(PointType *point, double relTime)
{
  if (deskewFlag == -1 || cloudInfo.imu_available == false)
    return *point;

  double pointTime = timeScanCur + relTime;

  float rotXCur, rotYCur, rotZCur;
  findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

  float posXCur, posYCur, posZCur;
  findPosition(relTime, &posXCur, &posYCur, &posZCur);

  if (firstPointFlag == true)
  {
    transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
    firstPointFlag = false;
  }

  // transform points to start
  Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
  Eigen::Affine3f transBt = transStartInverse * transFinal;

  PointType newPoint;
  newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
  newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
  newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
  newPoint.intensity = point->intensity;

  return newPoint;
}


void ImageProjection::projectPointCloud()
{
  HSV = cv::Mat(N_SCAN, Horizon_SCAN, CV_8UC3, cv::Scalar::all(FLT_MAX));


  int cloudSize = laserCloudIn->points.size();
  // range image projection
  for (int i = 0; i < cloudSize; ++i)
  {
    PointType thisPoint;
    thisPoint.x = laserCloudIn->points[i].x;
    thisPoint.y = laserCloudIn->points[i].y;
    thisPoint.z = laserCloudIn->points[i].z;
    thisPoint.intensity = laserCloudIn->points[i].intensity;

    float range = utils->pointDistance(thisPoint);
    if (range < lidarMinRange || range > lidarMaxRange)
      continue;

    int rowIdn = laserCloudIn->points[i].ring;
    // if sensor is a velodyne (ringFlag = 2) calculate rowIdn based on number of scans
    if (ringFlag == 2) {
      float verticalAngle =
        atan2(thisPoint.z,
              sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) *
        180 / M_PI;
      rowIdn = (verticalAngle + (N_SCAN - 1)) / 2.0;
    }

    if (rowIdn < 0 || rowIdn >= N_SCAN)
      continue;

//    if (rowIdn % downsampleRate != 0)
//      continue;

    int columnIdn = -1;
    float horizonAngle;
    static float ang_res_x;

    horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
    ang_res_x = 360.0/float(Horizon_SCAN);
    columnIdn = -round((horizonAngle)/ang_res_x) + Horizon_SCAN/2;
//    columnIdn = round((horizonAngle)/ang_res_x);
    if (columnIdn >= Horizon_SCAN)
      columnIdn -= Horizon_SCAN;


    if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
      continue;

    if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
      continue;

    thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

    rangeMat.at<float>(rowIdn, columnIdn) = range;
//                rangeMat.at<float>(rowIdn, columnIdn) = horizonAngle;

    //            uchar hue = static_cast<uchar>((horizonAngle * 180.0) / 360 + 90);
    uchar hue = static_cast<uchar>((range * 180.0) / 60);
    HSV.at<cv::Vec3b>(rowIdn, columnIdn) = cv::Vec3b(hue, 255.0, 255.0);

    int index = columnIdn + rowIdn * Horizon_SCAN;
    fullCloud->points[index] = thisPoint;
  }


//  cv::Mat BGR;
//  cv::cvtColor(HSV, BGR, cv::COLOR_HSV2BGR);
//
//  cv::Mat bgr_resized;
//  cv::resize(BGR, bgr_resized, cv::Size(), 1.0, 20.0);
//
//  cv_bridge::CvImage cv_image;
//  cv_image.header.frame_id = "map";
//  cv_image.header.stamp = this->get_clock()->now();
//  cv_image.encoding = "bgr8";
//  cv_image.image = bgr_resized;
//
//  sensor_msgs::msg::Image image;
//  cv_image.toImageMsg(image);
//
//  pubRangeImage->publish(image);
}



void ImageProjection::cloudExtraction()
{
  int count = 0;
  // extract segmented cloud for lidar odometry
  for (int i = 0; i < N_SCAN; ++i)
  {
    cloudInfo.start_ring_index[i] = count - 1 + 5;
    for (int j = 0; j < Horizon_SCAN; ++j)
    {
      if (rangeMat.at<float>(i,j) != FLT_MAX)
      {
        // mark the points' column index for marking occlusion later
        cloudInfo.point_col_index[count] = j;
        // save range info
        cloudInfo.point_range[count] = rangeMat.at<float>(i,j);
        // save extracted cloud
        extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
        // size of extracted cloud
        ++count;
      }
    }
    cloudInfo.end_ring_index[i] = count -1 - 5;
  }

}



}