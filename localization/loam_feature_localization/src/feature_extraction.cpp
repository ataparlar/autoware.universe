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

#include "loam_feature_localization/feature_extraction.hpp"

namespace loam_feature_localization
{

FeatureExtraction::FeatureExtraction(
  int N_SCAN, int Horizon_SCAN, int odometrySurfLeafSize, double edgeThreshold,
  double surfThreshold)
{
  N_SCAN_ = N_SCAN;
  Horizon_SCAN_ = Horizon_SCAN;
  odometrySurfLeafSize_ = odometrySurfLeafSize;
  edgeThreshold_ = edgeThreshold;
  surfThreshold_ = surfThreshold;

  initializationValue();
}

void FeatureExtraction::initializationValue()
{
  cloudSmoothness.resize(N_SCAN_ * Horizon_SCAN_);

  downSizeFilter.setLeafSize(odometrySurfLeafSize_, odometrySurfLeafSize_, odometrySurfLeafSize_);

  extractedCloud.reset(new pcl::PointCloud<PointType>());
  cornerCloud.reset(new pcl::PointCloud<PointType>());
  surfaceCloud.reset(new pcl::PointCloud<PointType>());

  cloudCurvature = new float[N_SCAN_ * Horizon_SCAN_];
  cloudNeighborPicked = new int[N_SCAN_ * Horizon_SCAN_];
  cloudLabel = new int[N_SCAN_ * Horizon_SCAN_];
}

void FeatureExtraction::laserCloudInfoHandler(
  const Utils::CloudInfo & msgIn, const std_msgs::msg::Header & cloudHeaderInp,
  const pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn)
{
  cloudInfo = msgIn;             // new cloud info
  cloudHeader = cloudHeaderInp;  // new cloud header
  //  pcl::fromROSMsg(msgIn.cloud_deskewed, *extractedCloud); // new cloud for extraction

  extractedCloud->resize(laserCloudIn->size());
  for (size_t i = 0; i < laserCloudIn->size(); i++) {
    extractedCloud->at(i).x = laserCloudIn->at(i).x;
    extractedCloud->at(i).y = laserCloudIn->at(i).y;
    extractedCloud->at(i).z = laserCloudIn->at(i).z;
    extractedCloud->at(i).intensity = laserCloudIn->at(i).intensity;
  }

  calculateSmoothness();

  markOccludedPoints();

  extractFeatures();

//  publishFeatureCloud();
}

void FeatureExtraction::calculateSmoothness()
{
  int cloudSize = extractedCloud->points.size();
  std::cout << "cloudSize: " << cloudSize << std::endl;
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange =
      cloudInfo.point_range[i - 5] + cloudInfo.point_range[i - 4] + cloudInfo.point_range[i - 3] +
      cloudInfo.point_range[i - 2] + cloudInfo.point_range[i - 1] - cloudInfo.point_range[i] * 10 +
      cloudInfo.point_range[i + 1] + cloudInfo.point_range[i + 2] + cloudInfo.point_range[i + 3] +
      cloudInfo.point_range[i + 4] + cloudInfo.point_range[i + 5];

    cloudCurvature[i] = diffRange * diffRange;  // diffX * diffX + diffY * diffY + diffZ * diffZ;

    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
    // cloudSmoothness for sorting
    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void FeatureExtraction::markOccludedPoints()
{
  int cloudSize = extractedCloud->points.size();
  // mark occluded points and parallel beam points
  for (int i = 5; i < cloudSize - 6; ++i) {
    // occluded points
    float depth1 = cloudInfo.point_range[i];
    float depth2 = cloudInfo.point_range[i + 1];
    int columnDiff = std::abs(int(cloudInfo.point_col_index[i + 1] - cloudInfo.point_col_index[i]));
    if (columnDiff < 10) {
      // 10 pixel diff in range image
      if (depth1 - depth2 > 0.3) {
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
      }
    }
    // parallel beam
    float diff1 = std::abs(float(cloudInfo.point_range[i - 1] - cloudInfo.point_range[i]));
    float diff2 = std::abs(float(cloudInfo.point_range[i + 1] - cloudInfo.point_range[i]));

    if (diff1 > 0.02 * cloudInfo.point_range[i] && diff2 > 0.02 * cloudInfo.point_range[i])
      cloudNeighborPicked[i] = 1;
  }
}

void FeatureExtraction::extractFeatures()
{
  cornerCloud->clear();
  surfaceCloud->clear();

  pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

  //        std::cout << "cloudSmoothness.size(): " << cloudSmoothness.size() << std::endl;

  for (int i = 0; i < N_SCAN_; i++) {
    surfaceCloudScan->clear();

    for (int j = 0; j < 6; j++) {
      int sp = (cloudInfo.start_ring_index[i] * (6 - j) + cloudInfo.end_ring_index[i] * j) / 6;
      int ep =
        (cloudInfo.start_ring_index[i] * (5 - j) + cloudInfo.end_ring_index[i] * (j + 1)) / 6 - 1;

      if (sp >= ep) continue;

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold_) {
          largestPickedNum++;
          if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerCloud->push_back(extractedCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            int columnDiff = std::abs(
              int(cloudInfo.point_col_index[ind + l] - cloudInfo.point_col_index[ind + l - 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff = std::abs(
              int(cloudInfo.point_col_index[ind + l] - cloudInfo.point_col_index[ind + l + 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold_) {
          cloudLabel[ind] = -1;
          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++) {
            int columnDiff = std::abs(
              int(cloudInfo.point_col_index[ind + l] - cloudInfo.point_col_index[ind + l - 1]));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff = std::abs(
              int(cloudInfo.point_col_index[ind + l] - cloudInfo.point_col_index[ind + l + 1]));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfaceCloudScan->push_back(extractedCloud->points[k]);
        }
      }
    }

    surfaceCloudScanDS->clear();
    downSizeFilter.setInputCloud(surfaceCloudScan);
    downSizeFilter.filter(*surfaceCloudScanDS);

    *surfaceCloud += *surfaceCloudScanDS;
  }
}

void FeatureExtraction::freeCloudInfoMemory()
{
  cloudInfo.start_ring_index.clear();
  cloudInfo.end_ring_index.clear();
  cloudInfo.point_col_index.clear();
  cloudInfo.point_range.clear();
}

// void FeatureExtraction::publishFeatureCloud()
//{
//   // free cloud info memory
//   freeCloudInfoMemory();
//   // save newly extracted features
//   cloudInfo.cloud_corner = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp,
//   lidarFrame); cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud,
//   cloudHeader.stamp, lidarFrame);
//   // publish to mapOptimization
//   pubLaserCloudInfo->publish(cloudInfo);
// }

}  // namespace loam_feature_localization
