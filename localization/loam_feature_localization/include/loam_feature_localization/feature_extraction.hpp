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

#ifndef LOAM_FEATURE_LOCALIZATION__FEATURE_EXTRACTION_HPP_
#define LOAM_FEATURE_LOCALIZATION__FEATURE_EXTRACTION_HPP_

#include <loam_feature_localization/image_projection.hpp>

#include <pcl/filters/voxel_grid.h>

struct smoothness_t
{
  float value;
  size_t ind;
};

struct by_value
{
  bool operator()(smoothness_t const & left, smoothness_t const & right)
  {
    return left.value < right.value;
  }
};

namespace loam_feature_localization
{
class FeatureExtraction
{
public:
  using SharedPtr = std::shared_ptr<FeatureExtraction>;
  using ConstSharedPtr = const SharedPtr;

  explicit FeatureExtraction(
    int N_SCAN, int Horizon_SCAN, int odometrySurfLeafSize, double edgeThreshold,
    double surfThreshold);

  pcl::PointCloud<PointType>::Ptr extractedCloud;
  pcl::PointCloud<PointType>::Ptr cornerCloud;
  pcl::PointCloud<PointType>::Ptr surfaceCloud;

  pcl::VoxelGrid<PointType> downSizeFilter;

  Utils::CloudInfo cloudInfo;
  std_msgs::msg::Header cloudHeader;

  std::vector<smoothness_t> cloudSmoothness;
  float * cloudCurvature;
  int * cloudNeighborPicked;
  int * cloudLabel;

  int N_SCAN_;
  int Horizon_SCAN_;

  int odometrySurfLeafSize_;
  double edgeThreshold_;
  double surfThreshold_;

  void initializationValue();
  void laserCloudInfoHandler(
    const Utils::CloudInfo & msgIn, const std_msgs::msg::Header & cloudHeaderInp,
    const pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn);
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();
  void freeCloudInfoMemory();
  //  void publishFeatureCloud();

private:
};
}  // namespace loam_feature_localization

#endif  // LOAM_FEATURE_LOCALIZATION__FEATURE_EXTRACTION_HPP_
