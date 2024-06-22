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



#include <rclcpp/rclcpp.hpp>

namespace loam_feature_localization
{
class LoamFeatureLocalization : public rclcpp::Node
//class LoamFeatureLocalization
{
public:
  using SharedPtr = std::shared_ptr<LoamFeatureLocalization>;
  using ConstSharedPtr = const std::shared_ptr<LoamFeatureLocalization>;

  explicit LoamFeatureLocalization(const rclcpp::NodeOptions & options);

private:

};
}




#endif  // LOAM_FEATURE_LOCALIZATION__LOAM_FEATURE_LOCALIZATION_HPP_
