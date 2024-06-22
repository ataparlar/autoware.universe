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

namespace loam_feature_localization
{
LoamFeatureLocalization::LoamFeatureLocalization(const rclcpp::NodeOptions & options)
: Node("loam_feature_localization")
{



}
}  // namespace loam_feature_localization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(loam_feature_localization::LoamFeatureLocalization)
