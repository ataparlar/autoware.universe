// Copyright 2022 The Autoware Contributors
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

#include "pointcloud_map_loader_node.hpp"

#include <glob.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
bool is_pcd_file(const std::string & p)
{
  if (fs::is_directory(p)) {
    return false;
  }

  const std::string ext = fs::path(p).extension();

  return !(ext != ".pcd" && ext != ".PCD");
}
}  // namespace

PointCloudMapLoaderNode::PointCloudMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_map_loader", options)
{
  const auto pcd_paths_vector =
    get_pcd_paths(declare_parameter<std::vector<std::string>>("pcd_paths_or_directory"));
  const auto & pcd_paths = pcd_paths_vector.at(0);
  const auto & corner_pcd_paths = pcd_paths_vector.at(1);
  const auto & surface_pcd_paths = pcd_paths_vector.at(2);

  std::string pcd_metadata_path, corner_pcd_metadata_path, surface_pcd_metadata_path;
  const auto pcd_metadatas =
    get_pcd_metadata_paths(declare_parameter<std::string>("pcd_metadata_folder"));
  for (const auto & metadata_path : pcd_metadatas) {
    auto filename = metadata_path.substr(metadata_path.find_last_of("/\\") + 1);
    if (filename == "pointcloud_map_metadata.yaml"){
      pcd_metadata_path = metadata_path;
    } else if (filename == "corner_pointcloud_map_metadata.yaml") {
      corner_pcd_metadata_path = metadata_path;
    } else if (filename == "surface_ğpointcloud_map_metadata.yaml") {
      surface_pcd_metadata_path = metadata_path;
    }
  }

  bool enable_whole_load = declare_parameter<bool>("enable_whole_load");
  bool enable_downsample_whole_load = declare_parameter<bool>("enable_downsampled_whole_load");
  bool enable_partial_load = declare_parameter<bool>("enable_partial_load");
  bool enable_selected_load = declare_parameter<bool>("enable_selected_load");

  if (enable_whole_load) {
    std::string publisher_name = "output/pointcloud_map";
    pcd_map_loader_ =
      std::make_unique<PointcloudMapLoaderModule>(this, pcd_paths, publisher_name, false);
  }

  if (enable_downsample_whole_load) {
    std::string publisher_name = "output/debug/downsampled_pointcloud_map";
    downsampled_pcd_map_loader_ =
      std::make_unique<PointcloudMapLoaderModule>(this, pcd_paths, publisher_name, true);
  }

  // Parse the metadata file and get the map of (absolute pcd path, pcd file metadata)
  auto pcd_metadata_dict = get_pcd_metadata(pcd_metadata_path, pcd_paths);
  auto corner_pcd_metadata_dict = get_pcd_metadata(surface_pcd_metadata_path, corner_pcd_paths);
  auto surface_pcd_metadata_dict = get_pcd_metadata(surface_pcd_metadata_path, surface_pcd_paths);

  if (enable_partial_load) {
    partial_map_loader_ = std::make_unique<PartialMapLoaderModule>(this, pcd_metadata_dict);
  }

  differential_map_loader_ = std::make_unique<DifferentialMapLoaderModule>(this, pcd_metadata_dict);
  differential_corner_map_loader_ =
    std::make_unique<DifferentialMapLoaderModule>(this, corner_pcd_metadata_dict);
  differential_surface_map_loader_ =
    std::make_unique<DifferentialMapLoaderModule>(this, surface_pcd_metadata_dict);

  if (enable_selected_load) {
    selected_map_loader_ = std::make_unique<SelectedMapLoaderModule>(this, pcd_metadata_dict);
  }
}

std::map<std::string, PCDFileMetadata> PointCloudMapLoaderNode::get_pcd_metadata(
  const std::string & pcd_metadata_path, const std::vector<std::string> & pcd_paths) const
{
  if (fs::exists(pcd_metadata_path)) {
    std::set<std::string> missing_pcd_names;
    auto pcd_metadata_dict = load_pcd_metadata(pcd_metadata_path);

    pcd_metadata_dict = replace_with_absolute_path(pcd_metadata_dict, pcd_paths, missing_pcd_names);

    // Warning if some segments are missing
    if (!missing_pcd_names.empty()) {
      std::ostringstream oss;

      oss << "The following segment(s) are missing from the input PCDs: ";

      for (const auto & fname : missing_pcd_names) {
        oss << std::endl << fname;
      }

      RCLCPP_ERROR_STREAM(get_logger(), oss.str());
      throw std::runtime_error("Missing PCD segments. Exiting map loader...");
    }

    return pcd_metadata_dict;
  }

  if (pcd_paths.size() == 1) {
    // An exception when using a single file PCD map so that the users do not have to provide
    // a metadata file.
    // Note that this should ideally be avoided and thus eventually be removed by someone, until
    // Autoware users get used to handling the PCD file(s) with metadata.
    RCLCPP_DEBUG_STREAM(get_logger(), "Create PCD metadata, as the pointcloud is a single file.");
    pcl::PointCloud<pcl::PointXYZ> single_pcd;
    const auto & pcd_path = pcd_paths.front();
    if (pcl::io::loadPCDFile(pcd_path, single_pcd) == -1) {
      throw std::runtime_error("PCD load failed: " + pcd_path);
    }
    PCDFileMetadata metadata = {};
    pcl::getMinMax3D(single_pcd, metadata.min, metadata.max);
    return std::map<std::string, PCDFileMetadata>{{pcd_path, metadata}};
  }
  throw std::runtime_error("PCD metadata file not found: " + pcd_metadata_path);
}

std::vector<std::vector<std::string>> PointCloudMapLoaderNode::get_pcd_paths(
  const std::vector<std::string> & pcd_paths_or_directory) const
{
  for (const auto & p : pcd_paths_or_directory) {
    RCLCPP_INFO(get_logger(), "\n\nPCD_PATH: %s\n\n", p.c_str());
  }
  std::vector<std::string> pcd_paths;
  std::vector<std::string> corner_pcd_paths;
  std::vector<std::string> surface_pcd_paths;
  for (const auto & p : pcd_paths_or_directory) {
    if (!fs::exists(p)) {
      RCLCPP_ERROR_STREAM(get_logger(), "invalid path: " << p);
    }

    if (is_pcd_file(p)) {
      pcd_paths.push_back(p);
    }

    if (fs::is_directory(p)) {
      for (const auto & folder1 : fs::directory_iterator(p)) {
        auto folder1_filename = folder1.path().string().substr(folder1.path().string().find_last_of("/\\") + 1);
        RCLCPP_INFO(get_logger(), "\n\nfolder1 name : %s\n\n", folder1_filename.c_str());
        if (folder1_filename == "pointcloud_map.pcd" && fs::is_directory(folder1)) {
          for (const auto & folder2 : fs::directory_iterator(folder1)) {
            auto folder2_filename = folder2.path().string().substr(folder2.path().string().find_last_of("/\\") + 1);
            if (folder2_filename == "full") {
              for (const auto & file : fs::directory_iterator(p)) {
                const auto filename = file.path().string();
                if (is_pcd_file(filename)) {
                  pcd_paths.push_back(filename);
                }
              }
              RCLCPP_INFO(get_logger(), "\n\nDIR NAME IS FULL: \n\n");

            } else if (folder2_filename == "corner") {
              for (const auto & file : fs::directory_iterator(p)) {
                const auto filename = file.path().string();
                if (is_pcd_file(filename)) {
                  corner_pcd_paths.push_back(filename);
                }
              }
              RCLCPP_INFO(get_logger(), "\n\nDIR NAME IS CORNER: \n\n");
            } else if (folder2_filename == "surface") {
              for (const auto & file : fs::directory_iterator(p)) {
                const auto filename = file.path().string();
                if (is_pcd_file(filename)) {
                  surface_pcd_paths.push_back(filename);
                }
              }
              RCLCPP_INFO(get_logger(), "\n\nDIR NAME IS SURFACE: \n\n");
            }
          }
        }
      }
    }
  }
  std::vector<std::vector<std::string>> vector;
  vector.push_back(pcd_paths);
  vector.push_back(corner_pcd_paths);
  vector.push_back(surface_pcd_paths);
  return vector;
}

std::vector<std::string> PointCloudMapLoaderNode::get_pcd_metadata_paths(
  const std::string & pcd_metadata_folder_path) const
{
  std::vector<std::string> pcd_metadata_paths;

  if (!fs::exists(pcd_metadata_folder_path)) {
    RCLCPP_ERROR_STREAM(get_logger(), "invalid path for metadata file folder: " << pcd_metadata_folder_path);
  }

  for (const auto & file : fs::directory_iterator(pcd_metadata_folder_path)) {
    const auto filename = file.path().string();

    const std::string ext = fs::path(filename).extension();
    if (ext == ".yaml") {
      pcd_metadata_paths.push_back(filename);
    }
  }
  return pcd_metadata_paths;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudMapLoaderNode)
