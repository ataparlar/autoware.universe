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

#ifndef LOAM_FEATURE_LOCALIZATION__UTILS_HPP_
#define LOAM_FEATURE_LOCALIZATION__UTILS_HPP_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/imu.hpp"

#include <pcl/register_point_struct.h>
#include <pcl/impl/point_types.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

struct VelodynePointXYZIRT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                  (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  uint16_t reflectivity;
  uint8_t ring;
  uint16_t noise;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
                                      (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

using PointXYZIRT = VelodynePointXYZIRT;



namespace loam_feature_localization
{

typedef pcl::PointXYZI PointType;

class Utils
{
public:
  using SharedPtr = std::shared_ptr<Utils>;
  using ConstSharedPtr = const SharedPtr;

  explicit Utils();

  std::string byte_hex_to_string(uint8_t byte_hex);
  std::string bytes_hexes_to_string(const std::vector<uint8_t> & bytes_hexes);
  sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in);
//  static std::vector<std::string> string_to_vec_split_by(const std::string & input, char splitter);
//  static Eigen::Matrix3d ned2enu_converter_for_matrices(const Eigen::Matrix3d & matrix3d);

  Eigen::Matrix3d extRot = Eigen::Matrix3d::Zero(4, 4);
  Eigen::Matrix3d extRPY = Eigen::Matrix3d::Zero(4, 4);
  Eigen::Quaterniond extQRPY = Eigen::Quaterniond::Identity();


  template <typename T>
  T deg_to_rad(T deg)
  {
    constexpr double multiplier = M_PI / 180.0;
    return static_cast<T>(deg * multiplier);
  }

  template<typename T>
  double stamp2Sec(const T& stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

  rclcpp::Time get_time()
  {
    return rclcpp::Clock().now();
  }

//  template<typename T>
  void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, double *angular_x, double *angular_y, double *angular_z);

//  template<typename T>
  void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, double *rosRoll, double *rosPitch, double *rosYaw);

  float pointDistance(PointType p);

  struct CloudInfo
  {
    bool imu_available;
    bool odom_available;
    double imu_roll_init;
    double imu_pitch_init;
    double imu_yaw_init;
    double initial_guess_x;
    double initial_guess_y;
    double initial_guess_z;
    double initial_guess_roll;
    double initial_guess_pitch;
    double initial_guess_yaw;
    std::vector<float> point_range;
    std::vector<uint32_t> start_ring_index;
    std::vector<uint32_t> point_col_index;
    std::vector<int32_t> end_ring_index;
  };
};

}

#endif  // LOAM_FEATURE_LOCALIZATION__UTILS_HPP_
