//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"

#include <inttypes.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdint.h>

#include "include/ros2_headers.h"

#include "driver_node.h"
#include "lds_lidar.h"

namespace livox_ros {

/** Lidar Data Distribute Control--------------------------------------------*/
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
           double frq, std::string &frame_id, bool merge_pointcloud)
    : transfer_format_(format),
      use_multi_topic_(multi_topic),
      data_src_(data_src),
      output_type_(output_type),
      publish_frq_(frq),
      frame_id_(frame_id),
      merge_pointcloud_(merge_pointcloud){
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
}

Lddc::~Lddc() {
  PrepareExit();
  std::cout << "lddc destroyed" << std::endl;
}

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::DistributePointCloudData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributePointCloudData is RequestExit" << std::endl;
    return;
  }

  lds_->pcd_semaphore_.Wait();

  std::vector<PointCloud2> pcd2_buffer;
  std::vector<CustomMsg> custom_buffer;

  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar, pcd2_buffer, custom_buffer);
  }

  if (merge_pointcloud_ && kPointCloud2Msg == transfer_format_ && !pcd2_buffer.empty()) {
    PointCloud2 merged_cloud = MergeMessages(pcd2_buffer);
    if (!merged_pcd_pub_) {
      std::string topic_name("livox/merged_cloud");
      uint32_t queue_size = 256;

      auto base_pub = CreatePublisher(transfer_format_, topic_name, queue_size);
      merged_pcd_pub_ = std::dynamic_pointer_cast<rclcpp::Publisher<PointCloud2>>(base_pub);
    }
    merged_pcd_pub_->publish(merged_cloud);
  } else if (kLivoxCustomMsg == transfer_format_ && !custom_buffer.empty()) {
    CustomMsg merged_msg = MergeMessages(custom_buffer);
    if (!merged_custom_pub_) {
      std::string topic_name("livox/merged_custom_cloud");
      uint32_t queue_size = 256;

      auto base_pub = CreatePublisher(transfer_format_, topic_name, queue_size);
      merged_custom_pub_ = std::dynamic_pointer_cast<rclcpp::Publisher<CustomMsg>>(base_pub);
    }
    merged_custom_pub_->publish(merged_msg);
  }
}

void Lddc::DistributeImuData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributeImuData is RequestExit" << std::endl;
    return;
  }

  lds_->imu_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarImuDataQueue *p_queue = &lidar->imu_data;
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) {
      continue;
    }
    PollingLidarImuData(lidar_id, lidar);
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar,
                                      std::vector<PointCloud2>& pcd2_buffer, std::vector<CustomMsg>& custom_buffer) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }

  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    if (kPointCloud2Msg == transfer_format_) {
      PublishPointcloud2(p_queue, index, pcd2_buffer);
    } else if (kLivoxCustomMsg == transfer_format_) {
      PublishCustomPointcloud(p_queue, index, custom_buffer);
    } else if (kPclPxyziMsg == transfer_format_) {
      PublishPclMsg(p_queue, index);
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar) {
  LidarImuDataQueue& p_queue = lidar->imu_data;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishImuData(p_queue, index);
  }
}

void Lddc::PrepareExit(void) {
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

void Lddc::PublishPointcloud2(LidarDataQueue *queue, uint8_t index, std::vector<PointCloud2>& pcd2_buffer) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud2 failed, the pkg points is empty.\n");
      continue;
    }

    PointCloud2 cloud;
    uint64_t timestamp = 0;
    InitPointcloud2Msg(pkg, cloud, timestamp);

    if (merge_pointcloud_) pcd2_buffer.push_back(cloud);
    else PublishPointcloud2Data(index, timestamp, cloud);
  }
}

void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index, std::vector<CustomMsg>& custom_buffer) {
  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish custom point cloud failed, the pkg points is empty.\n");
      continue;
    }

    CustomMsg livox_msg;
    InitCustomMsg(livox_msg, pkg, index);
    FillPointsToCustomMsg(livox_msg, pkg);

    if (merge_pointcloud_) custom_buffer.push_back(livox_msg);
    else PublishCustomPointData(livox_msg, index);
  }
}

/* for pcl::pxyzi */
void Lddc::PublishPclMsg(LidarDataQueue *queue, uint8_t index) {
  static bool first_log = true;
  if (first_log) {
    std::cout << "error: message type 'pcl::PointCloud' is NOT supported in ROS2, "
              << "please modify the 'xfer_format' field in the launch file"
              << std::endl;
  }
  first_log = false;
  return;

  while(!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud failed, the pkg points is empty.\n");
      continue;
    }

    PointCloud cloud;
    uint64_t timestamp = 0;
    InitPclMsg(pkg, cloud, timestamp);
    FillPointsToPclMsg(pkg, cloud);
    PublishPclData(index, timestamp, cloud);
  }
  return;
}

void Lddc::InitPointcloud2MsgHeader(PointCloud2& cloud) {
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(7);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = PointField::UINT8;
  cloud.fields[6].offset = 18;
  cloud.fields[6].name = "timestamp";
  cloud.fields[6].count = 1;
  cloud.fields[6].datatype = PointField::FLOAT64;
  cloud.point_step = sizeof(LivoxPointXyzrtlt);
}

void Lddc::InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2& cloud, uint64_t& timestamp) {
  InitPointcloud2MsgHeader(cloud);

  cloud.point_step = sizeof(LivoxPointXyzrtlt);

  cloud.width = pkg.points_num;
  cloud.row_step = cloud.width * cloud.point_step;

  cloud.is_bigendian = false;
  cloud.is_dense     = true;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }

  cloud.header.stamp = rclcpp::Time(timestamp);

  std::vector<LivoxPointXyzrtlt> points;
  for (size_t i = 0; i < pkg.points_num; ++i) {
    LivoxPointXyzrtlt point;
    point.x = pkg.points[i].x;
    point.y = pkg.points[i].y;
    point.z = pkg.points[i].z;
    point.reflectivity = pkg.points[i].intensity;
    point.tag = pkg.points[i].tag;
    point.line = pkg.points[i].line;
    point.timestamp = static_cast<double>(pkg.points[i].offset_time);
    points.push_back(std::move(point));
  }
  cloud.data.resize(pkg.points_num * sizeof(LivoxPointXyzrtlt));
  memcpy(cloud.data.data(), points.data(), pkg.points_num * sizeof(LivoxPointXyzrtlt));
}

void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp, const PointCloud2& cloud) {
  Publisher<PointCloud2>::SharedPtr publisher_ptr =
    std::dynamic_pointer_cast<Publisher<PointCloud2>>(GetCurrentPublisher(index));

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud);
  }
}

void Lddc::InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index) {
  livox_msg.header.frame_id.assign(frame_id_);

  uint64_t timestamp = 0;
  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }
  livox_msg.timebase = timestamp;
  livox_msg.header.stamp = rclcpp::Time(timestamp);

  livox_msg.point_num = pkg.points_num;
  if (lds_->lidars_[index].lidar_type == kLivoxLidarType) {
    livox_msg.lidar_id = lds_->lidars_[index].handle;
  } else {
    printf("Init custom msg lidar id failed, the index:%u.\n", index);
    livox_msg.lidar_id = 0;
  }
}

void Lddc::FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg) {
  uint32_t points_num = pkg.points_num;
  const std::vector<PointXyzlt>& points = pkg.points;
  for (uint32_t i = 0; i < points_num; ++i) {
    CustomPoint point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.reflectivity = points[i].intensity;
    point.tag = points[i].tag;
    point.line = points[i].line;
    point.offset_time = static_cast<uint32_t>(points[i].offset_time - pkg.base_time);

    livox_msg.points.push_back(std::move(point));
  }
}

void Lddc::PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index) {
  Publisher<CustomMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<CustomMsg>>(GetCurrentPublisher(index));

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(livox_msg);
  }
}

void Lddc::InitPclMsg(const StoragePacket& pkg, PointCloud& cloud, uint64_t& timestamp) {
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, please check code logic" << std::endl;
  return;
}

void Lddc::FillPointsToPclMsg(const StoragePacket& pkg, PointCloud& pcl_msg) {
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, please check code logic" << std::endl;
  return;
}

void Lddc::PublishPclData(const uint8_t index, const uint64_t timestamp, const PointCloud& cloud) {
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, please check code logic" << std::endl;
  return;
}

void Lddc::InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp, uint8_t index) {
  imu_msg.header.frame_id = frame_id_;

  timestamp = imu_data.time_stamp;
  imu_msg.header.stamp = rclcpp::Time(timestamp);  // to ros time stamp

  const ExtParameter& ext = lds_->lidars_[index].livox_config.extrinsic_param;
  const double roll  = ext.roll  * M_PI / 180.0;
  const double pitch = ext.pitch * M_PI / 180.0;
  const double yaw   = ext.yaw   * M_PI / 180.0;

  const double cr = cos(roll),  sr = sin(roll);
  const double cp = cos(pitch), sp = sin(pitch);
  const double cy = cos(yaw),   sy = sin(yaw);

  // ZYX rotation matrix (same convention as pub_handler SetLidarsExtParam)
  const double r00 = cp * cy,  r01 = sr * sp * cy - cr * sy,  r02 = cr * sp * cy + sr * sy;
  const double r10 = cp * sy,  r11 = sr * sp * sy + cr * cy,  r12 = cr * sp * sy - sr * cy;
  const double r20 = -sp,      r21 = sr * cp,                 r22 = cr * cp;

  imu_msg.angular_velocity.x = r00 * imu_data.gyro_x + r01 * imu_data.gyro_y + r02 * imu_data.gyro_z;
  imu_msg.angular_velocity.y = r10 * imu_data.gyro_x + r11 * imu_data.gyro_y + r12 * imu_data.gyro_z;
  imu_msg.angular_velocity.z = r20 * imu_data.gyro_x + r21 * imu_data.gyro_y + r22 * imu_data.gyro_z;

  imu_msg.linear_acceleration.x = 9.81 * (r00 * imu_data.acc_x + r01 * imu_data.acc_y + r02 * imu_data.acc_z);
  imu_msg.linear_acceleration.y = 9.81 * (r10 * imu_data.acc_x + r11 * imu_data.acc_y + r12 * imu_data.acc_z);
  imu_msg.linear_acceleration.z = 9.81 * (r20 * imu_data.acc_x + r21 * imu_data.acc_y + r22 * imu_data.acc_z);
}

void Lddc::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {
  ImuData imu_data;
  if (!imu_data_queue.Pop(imu_data)) {
    //printf("Publish imu data failed, imu data queue pop failed.\n");
    return;
  }

  ImuMsg imu_msg;
  uint64_t timestamp;
  InitImuMsg(imu_data, imu_msg, timestamp, index);

  Publisher<ImuMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<ImuMsg>>(GetCurrentImuPublisher(index));

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(imu_msg);
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::CreatePublisher(uint8_t msg_type,
    std::string &topic_name, uint32_t queue_size) {

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(queue_size);

    if (kPointCloud2Msg == msg_type) {
      DRIVER_INFO(*cur_node_,
          "%s publish use PointCloud2 format", topic_name.c_str());
      return cur_node_->create_publisher<PointCloud2>(topic_name, qos);
    } else if (kLivoxCustomMsg == msg_type) {
      DRIVER_INFO(*cur_node_,
          "%s publish use livox custom format", topic_name.c_str());
      return cur_node_->create_publisher<CustomMsg>(topic_name, qos);
    } else if (kLivoxImuMsg == msg_type)  {
      DRIVER_INFO(*cur_node_,
          "%s publish use imu format", topic_name.c_str());
      return cur_node_->create_publisher<ImuMsg>(topic_name, qos);
    } else {
      PublisherPtr null_publisher(nullptr);
      return null_publisher;
    }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));

      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
          ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_pub_[handle] = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return private_pub_[handle];
  } else {
    if (!global_pub_) {
      std::string topic_name("livox/lidar");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return global_pub_;
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentImuPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_imu_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
          ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_imu_pub_[handle] = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return private_imu_pub_[handle];
  } else {
    if (!global_imu_pub_) {
      std::string topic_name("livox/imu");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_imu_pub_ = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return global_imu_pub_;
  }
}

PointCloud2 Lddc::MergeMessages(const std::vector<PointCloud2>& buffer) {
  if (buffer.empty()) return PointCloud2();

  PointCloud2 merged = buffer[0];
  merged.width = 0;
  merged.row_step = 0;
  merged.data.clear();

  size_t total_data_size = 0;
  for (const auto& cloud : buffer) {
    total_data_size += cloud.data.size();
  }
  merged.data.reserve(total_data_size);

  for (const auto& cloud : buffer) {
    merged.width += cloud.width;
    merged.row_step += cloud.row_step;
    merged.data.insert(merged.data.end(), cloud.data.begin(), cloud.data.end());
  }

  return merged;
}

CustomMsg Lddc::MergeMessages(const std::vector<CustomMsg>& buffer) {
  if (buffer.empty()) return CustomMsg();

  CustomMsg merged = buffer[0];
  merged.point_num = 0;
  merged.points.clear();

  size_t total_points = 0;
  for (const auto& msg : buffer) {
    total_points += msg.points.size();
  }
  merged.points.reserve(total_points);

  for (const auto& msg : buffer) {
    int64_t delta_time_ns = msg.timebase - merged.timebase;

    if (delta_time_ns == 0) {
      merged.points.insert(merged.points.end(), msg.points.begin(), msg.points.end());
    } else {
      for (auto point : msg.points) {
        point.offset_time += delta_time_ns;
        merged.points.push_back(point);
      }
    }

    merged.point_num += msg.point_num;
  }

  return merged;
}

}  // namespace livox_ros
