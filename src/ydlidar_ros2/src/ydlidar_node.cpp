/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <limits>
#include <signal.h>
#include <cmath>

#define ROS2Verision "1.0.1"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  std::string str_optvalue = "/dev/ydlidar";
  node->declare_parameter("port", str_optvalue);
  node->get_parameter("port", str_optvalue);
  ///lidar port
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());
  ///ignore array
  str_optvalue = "";
  node->declare_parameter("ignore_array", str_optvalue);
  node->get_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id = "laser_frame";
  node->declare_parameter("frame_id", frame_id);
  node->get_parameter("frame_id", frame_id);

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 230400;
  node->declare_parameter("baudrate", optval);
  node->get_parameter("baudrate", optval);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
  node->declare_parameter("lidar_type", optval);
  node->get_parameter("lidar_type", optval);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter("device_type", optval);
  node->get_parameter("device_type", optval);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 9;
  node->declare_parameter("sample_rate", optval);
  node->get_parameter("sample_rate", optval);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  node->declare_parameter("abnormal_check_count", optval);
  node->get_parameter("abnormal_check_count", optval);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  /// Intenstiy bit count
  optval = 8;
  node->declare_parameter("intensity_bit", optval);
  node->get_parameter("intensity_bit", optval);
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));
     
  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  node->declare_parameter("fixed_resolution", b_optvalue);
  node->get_parameter("fixed_resolution", b_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = true;
  node->declare_parameter("reversion", b_optvalue);
  node->get_parameter("reversion", b_optvalue);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = true;
  node->declare_parameter("inverted", b_optvalue);
  node->get_parameter("inverted", b_optvalue);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  node->declare_parameter("auto_reconnect", b_optvalue);
  node->get_parameter("auto_reconnect", b_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  node->declare_parameter("isSingleChannel", b_optvalue);
  node->get_parameter("isSingleChannel", b_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  // b_optvalue = false;
  // node->declare_parameter("intensity", b_optvalue);
  // node->get_parameter("intensity", b_optvalue);
  // laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  laser.setAutoIntensity(true);
  /// Motor DTR
  b_optvalue = false;
  node->declare_parameter("support_motor_dtr", b_optvalue);
  node->get_parameter("support_motor_dtr", b_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  node->declare_parameter("angle_max", f_optvalue);
  node->get_parameter("angle_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  node->declare_parameter("angle_min", f_optvalue);
  node->get_parameter("angle_min", f_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 64.f;
  node->declare_parameter("range_max", f_optvalue);
  node->get_parameter("range_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  node->declare_parameter("range_min", f_optvalue);
  node->get_parameter("range_min", f_optvalue);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 10.f;
  node->declare_parameter("frequency", f_optvalue);
  node->get_parameter("frequency", f_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = false;
  node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  // Some lidar firmware timestamps are in a clock domain that is about 56 ms
  // behind the ROS clock.  Using that stamp directly can make TF lookups fail
  // when the scan arrives near the edge of the TF buffer.  Use the ROS receive
  // time by default; hardware stamping remains available for synchronized
  // systems that explicitly need it.
  bool use_hardware_timestamp = false;
  node->declare_parameter("use_hardware_timestamp", use_hardware_timestamp);
  node->get_parameter("use_hardware_timestamp", use_hardware_timestamp);

  // /point_cloud is a convenience topic for debugging.  Nav2, slam_toolbox and
  // RViz all consume /scan, so rebuilding a second point cloud every frame only
  // wastes CPU and DDS bandwidth on Jetson.  Enable it explicitly when needed.
  bool publish_point_cloud = false;
  node->declare_parameter("publish_point_cloud", publish_point_cloud);
  node->get_parameter("publish_point_cloud", publish_point_cloud);

  // Do not spam the console when the WiFi link is down.  The SDK reconnects in
  // its own thread; a throttled message keeps the failure visible without
  // flooding the terminal (and starving the rest of the launch).
  // RCLCPP_*_THROTTLE expects the period in milliseconds.
  const int64_t scan_error_report_period_ms = 2000;


  bool ret = false;
  while (rclcpp::ok() && !ret)
  {
    ret = laser.initialize() && laser.turnOn();
    if (!ret)
    {
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(),
                            scan_error_report_period_ms,
                            "Lidar not ready: %s; retrying in 2s",
                            laser.DescribeError());
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
  }
  
  // LaserScan is a latest-sample sensor stream.  Best-effort prevents a slow
  // RViz/network subscriber from applying back-pressure to the driver and
  // growing stale data in the DDS queue.  Consumers should use SensorDataQoS.
  auto scan_qos = rclcpp::SensorDataQoS();
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", scan_qos);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pc_pub;
  if (publish_point_cloud)
  {
    pc_pub = node->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", scan_qos);
    RCLCPP_INFO(node->get_logger(), "point_cloud publishing enabled");
  }
  
  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    LaserScan scan;//

    if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      if (use_hardware_timestamp) {
        scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
        scan_msg->header.stamp.nanosec =
          scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      } else {
        const auto ros_stamp = node->now();
        scan_msg->header.stamp = ros_stamp;
      }
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      
      if (!std::isfinite(scan.config.angle_increment) ||
          !std::isfinite(scan.config.min_angle) ||
          !std::isfinite(scan.config.max_angle) ||
          !(scan.config.angle_increment > 0.0f) ||
          !(scan.config.max_angle > scan.config.min_angle))
      {
        RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(),
                              scan_error_report_period_ms,
                              "Dropped scan with invalid angle configuration");
        rclcpp::spin_some(node);
        loop_rate.sleep();
        continue;
      }
      const double sample_count =
        (static_cast<double>(scan.config.max_angle) - scan.config.min_angle) /
        scan.config.angle_increment + 1.0;
      if (!std::isfinite(sample_count) || sample_count < 1.0 ||
          sample_count > 200000.0)
      {
        RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(),
                              scan_error_report_period_ms,
                              "Dropped scan with invalid sample count: %.1f",
                              sample_count);
        rclcpp::spin_some(node);
        loop_rate.sleep();
        continue;
      }
      const int size = static_cast<int>(sample_count);
      // resize() zero-fills missing rays, which downstream consumers read as a
      // real obstacle at 0 m.  Fill unwritten cells with the configured
      // "no return" value instead.
      const float no_return = invalid_range_is_inf
        ? std::numeric_limits<float>::infinity()
        : std::numeric_limits<float>::quiet_NaN();
      scan_msg->ranges.assign(size, no_return);
      scan_msg->intensities.assign(size, 0.0f);

      std::shared_ptr<sensor_msgs::msg::PointCloud> pc_msg;
      if (pc_pub)
      {
        pc_msg = std::make_shared<sensor_msgs::msg::PointCloud>();
        pc_msg->header = scan_msg->header;
        pc_msg->channels.resize(2);
        pc_msg->channels[0].name = "intensities";
        pc_msg->channels[1].name = "stamps";
      }

      for (size_t i = 0; i < scan.points.size(); ++i) {
        const auto &point_data = scan.points[i];
        if (!std::isfinite(point_data.angle) ||
            !std::isfinite(point_data.range) ||
            point_data.range < scan.config.min_range ||
            point_data.range > scan.config.max_range) {
          continue;
        }
        const double sample_index = std::ceil(
          (static_cast<double>(point_data.angle) - scan.config.min_angle) /
          scan.config.angle_increment);
        if (sample_index >= 0.0 && sample_index < size) {
          const int index = static_cast<int>(sample_index);
          scan_msg->ranges[index] = point_data.range;
          scan_msg->intensities[index] = point_data.intensity;
        }

        if (pc_msg) {
          geometry_msgs::msg::Point32 point;
          point.x = point_data.range * std::cos(point_data.angle);
          point.y = point_data.range * std::sin(point_data.angle);
          point.z = 0.0;
          pc_msg->points.push_back(point);
          pc_msg->channels[0].values.push_back(point_data.intensity);
          pc_msg->channels[1].values.push_back(i * scan.config.time_increment);
        }
      }

      laser_pub->publish(*scan_msg);
      if (pc_pub && pc_msg)
        pc_pub->publish(*pc_msg);

    } else {
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(),
                            scan_error_report_period_ms,
                            "Failed to get scan (waiting for lidar or reconnecting)");
    }
    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
