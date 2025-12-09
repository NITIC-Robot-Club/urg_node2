#pragma once
#include <chrono>
#include <csignal>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "urg_sensor.h"
#include "urg_utils.h"

using namespace std::chrono_literals;

namespace urg_node2_nl
{
struct UrgNode2SensorDatas
{
  std::vector<long> distance_;
  std::vector<unsigned short> intensity_;
  std::string header_frame_id_;
  double topic_angle_min_;
  double topic_angle_max_;
  double topic_angle_increment_;
  double topic_time_increment_;
  double topic_range_min_;
  double topic_range_max_;
  double scan_period_;
};

struct UrgNode2States
{
  bool use_multiecho_{false};
  bool use_intensity_{false};
  bool is_connected_{false};
  bool is_measurement_started_{false};
  bool is_stable_{false};
  rclcpp::Duration user_latency_{0ns};
  rclcpp::Duration system_latency_{0ns};
  std::string device_status_;
  std::string sensor_status_;
  std::string product_name_;
  std::string firmware_version_;
  std::string device_id_;
  double hardware_clock_;
  long int last_hardware_time_stamp_;
  double hardware_clock_adj_;
  const double adj_alpha_{0.01};
  int adj_count_;
  int reconnect_count_;
  int error_count_;
  int total_error_count_;
  int first_step_;
  int last_step_;
  urg_measurement_type_t measurement_type_;
};

struct UrgNode2Config
{
  // ---- ros2 parameters ----
  std::string ip_address_;
  int ip_port_;
  std::string serial_port_;
  int serial_baud_;
  std::string frame_id_;
  bool calibrate_time_;
  bool synchronize_time_;
  bool publish_intensity_;
  bool publish_multiecho_;
  int error_limit_;
  double error_reset_period_;
  double diagnostics_tolerance_;
  double diagnostics_window_time_;
  double time_offset_;
  double angle_min_;
  double angle_max_;
  int skip_;
  int cluster_;
};

class UrgNode2NL : public rclcpp::Node
{
public:
  explicit UrgNode2NL(const rclcpp::NodeOptions & options);
  ~UrgNode2NL(void);

private:
  void init_urgnode2_parameters(void);
  void set_scan_parameters(void);

  bool connect(void);
  void disconnect(void);
  bool reconnect(void);

  void update_sensor_states(void);
  bool is_intensity_supported(void);
  bool is_multiecho_supported(void);
  rclcpp::Duration get_angular_time_offset(void);

  bool create_laserscan_message(sensor_msgs::msg::LaserScan::UniquePtr & msg);
  void convert_laserscan_to_pointcloud2(
    const sensor_msgs::msg::LaserScan::UniquePtr & laserscan_msg,
    sensor_msgs::msg::PointCloud2::UniquePtr & pointcloud2_msg);

  void scan_thread_func(void);
  void start_thread(void);
  void close_thread(void);

  // URG Sensor manage struct from urg_library
  urg_t urg_;

  UrgNode2Config config_;
  UrgNode2States states_;
  UrgNode2SensorDatas sensor_data_;

  std::thread scan_thread_;
  std::atomic_bool close_thread_flag_;

  bool publish_laserscan_;
  bool publish_pointcloud2_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> laserscan_pub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud2_pub_;
  static constexpr int URG_NODE2_NL_MAX_DATA_SIZE{5000};
};
}  // namespace urg_node2_nl
