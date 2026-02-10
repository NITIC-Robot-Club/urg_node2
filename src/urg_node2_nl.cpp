#include "urg_node2_nl.hpp"

#include <rcl/time.h>

#include "urg_connection.h"
#include "urg_sensor.h"
#include "urg_utils.h"

namespace urg_node2_nl
{
UrgNode2NL::UrgNode2NL(const rclcpp::NodeOptions & options) : Node("urg_node2_nl", options)
{
  RCLCPP_INFO(this->get_logger(), "Started UrgNode2NL. Initializing...");
  // set SIGPIPE signal ignore setting
  // if sensor's pwr is not turned on, connection trial causes SIGPIPE signal
  std::signal(SIGPIPE, SIG_IGN);

  // declare parameters
  init_urgnode2_parameters();

  if (states_.use_multiecho_) {
    // TODO: implement multiecho func
  } else {
    if (publish_laserscan_) {
      laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(20));
    }
    if (publish_pointcloud2_) {
      pointcloud2_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud2", rclcpp::QoS(20));
    }
    if (!publish_laserscan_ && !publish_pointcloud2_) {
      RCLCPP_FATAL(
        this->get_logger(), "No publisher is enabled. Please enable at least one publisher.");
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Initialize complete. Starting scan thread...");
  start_thread();
}
UrgNode2NL::~UrgNode2NL(void)
{
  RCLCPP_INFO(this->get_logger(), "Attempt to Cleaning up this node...");

  close_thread();

  RCLCPP_INFO(this->get_logger(), "stopping publisher...");
  if (states_.use_multiecho_) {
    // TODO: implement multiecho func
  } else {
    if (publish_laserscan_) {
      laserscan_pub_.reset();
    }

    if (publish_pointcloud2_) {
      pointcloud2_pub_.reset();
    }
  }
}

void UrgNode2NL::init_urgnode2_parameters(void)
{
  config_.ip_address_ = declare_parameter<std::string>("ip_address", "");
  config_.ip_port_ = declare_parameter<int>("ip_port", 10940);
  config_.serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  config_.serial_baud_ = declare_parameter<int>("serial_baud", 115200);
  config_.frame_id_ = declare_parameter<std::string>("frame_id", "laser");
  config_.calibrate_time_ = declare_parameter<bool>("calibrate_time", false);
  config_.synchronize_time_ = declare_parameter<bool>("synchronize_time", false);
  config_.publish_intensity_ = declare_parameter<bool>("publish_intensity", false);
  config_.publish_multiecho_ = declare_parameter<bool>("publish_multiecho", false);
  config_.error_limit_ = declare_parameter<int>("error_limit", 4);
  config_.error_reset_period_ = declare_parameter<double>("error_reset_period", 5.0),
  config_.diagnostics_tolerance_ = declare_parameter<double>("diagnostics_tolerance", 0.05);
  config_.diagnostics_window_time_ = declare_parameter<double>("diagnostics_window_time", 5.0);
  config_.time_offset_ = declare_parameter<double>("time_offset", 0.0);
  config_.angle_min_ = declare_parameter<double>("angle_min", -M_PI);
  config_.angle_max_ = declare_parameter<double>("angle_max", M_PI);
  config_.skip_ = declare_parameter<int>("skip", 0);
  config_.cluster_ = declare_parameter<int>("cluster", 1);

  publish_laserscan_ = declare_parameter<bool>("publish_laserscan", true);
  publish_pointcloud2_ = declare_parameter<bool>("publish_pointcloud2", true);
  reverse_ = declare_parameter<bool>("reverse", false);

  // 範囲チェック
  config_.angle_min_ = (config_.angle_min_ < -M_PI)
                         ? -M_PI
                         : ((config_.angle_min_ > M_PI) ? M_PI : config_.angle_min_);
  config_.angle_max_ = (config_.angle_max_ < -M_PI)
                         ? -M_PI
                         : ((config_.angle_max_ > M_PI) ? M_PI : config_.angle_max_);
  config_.skip_ = (config_.skip_ < 0) ? 0 : ((config_.skip_ > 9) ? 9 : config_.skip_);
  config_.cluster_ = (config_.cluster_ < 1) ? 1 : ((config_.cluster_ > 99) ? 99 : config_.cluster_);

  // 内部変数初期化
  states_.is_connected_ = false;
  states_.is_measurement_started_ = false;
  states_.is_stable_ = false;
  states_.user_latency_ = rclcpp::Duration::from_seconds(config_.time_offset_);

  // メッセージヘッダのframe_id設定
  sensor_data_.header_frame_id_ =
    (config_.frame_id_.find_first_not_of('/') == std::string::npos)
      ? ""
      : config_.frame_id_.substr(config_.frame_id_.find_first_not_of('/'));

  states_.hardware_clock_ = 0.0;
  states_.last_hardware_time_stamp_ = 0;
  states_.hardware_clock_adj_ = 0;
  states_.adj_count_ = 0;
}

void UrgNode2NL::set_scan_parameters(void)
{
  int urg_data_size = urg_max_data_size(&urg_);
  if (urg_data_size > URG_NODE2_NL_MAX_DATA_SIZE) {
    urg_data_size = URG_NODE2_NL_MAX_DATA_SIZE;
  }

  sensor_data_.distance_.resize(urg_data_size * URG_MAX_ECHO);
  sensor_data_.intensity_.resize(urg_data_size * URG_MAX_ECHO);

  states_.first_step_ = urg_rad2step(&urg_, config_.angle_min_);
  states_.last_step_ = urg_rad2step(&urg_, config_.angle_max_);

  if (states_.last_step_ < states_.first_step_) {
    std::swap(states_.first_step_, states_.last_step_);
  }
  if (states_.last_step_ == states_.first_step_) {
    int min_step, max_step;
    urg_step_min_max(&urg_, &min_step, &max_step);
    if (states_.first_step_ == min_step) {
      states_.last_step_ = states_.first_step_ + 1;
    } else {
      states_.first_step_ = states_.last_step_ - 1;
    }
  }

  urg_set_scanning_parameter(&urg_, states_.first_step_, states_.last_step_, config_.cluster_);

  // update config value
  config_.angle_min_ = urg_step2rad(&urg_, states_.first_step_);
  config_.angle_max_ = urg_step2rad(&urg_, states_.last_step_);

  sensor_data_.topic_angle_min_ = config_.angle_min_;
  sensor_data_.topic_angle_max_ = config_.angle_max_;
  sensor_data_.topic_angle_increment_ = config_.cluster_ * urg_step2rad(&urg_, 1);

  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  sensor_data_.topic_time_increment_ =
    config_.cluster_ *
    ((urg_step2rad(&urg_, max_step) - urg_step2rad(&urg_, min_step)) / (2.0 * M_PI)) *
    sensor_data_.scan_period_ / static_cast<double>(max_step - min_step);

  long min_dis;
  long max_dis;
  urg_distance_min_max(&urg_, &min_dis, &max_dis);
  sensor_data_.topic_range_min_ = static_cast<double>(min_dis) / 1000.0;
  sensor_data_.topic_range_max_ = static_cast<double>(max_dis) / 1000.0;
}

bool UrgNode2NL::connect(void)
{
  if (!config_.ip_address_.empty()) {
    // Ethernet connection
    int result = urg_open(&urg_, URG_ETHERNET, config_.ip_address_.c_str(), config_.ip_port_);
    if (result < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not open network Hokuyo 2D LiDAR\n%s:%d\n%s",
        config_.ip_address_.c_str(), config_.ip_port_, urg_error(&urg_));
      return false;
    }
  } else {
    // Serial(USB) connection
    int result = urg_open(&urg_, URG_SERIAL, config_.serial_port_.c_str(), config_.serial_baud_);
    if (result < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not open serial port Hokuyo 2D LiDAR\n%s:%d\n%s",
        config_.serial_port_.c_str(), config_.serial_baud_, urg_error(&urg_));
      return false;
    }
  }

  states_.is_connected_ = true;

  std::stringstream ss;
  ss << "Connected to a ";
  if (!config_.ip_address_.empty()) {
    ss << "network ";
  } else {
    ss << "serial ";
  }
  ss << "device with ";
  if (config_.publish_multiecho_) {
    ss << "multiecho ";
  } else {
    ss << "single ";
  }
  if (config_.publish_intensity_) {
    ss << "and intensity ";
  }
  ss << "scan. Hardware ID: " << urg_sensor_serial_id(&urg_);
  RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

  // get sensor's hardware data
  states_.device_status_ = urg_sensor_status(&urg_);
  states_.sensor_status_ = urg_sensor_state(&urg_);
  states_.product_name_ = urg_sensor_product_type(&urg_);
  states_.firmware_version_ = urg_sensor_firmware_version(&urg_);
  states_.device_id_ = urg_sensor_serial_id(&urg_);
  sensor_data_.scan_period_ = 1.e-6 * static_cast<double>(urg_scan_usec(&urg_));

  // check config and sensor support to publish intensity data
  if (config_.publish_intensity_) {
    states_.use_intensity_ = is_intensity_supported();

    // if publish_intensity config is true, but connected sensor is not supported intensity data
    if (!states_.use_intensity_) {
      RCLCPP_WARN(
        get_logger(),
        "parameter 'publish_intensity' is true, but this device does not support intensity mode. "
        "disable intensity mode.");
    }
  }
  // check config and sensor support to publish intensity data
  if (config_.publish_multiecho_) {
    states_.use_multiecho_ = is_multiecho_supported();

    // if publish_multiecho config is true, but connected sensor is not supported multiecho function
    if (!states_.use_multiecho_) {
      RCLCPP_WARN(
        get_logger(),
        "parameter 'publish_multiecho' is true, but this device does not support multiecho scan "
        "mode. switch single scan mode.");
    }
  }

  // setting
  if (states_.use_intensity_ && states_.use_multiecho_) {
    states_.measurement_type_ = URG_MULTIECHO_INTENSITY;
  } else if (states_.use_intensity_) {
    states_.measurement_type_ = URG_DISTANCE_INTENSITY;
  } else if (states_.use_multiecho_) {
    states_.measurement_type_ = URG_MULTIECHO;
  } else {
    states_.measurement_type_ = URG_DISTANCE;
  }

  return true;
}

void UrgNode2NL::disconnect(void)
{
  if (states_.is_connected_) {
    urg_close(&urg_);
    states_.is_connected_ = false;
  }
}

bool UrgNode2NL::reconnect(void)
{
  disconnect();
  return connect();
}

bool UrgNode2NL::is_intensity_supported(void)
{
  if (states_.is_measurement_started_) {
    return false;
  }

  if (urg_start_measurement(&urg_, URG_DISTANCE_INTENSITY, 0, 0, 0) < 0) {
    RCLCPP_WARN(this->get_logger(), "Could not start Hokuyo measurement\n%s", urg_error(&urg_));
    return false;
  }
  states_.is_measurement_started_ = true;
  int ret = urg_get_distance_intensity(
    &urg_, &sensor_data_.distance_[0], &sensor_data_.intensity_[0], NULL);
  if (ret <= 0) {
    // intensity output is not supported
    return false;
  }
  urg_stop_measurement(&urg_);
  states_.is_measurement_started_ = false;
  return true;
}

bool UrgNode2NL::is_multiecho_supported(void)
{
  // 計測は停止している必要がある
  if (states_.is_measurement_started_) {
    return false;
  }

  if (urg_start_measurement(&urg_, URG_MULTIECHO_INTENSITY, 0, 0, 0) < 0) {
    RCLCPP_WARN(get_logger(), "Could not start Hokuyo measurement\n%s", urg_error(&urg_));
    return false;
  }
  states_.is_measurement_started_ = true;
  int ret = urg_get_multiecho(&urg_, &sensor_data_.distance_[0], NULL);
  if (ret <= 0) {
    // マルチエコー出力非対応
    return false;
  }

  urg_stop_measurement(&urg_);
  states_.is_measurement_started_ = false;
  return true;
}

bool UrgNode2NL::create_laserscan_message(sensor_msgs::msg::LaserScan::UniquePtr & msg)
{
  msg->header.frame_id = sensor_data_.header_frame_id_;
  msg->angle_min = sensor_data_.topic_angle_min_;
  msg->angle_max = sensor_data_.topic_angle_max_;
  msg->angle_increment = sensor_data_.topic_angle_increment_;
  msg->scan_time = sensor_data_.scan_period_;
  msg->time_increment = sensor_data_.topic_time_increment_;
  msg->range_min = sensor_data_.topic_range_min_;
  msg->range_max = sensor_data_.topic_range_max_;

  int num_beams = 0;
  long time_stamp = 0;

  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  rclcpp::Time system_time_stamp = system_clock.now();
  if (states_.use_intensity_) {
    num_beams = urg_get_distance_intensity(
      &urg_, &sensor_data_.distance_[0], &sensor_data_.intensity_[0], &time_stamp);
  } else {
    num_beams = urg_get_distance(&urg_, &sensor_data_.distance_[0], &time_stamp);
  }
  if (num_beams <= 0) {
    return false;
  }

  if (config_.synchronize_time_) {
    // TODO: time synchronize func
  }
  msg->header.stamp =
    system_time_stamp + states_.system_latency_ + states_.user_latency_ + get_angular_time_offset();

  msg->ranges.resize(num_beams);
  if (states_.use_intensity_) {
    msg->intensities.resize(num_beams);
  }

  for (int i = 0; i < num_beams; i++) {
    
    int data_index;
    if(reverse_) {
      data_index = num_beams - i - 1;
    } else {
      data_index = i;
    }

    if (sensor_data_.distance_[i] != 0) {
      msg->ranges[data_index] = static_cast<float>(sensor_data_.distance_[i]) / 1000.0;
      if (states_.use_intensity_) {
        msg->intensities[data_index] = sensor_data_.intensity_[i];
      }
    } else {
      msg->ranges[data_index] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
  }

  return true;
}

void UrgNode2NL::convert_laserscan_to_pointcloud2(
  const sensor_msgs::msg::LaserScan::UniquePtr & laserscan_msg,
  sensor_msgs::msg::PointCloud2::UniquePtr & pointcloud2_msg)
{
  pointcloud2_msg->header = laserscan_msg->header;

  sensor_msgs::PointCloud2Modifier pointcloud2_modifier(*pointcloud2_msg);
  pointcloud2_modifier.setPointCloud2FieldsByString(1, "xyz");
  pointcloud2_modifier.resize(laserscan_msg->ranges.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud2_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud2_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud2_msg, "z");

  for (size_t i = 0; i < laserscan_msg->ranges.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
    float r = laserscan_msg->ranges[i];
    if (std::isnan(r) || r < laserscan_msg->range_min || r > laserscan_msg->range_max) {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
    float angle = laserscan_msg->angle_min + i * laserscan_msg->angle_increment;
    *iter_x = r * std::cos(angle);
    *iter_y = r * std::sin(angle);
    *iter_z = 0.0;
  }
}

void UrgNode2NL::update_sensor_states(void)
{
  // update sensor states
  states_.device_status_ = urg_sensor_status(&urg_);
  states_.sensor_status_ = urg_sensor_state(&urg_);
  states_.is_stable_ = urg_is_stable(&urg_);
}

void UrgNode2NL::scan_thread_func(void)
{
  states_.reconnect_count_ = 0;
  while (!close_thread_flag_.load()) {
    if (!states_.is_connected_) {
      if (!connect()) {
        rclcpp::sleep_for(500ms);
        continue;
      }
    }

    set_scan_parameters();

    // TODO: add calibrate mode

    update_sensor_states();

    // start scan
    int ret = urg_start_measurement(&urg_, states_.measurement_type_, 0, config_.skip_, 0);
    if (ret < 0) {
      RCLCPP_WARN(get_logger(), "Could not start Hokuyo measurement\n%s", urg_error(&urg_));

      reconnect();
      states_.reconnect_count_++;

      continue;
    }

    states_.is_measurement_started_ = true;
    states_.error_count_ = 0;

    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    rclcpp::Time prev_time = system_clock.now();
    while (!close_thread_flag_.load()) {
      if (states_.use_multiecho_) {
        // TODO: implement multiecho func
        continue;
      } else {
        sensor_msgs::msg::LaserScan::UniquePtr msg =
          std::make_unique<sensor_msgs::msg::LaserScan>();
        if (create_laserscan_message(msg)) {
          if (publish_pointcloud2_) {
            sensor_msgs::msg::PointCloud2::UniquePtr pc2_msg =
              std::make_unique<sensor_msgs::msg::PointCloud2>();
            convert_laserscan_to_pointcloud2(msg, pc2_msg);
            pointcloud2_pub_->publish(std::move(pc2_msg));
          }
          if (publish_laserscan_) {
            laserscan_pub_->publish(std::move(msg));
          }
        } else {
          RCLCPP_WARN(get_logger(), "could not get single echo scan.");
          states_.error_count_++;
          states_.total_error_count_++;
          update_sensor_states();
        }
      }

      // If the error count exceeds the threshold, attempt reconnection.
      if (states_.error_count_ > config_.error_limit_) {
        RCLCPP_ERROR(get_logger(), "Error count exceeded limit, reconnecting.");
        // reconnect sensor
        states_.is_measurement_started_ = false;
        reconnect();
        states_.reconnect_count_++;
        break;
      } else {
        // If the error remains below the threshold for error reset period, error_count reset.
        rclcpp::Time current_time = system_clock.now();
        rclcpp::Duration period = current_time - prev_time;
        if (period.seconds() >= config_.error_reset_period_) {
          prev_time = current_time;
          states_.error_count_ = 0;
        }
      }
    }
  }

  disconnect();
}

void UrgNode2NL::start_thread(void)
{
  // clear flag for thread finish
  close_thread_flag_.store(false);
  scan_thread_ = std::thread(std::bind(&UrgNode2NL::scan_thread_func, this));
}

void UrgNode2NL::close_thread(void)
{
  // set thread finish
  close_thread_flag_.store(true);

  // join thread
  if (scan_thread_.joinable()) {
    scan_thread_.join();
  }
}

// calc the time offset from start position
rclcpp::Duration UrgNode2NL::get_angular_time_offset(void)
{
  // The timestamp for the scan data is for the position directly behind.
  // So, the time required to reach the starting angle position must be added as an offset.
  double circle_fraction = 0.0;
  if (states_.first_step_ == 0 && states_.last_step_ == 0) {
    int min_step, max_step;
    urg_step_min_max(&urg_, &min_step, &max_step);
    // zero position is sensor's front, so M_PI is added
    circle_fraction = (urg_step2rad(&urg_, min_step) + M_PI) / (2.0 * M_PI);
  } else {
    circle_fraction = (urg_step2rad(&urg_, states_.first_step_) + M_PI) / (2.0 * M_PI);
  }
  return rclcpp::Duration::from_seconds(circle_fraction * sensor_data_.scan_period_);
}

}  // namespace urg_node2_nl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg_node2_nl::UrgNode2NL)
