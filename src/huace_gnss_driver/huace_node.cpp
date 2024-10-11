// Eigen include
#include <Eigen/Geometry>
#include <huace_gnss_driver/huace_node.hpp>

/**
 * @file huace_node.cpp
 * @date 22/08/20
 * @brief The heart of the ROSaic driver: The ROS node that represents it
 */

huace_node::HuaceNode::HuaceNode(const rclcpp::NodeOptions & options)
: HuaceNodeBase(options), tfBuffer_(this->get_clock())
{
  param("activate_debug_log", settings_.activate_debug_log, false);
  if (settings_.activate_debug_log) {
    auto ret =
      rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        this->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
      rcutils_reset_error();
    }
  }

  this->log(log_level::DEBUG, "Called HuaceNode() constructor..");

  tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));

  // Parameters must be set before initializing IO
  if (!getROSParams()) return;

  IO_ = std::make_unique<io::AsyncManager>(this);
  // Initializes Connection
  connect();

  this->log(log_level::DEBUG, "Leaving HuaceNode() constructor..");
}
void huace_node::HuaceNode::connect()
{
  this->log(
    log_level::DEBUG, "Started timer for calling connect() method until connection succeeds");

  boost::asio::io_context io;
  boost::posix_time::millisec wait_ms(static_cast<uint32_t>(settings_.reconnect_delay_s * 1000));
  while (this->ok()) {
    boost::asio::deadline_timer t(io, wait_ms);

    if (IO_->connect()) {
      break;
    }

    t.wait();
  }
  // If node is shut down before a connection could be established
  if (!this->ok()) return;

  this->log(log_level::DEBUG, "Successully connected. Leaving connect() method");
}

[[nodiscard]] bool huace_node::HuaceNode::getROSParams()
{
  param("ntp_server", settings_.ntp_server, false);
  param("ptp_server_clock", settings_.ptp_server_clock, false);
  param("use_gnss_time", settings_.use_gnss_time, false);
  param("latency_compensation", settings_.latency_compensation, false);

  param("frame_id", settings_.frame_id, static_cast<std::string>("gnss"));
  param("imu_frame_id", settings_.imu_frame_id, static_cast<std::string>("imu"));
  param("poi_frame_id", settings_.poi_frame_id, static_cast<std::string>("base_link"));
  param("vsm_frame_id", settings_.vsm_frame_id, static_cast<std::string>("vsm"));
  param("aux1_frame_id", settings_.aux1_frame_id, static_cast<std::string>("aux1"));
  param("vehicle_frame_id", settings_.vehicle_frame_id, settings_.poi_frame_id);
  param("local_frame_id", settings_.local_frame_id, static_cast<std::string>("odom"));
  param("insert_local_frame", settings_.insert_local_frame, false);

  std::string io_type_str{};
  param("io_type", io_type_str, static_cast<std::string>(""));
  settings_.io_type = io_type_str == "serial"       ? device_type::SERIAL
                      : io_type_str == "udp_server" ? device_type::UDP_SERVER
                      : io_type_str == "tcp_client" ? device_type::TCP_CLIENT
                                                    : device_type::NONE;
  if (settings_.io_type == device_type::TCP_CLIENT) {
    param("tcp_client.remote_ip", settings_.io_tcp_remote_ip, static_cast<std::string>(""));
    getUint32Param(
      "tcp_client.remote_port", settings_.io_tcp_remote_port, static_cast<uint32_t>(0));
  } else if (settings_.io_type == device_type::UDP_SERVER) {
    // param("udp.type", settings_.io_udp_type, static_cast<std::string>(""));
    // param("udp.host_ip", settings_.io_udp_host_ip, static_cast<std::string>(""));
    getUint32Param("udp_server.host_port", settings_.io_udp_host_port, static_cast<uint32_t>(0));
    // param("udp.remote_ip", settings_.io_udp_remote_ip, static_cast<std::string>(""));
    // getUint32Param("udp.remote_port", settings_.io_udp_remote_port, static_cast<uint32_t>(0));
  } else if (settings_.io_type == device_type::SERIAL) {
    param("serial.portname", settings_.io_serial_portname, static_cast<std::string>(""));
    param("serial.baudrate", settings_.io_serial_baudrate, static_cast<int32_t>(0));
    // param("serial.baudrate", settings_.io_serial_baudrate, static_cast<int64_t>(0));
    // } else if (this->has_parameter("can")) {
  }

  // Publishing parameters
  param("publish.auto_publish", settings_.auto_publish, false);
  param("publish.publish_only_valid", settings_.publish_only_valid, false);
  param("publish.gpgga", settings_.publish_gpgga, false);
  param("publish.gprmc", settings_.publish_gprmc, false);
  param("publish.gpgsa", settings_.publish_gpgsa, false);
  param("publish.gpgsv", settings_.publish_gpgsv, false);
  param("publish.pose", settings_.publish_pose, false);
  param("publish.imu", settings_.publish_imu, false);
  param("publish.localization", settings_.publish_localization, false);
  param("publish.twist", settings_.publish_twist, false);
  param("publish.tf", settings_.publish_tf, false);

  settings_.reconnect_delay_s = 60;
  // To be implemented: RTCM, raw data settings, PPP, SBAS ...
  this->log(log_level::DEBUG, "Finished getROSParams() method");
  return true;
}

[[nodiscard]] bool huace_node::HuaceNode::validPeriod(uint32_t period, bool isIns) const
{
  return (
    (period == 0) || ((period == 5 && isIns)) || (period == 10) || (period == 20) ||
    (period == 40) || (period == 50) || (period == 100) || (period == 200) || (period == 500) ||
    (period == 1000) || (period == 2000) || (period == 5000) || (period == 10000) ||
    (period == 15000) || (period == 30000) || (period == 60000) || (period == 120000) ||
    (period == 300000) || (period == 600000) || (period == 900000) || (period == 1800000) ||
    (period == 3600000));
}

void huace_node::HuaceNode::getTransform(
  const std::string & targetFrame, const std::string & sourceFrame,
  TransformStampedMsg & T_s_t) const
{
  bool found = false;
  while (!found) {
    try {
      // try to get tf from source frame to target frame
      T_s_t = tfBuffer_.lookupTransform(targetFrame, sourceFrame, rclcpp::Time(0));
      found = true;
    } catch (const tf2::TransformException & ex) {
      this->log(
        log_level::WARN, "Waiting for transform from " + sourceFrame + " to " + targetFrame + ": " +
                           ex.what() + ".");
      found = false;
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(2000ms);
    }
  }
}

void huace_node::HuaceNode::getRPY(
  const QuaternionMsg & qm, double & roll, double & pitch, double & yaw) const
{
  Eigen::Quaterniond q(qm.w, qm.x, qm.y, qm.z);
  Eigen::Quaterniond::RotationMatrixType C = q.matrix();

  roll = std::atan2(C(2, 1), C(2, 2));
  pitch = std::asin(-C(2, 0));
  yaw = std::atan2(C(1, 0), C(0, 0));
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(huace_node::HuaceNode)