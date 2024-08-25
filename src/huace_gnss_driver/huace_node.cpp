// Eigen include
#include <Eigen/Geometry>
#include <huace_gnss_driver/huace_node.hpp>

/**
 * @file huace_node.cpp
 * @date 22/08/20
 * @brief The heart of the ROSaic driver: The ROS node that represents it
 */

huace_node::HuaceNode::HuaceNode(const rclcpp::NodeOptions& options) :
    HuaceNodeBase(options), IO_(this), tfBuffer_(this->get_clock())
{
    param("activate_debug_log", settings_.activate_debug_log, false);
    if (settings_.activate_debug_log)
    {
        auto ret = rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                                    RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Error setting severity: %s",
                         rcutils_get_error_string().str);
            rcutils_reset_error();
        }
    }

    this->log(log_level::DEBUG, "Called HuaceNode() constructor..");

    tfListener_.reset(new tf2_ros::TransformListener(tfBuffer_));

    // Parameters must be set before initializing IO
    if (!getROSParams())
        return;

    // Initializes Connection
    IO_.connect();

    this->log(log_level::DEBUG, "Leaving HuaceNode() constructor..");
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

    if (this->has_parameter("io.tcp")) {
        param("io.tcp.host_ip", settings_.io_tcp_host_ip, static_cast<std::string>(""));
        getUint32Param("io.tcp.host_port", settings_.io_tcp_host_port, static_cast<uint32_t>(0));
        param("io.tcp.remote_ip", settings_.io_tcp_remote_ip, static_cast<std::string>(""));
        getUint32Param("io.tcp.remote_port", settings_.io_tcp_remote_port, static_cast<uint32_t>(0));
        settings_.io_type = device_type::TCP;
    } else if (this->has_parameter("io.udp")) {
        param("io.udp.type", settings_.io_udp_type, static_cast<std::string>(""));
        param("io.udp.host_ip", settings_.io_udp_host_ip, static_cast<std::string>(""));
        getUint32Param("io.udp.host_port", settings_.io_udp_host_port, static_cast<uint32_t>(0));
        param("io.udp.remote_ip", settings_.io_udp_remote_ip, static_cast<std::string>(""));
        getUint32Param("io.udp.remote_port", settings_.io_udp_remote_port, static_cast<uint32_t>(0));
        settings_.io_type = device_type::UDP;
    // } else if (this->has_parameter("io.serial")) {
    // } else if (this->has_parameter("io.can")) {
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

[[nodiscard]] bool huace_node::HuaceNode::validPeriod(uint32_t period,
                                                        bool isIns) const
{
    return ((period == 0) || ((period == 5 && isIns)) || (period == 10) ||
            (period == 20) || (period == 40) || (period == 50) || (period == 100) ||
            (period == 200) || (period == 500) || (period == 1000) ||
            (period == 2000) || (period == 5000) || (period == 10000) ||
            (period == 15000) || (period == 30000) || (period == 60000) ||
            (period == 120000) || (period == 300000) || (period == 600000) ||
            (period == 900000) || (period == 1800000) || (period == 3600000));
}

void huace_node::HuaceNode::getTransform(const std::string& targetFrame,
                                           const std::string& sourceFrame,
                                           TransformStampedMsg& T_s_t) const
{
    bool found = false;
    while (!found)
    {
        try
        {
            // try to get tf from source frame to target frame
            T_s_t =
                tfBuffer_.lookupTransform(targetFrame, sourceFrame, rclcpp::Time(0));
            found = true;
        } catch (const tf2::TransformException& ex)
        {
            this->log(log_level::WARN, "Waiting for transform from " + sourceFrame +
                                           " to " + targetFrame + ": " + ex.what() +
                                           ".");
            found = false;
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(2000ms);
        }
    }
}

void huace_node::HuaceNode::getRPY(const QuaternionMsg& qm, double& roll,
                                     double& pitch, double& yaw) const
{
    Eigen::Quaterniond q(qm.w, qm.x, qm.y, qm.z);
    Eigen::Quaterniond::RotationMatrixType C = q.matrix();

    roll = std::atan2(C(2, 1), C(2, 2));
    pitch = std::asin(-C(2, 0));
    yaw = std::atan2(C(1, 0), C(0, 0));
}

void huace_node::HuaceNode::sendVelocity(const std::string& velNmea)
{
    IO_.sendVelocity(velNmea);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(huace_node::HuaceNode)