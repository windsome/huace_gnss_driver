#pragma once

// std includes
#include <any>
#include <iomanip>
#include <sstream>
#include <unordered_map>
// ROS includes
#include <rclcpp/rclcpp.hpp>
// tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// ROS msg includes
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
// GNSS msg includes
#include "huace_gnss_driver/msg/gpchc.hpp"
#include "huace_gnss_driver/msg/gpchcx.hpp"
// NMEA msg includes
#include <nmea_msgs/msg/gpgga.hpp>
#include <nmea_msgs/msg/gpgsa.hpp>
#include <nmea_msgs/msg/gpgsv.hpp>
#include <nmea_msgs/msg/gprmc.hpp>
// Rosaic includes
#include <huace_gnss_driver/settings.hpp>
#include <huace_gnss_driver/parsers/string_utilities.hpp>
#include <huace_gnss_driver/parsers/sbf_utilities.hpp>

// Timestamp in nanoseconds (Unix epoch)
typedef uint64_t Timestamp;
// ROS timestamp
typedef rclcpp::Time TimestampRos;

// ROS messages
typedef diagnostic_msgs::msg::DiagnosticArray DiagnosticArrayMsg;
typedef diagnostic_msgs::msg::DiagnosticStatus DiagnosticStatusMsg;
typedef geometry_msgs::msg::Quaternion QuaternionMsg;
typedef geometry_msgs::msg::PoseWithCovarianceStamped PoseWithCovarianceStampedMsg;
typedef geometry_msgs::msg::TwistWithCovarianceStamped TwistWithCovarianceStampedMsg;
typedef geometry_msgs::msg::TransformStamped TransformStampedMsg;
typedef gps_msgs::msg::GPSFix GpsFixMsg;
typedef gps_msgs::msg::GPSStatus GpsStatusMsg;
typedef sensor_msgs::msg::NavSatFix NavSatFixMsg;
typedef sensor_msgs::msg::NavSatStatus NavSatStatusMsg;
typedef sensor_msgs::msg::TimeReference TimeReferenceMsg;
typedef sensor_msgs::msg::Imu ImuMsg;
typedef nav_msgs::msg::Odometry LocalizationMsg;

// Septentrio GNSS SBF messages
typedef huace_gnss_driver::msg::Gpchc GpchcMsg;
typedef huace_gnss_driver::msg::Gpchcx GpchcxMsg;

// NMEA message
typedef nmea_msgs::msg::Gpgga GpggaMsg;
typedef nmea_msgs::msg::Gpgsa GpgsaMsg;
typedef nmea_msgs::msg::Gpgsv GpgsvMsg;
typedef nmea_msgs::msg::Gprmc GprmcMsg;

/**
 * @brief Convert nsec timestamp to ROS timestamp
 * @param[in] ts timestamp in nanoseconds (Unix epoch)
 * @return ROS timestamp
 */
inline TimestampRos timestampToRos(Timestamp ts) { return TimestampRos(ts); }

/**
 * @brief Convert ROS timestamp to nsec timestamp
 * @param[in] ts ROS timestamp
 * @return timestamp in nanoseconds (Unix epoch)
 */
inline Timestamp timestampFromRos(const TimestampRos& tsr)
{
    return tsr.nanoseconds();
}

/**
 * @brief Log level for ROS logging
 */
namespace log_level {
    enum LogLevel
    {
        DEBUG,
        INFO,
        WARN,
        ERROR,
        FATAL
    };
} // namespace log_level

/**
 * @class HuaceNodeBase
 * @brief This class is the base class for abstraction
 */
class HuaceNodeBase : public rclcpp::Node
{
public:
    HuaceNodeBase(const rclcpp::NodeOptions& options) :
        Node("huace_gnss", options), tf2Publisher_(this),
        tfBuffer_(this->get_clock()), tfListener_(tfBuffer_)
    {
    }

    ~HuaceNodeBase() {}

    bool ok() { return rclcpp::ok(); }

    const Settings* settings() const { return &settings_; }

    /**
     * @brief Gets an integer or unsigned integer value from the parameter server
     * @param[in] name The key to be used in the parameter server's dictionary
     * @param[out] val Storage for the retrieved value, of type U, which can be
     * either unsigned int or int
     * @param[in] defaultVal Value to use if the server doesn't contain this
     * parameter
     */
    bool getUint32Param(const std::string& name, uint32_t& val, uint32_t defaultVal)
    {
        int32_t tempVal;
        if ((!this->param(name, tempVal, -1)) || (tempVal < 0))
        {
            val = defaultVal;
            return false;
        }
        val = tempVal;
        return true;
    }

    /**
     * @brief Gets parameter of type T from the parameter server
     * @param[in] name The key to be used in the parameter server's dictionary
     * @param[out] val Storage for the retrieved value, of type T
     * @param[in] defaultVal Value to use if the server doesn't contain this
     * parameter
     * @return True if it could be retrieved, false if not
     */
    template <typename T>
    bool param(const std::string& name, T& val, const T& defaultVal)
    {
        if (this->has_parameter(name))
            this->undeclare_parameter(name);

        try
        {
            val = this->declare_parameter<T>(name, defaultVal);
        } catch (std::runtime_error& e)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), e.what());
            return false;
        }
        return true;
    }

    /**
     * @brief Log function to provide abstraction of ROS loggers
     * @param[in] logLevel Log level
     * @param[in] s String to log
     */
    void log(log_level::LogLevel logLevel, const std::string& s) const
    {
        switch (logLevel)
        {
        case log_level::DEBUG:
            RCLCPP_DEBUG_STREAM(this->get_logger(), s);
            break;
        case log_level::INFO:
            RCLCPP_INFO_STREAM(this->get_logger(), s);
            break;
        case log_level::WARN:
            RCLCPP_WARN_STREAM(this->get_logger(), s);
            break;
        case log_level::ERROR:
            RCLCPP_ERROR_STREAM(this->get_logger(), s);
            break;
        case log_level::FATAL:
            RCLCPP_FATAL_STREAM(this->get_logger(), s);
            break;
        default:
            break;
        }
    }

    /**
     * @brief Gets current timestamp
     * @return Timestamp
     */
    Timestamp getTime() const { return this->now().nanoseconds(); }

    /**
     * @brief Publishing function
     * @param[in] topic String of topic
     * @param[in] msg ROS message to be published
     */
    template <typename M>
    void publishMessage(const std::string& topic, const M& msg)
    {
        if constexpr (has_block_header<M>::value)
        {
            if (settings_.publish_only_valid && !validValue(msg.block_header.tow))
                return;
        }

        auto it = topicMap_.find(topic);
        if (it != topicMap_.end())
        {
            typename rclcpp::Publisher<M>::SharedPtr ptr =
                std::any_cast<typename rclcpp::Publisher<M>::SharedPtr>(it->second);
            ptr->publish(msg);
        } else
        {
            if (this->ok())
            {
                typename rclcpp::Publisher<M>::SharedPtr pub =
                    this->create_publisher<M>(
                        topic, rclcpp::QoS(rclcpp::KeepLast(queueSize_))
                                   .durability_volatile()
                                   .reliable());
                topicMap_.insert(std::make_pair(topic, pub));
                pub->publish(msg);
            }
        }
    }

    /**
     * @brief Publishing function for tf
     * @param[in] msg ROS localization message to be converted to tf
     */
    void publishTf(const LocalizationMsg& loc)
    {
        if (std::isnan(loc.pose.pose.orientation.w))
            return;

        Timestamp currentStamp = timestampFromRos(loc.header.stamp);
        if (lastTfStamp_ == currentStamp)
            return;

        lastTfStamp_ = currentStamp;

        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = loc.header.stamp;
        transformStamped.header.frame_id = loc.header.frame_id;
        transformStamped.child_frame_id = loc.child_frame_id;
        transformStamped.transform.translation.x = loc.pose.pose.position.x;
        transformStamped.transform.translation.y = loc.pose.pose.position.y;
        transformStamped.transform.translation.z = loc.pose.pose.position.z;
        transformStamped.transform.rotation.x = loc.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = loc.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = loc.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = loc.pose.pose.orientation.w;

        if (settings_.insert_local_frame)
        {
            geometry_msgs::msg::TransformStamped T_l_b;
            try
            {
                // try to get tf at timestamp of message
                T_l_b = tfBuffer_.lookupTransform(
                    loc.child_frame_id, settings_.local_frame_id, loc.header.stamp);
            } catch (const tf2::TransformException& ex)
            {
                try
                {
                    RCLCPP_INFO_STREAM_THROTTLE(
                        this->get_logger(), *this->get_clock(), 10000,
                        ": No transform for insertion of local frame at t="
                            << std::to_string(currentStamp)
                            << ". Exception: " << std::string(ex.what()));
                    // try to get latest tf
                    T_l_b = tfBuffer_.lookupTransform(loc.child_frame_id,
                                                      settings_.local_frame_id,
                                                      rclcpp::Time(0));
                } catch (const tf2::TransformException& ex)
                {
                    RCLCPP_WARN_STREAM_THROTTLE(
                        this->get_logger(), *this->get_clock(), 10000,
                        ": No most recent transform for insertion of local frame. Exception: "
                            << std::string(ex.what()));
                    return;
                }
            }

            // T_l_g = T_b_l^-1 * T_b_g;
            transformStamped =
                tf2::eigenToTransform(tf2::transformToEigen(transformStamped) *
                                      tf2::transformToEigen(T_l_b));
            transformStamped.header.stamp = loc.header.stamp;
            transformStamped.header.frame_id = loc.header.frame_id;
            transformStamped.child_frame_id = settings_.local_frame_id;
        }

        tf2Publisher_.sendTransform(transformStamped);
    }

    /**
     * @brief Set INS to true
     */
    void setIsIns() { capabilities_.is_ins = true; }

    /**
     * @brief Set has heading to true
     */
    void setHasHeading() { capabilities_.has_heading = true; }

    /**
     * @brief Set improved VSM handling to true
     */
    void setImprovedVsmHandling() { capabilities_.has_improved_vsm_handling = true; }

    /**
     * @brief Check if Rx is INS
     */
    bool isIns() { return capabilities_.is_ins; }

    /**
     * @brief Check if Rx has heading
     */
    bool hasHeading() { return capabilities_.has_heading; }

    /**
     * @brief Check if Rx has improved VSM handling
     */
    bool hasImprovedVsmHandling() { return capabilities_.has_improved_vsm_handling; }

protected:
    //! Settings
    Settings settings_;

private:
    //! Map of topics and publishers
    std::unordered_map<std::string, std::any> topicMap_;
    //! Publisher queue size
    uint32_t queueSize_ = 1;
    //! Transform publisher
    tf2_ros::TransformBroadcaster tf2Publisher_;
    //! Odometry subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySubscriber_;
    //! Twist subscriber
    rclcpp::Subscription<TwistWithCovarianceStampedMsg>::SharedPtr twistSubscriber_;
    //! Last tf stamp
    Timestamp lastTfStamp_ = 0;
    //! tf buffer
    tf2_ros::Buffer tfBuffer_;
    // tf listener
    tf2_ros::TransformListener tfListener_;
    // Capabilities of Rx
    Capabilities capabilities_;
};