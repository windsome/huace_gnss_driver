#pragma once

/**
 * @file huace_node.hpp
 * @date 21/08/20
 * @brief The heart of the ROSaic driver: The ROS node that represents it
 */

// tf2 includes
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// ROSaic includes
#include <huace_gnss_driver/huace_node_base.hpp>
#include <huace_gnss_driver/async_manager.hpp>

/**
 * @namespace huace_node
 * This namespace is for the ROSaic node, handling all aspects regarding
 * ROS parameters, ROS message publishing etc.
 */
namespace huace_node {
    /**
     * @class HuaceNode
     * @brief This class represents the ROsaic node, to be extended..
     */
    class HuaceNode : public HuaceNodeBase
    {
    public:
        //! The constructor initializes and runs the ROSaic node, if everything works
        //! fine. It loads the user-defined ROS parameters, subscribes to Rx
        //! messages, and publishes requested ROS messages...
        HuaceNode(const rclcpp::NodeOptions& options);

    private:
        void connect();
        /**
         * @brief Gets the node parameters from the ROS Parameter Server, parts of
         * which are specified in a YAML file
         *
         * The other ROSaic parameters are specified via the command line.
         */
        [[nodiscard]] bool getROSParams();
        /**
         * @brief Checks if the period has a valid value
         * @param[in] period period [ms]
         * @param[in] isIns wether recevier is an INS
         * @return wether the period is valid
         */
        [[nodiscard]] bool validPeriod(uint32_t period, bool isIns) const;
        /**
         * @brief Gets transforms from tf2
         * @param[in] targetFrame traget frame id
         * @param[in] sourceFrame source frame id
         * @param[out] T_s_t transfrom from source to target
         */
        void getTransform(const std::string& targetFrame,
                          const std::string& sourceFrame,
                          TransformStampedMsg& T_s_t) const;
        /**
         * @brief Gets Euler angles from quaternion message
         * @param[in] qm quaternion message
         * @param[out] roll roll angle
         * @param[out] pitch pitch angle
         * @param[out] yaw yaw angle
         */
        void getRPY(const QuaternionMsg& qm, double& roll, double& pitch,
                    double& yaw) const;

        //! Handles communication with the Rx
        std::unique_ptr<io::AsyncManager> IO_;
        //! tf2 buffer and listener
        tf2_ros::Buffer tfBuffer_;
        std::unique_ptr<tf2_ros::TransformListener> tfListener_;
    };
} // namespace huace_node