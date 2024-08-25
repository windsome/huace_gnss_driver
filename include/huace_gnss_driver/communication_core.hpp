// 从udp/tcp/serial/can获取华测gnss-ins设备上报的消息,转发到ros2/topic消息中
#pragma once

// Boost includes
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
// C++ library includes
#include <fstream>
#include <memory>
#include <sstream>
// ROSaic includes
#include <huace_gnss_driver/huace_node_base.hpp>
#include <huace_gnss_driver/message_handler.hpp>
#include <huace_gnss_driver/io.hpp>

/**
 * @file communication_core.hpp
 * @date 22/08/20
 * @brief Highest-Level view on communication services
 */

/**
 * @namespace io
 * This namespace is for the communication interface, handling all aspects related to
 * serial and TCP/IP communication..
 */
namespace io {

    /**
     * @class CommunicationCore
     * @brief Handles communication with and configuration of the mosaic (and beyond)
     * receiver(s)
     */
    class CommunicationCore
    {
    public:
        /**
         * @brief Constructor of the class CommunicationCore
         * @param[in] node Pointer to node
         */
        CommunicationCore(HuaceNodeBase* node);
        /**
         * @brief Default destructor of the class CommunicationCore
         */
        ~CommunicationCore();

        /**
         * @brief Connects the data stream
         */
        void connect();

        /**
         * @brief Hands over NMEA velocity message over to the send() method of
         * manager_
         * @param cmd The command to hand over
         */
        void sendVelocity(const std::string& velNmea);

    private:
        /**
         * @brief Initializes the I/O handling
         * * @return Wether connection was successful
         */
        [[nodiscard]] bool initializeIo();

        void processTelegrams();

        /**
         * @brief Hands over to the send() method of manager_
         * @param cmd The command to hand over
         */
        void send(const std::string& cmd);

        //! Pointer to Node
        HuaceNodeBase* node_;
        //! Settings
        const Settings* settings_;
        //! TelegramQueue
        TelegramQueue telegramQueue_;
        //! MessageHandler parser
        MessageHandler messageHandler_;
        // //! TelegramHandler
        // TelegramHandler telegramHandler_;
        //! Processing thread
        std::thread processingThread_;

        //! Whether connecting was successful
        bool initializedIo_ = false;
        std::unique_ptr<IoBase> io_;
        //! Processes I/O stream data

        //! Indicator for threads to run
        std::atomic<bool> running_;
    };
} // namespace io
