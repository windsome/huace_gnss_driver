// *****************************************************************************
// 用于定义所有的IO类，包含了输入输出。
// 所有的类实现统一的基类
// *****************************************************************************

#pragma once

// C++
#include <thread>

// Linux
#include <linux/input.h>
#include <linux/serial.h>

// Boost
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

// ROSaic
#include "boost_tcp_driver/tcp_driver.hpp"
#include "boost_udp_driver/udp_driver.hpp"

#include <huace_gnss_driver/huace_node_base.hpp>
#include <huace_gnss_driver/telegram.hpp>

//! Possible baudrates for the Rx
const static std::array<uint32_t, 21> baudrates = {
    1200,    2400,    4800,    9600,    19200,   38400,   57600,
    115200,  230400,  460800,  500000,  576000,  921600,  1000000,
    1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000};

namespace io {
    class IoBase
    {
    public:
        virtual std::size_t send(std::vector<uint8_t>& buff) = 0;
        virtual bool connect() = 0;

    };

    class UdpIo : public IoBase
    {
        // Udp终端没有服务端\客户端概念,打开端口就可以接收发送.有三种地址方式:
        // 单播 (Unicast)：一对一通信方式，接收方只需监听指定的 IP 地址和端口。
        // 组播
        // (Multicast)：一对多通信方式，接收方需要加入一个特定的组播组才能接收消息。
        // 广播
        // (Broadcast)：一对全网通信方式，接收方通过监听特定端口接收网络上的广播消息。
    public:
        UdpIo(HuaceNodeBase* node, TelegramQueue* telegramQueue) :
            node_(node), running_(true), telegramQueue_(telegramQueue),
            m_owned_ctx{new drivers::common::IoContext(1)},
            m_udp_driver{new drivers::udp_driver::UdpDriver(*m_owned_ctx)}
        {
            connect();
        }

        ~UdpIo()
        {
            running_ = false;
            m_udp_driver->receiver()->close();
            if (m_owned_ctx)
            {
                m_owned_ctx->waitForExit();
            }
            node_->log(log_level::INFO, "UDP client shutting down threads");
        }
        std::size_t send(std::vector<uint8_t>& buff)
        {
            return m_udp_driver->receiver()->send(buff);
        }
        bool connect()
        {
            std::string type = node_->settings()->io_udp_type;
            std::string remote_ip = node_->settings()->io_udp_remote_ip;
            uint16_t remote_port = node_->settings()->io_udp_remote_port;
            std::string host_ip = node_->settings()->io_udp_host_ip;
            uint16_t host_port = node_->settings()->io_udp_host_port;
            try
            {
                m_udp_driver->init_receiver(remote_ip, remote_port, host_ip,
                                            host_port);
                if (type == "multicast")
                    m_udp_driver->receiver()
                        ->setMulticast(true);
                m_udp_driver->receiver()->open();
                m_udp_driver->receiver()->bind();
                m_udp_driver->receiver()->asyncReceive(std::bind(
                    &UdpIo::receiver_callback, this, std::placeholders::_1));
            } catch (const std::exception& ex)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Error creating UDP receiver: %s:%i - %s",
                             host_ip.c_str(), host_port, ex.what());
                return false;
            }
            node_->log(log_level::INFO,
                       "Listening on UDP port " + std::to_string(host_port));
            return true;
        }
        void receiver_callback(const std::vector<uint8_t>& buffer)
        {
            Timestamp stamp = node_->getTime();
            std::shared_ptr<Telegram> telegram(new Telegram);
            telegram->stamp = stamp;
            // telegram->message.assign(buffer);
            telegram->message = buffer;
            telegram->type = telegram_type::NMEA;
            telegramQueue_->push(telegram);
        }

    private:
        //! Pointer to the node
        HuaceNodeBase* node_;
        TelegramQueue* telegramQueue_;
        std::atomic<bool> running_;
        std::unique_ptr<drivers::common::IoContext> m_owned_ctx{};
        std::unique_ptr<drivers::udp_driver::UdpDriver> m_udp_driver;
    };

    class TcpIo : public IoBase
    {
        // 只作为客户端
    public:
        TcpIo(HuaceNodeBase* node, TelegramQueue* telegramQueue) :
            node_(node), running_(true), telegramQueue_(telegramQueue),
            m_owned_ctx{new boost::asio::io_context(1)},
            m_tcp_driver{new drivers::tcp_driver::TcpDriver(m_owned_ctx)}
        {
            connect();
        }

        ~TcpIo()
        {
            // stream_->close();
            if (m_tcp_driver)
            {
                if (m_tcp_driver->isOpen())
                {
                    m_tcp_driver->close();
                }
            }
        }

        bool connect()
        {
            std::string remote_ip = node_->settings()->io_tcp_remote_ip;
            uint16_t remote_port = node_->settings()->io_tcp_remote_port;
            std::string host_ip = node_->settings()->io_tcp_host_ip;
            uint16_t host_port = node_->settings()->io_tcp_host_port;
            m_tcp_driver->init_socket(remote_ip, remote_port, host_ip, host_port);
            if (!m_tcp_driver->open())
            {
                m_tcp_driver->closeSync();
                return false;
            }
            m_tcp_driver->socket()->asyncReceive(
                std::bind(&TcpIo::receiver_callback, this, std::placeholders::_1));

            node_->log(log_level::INFO, "Connected to " + remote_ip + ".");
            return true;
        }
        std::size_t send(std::vector<uint8_t>& buff)
        {
            if (m_tcp_driver->asyncSend(buff))
            {
                return buff.size();
            }
            return 0;
        }
        void receiver_callback(const std::vector<uint8_t>& buffer)
        {
            Timestamp stamp = node_->getTime();
            std::shared_ptr<Telegram> telegram(new Telegram);
            telegram->stamp = stamp;
            // telegram->message.assign(buffer);
            telegram->message = buffer;
            telegram->type = telegram_type::NMEA;
            telegramQueue_->push(telegram);
        }

    private:
        HuaceNodeBase* node_;
        TelegramQueue* telegramQueue_;
        std::atomic<bool> running_;
        std::shared_ptr<boost::asio::io_context> m_owned_ctx;
        std::unique_ptr<drivers::tcp_driver::TcpDriver> m_tcp_driver;

    public:
    };

} // namespace io