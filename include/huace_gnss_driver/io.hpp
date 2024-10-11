// *****************************************************************************
// 用于定义所有的IO类，包含了输入输出。
// 所有的类实现统一的基类
// *****************************************************************************

#pragma once

// C++
#include <iostream>
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
// #include "boost_tcp_driver/tcp_driver.hpp"
// #include "boost_udp_driver/udp_driver.hpp"

// #include <huace_gnss_driver/huace_node_base.hpp>
// #include <huace_gnss_driver/telegram.hpp>

//! Possible baudrates for the Rx
const static std::array<uint32_t, 21> baudrates = {
  1200,   2400,   4800,    9600,    19200,   38400,   57600,   115200,  230400,  460800, 500000,
  576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 3500000, 4000000};

using boost::asio::ip::tcp;
using boost::asio::ip::udp;

#define MAX_HUACE_PACKET_SIZE 1024

namespace io
{
class IoBase
{
public:
  virtual bool connect() = 0;
  virtual std::size_t send(const uint8_t * buff, int count = -1) = 0;
  virtual void receive(const std::function<int(uint8_t *, int)> & callback) = 0;

protected:
  std::array<uint8_t, MAX_HUACE_PACKET_SIZE> receive_buffer_;  // 接收缓冲区
};

class UdpServer : public IoBase
{
  // Udp终端没有服务端\客户端概念,打开端口就可以接收发送.有三种地址方式:
  // 单播 (Unicast)：一对一通信方式，接收方只需监听指定的 IP 地址和端口。
  // 组播
  // (Multicast)：一对多通信方式，接收方需要加入一个特定的组播组才能接收消息。
  // 广播
  // (Broadcast)：一对全网通信方式，接收方通过监听特定端口接收网络上的广播消息。
public:
  UdpServer(std::shared_ptr<boost::asio::io_context> ctx, unsigned short port)
  : m_owned_ctx{ctx}, port_(port), socket_(*ctx)
  {
  }

  ~UdpServer()
  {
    // node_->log(log_level::INFO, "UDP client shutting down threads");
  }
  bool connect()
  {
    // 在connect中创建和绑定socket
    boost::system::error_code ec;
    socket_.open(udp::v4(), ec);  // 打开IPv4的UDP socket
    if (ec) {
      std::cerr << "Failed to open socket: " << ec.message() << std::endl;
      return false;
    }

    socket_.bind(udp::endpoint(udp::v4(), port_), ec);  // 绑定到指定的端口
    if (ec) {
      std::cerr << "Failed to bind socket: " << ec.message() << std::endl;
      return false;
    }

    return true;
  }

  std::size_t send(const uint8_t * buff, int count)
  {
    return socket_.send_to(boost::asio::buffer(buff, count), remote_endpoint_);
  }

  void receive(const std::function<int(uint8_t *, int)> & callback)
  {
    socket_.async_receive_from(
      boost::asio::buffer(receive_buffer_), remote_endpoint_,
      [this, callback](boost::system::error_code ec, std::size_t bytes_recvd) {
        if (!ec && bytes_recvd > 0) {
          std::string str(reinterpret_cast<const char *>(receive_buffer_.data()), bytes_recvd);
          std::cout << "Received(" << bytes_recvd << "): " << str << std::endl;
          // 回发收到的消息
          callback(reinterpret_cast<uint8_t *>(receive_buffer_.data()), bytes_recvd);
          // send(reinterpret_cast<uint8_t *>(receive_buffer_.data()), bytes_recvd);
        }
        receive(callback);  // 再次开始接收数据
      });
  }

private:
  std::shared_ptr<boost::asio::io_context> m_owned_ctx;
  unsigned int port_;
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
};

class TcpClient : public IoBase
{
  // 只作为客户端
public:
  TcpClient(
    std::shared_ptr<boost::asio::io_context> ctx, const std::string host, unsigned short port)
  : m_owned_ctx(ctx), host_(host), port_(port), resolver_(*ctx), socket_(*ctx)
  {
    // connect();
  }

  ~TcpClient() { socket_.close(); }

  bool connect()
  {
    tcp::resolver::results_type endpoints = resolver_.resolve(host_, std::to_string(port_));
    boost::asio::connect(socket_, endpoints);
    return true;
  }
  std::size_t send(const uint8_t * buff, int count)
  {
    return boost::asio::write(socket_, boost::asio::buffer(buff, count));
  }
  void receive(const std::function<int(uint8_t *, int)> & callback)
  {
    socket_.async_read_some(
      boost::asio::buffer(receive_buffer_),
      [this, callback](const boost::system::error_code & ec, std::size_t bytes_recvd) {
        if (!ec && bytes_recvd > 0) {
          std::string str(reinterpret_cast<const char *>(receive_buffer_.data()), bytes_recvd);
          std::cout << "Received(" << bytes_recvd << "): " << str << std::endl;
          // 回发收到的消息
          callback(reinterpret_cast<uint8_t *>(receive_buffer_.data()), bytes_recvd);
          // send(reinterpret_cast<uint8_t *>(receive_buffer_.data()), bytes_recvd);
        }
        receive(callback);  // 再次开始接收数据
      });
  }

private:
  std::string host_;
  unsigned int port_;
  std::shared_ptr<boost::asio::io_context> m_owned_ctx;
  tcp::resolver resolver_;
  tcp::socket socket_;
};

class SerialIo : public IoBase
{
public:
  SerialIo(
    std::shared_ptr<boost::asio::io_context> ctx, const std::string & portname,
    unsigned int baudrate)
  : m_owned_ctx(ctx),
    portname_(portname),
    baudrate_(baudrate),
    tempbuff_(MAX_HUACE_PACKET_SIZE + 1, '\0'),
    serial_port_(std::make_unique<boost::asio::serial_port>(*m_owned_ctx))
  {
  }

  ~SerialIo() { serial_port_->close(); }

  bool connect()
  {
    if (serial_port_->is_open()) {
      serial_port_->close();
    }

    bool opened = false;

    while (!opened) {
      try {
        serial_port_->open(portname_);
        opened = true;
      } catch (const boost::system::system_error & err) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1s);
      }
    }
    if (!opened) return false;

    // No Parity, 8bits data, 1 stop Bit
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));
    serial_port_->set_option(
      boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(
      boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    // Set low latency
    int fd = serial_port_->native_handle();
    struct serial_struct serialInfo;
    memset(&serialInfo, 0, sizeof(serialInfo));  // 清空结构体
    if (ioctl(fd, TIOCGSERIAL, &serialInfo) < 0) {
      std::cerr << "Failed to get serial info: " << strerror(errno) << std::endl;
    }

    serialInfo.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(fd, TIOCSSERIAL, &serialInfo) < 0) {
      std::cerr << "Failed to set serial info: " << strerror(errno) << std::endl;
    }

    // clear io
    ::tcflush(fd, TCIOFLUSH);
    return true;
  }
  std::size_t send(const uint8_t * buff, int count)
  {
    return boost::asio::write(*serial_port_, boost::asio::buffer(buff, count));
  }
  void receive(const std::function<int(uint8_t *, int)> & callback)
  {
    serial_port_->async_read_some(
      boost::asio::buffer(receive_buffer_),
      [this, callback](const boost::system::error_code & ec, std::size_t bytes_recvd) {
        if (!ec && bytes_recvd > 0) {
          // tempbuff_.assign(reinterpret_cast<const char *>(receive_buffer_.data()), bytes_recvd);
          // std::cout << "Received(" << bytes_recvd << "): " << tempbuff_ << std::endl;
          std::string str(reinterpret_cast<const char*>(receive_buffer_.data()), bytes_recvd);
          std::cout << "Received("<< bytes_recvd << "): " << str << std::endl;
          // 回发收到的消息
          callback(reinterpret_cast<uint8_t *>(receive_buffer_.data()), bytes_recvd);
          // send(reinterpret_cast<uint8_t *>(receive_buffer_.data()), bytes_recvd);
        }
        receive(callback);  // 再次开始接收数据
      });
  }

private:
  std::shared_ptr<boost::asio::io_context> m_owned_ctx;
  std::string portname_;
  uint32_t baudrate_;

  std::string tempbuff_;

public:
  std::unique_ptr<boost::asio::serial_port> serial_port_;  // 使用 std::unique_ptr 管理串口
};

}  // namespace io
