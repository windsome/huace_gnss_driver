#pragma once

// Boost includes
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/regex.hpp>

// ROSaic includes
// #include <huace_gnss_driver/crc/crc.hpp>
#include <huace_gnss_driver/parsers/parsing_utilities.hpp>

// local includes
#include <huace_gnss_driver/io.hpp>
#include <huace_gnss_driver/message_handler.hpp>
#include <huace_gnss_driver/telegram.hpp>

// #define HEADER_S "$GPCHC"
// #define ENDER_S "\r\n"

namespace io
{

void printBuffer(uint8_t * buff, size_t size)
{
  // // 打印十六进制格式
  // std::cout << "HEX(" << size << "): " << std::endl;
  // for (size_t i = 0; i < size; ++i) {
  //   std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buff[i]) << " ";
  // }
  // std::cout << std::dec << std::endl;  // 恢复为十进制输出

  // 打印字符串格式（注意要确保buff是有效的字符串）
  std::cout << "STR(" << size << "): " << std::endl;
  for (size_t i = 0; i < size; ++i) {
    // 这里假设buff中包含有效的ASCII字符
    // 也可以根据需求添加条件来判断是否打印字符
    std::cout << static_cast<char>(buff[i]);
  }
  std::cout << std::endl;
}

/**
 * @class AsyncManager
 * @brief This is the central interface between ROSaic and the Rx(s), managing
 * I/O operations such as reading messages and sending commands..
 *
 * IoType is either boost::asio::serial_port or boost::asio::tcp::ip
 */
class AsyncManager
{
public:
  /**
   * @brief Class constructor
   * @param[in] node Pointer to node
   * @param[in] telegramQueue Telegram queue
   */
  AsyncManager(HuaceNodeBase * node)
  : node_(node), ioContext_(new boost::asio::io_context), messageHandler_(node),
    tempbuff_(2 * MAX_HUACE_PACKET_SIZE + 1, '\0')
  {
    const Settings * settings = node->settings();
    switch (settings->io_type) {
      case device_type::TCP_CLIENT: {
        ioInterface_ = std::make_unique<io::TcpClient>(
          ioContext_, settings->io_tcp_remote_ip, settings->io_tcp_remote_port);
        // io_->connect();
        break;
      }
      case device_type::UDP_SERVER: {
        ioInterface_ = std::make_unique<io::UdpServer>(ioContext_, settings->io_udp_host_port);
        // io_->connect();
        break;
      }
      case device_type::SERIAL: {
        ioInterface_ = std::make_unique<io::SerialIo>(
          ioContext_, settings->io_serial_portname, settings->io_serial_baudrate);
        break;
      }
      default: {
        node_->log(log_level::WARN, "Unsupported device.");
      }
    }

    node_->log(
      log_level::INFO, "AsyncManager created. type=" + static_cast<int>(settings->io_type));
  }

  ~AsyncManager()
  {
    running_ = false;
    // ioInterface_.close();
    node_->log(log_level::DEBUG, "AsyncManager shutting down threads");
    if (ioThread_.joinable()) {
      ioContext_->stop();
      ioThread_.join();
    }
    if (watchdogThread_.joinable()) watchdogThread_.join();

    // 结束处理线程，需要先等IO退出，先发一个消息，是避免无法触发到
    std::shared_ptr<Telegram> telegram(new Telegram);
    telegramQueue_.push(telegram);
    if (watchdogThread_.joinable()) processingThread_.join();

    node_->log(log_level::DEBUG, "AsyncManager threads stopped");
  }

  bool connect()
  {
    running_ = true;

    if (!ioInterface_->connect()) {
      return false;
    }
    connected_ = true;
    receive();

    return true;
  }

  void send(const std::string & cmd)
  {
    if (cmd.size() == 0) {
      node_->log(log_level::ERROR, "AsyncManager message size to be sent to the Rx would be 0");
      return;
    }

    ioInterface_->send(reinterpret_cast<const uint8_t *>(cmd.data()), cmd.size());
  }

  bool connected() { return connected_; }

private:
  int receive_callback(uint8_t * buff, int size)
  {
    std::cout << "接收字节数: " << size << ", 处理前remain_buffer_size_=" << remain_buffer_size_ << std::endl;
    if (size <= 0 || size > MAX_HUACE_PACKET_SIZE) {
      std::cout << "receive_callback, wrong buff size = " << size << std::endl;
      remain_buffer_size_ = 0;
      return 0;
    }
    if (remain_buffer_size_ > MAX_HUACE_PACKET_SIZE) {
      // 残留数据超过1个包大小，清空原来的数据.
      remain_buffer_size_ = 0;
    }
    // 添加新数据到末尾
    std::copy(buff, buff + size, remain_buffer_.begin() + remain_buffer_size_);
    remain_buffer_size_ += size;
    printBuffer(remain_buffer_.data(), remain_buffer_size_);

    // 开始处理remain_buffer_, 都是NMEA消息.
    // 查找$,如果未找到说明消息缺少头，重置
    // 找到$后，找\r\n, 找到则表示发现一条消息, 继续发现下一条消息，
    // 没有新消息后整理remain_buffer_为剩余子buff，修改remain_buffer_size_

    std::vector<uint8_t> pattern1 = {SYNC_BYTE_1, NMEA_SYNC_BYTE_2};  // 开始子串$G
    // std::vector<uint8_t> pattern2 = {CR, LF};                         // 结束子串\r\n
    std::vector<uint8_t> pattern2 = {SYNC_BYTE_END, SYNC_BYTE_END};   // 结束子串\r\n

    int pos = 0;
    while (pos < remain_buffer_size_) {
      // 查找$G位置
      auto it1 = std::search(
        remain_buffer_.begin() + pos, remain_buffer_.begin() + remain_buffer_size_,
        pattern1.begin(), pattern1.end());
      if (it1 != remain_buffer_.begin() + remain_buffer_size_) {
        pos = std::distance(remain_buffer_.begin(), it1);
        // 找到:$G
        std::cout << "找到开始符$G,位置: " << pos << std::endl;
      } else {
        // 未找到:$G, 表示数据未收集器, 需要继续收集数据
        break;
      }
      // 查找\r\n位置
      auto it2 = std::search(
        remain_buffer_.begin() + pos, remain_buffer_.begin() + remain_buffer_size_,
        pattern2.begin(), pattern2.end());
      if (it2 != remain_buffer_.begin() + remain_buffer_size_) {
        int end = std::distance(remain_buffer_.begin(), it2);
        // 找到:\r\n
        int size = end - pos + pattern2.size();
        std::cout << "找到结束符,位置: " << end << ",size=" << size << std::endl;
        telegram_.reset(new Telegram);
        telegram_->message.resize(size);
        telegram_->type = telegram_type::NMEA;
        telegram_->stamp = node_->getTime();
        std::copy(
          remain_buffer_.begin() + pos, remain_buffer_.begin() + end + pattern2.size(), telegram_->message.begin());
        telegramQueue_.push(telegram_);
        pos = end + pattern2.size();
      } else {
        // 未找到:\r\n, 表示剩余buff不够一个包
        break;
      }
    }
    if (pos > 0 && pos < remain_buffer_size_) {
      // pos表示已经处理掉的数据.
      // int need_move = remain_buffer_size_ - pos;
      // 将未用的buffer提前到开头，即去掉已处理的buffer.
      std::move(
        remain_buffer_.begin() + pos, remain_buffer_.begin() + remain_buffer_size_,
        remain_buffer_.begin());
      remain_buffer_size_ -= pos;
    } else if (pos == remain_buffer_size_) {
      // 已经到末尾了, 说明没有数据了.
      remain_buffer_size_ = 0;
    }
    std::cout << "结束整理缓存后,remain_buffer_size_=" << remain_buffer_size_ << ", pos=" << pos << std::endl;

    return 0;
  }

  void receive()
  {
    resync();
    ioThread_ = std::thread(std::bind(&AsyncManager::runIoService, this));
    if (!watchdogThread_.joinable())
      watchdogThread_ = std::thread(std::bind(&AsyncManager::runWatchdog, this));
    if (!processingThread_.joinable())
      processingThread_ = std::thread(std::bind(&AsyncManager::processTelegrams, this));
  }
  void runIoService()
  {
    // 设置receive
    ioInterface_->receive(std::bind(
      &AsyncManager::receive_callback, this, std::placeholders::_1, std::placeholders::_2));
    ioContext_->run();
    node_->log(log_level::DEBUG, "AsyncManager ioContext terminated.");
  }
  void runWatchdog()
  {
    while (running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if (running_ && ioContext_->stopped()) {
        connected_ = false;
        node_->log(log_level::ERROR, "AsyncManager connection lost. Trying to reconnect.");
        ioContext_->reset();
        ioThread_.join();
        while (!ioInterface_->connect())
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        connected_ = true;
        receive();
      }
    }
  }
  void resync()
  {
    remain_buffer_size_ = 0;
    telegram_.reset(new Telegram);
    // readSync<0>();
  }
  void processTelegrams()
  {
    while (running_) {
      std::shared_ptr<Telegram> telegram;
      telegramQueue_.pop(telegram);
      // node_->log(log_level::INFO, "AsyncManager.processTelegrams: telegramQueue_.pop()");
      std::cout << "开始处理报文processTelegrams,剩余:" << telegramQueue_.size() << std::endl;

      if (telegram->type != telegram_type::EMPTY) {
        // 处理消息
        messageHandler_.parseNmea(telegram);
      }
    }
  }

  //! Pointer to the node
  HuaceNodeBase * node_;
  std::shared_ptr<boost::asio::io_context> ioContext_;
  std::unique_ptr<io::IoBase> ioInterface_;
  std::atomic<bool> running_;
  std::thread ioThread_;
  std::thread watchdogThread_;
  std::thread processingThread_;

  bool connected_ = false;
  std::array<uint8_t, MAX_HUACE_PACKET_SIZE * 2>
    remain_buffer_;  // 未处理数据缓冲区，将与收到的数据合并处理，所以最多不能超过2个包的大小。
  int remain_buffer_size_{0};

  TelegramQueue telegramQueue_;  // 解析出来的完整NMEA包列表
  Timestamp recvStamp_;          // 最后接收消息的时间
  MessageHandler messageHandler_;
  std::string tempbuff_;

  std::shared_ptr<Telegram> telegram_;  //
};

}  // namespace io