#include <chrono>
#include <filesystem>
#include <fstream>
#include <linux/serial.h>

// Boost includes
#include <boost/regex.hpp>
#include <huace_gnss_driver/communication_core.hpp>

static const int16_t ANGLE_MAX = 180;
static const int16_t ANGLE_MIN = -180;
static const int8_t THETA_Y_MAX = 90;
static const int8_t THETA_Y_MIN = -90;
static const int8_t LEVER_ARM_MAX = 100;
static const int8_t LEVER_ARM_MIN = -100;
static const int16_t HEADING_MAX = 360;
static const int16_t HEADING_MIN = -360;
static const int8_t PITCH_MAX = 90;
static const int8_t PITCH_MIN = -90;
static const int8_t ATTSTD_DEV_MIN = 0;
static const int8_t ATTSTD_DEV_MAX = 5;
static const int8_t POSSTD_DEV_MIN = 0;
static const int8_t POSSTD_DEV_MAX = 100;

/**
 * @file communication_core.cpp
 * @date 22/08/20
 * @brief Highest-Level view on communication services
 */

namespace io {

    CommunicationCore::CommunicationCore(HuaceNodeBase* node) :
        node_(node),
        settings_(node->settings()),
        io_(nullptr),
        messageHandler_(node),
        running_(true)
    {
        processingThread_ =
            std::thread(std::bind(&CommunicationCore::processTelegrams, this));
    }

    CommunicationCore::~CommunicationCore()
    {
        // telegramHandler_.clearSemaphores();

        running_ = false;
        std::shared_ptr<Telegram> telegram(new Telegram);
        telegramQueue_.push(telegram);
        processingThread_.join();
    }

    void CommunicationCore::connect()
    {
        node_->log(log_level::DEBUG, "Called connect() method");
        node_->log(
            log_level::DEBUG,
            "Started timer for calling connect() method until connection succeeds");

        boost::asio::io_service io;
        boost::posix_time::millisec wait_ms(
            static_cast<uint32_t>(settings_->reconnect_delay_s * 1000));
        if (initializeIo())
        {
            while (running_ && node_->ok())
            {
                boost::asio::deadline_timer t(io, wait_ms);

                if (io_->connect())
                {
                    initializedIo_ = true;
                    break;
                }

                t.wait();
            }
        }
        // If node is shut down before a connection could be established
        if (!node_->ok())
            return;

        node_->log(log_level::DEBUG,
                   "Successully connected. Leaving connect() method");
    }

    [[nodiscard]] bool CommunicationCore::initializeIo()
    {
        node_->log(log_level::DEBUG, "Called initializeIo() method");
        switch (settings_->io_type)
        {
        case device_type::TCP:
        {
            io_ = std::make_unique<TcpIo>(node_, &telegramQueue_);
            // io_->connect();
            break;
        }
        case device_type::UDP:
        {
            io_ = std::make_unique<UdpIo>(node_, &telegramQueue_);
            // io_->connect();
            break;
        }
        // case device_type::SERIAL:
        // {
        //     manager_.reset(new AsyncManager<SerialIo>(node_, &telegramQueue_));
        //     break;
        // }
        // case device_type::SBF_FILE:
        // {
        //     manager_.reset(new AsyncManager<SbfFileIo>(node_, &telegramQueue_));
        //     break;
        // }
        // case device_type::PCAP_FILE:
        // {
        //     manager_.reset(new AsyncManager<PcapFileIo>(node_, &telegramQueue_));
        //     break;
        // }
        default:
        {
            node_->log(log_level::DEBUG, "Unsupported device.");
            return false;
        }
        }
        return true;
    }

    void CommunicationCore::sendVelocity(const std::string& velNmea)
    {
        // if (nmeaActivated_)
        // {
        //     if (settings_->ins_vsm.use_stream_device)
        //     {
        //         if (tcpClient_)
        //             tcpClient_.get()->send(velNmea);
        //     } else
        //     {
        //         if (tcpVsm_)
        //             tcpVsm_.get()->send(velNmea);
        //     }
        // }
    }

    void CommunicationCore::processTelegrams()
    {
        while (running_)
        {
            std::shared_ptr<Telegram> telegram;
            telegramQueue_.pop(telegram);

            if (telegram->type != telegram_type::EMPTY) {
                // 处理消息
                // telegramHandler_.handleTelegram(telegram);
                if (telegram->type != telegram_type::NMEA)
                    messageHandler_.parseNmea(telegram);
            }
        }
    }

    void CommunicationCore::send(const std::string& cmd)
    {
        std::vector<uint8_t> vec(cmd.begin(), cmd.end());
        if(io_) io_->send(vec);
        // telegramHandler_.waitForResponse();
    }

} // namespace io