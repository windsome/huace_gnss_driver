#pragma once

// C++ libraries
#include <cassert> // for assert
#include <cstddef>
#include <map>
#include <sstream>
// Boost includes
#include <boost/call_traits.hpp>
#include <boost/format.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/tokenizer.hpp>
// Huace includes
#include <huace_gnss_driver/huace_node_base.hpp>
#include <huace_gnss_driver/telegram.hpp>
#include <huace_gnss_driver/parsers/nmea_parsers/gpgga.hpp>
#include <huace_gnss_driver/parsers/nmea_parsers/gpgsa.hpp>
#include <huace_gnss_driver/parsers/nmea_parsers/gpgsv.hpp>
#include <huace_gnss_driver/parsers/nmea_parsers/gprmc.hpp>
#include <huace_gnss_driver/parsers/nmea_parsers/gpchc.hpp>
#include <huace_gnss_driver/parsers/nmea_parsers/gpchcx.hpp>
#include <huace_gnss_driver/parsers/string_utilities.hpp>

/**
 * @file message_parser.hpp
 * @brief Defines a class that reads messages handed over from the circular buffer
 */

namespace io {

    /**
     * @class MessageHandler
     * @brief Can search buffer for messages, read/parse them, and so on
     */
    class MessageHandler
    {
    public:
        /**
         * @brief Constructor of the MessageHandler class
         * @param[in] node Pointer to the node)
         */
        MessageHandler(HuaceNodeBase* node) :
            node_(node), settings_(node->settings()), unix_time_(0)
        {
        }

        /**
         * @brief Parse NMEA block
         * @param[in] telegram Telegram to be parsed
         */
        void parseNmea(const std::shared_ptr<Telegram>& telegram);

    private:
        /**
         * @brief Publishing function
         * @param[in] topic String of topic
         * @param[in] msg ROS message to be published
         */
        template <typename M>
        void publish(const std::string& topic, const M& msg);

        /**
         * @brief Publishing function
         * @param[in] msg Localization message
         */

        void publishTf(const LocalizationMsg& msg);

        /**
         * @brief Pointer to the node
         */
        HuaceNodeBase* node_;

        /**
         * @brief Pointer to settings struct
         */
        const Settings* settings_;

        /**
         * @brief Map of NMEA messgae IDs and uint8_t
         */
        std::unordered_map<std::string, uint8_t> nmeaMap_{
            {"$GPGGA", 0}, {"$INGGA", 0}, {"$GPRMC", 1}, {"$INRMC", 1},
            {"$GPGSA", 2}, {"$INGSA", 2}, {"$GAGSV", 3}, {"$INGSV", 3},
            {"$GPCHC", 4}, {"$GPCHCX", 5} };

        //! When reading from an SBF file, the ROS publishing frequency is governed
        //! by the time stamps found in the SBF blocks therein.
        Timestamp unix_time_;

        //! Last reported PVT processing latency
        mutable uint64_t last_pvt_latency_ = 0;

        //! Current leap seconds as received, do not use value is -128
        int32_t current_leap_seconds_ = -128;

        /**
         * @brief Set status of NavSatFix messages
         */
        void setStatus(uint8_t mode, NavSatFixMsg& msg);

        // /**
        //  * @brief "Callback" function when constructing
        //  * ImuMsg messages
        //  */
        // void assembleImu();

        // /**
        //  * @brief "Callback" function when constructing
        //  * LocalizationMsg messages in UTM
        //  */
        // void assembleLocalizationUtm();

        // /**
        //  * @brief function to fill twist part of LocalizationMsg
        //  * @param[in] roll roll [rad]
        //  * @param[in] pitch pitch [rad]
        //  * @param[in] yaw yaw [rad]
        //  * @param[inout] msg LocalizationMsg to be filled
        //  */
        // void assembleLocalizationMsgTwist(double roll, double pitch, double yaw,
        //                                   LocalizationMsg& msg) const;

        // /**
        //  * @brief "Callback" function when constructing
        //  * TwistWithCovarianceStampedMsg messages
        //  * @param[in] fromIns Wether to contruct message from INS data
        //  */
        // void assembleTwist(bool fromIns = false);

        // /**
        //  * @brief  function when constructing
        //  * TimeReferenceMsg messages
        //  * @param[in] telegram telegram from which the msg was assembled
        //  */
        // void assembleTimeReference(const std::shared_ptr<Telegram>& telegram);

        /**
         * @brief Waits according to time when reading from file
         * @param[in] time_obj wait until time
         */
        void wait(Timestamp time_obj);

        /**
         * @brief Fixed UTM zone
         */
        std::shared_ptr<std::string> fixedUtmZone_;

        /**
         * @brief Calculates the timestamp, in the Unix Epoch time format
         * This is either done using the TOW as transmitted with the SBF block (if
         * "use_gnss" is true), or using the current time.
         * @param[in] tow (Time of Week) Number of milliseconds that elapsed since
         * the beginning of the current GPS week as transmitted by the SBF block
         * @param[in] wnc (Week Number Counter) counts the number of complete weeks
         * elapsed since January 6, 1980
         * @return Timestamp object containing seconds and nanoseconds since last
         * epoch
         */
        Timestamp timestampHuace(float tow, uint32_t wnc) const;
    };
} // namespace io