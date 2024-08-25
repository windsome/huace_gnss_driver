// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#pragma once

#include <stdint.h>
#include <string>
#include <vector>

struct InsVsm
{
    //! VSM source for INS
    std::string ros_source;
    //! Whether or not to use individual elements of 3D velocity (v_x, v_y, v_z)
    std::vector<bool> ros_config = {false, false, false};
    //! Whether or not to use variance defined by ROS parameter
    bool ros_variances_by_parameter = false;
    //! Variances of the 3D velocity (var_x, var_y, var_z)
    std::vector<double> ros_variances = {-1.0, -1.0, -1.0};
    //! Wether to use stream device tcp
    bool use_stream_device = false;
    //! VSM IP server id
    std::string ip_server;
    //! VSM tcp port
    uint32_t ip_server_port;
    //! Wether VSM shall be kept open om shutdown
    bool ip_server_keep_open;
    //! VSM serial port
    std::string serial_port;
    //! VSM serial baud rate
    uint32_t serial_baud_rate;
    //! Wether VSM shall be kept open om shutdown
    bool serial_keep_open;
};

namespace device_type {
    enum DeviceType
    {
        TCP,
        UDP,
        SERIAL,
        SBF_FILE,
        PCAP_FILE
    };
} // namespace device_type

//! Settings struct
struct Settings
{
    //! Set logger level to DEBUG
    bool activate_debug_log;

    //! If true, the ROS message headers' unix time field is constructed from the TOW
    //! (in the SBF case) and UTC (in the NMEA case) data. If false, times are
    //! constructed within the driver via ROS time.
    bool use_gnss_time;
    //! Wether NTP server shall be activated
    bool ntp_server;
    //! Wether PTP grandmaster clock shall be activated
    bool ptp_server_clock;
    //! Wether processing latency shall be compensated for in ROS timestamp
    bool latency_compensation;

    //! Wether local frame should be inserted into tf
    bool insert_local_frame = false;
    //! Frame id of the local frame to be inserted
    std::string local_frame_id;
    //! Septentrio receiver type, either "gnss" or "ins"
    std::string septentrio_receiver_type;
    //! The frame ID used in the header of every published ROS message
    std::string frame_id;
    //! The frame ID used in the header of published ROS Imu message
    std::string imu_frame_id;
    //! The frame ID used in the header of published ROS Localization message if poi
    //! is used
    std::string poi_frame_id;
    //! The frame ID of the velocity sensor
    std::string vsm_frame_id;
    //! The frame ID of the aux1 antenna
    std::string aux1_frame_id;
    //! The frame ID of the vehicle frame
    std::string vehicle_frame_id;

    //! Device type
    device_type::DeviceType io_type;
    //! TCP IP
    std::string io_tcp_host_ip;
    //! TCP port
    uint32_t io_tcp_host_port;
    //! TCP IP
    std::string io_tcp_remote_ip;
    //! TCP port
    uint32_t io_tcp_remote_port;
    //! UDP TYPE
    std::string io_udp_type;
    //! TCP IP
    std::string io_udp_host_ip;
    //! TCP port
    uint32_t io_udp_host_port;
    //! TCP IP
    std::string io_udp_remote_ip;
    //! TCP port
    uint32_t io_udp_remote_port;

    //! Wether to publish automatically for cinfigure_rx = false
    bool auto_publish;
    //! Wether to publish only valid messages
    bool publish_only_valid;
    //! Whether or not to publish the GGA message
    bool publish_gpgga;
    //! Whether or not to publish the RMC message
    bool publish_gprmc;
    //! Whether or not to publish the GSA message
    bool publish_gpgsa;
    //! Whether or not to publish the GSV message
    bool publish_gpgsv;
    //! Whether or not to publish the PoseWithCovarianceStampedMsg message
    bool publish_pose;
    //! Whether or not to publish the ImuMsg message
    bool publish_imu;
    //! Whether or not to publish the LocalizationMsg message
    bool publish_localization;
    //! Whether or not to publish the TwistWithCovarianceStampedMsg message
    bool publish_twist;
    //! Whether or not to publish the tf of the localization
    bool publish_tf;

    bool use_ros_axis_orientation;
    uint32_t reconnect_delay_s;
    //! INS VSM setting
    InsVsm ins_vsm;
};

//! Capabilities struct
struct Capabilities
{
    //! Wether Rx is INS
    bool is_ins = false;
    //! Wether Rx has heading
    bool has_heading = false;
    //! Wether Rx has improved VSM handling
    bool has_improved_vsm_handling = false;
};