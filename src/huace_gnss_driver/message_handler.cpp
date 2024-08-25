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

#include <GeographicLib/UTMUPS.hpp>
#include <boost/tokenizer.hpp>
#include <huace_gnss_driver/message_handler.hpp>
#include <thread>

/**
 * The position_covariance array is populated in row-major order, where the basis of
 * the correspond matrix is (E, N, U, Roll, Pitch, Heading). Important: The Euler
 * angles (Roll, Pitch, Heading) are with respect to a vehicle-fixed (e.g. for
 * mosaic-x5 in moving base mode via the command setAntennaLocation, ...) !local! NED
 * frame or ENU frame if use_ros_axis_directions is set true Thus the orientation
 * is !not! given with respect to the same frame as the position is given in. The
 * cross-covariances are hence (apart from the fact that e.g. mosaic receivers do
 * not calculate these quantities) set to zero. The position and the partial
 * (with 2 antennas) or full (for INS receivers) orientation have covariance matrices
 * available e.g. in the PosCovGeodetic or AttCovEuler blocks, yet those are separate
 * computations.
 */

using parsing_utilities::convertEulerToQuaternionMsg;
using parsing_utilities::deg2rad;
using parsing_utilities::deg2radSq;
using parsing_utilities::rad2deg;
using parsing_utilities::square;

namespace io {

    double convertAutoCovariance(double val)
    {
        return std::isnan(val) ? -1.0 : deg2radSq(val);
    }

    double convertCovariance(double val)
    {
        return std::isnan(val) ? 0.0 : deg2radSq(val);
    }

    // void MessageHandler::assemblePoseWithCovarianceStamped()
    // {
    //     if (!settings_->publish_pose)
    //         return;

    //     thread_local auto last_ins_tow = last_insnavgeod_.block_header.tow;

    //     PoseWithCovarianceStampedMsg msg;
    //     if (settings_->huace_receiver_type == "ins")
    //     {
    //         if (!validValue(last_insnavgeod_.block_header.tow) ||
    //             (last_insnavgeod_.block_header.tow == last_ins_tow))
    //             return;
    //         last_ins_tow = last_insnavgeod_.block_header.tow;

    //         msg.header = last_insnavgeod_.header;

    //         msg.pose.pose.position.x = rad2deg(last_insnavgeod_.longitude);
    //         msg.pose.pose.position.y = rad2deg(last_insnavgeod_.latitude);
    //         msg.pose.pose.position.z = last_insnavgeod_.height;

    //         // Filling in the pose data
    //         if ((last_insnavgeod_.sb_list & 1) != 0)
    //         {
    //             // Pos autocov
    //             msg.pose.covariance[0] = square(last_insnavgeod_.longitude_std_dev);
    //             msg.pose.covariance[7] = square(last_insnavgeod_.latitude_std_dev);
    //             msg.pose.covariance[14] = square(last_insnavgeod_.height_std_dev);
    //         } else
    //         {
    //             msg.pose.covariance[0] = -1.0;
    //             msg.pose.covariance[7] = -1.0;
    //             msg.pose.covariance[14] = -1.0;
    //         }
    //         if ((last_insnavgeod_.sb_list & 2) != 0)
    //         {
    //             double yaw = last_insnavgeod_.heading;
    //             double pitch = last_insnavgeod_.pitch;
    //             double roll = last_insnavgeod_.roll;
    //             // Attitude
    //             msg.pose.pose.orientation = convertEulerToQuaternionMsg(
    //                 deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
    //         } else
    //         {
    //             msg.pose.pose.orientation.w =
    //                 std::numeric_limits<double>::quiet_NaN();
    //             msg.pose.pose.orientation.x =
    //                 std::numeric_limits<double>::quiet_NaN();
    //             msg.pose.pose.orientation.y =
    //                 std::numeric_limits<double>::quiet_NaN();
    //             msg.pose.pose.orientation.z =
    //                 std::numeric_limits<double>::quiet_NaN();
    //         }
    //         if ((last_insnavgeod_.sb_list & 4) != 0)
    //         {
    //             // Attitude autocov
    //             msg.pose.covariance[21] =
    //                 convertAutoCovariance(last_insnavgeod_.roll_std_dev);
    //             msg.pose.covariance[28] =
    //                 convertAutoCovariance(last_insnavgeod_.pitch_std_dev);
    //             msg.pose.covariance[35] =
    //                 convertAutoCovariance(last_insnavgeod_.heading_std_dev);

    //         } else
    //         {
    //             msg.pose.covariance[21] = -1.0;
    //             msg.pose.covariance[28] = -1.0;
    //             msg.pose.covariance[35] = -1.0;
    //         }
    //         if ((last_insnavgeod_.sb_list & 32) != 0)
    //         {
    //             // Pos cov
    //             msg.pose.covariance[1] = last_insnavgeod_.latitude_longitude_cov;
    //             msg.pose.covariance[2] = last_insnavgeod_.longitude_height_cov;
    //             msg.pose.covariance[6] = last_insnavgeod_.latitude_longitude_cov;
    //             msg.pose.covariance[8] = last_insnavgeod_.latitude_height_cov;
    //             msg.pose.covariance[12] = last_insnavgeod_.longitude_height_cov;
    //             msg.pose.covariance[13] = last_insnavgeod_.latitude_height_cov;
    //         }
    //         if ((last_insnavgeod_.sb_list & 64) != 0)
    //         {
    //             // Attitude cov
    //             msg.pose.covariance[22] =
    //                 convertCovariance(last_insnavgeod_.pitch_roll_cov);
    //             msg.pose.covariance[23] =
    //                 convertCovariance(last_insnavgeod_.heading_roll_cov);
    //             msg.pose.covariance[27] =
    //                 convertCovariance(last_insnavgeod_.pitch_roll_cov);

    //             msg.pose.covariance[29] =
    //                 convertCovariance(last_insnavgeod_.heading_pitch_cov);
    //             msg.pose.covariance[33] =
    //                 convertCovariance(last_insnavgeod_.heading_roll_cov);
    //             msg.pose.covariance[34] =
    //                 convertCovariance(last_insnavgeod_.heading_pitch_cov);
    //         }
    //     } else
    //     {
    //         if ((!validValue(last_pvtgeodetic_.block_header.tow)) ||
    //             (last_pvtgeodetic_.block_header.tow !=
    //              last_atteuler_.block_header.tow) ||
    //             (last_pvtgeodetic_.block_header.tow !=
    //              last_poscovgeodetic_.block_header.tow) ||
    //             (last_pvtgeodetic_.block_header.tow !=
    //              last_attcoveuler_.block_header.tow))
    //             return;

    //         msg.header = last_pvtgeodetic_.header;

    //         // Filling in the pose data
    //         double yaw = last_atteuler_.heading;
    //         double pitch = last_atteuler_.pitch;
    //         double roll = last_atteuler_.roll;

    //         roll = std::isnan(roll) ? 0.0 : roll;
    //         pitch = std::isnan(pitch) ? 0.0 : pitch;

    //         msg.pose.pose.orientation = convertEulerToQuaternionMsg(
    //             deg2rad(roll), deg2rad(pitch), deg2rad(yaw));
    //         msg.pose.pose.position.x = rad2deg(last_pvtgeodetic_.longitude);
    //         msg.pose.pose.position.y = rad2deg(last_pvtgeodetic_.latitude);
    //         msg.pose.pose.position.z = last_pvtgeodetic_.height;
    //         // Filling in the covariance data in row-major order
    //         msg.pose.covariance[0] = last_poscovgeodetic_.cov_lonlon;
    //         msg.pose.covariance[1] = last_poscovgeodetic_.cov_latlon;
    //         msg.pose.covariance[2] = last_poscovgeodetic_.cov_lonhgt;
    //         msg.pose.covariance[6] = last_poscovgeodetic_.cov_latlon;
    //         msg.pose.covariance[7] = last_poscovgeodetic_.cov_latlat;
    //         msg.pose.covariance[8] = last_poscovgeodetic_.cov_lathgt;
    //         msg.pose.covariance[12] = last_poscovgeodetic_.cov_lonhgt;
    //         msg.pose.covariance[13] = last_poscovgeodetic_.cov_lathgt;
    //         msg.pose.covariance[14] = last_poscovgeodetic_.cov_hgthgt;
    //         msg.pose.covariance[21] =
    //             convertAutoCovariance(last_attcoveuler_.cov_rollroll);
    //         msg.pose.covariance[22] =
    //             convertCovariance(last_attcoveuler_.cov_pitchroll);
    //         msg.pose.covariance[23] =
    //             convertCovariance(last_attcoveuler_.cov_headroll);
    //         msg.pose.covariance[27] =
    //             convertCovariance(last_attcoveuler_.cov_pitchroll);
    //         msg.pose.covariance[28] =
    //             convertAutoCovariance(last_attcoveuler_.cov_pitchpitch);
    //         msg.pose.covariance[29] =
    //             convertCovariance(last_attcoveuler_.cov_headpitch);
    //         msg.pose.covariance[33] =
    //             convertCovariance(last_attcoveuler_.cov_headroll);
    //         msg.pose.covariance[34] =
    //             convertCovariance(last_attcoveuler_.cov_headpitch);
    //         msg.pose.covariance[35] =
    //             convertAutoCovariance(last_attcoveuler_.cov_headhead);
    //     }
    //     publish<PoseWithCovarianceStampedMsg>("pose", msg);
    // };

    // void MessageHandler::assembleImu()
    // {
    //     ImuMsg msg;

    //     msg.header = last_extsensmeas_.header;

    //     msg.linear_acceleration.x = last_extsensmeas_.acceleration_x;
    //     msg.linear_acceleration.y = last_extsensmeas_.acceleration_y;
    //     msg.linear_acceleration.z = last_extsensmeas_.acceleration_z;

    //     msg.angular_velocity.x = deg2rad(last_extsensmeas_.angular_rate_x);
    //     msg.angular_velocity.y = deg2rad(last_extsensmeas_.angular_rate_y);
    //     msg.angular_velocity.z = deg2rad(last_extsensmeas_.angular_rate_z);

    //     bool valid_orientation = false;
    //     if (validValue(last_insnavgeod_.block_header.tow))
    //     {
    //         // INS tow and extsens meas tow have the same time scale
    //         Timestamp tsImu = timestampSBF(last_extsensmeas_.block_header.tow,
    //                                        last_extsensmeas_.block_header.wnc);
    //         Timestamp tsIns = timestampSBF(last_insnavgeod_.block_header.tow,
    //                                        last_insnavgeod_.block_header.wnc);

    //         thread_local int64_t maxDt =
    //             (settings_->polling_period_pvt == 0)
    //                 ? 10000000
    //                 : settings_->polling_period_pvt * 1000000;
    //         if ((tsImu - tsIns) > maxDt)
    //         {
    //             valid_orientation = false;
    //         } else
    //         {
    //             if ((last_insnavgeod_.sb_list & 2) != 0)
    //             {
    //                 // Attitude
    //                 if (validValue(last_insnavgeod_.heading) &&
    //                     validValue(last_insnavgeod_.pitch) &&
    //                     validValue(last_insnavgeod_.roll))
    //                 {
    //                     msg.orientation = convertEulerToQuaternionMsg(
    //                         deg2rad(last_insnavgeod_.roll),
    //                         deg2rad(last_insnavgeod_.pitch),
    //                         deg2rad(last_insnavgeod_.heading));
    //                     valid_orientation = true;
    //                 }
    //             }
    //             if ((last_insnavgeod_.sb_list & 4) != 0)
    //             {
    //                 // Attitude autocov
    //                 if (validValue(last_insnavgeod_.roll_std_dev) &&
    //                     validValue(last_insnavgeod_.pitch_std_dev) &&
    //                     validValue(last_insnavgeod_.heading_std_dev))
    //                 {
    //                     msg.orientation_covariance[0] =
    //                         convertAutoCovariance(last_insnavgeod_.roll_std_dev);
    //                     msg.orientation_covariance[4] =
    //                         convertAutoCovariance(last_insnavgeod_.pitch_std_dev);
    //                     msg.orientation_covariance[8] =
    //                         convertAutoCovariance(last_insnavgeod_.heading_std_dev);

    //                     if ((last_insnavgeod_.sb_list & 64) != 0)
    //                     {
    //                         // Attitude cov
    //                         msg.orientation_covariance[1] =
    //                             convertCovariance(last_insnavgeod_.pitch_roll_cov);
    //                         msg.orientation_covariance[2] =
    //                             convertCovariance(last_insnavgeod_.heading_roll_cov);
    //                         msg.orientation_covariance[3] =
    //                             convertCovariance(last_insnavgeod_.pitch_roll_cov);

    //                         msg.orientation_covariance[5] = convertCovariance(
    //                             last_insnavgeod_.heading_pitch_cov);
    //                         msg.orientation_covariance[6] =
    //                             convertCovariance(last_insnavgeod_.heading_roll_cov);
    //                         msg.orientation_covariance[7] = convertCovariance(
    //                             last_insnavgeod_.heading_pitch_cov);
    //                     }
    //                 } else
    //                 {
    //                     msg.orientation_covariance[0] = -1.0;
    //                     msg.orientation_covariance[4] = -1.0;
    //                     msg.orientation_covariance[8] = -1.0;
    //                 }
    //             }
    //         }
    //     }

    //     if (!valid_orientation)
    //     {
    //         msg.orientation.w = std::numeric_limits<double>::quiet_NaN();
    //         msg.orientation.x = std::numeric_limits<double>::quiet_NaN();
    //         msg.orientation.y = std::numeric_limits<double>::quiet_NaN();
    //         msg.orientation.z = std::numeric_limits<double>::quiet_NaN();
    //     }

    //     publish<ImuMsg>("imu", msg);
    // };

    // void MessageHandler::assembleTwist(bool fromIns /* = false*/)
    // {
    //     if (!settings_->publish_twist)
    //         return;
    //     TwistWithCovarianceStampedMsg msg;

    //     // Autocovariances of angular velocity
    //     msg.twist.covariance[21] = -1.0;
    //     msg.twist.covariance[28] = -1.0;
    //     msg.twist.covariance[35] = -1.0;
    //     // Set angular velocities to NaN
    //     msg.twist.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
    //     msg.twist.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
    //     msg.twist.twist.angular.z = std::numeric_limits<double>::quiet_NaN();

    //     if (fromIns)
    //     {
    //         msg.header = last_insnavgeod_.header;

    //         if ((last_insnavgeod_.sb_list & 8) != 0)
    //         {
    //             // Linear velocity in navigation frame
    //             double ve = last_insnavgeod_.ve;
    //             double vn = last_insnavgeod_.vn;
    //             double vu = last_insnavgeod_.vu;
    //             Eigen::Vector3d vel;
    //             if (settings_->use_ros_axis_orientation)
    //             {
    //                 // (ENU)
    //                 vel << ve, vn, vu;
    //             } else
    //             {
    //                 // (NED)
    //                 vel << vn, ve, -vu;
    //             }
    //             // Linear velocity
    //             msg.twist.twist.linear.x = vel(0);
    //             msg.twist.twist.linear.y = vel(1);
    //             msg.twist.twist.linear.z = vel(2);
    //         } else
    //         {
    //             msg.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
    //             msg.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
    //             msg.twist.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
    //         }

    //         if (((last_insnavgeod_.sb_list & 16) != 0) &&
    //             ((last_insnavgeod_.sb_list & 2) != 0) &&
    //             ((last_insnavgeod_.sb_list & 8) != 0))
    //         {
    //             Eigen::Matrix3d covVel_local = Eigen::Matrix3d::Zero();
    //             if ((last_insnavgeod_.sb_list & 128) != 0)
    //             {
    //                 // Linear velocity covariance
    //                 if (validValue(last_insnavgeod_.ve_std_dev))
    //                     if (settings_->use_ros_axis_orientation)
    //                         covVel_local(0, 0) = square(last_insnavgeod_.ve_std_dev);
    //                     else
    //                         covVel_local(1, 1) = square(last_insnavgeod_.ve_std_dev);
    //                 else
    //                     covVel_local(0, 0) = -1.0;
    //                 if (validValue(last_insnavgeod_.vn_std_dev))
    //                     if (settings_->use_ros_axis_orientation)
    //                         covVel_local(1, 1) = square(last_insnavgeod_.vn_std_dev);
    //                     else
    //                         covVel_local(0, 0) = square(last_insnavgeod_.vn_std_dev);
    //                 else
    //                     covVel_local(1, 1) = -1.0;
    //                 if (validValue(last_insnavgeod_.vu_std_dev))
    //                     covVel_local(2, 2) = square(last_insnavgeod_.vu_std_dev);
    //                 else
    //                     covVel_local(2, 2) = -1.0;

    //                 if (validValue(last_insnavgeod_.ve_vn_cov))
    //                     covVel_local(0, 1) = covVel_local(1, 0) =
    //                         last_insnavgeod_.ve_vn_cov;
    //                 if (settings_->use_ros_axis_orientation)
    //                 {
    //                     if (validValue(last_insnavgeod_.ve_vu_cov))
    //                         covVel_local(0, 2) = covVel_local(2, 0) =
    //                             last_insnavgeod_.ve_vu_cov;
    //                     if (validValue(last_insnavgeod_.vn_vu_cov))
    //                         covVel_local(2, 1) = covVel_local(1, 2) =
    //                             last_insnavgeod_.vn_vu_cov;
    //                 } else
    //                 {
    //                     if (validValue(last_insnavgeod_.vn_vu_cov))
    //                         covVel_local(0, 2) = covVel_local(2, 0) =
    //                             -last_insnavgeod_.vn_vu_cov;
    //                     if (validValue(last_insnavgeod_.ve_vu_cov))
    //                         covVel_local(2, 1) = covVel_local(1, 2) =
    //                             -last_insnavgeod_.ve_vu_cov;
    //                 }
    //             } else
    //             {
    //                 covVel_local(0, 0) = -1.0;
    //                 covVel_local(1, 1) = -1.0;
    //                 covVel_local(2, 2) = -1.0;
    //             }

    //             msg.twist.covariance[0] = covVel_local(0, 0);
    //             msg.twist.covariance[1] = covVel_local(0, 1);
    //             msg.twist.covariance[2] = covVel_local(0, 2);
    //             msg.twist.covariance[6] = covVel_local(1, 0);
    //             msg.twist.covariance[7] = covVel_local(1, 1);
    //             msg.twist.covariance[8] = covVel_local(1, 2);
    //             msg.twist.covariance[12] = covVel_local(2, 0);
    //             msg.twist.covariance[13] = covVel_local(2, 1);
    //             msg.twist.covariance[14] = covVel_local(2, 2);
    //         } else
    //         {
    //             msg.twist.covariance[0] = -1.0;
    //             msg.twist.covariance[7] = -1.0;
    //             msg.twist.covariance[14] = -1.0;
    //         }

    //         publish<TwistWithCovarianceStampedMsg>("twist_ins", msg);
    //     } else
    //     {
    //         if ((!validValue(last_pvtgeodetic_.block_header.tow)) ||
    //             (last_pvtgeodetic_.block_header.tow !=
    //              last_velcovgeodetic_.block_header.tow))
    //             return;
    //         msg.header = last_pvtgeodetic_.header;

    //         if (last_pvtgeodetic_.error == 0)
    //         {
    //             // Linear velocity in navigation frame
    //             double ve = last_pvtgeodetic_.ve;
    //             double vn = last_pvtgeodetic_.vn;
    //             double vu = last_pvtgeodetic_.vu;
    //             Eigen::Vector3d vel;
    //             if (settings_->use_ros_axis_orientation)
    //             {
    //                 // (ENU)
    //                 vel << ve, vn, vu;
    //             } else
    //             {
    //                 // (NED)
    //                 vel << vn, ve, -vu;
    //             }
    //             // Linear velocity
    //             msg.twist.twist.linear.x = vel(0);
    //             msg.twist.twist.linear.y = vel(1);
    //             msg.twist.twist.linear.z = vel(2);
    //         } else
    //         {
    //             msg.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
    //             msg.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();
    //             msg.twist.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
    //         }

    //         if (last_velcovgeodetic_.error == 0)
    //         {
    //             Eigen::Matrix3d covVel_local = Eigen::Matrix3d::Zero();
    //             // Linear velocity covariance in navigation frame
    //             if (validValue(last_velcovgeodetic_.cov_veve))
    //                 if (settings_->use_ros_axis_orientation)
    //                     covVel_local(0, 0) = last_velcovgeodetic_.cov_veve;
    //                 else
    //                     covVel_local(1, 1) = last_velcovgeodetic_.cov_veve;
    //             else
    //                 covVel_local(0, 0) = -1.0;
    //             if (validValue(last_velcovgeodetic_.cov_vnvn))
    //                 if (settings_->use_ros_axis_orientation)
    //                     covVel_local(1, 1) = last_velcovgeodetic_.cov_vnvn;
    //                 else
    //                     covVel_local(0, 0) = last_velcovgeodetic_.cov_vnvn;
    //             else
    //                 covVel_local(1, 1) = -1.0;
    //             if (validValue(last_velcovgeodetic_.cov_vuvu))
    //                 covVel_local(2, 2) = last_velcovgeodetic_.cov_vuvu;
    //             else
    //                 covVel_local(2, 2) = -1.0;

    //             covVel_local(0, 1) = covVel_local(1, 0) =
    //                 last_velcovgeodetic_.cov_vnve;
    //             if (settings_->use_ros_axis_orientation)
    //             {
    //                 if (validValue(last_velcovgeodetic_.cov_vevu))
    //                     covVel_local(0, 2) = covVel_local(2, 0) =
    //                         last_velcovgeodetic_.cov_vevu;
    //                 if (validValue(last_velcovgeodetic_.cov_vnvu))
    //                     covVel_local(2, 1) = covVel_local(1, 2) =
    //                         last_velcovgeodetic_.cov_vnvu;
    //             } else
    //             {
    //                 if (validValue(last_velcovgeodetic_.cov_vnvu))
    //                     covVel_local(0, 2) = covVel_local(2, 0) =
    //                         -last_velcovgeodetic_.cov_vnvu;
    //                 if (validValue(last_velcovgeodetic_.cov_vevu))
    //                     covVel_local(2, 1) = covVel_local(1, 2) =
    //                         -last_velcovgeodetic_.cov_vevu;
    //             }

    //             msg.twist.covariance[0] = covVel_local(0, 0);
    //             msg.twist.covariance[1] = covVel_local(0, 1);
    //             msg.twist.covariance[2] = covVel_local(0, 2);
    //             msg.twist.covariance[6] = covVel_local(1, 0);
    //             msg.twist.covariance[7] = covVel_local(1, 1);
    //             msg.twist.covariance[8] = covVel_local(1, 2);
    //             msg.twist.covariance[12] = covVel_local(2, 0);
    //             msg.twist.covariance[13] = covVel_local(2, 1);
    //             msg.twist.covariance[14] = covVel_local(2, 2);
    //         } else
    //         {
    //             msg.twist.covariance[0] = -1.0;
    //             msg.twist.covariance[7] = -1.0;
    //             msg.twist.covariance[14] = -1.0;
    //         }

    //         publish<TwistWithCovarianceStampedMsg>("twist_gnss", msg);
    //     }
    // };

    /// If the current time shall be employed, it is calculated via the time(NULL)
    /// function found in the \<ctime\> library At the time of writing the code
    /// (2020), the GPS time was ahead of UTC time by 18 (leap) seconds. Adapt the
    /// settings_->leap_seconds ROSaic parameter accordingly as soon as the
    /// next leap second is inserted into the UTC time.
    Timestamp MessageHandler::timestampHuace(float tow, uint32_t wnc) const
    {
        Timestamp time_obj;

        // conversion from GPS time of week and week number to UTC taking leap
        // seconds into account
        constexpr uint64_t secToNSec = 1000000000;
        constexpr uint64_t mSec2NSec = 1000000;
        constexpr uint64_t nsOfGpsStart =
            315964800 *
            secToNSec; // GPS week counter starts at 1980-01-06 which is
                       // 315964800 seconds since Unix epoch (1970-01-01 UTC)
        constexpr uint64_t nsecPerWeek = 7 * 24 * 60 * 60 * secToNSec;

        time_obj = nsOfGpsStart + tow * secToNSec + wnc * nsecPerWeek;

        if (current_leap_seconds_ != -128)
            time_obj -= current_leap_seconds_ * secToNSec;
        // else: warn?

        return time_obj;
    }

    /**
     * If GNSS time is used, Publishing is only done with valid leap seconds
     */
    template <typename M>
    void MessageHandler::publish(const std::string& topic, const M& msg)
    {
        // TODO: maybe publish only if wnc and tow is valid?
        if (!settings_->use_gnss_time ||
            (settings_->use_gnss_time && (current_leap_seconds_ != -128)))
        {
            // if (settings_->read_from_sbf_log || settings_->read_from_pcap)
            // {
            //     wait(timestampFromRos(msg.header.stamp));
            // }
            node_->publishMessage<M>(topic, msg);
        } else
        {
            node_->log(
                log_level::DEBUG,
                "Not publishing message with GNSS time because no leap seconds are available yet.");
        }
    }

    /**
     * If GNSS time is used, Publishing is only done with valid leap seconds
     */
    void MessageHandler::publishTf(const LocalizationMsg& msg)
    {
        // TODO: maybe publish only if wnc and tow is valid?
        if (!settings_->use_gnss_time ||
            (settings_->use_gnss_time && (current_leap_seconds_ != -128)))
        {
            node_->publishTf(msg);
        } else
        {
            node_->log(
                log_level::DEBUG,
                "Not publishing tf with GNSS time because no leap seconds are available yet.");
        }
    }

    void MessageHandler::wait(Timestamp time_obj)
    {
        Timestamp unix_old = unix_time_;
        unix_time_ = time_obj;
        if ((unix_old != 0) && (unix_time_ > unix_old))
        {
            auto sleep_nsec = unix_time_ - unix_old;

            std::stringstream ss;
            ss << "Waiting for " << sleep_nsec / 1000000 << " milliseconds...";
            node_->log(log_level::DEBUG, ss.str());

            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_nsec));
        }
    }

    void MessageHandler::parseNmea(const std::shared_ptr<Telegram>& telegram)
    {
        std::string message(telegram->message.begin(), telegram->message.end());
        /*node_->log(
          LogLevel::DEBUG,
          "The NMEA message contains " + std::to_string(message.size()) +
              " bytes and is ready to be parsed. It reads: " + message);*/
        boost::char_separator<char> sep_2(",*", "", boost::keep_empty_tokens);
        boost::tokenizer<boost::char_separator<char>> tokens(message, sep_2);
        std::vector<std::string> body;
        body.reserve(50);
        for (boost::tokenizer<boost::char_separator<char>>::iterator tok_iter =
                 tokens.begin();
             tok_iter != tokens.end(); ++tok_iter)
        {
            body.push_back(*tok_iter);
        }

        std::string id(body[0].begin() + 1, body[0].end());

        auto it = nmeaMap_.find(body[0]);
        if (it != nmeaMap_.end())
        {
            switch (it->second)
            {
            case 0:
            {
                // Create NmeaSentence struct to pass to GpggaParser::parseASCII
                NMEASentence gga_message(id, body);
                GpggaMsg msg;
                GpggaParser parser_obj;
                try
                {
                    msg = parser_obj.parseASCII(gga_message, settings_->frame_id,
                                                settings_->use_gnss_time,
                                                telegram->stamp);
                } catch (ParseException& e)
                {
                    node_->log(log_level::DEBUG,
                               "GpggaMsg: " + std::string(e.what()));
                    break;
                }
                publish<GpggaMsg>("gpgga", msg);
                break;
            }
            case 1:
            {
                // Create NmeaSentence struct to pass to GprmcParser::parseASCII
                NMEASentence rmc_message(id, body);
                GprmcMsg msg;
                GprmcParser parser_obj;
                try
                {
                    msg = parser_obj.parseASCII(rmc_message, settings_->frame_id,
                                                settings_->use_gnss_time,
                                                telegram->stamp);
                } catch (ParseException& e)
                {
                    node_->log(log_level::DEBUG,
                               "GprmcMsg: " + std::string(e.what()));
                    break;
                }
                publish<GprmcMsg>("gprmc", msg);
                break;
            }
            // case 2:
            // {
            //     // Create NmeaSentence struct to pass to GpgsaParser::parseASCII
            //     NMEASentence gsa_message(id, body);
            //     GpgsaMsg msg;
            //     GpgsaParser parser_obj;
            //     try
            //     {
            //         msg = parser_obj.parseASCII(gsa_message, settings_->frame_id,
            //                                     settings_->use_gnss_time,
            //                                     node_->getTime());
            //     } catch (ParseException& e)
            //     {
            //         node_->log(log_level::DEBUG,
            //                    "GpgsaMsg: " + std::string(e.what()));
            //         break;
            //     }
            //     if (settings_->use_gnss_time)
            //     {
            //         if (settings_->huace_receiver_type == "gnss")
            //         {
            //             Timestamp time_obj =
            //                 timestampSBF(last_pvtgeodetic_.block_header.tow,
            //                              last_pvtgeodetic_.block_header.wnc);
            //             msg.header.stamp = timestampToRos(time_obj);
            //         }
            //         if (settings_->huace_receiver_type == "ins")
            //         {
            //             Timestamp time_obj =
            //                 timestampSBF(last_insnavgeod_.block_header.tow,
            //                              last_insnavgeod_.block_header.wnc);
            //             msg.header.stamp = timestampToRos(time_obj);
            //         }
            //     } else
            //         msg.header.stamp = timestampToRos(telegram->stamp);
            //     publish<GpgsaMsg>("gpgsa", msg);
            //     break;
            // }
            // case 3:
            // {
            //     // Create NmeaSentence struct to pass to GpgsvParser::parseASCII
            //     NMEASentence gsv_message(id, body);
            //     GpgsvMsg msg;
            //     GpgsvParser parser_obj;
            //     try
            //     {
            //         msg = parser_obj.parseASCII(gsv_message, settings_->frame_id,
            //                                     settings_->use_gnss_time,
            //                                     node_->getTime());
            //     } catch (ParseException& e)
            //     {
            //         node_->log(log_level::DEBUG,
            //                    "GpgsvMsg: " + std::string(e.what()));
            //         break;
            //     }
            //     if (settings_->use_gnss_time)
            //     {

            //         if (settings_->huace_receiver_type == "gnss")
            //         {
            //             Timestamp time_obj =
            //                 timestampSBF(last_pvtgeodetic_.block_header.tow,
            //                              last_pvtgeodetic_.block_header.wnc);
            //             msg.header.stamp = timestampToRos(time_obj);
            //         }
            //         if (settings_->huace_receiver_type == "ins")
            //         {
            //             Timestamp time_obj =
            //                 timestampSBF(last_insnavgeod_.block_header.tow,
            //                              last_insnavgeod_.block_header.wnc);
            //             msg.header.stamp = timestampToRos(time_obj);
            //         }
            //     } else
            //         msg.header.stamp = timestampToRos(telegram->stamp);
            //     publish<GpgsvMsg>("gpgsv", msg);
            //     break;
            // }
            case 4:
            {
                // Create NmeaSentence struct to pass to GpgsaParser::parseASCII
                NMEASentence chc_message(id, body);
                GpchcMsg msg;
                GpchcParser parser_obj;
                try
                {
                    msg = parser_obj.parseASCII(chc_message, settings_->frame_id,
                                                settings_->use_gnss_time,
                                                node_->getTime());
                } catch (ParseException& e)
                {
                    node_->log(log_level::DEBUG,
                               "GpchcMsg: " + std::string(e.what()));
                    break;
                }
                if (settings_->use_gnss_time)
                {
                    Timestamp time_obj = timestampHuace(msg.tow, msg.wnc);
                    msg.header.stamp = timestampToRos(time_obj);
                } else
                    msg.header.stamp = timestampToRos(telegram->stamp);
                publish<GpchcMsg>("gpchc", msg);
                break;
            }
            case 5:
            {
                // Create NmeaSentence struct to pass to GpgsaParser::parseASCII
                NMEASentence chcx_message(id, body);
                GpchcxMsg msg;
                GpchcxParser parser_obj;
                try
                {
                    msg = parser_obj.parseASCII(chcx_message, settings_->frame_id,
                                                settings_->use_gnss_time,
                                                node_->getTime());
                } catch (ParseException& e)
                {
                    node_->log(log_level::DEBUG, "GpchcxMsg: " + std::string(e.what()));
                    break;
                }
                if (settings_->use_gnss_time)
                {
                    Timestamp time_obj = timestampHuace(msg.tow, msg.wnc);
                    msg.header.stamp = timestampToRos(time_obj);
                } else
                    msg.header.stamp = timestampToRos(telegram->stamp);
                publish<GpchcxMsg>("gpchcx", msg);
                break;
            }

            }
        } else
        {
            node_->log(log_level::DEBUG, "Unknown NMEA message: " + body[0]);
        }
    }

} // namespace io
