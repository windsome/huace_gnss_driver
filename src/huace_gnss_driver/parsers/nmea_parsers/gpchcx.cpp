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

#include <huace_gnss_driver/parsers/nmea_parsers/gpchcx.hpp>

/**
 * @file gpchcx.cpp
 * @brief Derived class for parsing CHCX messages
 * @date 17/08/20
 */

const std::string GpchcxParser::MESSAGE_ID = "$GPCHCX";

const std::string GpchcxParser::getMessageID() const
{
    return GpchcxParser::MESSAGE_ID;
}

/**
 * Caution: Due to the occurrence of the throw keyword, this method parseASCII should
 * be called within a try / catch framework... Note: This method is called from
 * within the read() method of the RxMessage class by including the checksum part in
 * the argument "sentence" here, though the checksum is never parsed: It would be
 * sentence.get_body()[15] if anybody ever needs it.
 */
GpchcxMsg GpchcxParser::parseASCII(const NMEASentence& sentence,
                                 const std::string& frame_id, bool use_gnss_time,
                                 Timestamp time_obj) noexcept(false)
{
    // ROS_DEBUG("Just testing that first entry is indeed what we expect it to be:
    // %s", sentence.get_body()[0].c_str());
    // Check the length first, which should be 16 elements.
    const size_t LEN = 47;
    if (sentence.get_body().size() > LEN || sentence.get_body().size() < LEN)
    {
        std::stringstream error;
        error << "CHCX parsing failed: Expected GPCHCX length is " << LEN
              << ", but actual length is " << sentence.get_body().size();
        throw ParseException(error.str());
    }

    GpchcxMsg msg;
    msg.header.frame_id = frame_id;

    msg.message_id = sentence.get_body()[0];

  bool valid = true;

  valid = valid && string_utilities::toUInt32(sentence.get_body()[1], msg.wnc);
  valid = valid && string_utilities::toFloat(sentence.get_body()[2], msg.tow);

  valid = valid && string_utilities::toFloat(sentence.get_body()[3], msg.heading2);
  valid = valid && string_utilities::toFloat(sentence.get_body()[4], msg.pitch);
  valid = valid && string_utilities::toFloat(sentence.get_body()[5], msg.roll);

  valid = valid && string_utilities::toFloat(sentence.get_body()[6], msg.gyrox);
  valid = valid && string_utilities::toFloat(sentence.get_body()[7], msg.gyroy);
  valid = valid && string_utilities::toFloat(sentence.get_body()[8], msg.gyroz);
  valid = valid && string_utilities::toFloat(sentence.get_body()[9], msg.accx);
  valid = valid && string_utilities::toFloat(sentence.get_body()[10], msg.accy);
  valid = valid && string_utilities::toFloat(sentence.get_body()[11], msg.accz);

  valid = valid && string_utilities::toDouble(sentence.get_body()[12], msg.latitude);
  valid = valid && string_utilities::toDouble(sentence.get_body()[13], msg.longitude);
  valid = valid && string_utilities::toDouble(sentence.get_body()[14], msg.altitude);

  valid = valid && string_utilities::toFloat(sentence.get_body()[15], msg.ve);
  valid = valid && string_utilities::toFloat(sentence.get_body()[16], msg.vn);
  valid = valid && string_utilities::toFloat(sentence.get_body()[17], msg.vu);
  valid = valid && string_utilities::toFloat(sentence.get_body()[18], msg.v2d);

  valid = valid && string_utilities::toUInt32(sentence.get_body()[19], msg.nsv1);
  valid = valid && string_utilities::toUInt32(sentence.get_body()[20], msg.nsv2);

  valid = valid && string_utilities::toUInt32(sentence.get_body()[21], msg.status);
  valid = valid && string_utilities::toUInt32(sentence.get_body()[22], msg.age);
  valid = valid && string_utilities::toUInt32(sentence.get_body()[23], msg.warning);

  valid = valid && string_utilities::toFloat(sentence.get_body()[24], msg.latitude_std);
  valid = valid && string_utilities::toFloat(sentence.get_body()[25], msg.longitude_std);
  valid = valid && string_utilities::toFloat(sentence.get_body()[26], msg.altitude_std);

  valid = valid && string_utilities::toFloat(sentence.get_body()[27], msg.ve_std);
  valid = valid && string_utilities::toFloat(sentence.get_body()[28], msg.vn_std);
  valid = valid && string_utilities::toFloat(sentence.get_body()[29], msg.vu_std);

  valid = valid && string_utilities::toFloat(sentence.get_body()[30], msg.roll_std);
  valid = valid && string_utilities::toFloat(sentence.get_body()[31], msg.pitch_std);
  valid = valid && string_utilities::toFloat(sentence.get_body()[32], msg.heading2_std);

  msg.separator_x = sentence.get_body()[33];

  valid = valid && string_utilities::toFloat(sentence.get_body()[34], msg.heading);
  valid = valid && string_utilities::toFloat(sentence.get_body()[35], msg.heading_std);

  valid = valid && string_utilities::toFloat(sentence.get_body()[36], msg.ins2gnss_vector_x);
  valid = valid && string_utilities::toFloat(sentence.get_body()[37], msg.ins2gnss_vector_y);
  valid = valid && string_utilities::toFloat(sentence.get_body()[38], msg.ins2gnss_vector_z);

  valid = valid && string_utilities::toFloat(sentence.get_body()[39], msg.ins2body_angle_x);
  valid = valid && string_utilities::toFloat(sentence.get_body()[40], msg.ins2body_angle_y);
  valid = valid && string_utilities::toFloat(sentence.get_body()[41], msg.ins2body_angle_z);

  valid = valid && string_utilities::toFloat(sentence.get_body()[42], msg.gnss2body_angle_z);

  valid = valid && string_utilities::toUInt32(sentence.get_body()[43], msg.vis_nsv1);
  valid = valid && string_utilities::toUInt32(sentence.get_body()[44], msg.vis_nsv2);

  msg.sn = sentence.get_body()[45];

    if (!valid)
    {
        was_last_gpchcx_valid_ = false;
        throw ParseException("GPCHCX message was invalid.");
    }

    // If we made it this far, we successfully parsed the message and will consider
    // it to be valid.
    was_last_gpchcx_valid_ = true;

    return msg;
}

bool GpchcxParser::wasLastGPCHCXValid() const { return was_last_gpchcx_valid_; }
