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

#include <huace_gnss_driver/parsers/nmea_parsers/gpchc.hpp>
#include <huace_gnss_driver/parsers/string_utilities.hpp>

/**
 * @file gpchc.cpp
 * @brief Derived class for parsing CHC messages
 * @date 17/08/20
 */

const std::string GpchcParser::MESSAGE_ID = "$GPCHC";

const std::string GpchcParser::getMessageID() const
{
  return GpchcParser::MESSAGE_ID;
}

/**
 * Caution: Due to the occurrence of the throw keyword, this method parseASCII should
 * be called within a try / catch framework... Note: This method is called from
 * within the read() method of the RxMessage class by including the checksum part in
 * the argument "sentence" here, though the checksum is never parsed: It would be
 * sentence.get_body()[15] if anybody ever needs it.
 */
GpchcMsg GpchcParser::parseASCII(
  const NMEASentence & sentence, const std::string & frame_id, bool use_gnss_time,
  Timestamp time_obj) noexcept(false)
{
  // ROS_DEBUG("Just testing that first entry is indeed what we expect it to be:
  // %s", sentence.get_body()[0].c_str());
  // Check the length first, which should be 16 elements.
  const size_t LEN = 25;
  if (sentence.get_body().size() > LEN || sentence.get_body().size() < LEN) {
    std::stringstream error;
    error << "CHC parsing failed: Expected GPCHC length is " << LEN << ", but actual length is "
          << sentence.get_body().size();
    throw ParseException(error.str());
  }

  GpchcMsg msg;
  msg.header.frame_id = frame_id;

  msg.message_id = sentence.get_body()[0];

  bool valid = true;

  valid = valid && string_utilities::toUInt32(sentence.get_body()[1], msg.wnc);
  valid = valid && string_utilities::toFloat(sentence.get_body()[2], msg.tow);
  
  valid = valid && string_utilities::toFloat(sentence.get_body()[3], msg.heading);
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
  valid = valid && string_utilities::toFloat(sentence.get_body()[18], msg.v);

  valid = valid && string_utilities::toUInt32(sentence.get_body()[19], msg.nsv1);
  valid = valid && string_utilities::toUInt32(sentence.get_body()[20], msg.nsv2);

  valid = valid && string_utilities::toUInt32(sentence.get_body()[21], msg.status);
  valid = valid && string_utilities::toUInt32(sentence.get_body()[22], msg.age);
  valid = valid && string_utilities::toUInt32(sentence.get_body()[23], msg.warning);

  if (!valid) {
    was_last_gpchc_valid_ = false;
    throw ParseException("GPCHC message was invalid.");
  }

  // If we made it this far, we successfully parsed the message and will consider
  // it to be valid.
  was_last_gpchc_valid_ = true;

  return msg;
}

bool GpchcParser::wasLastGPCHCValid() const
{
  return was_last_gpchc_valid_;
}
