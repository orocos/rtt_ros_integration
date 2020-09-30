/*
 * (C) 2010 Ruben Smits, ruben.smits@mech.kuleuven.be
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __RTT_ROSCOMM_ROSTOPIC_H
#define __RTT_ROSCOMM_ROSTOPIC_H

#include <rtt/RTT.hpp>

#define ORO_ROS_PROTOCOL_ID 3

namespace rtt_roscomm {
  //! ROS topic protocol ID
  static const int protocol_id = 3;

  /**
   * Returns a ConnPolicy object for streaming to or from
   * the given ROS topic. No buffering is done within RTT.
   */
  RTT::ConnPolicy topic(const std::string& name);

  /**
   * Returns a ConnPolicy object for latched streaming to the
   * given ROS topic. No buffering is done within RTT.
   */
  RTT::ConnPolicy topicLatched(const std::string& name);

  /**
   * Returns a ConnPolicy object for streaming to or from
   * the given ROS topic. Also specifies the buffer size of
   * the connection to be created.
   */
  RTT::ConnPolicy topicBuffer(const std::string& name, int size);

  /**
   * Returns a ConnPolicy object for streaming to or from
   * the given ROS topic. Use this only for unbuffered
   * publishing, where the publish() method is called
   * in the thread of the writing TaskContext.
   */
  RTT::ConnPolicy topicUnbuffered(const std::string& name);
}

#endif // ifndef __RTT_ROSCOMM_ROSTOPIC_H
