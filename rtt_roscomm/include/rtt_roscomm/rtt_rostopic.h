/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:18:49 CET 2010  rtt_rostopic.h

                        rtt_rostopic.h -  description
                           -------------------
    begin                : Tue November 16 2010
    copyright            : (C) 2010 Ruben Smits
    email                : first.last@mech.kuleuven.be

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef __RTT_ROSCOMM_RTT_ROSTOPIC_H
#define __RTT_ROSCOMM_RTT_ROSTOPIC_H

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

#endif // ifndef __RTT_ROSCOMM_RTT_ROSTOPIC_H
