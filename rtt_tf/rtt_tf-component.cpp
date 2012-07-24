/* (C) 2011 Ruben Smits, ruben.smits@mech.kuleuven.be, Department of Mechanical
 * Engineering, Katholieke Universiteit Leuven, Belgium.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rtt_tf-component.hpp"
#include <rtt/Component.hpp>

namespace rtt_tf
{

  using namespace RTT;
  using namespace tf;

  RTT_TF::RTT_TF(const std::string& name) :
    TaskContext(name, PreOperational), prop_interpolating(true),
    prop_cache_time(Transformer::DEFAULT_CACHE_TIME),
    prop_buffer_size(DEFAULT_BUFFER_SIZE)
  {
    this->addProperty("interpolating", prop_interpolating);
    this->addProperty("cache_time", prop_cache_time);
    this->addProperty("buffer_size", prop_buffer_size);
    this->addEventPort("tf_in", port_tf_in);
    this->addPort("tf_out", port_tf_out);

    this->addOperation("lookupTransform", &RTT_TF::lookupTransform, this)
      .doc("lookup the most recent transform from source to target")
      .arg("target", "target frame")
      .arg("source", "source frame");

    this->addOperation("broadcastTransform", &RTT_TF::broadcastTransform, this, RTT::OwnThread)
      .doc("lookup the most recent transform from source to target")
      .arg("stamped transform", "geometry_msgs::TransformStamped");

    this->provides("tf")->addOperation("lookupTransform", &RTT_TF::lookupTransform, this)
      .doc("lookup the most recent transform from source to target")
      .arg("target", "target frame")
      .arg("source", "source frame");

    this->provides("tf")->addOperation("broadcastTransform", &RTT_TF::broadcastTransform, this, RTT::OwnThread)
      .doc("lookup the most recent transform from source to target")
      .arg("stamped transform", "geometry_msgs::TransformStamped");
  }

  bool RTT_TF::configureHook()
  {
    m_transformer.reset(new Transformer(prop_interpolating, ros::Duration(
            prop_cache_time)));

    ConnPolicy cp = ConnPolicy::buffer(prop_buffer_size);
    cp.transport = 3; //3=ROS
    cp.name_id = "/tf";

    return (port_tf_in.createStream(cp) && port_tf_out.createStream(cp));
  }

  void RTT_TF::updateHook()
  {
    Logger::In(this->getName());
#ifndef NDEBUG
    log(Debug) << "In update" << endlog();
#endif
    try
    {
      tf::tfMessage msg_in;
      while (port_tf_in.read(msg_in) == NewData)
      {
        for (unsigned int i = 0; i < msg_in.transforms.size(); i++)
        {
          StampedTransform trans;
          transformStampedMsgToTF(msg_in.transforms[i], trans);
          try
          {
            std::map<std::string, std::string>* msg_header_map =
              msg_in.__connection_header.get();
            std::string authority;
            std::map<std::string, std::string>::iterator it =
              msg_header_map->find("callerid");
            if (it == msg_header_map->end())
            {
              log(Warning) << "Message received without callerid"
                << endlog();
              authority = "no callerid";
            }
            else
            {
              authority = it->second;
            }
            m_transformer->setTransform(trans, authority);
          } catch (TransformException& ex)
          {

            log(Error) << "Failure to set received transform from "
              << msg_in.transforms[i].child_frame_id << " to "
              << msg_in.transforms[i].header.frame_id
              << " with error: " << ex.what() << endlog();
          }
        }
      }

    } catch (std::exception& ex)
    {
      log(Error) << ex.what() << endlog();
    }
  }

  geometry_msgs::TransformStamped RTT_TF::lookupTransform(
      const std::string& target,
      const std::string& source)
  {
    tf::StampedTransform stamped_tf;
    ros::Time common_time;
    m_transformer->getLatestCommonTime(source, target, common_time,NULL);
    m_transformer->lookupTransform(target, source, common_time, stamped_tf);
    geometry_msgs::TransformStamped msg;
    tf::transformStampedTFToMsg(stamped_tf,msg);
    return msg;
  }

  void RTT_TF::broadcastTransform(
      const geometry_msgs::TransformStamped &tform)
  {
    tf::tfMessage msg_out;
    msg_out.transforms.push_back(tform);
    port_tf_out.write(msg_out);
  }

}//namespace

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use 
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Rtt_tf)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(rtt_tf::RTT_TF)
