/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:19:02 CET 2010  ros_publish_activity.cpp

                        ros_publish_activity.cpp -  description
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



#include <rtt_roscomm/rtt_rostopic_ros_publish_activity.hpp>

namespace rtt_roscomm {

    using namespace RTT;

    RosPublishActivity::weak_ptr RosPublishActivity::ros_pub_act;

    RosPublishActivity::RosPublishActivity( const std::string& name)
      : Activity(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0.0, 0, name)
    {
      Logger::In in("RosPublishActivity");
      log(Debug)<<"Creating RosPublishActivity"<<endlog();
    }

    void RosPublishActivity::loop(){
      os::MutexLock lock(publishers_lock);
      for(iterator it = publishers.begin(); it != publishers.end(); ++it) {
        (*it)->publish();
      }
    }

    RosPublishActivity::shared_ptr RosPublishActivity::Instance() {
      shared_ptr ret = ros_pub_act.lock();
      if ( !ret ) {
        ret.reset(new RosPublishActivity("RosPublishActivity"));
        ros_pub_act = ret;
        ret->start();
      }
      return ret;
    }

    void RosPublishActivity::addPublisher(RosPublisher* pub) {
      os::MutexLock lock(publishers_lock);
      publishers.insert(pub);
    }

    void RosPublishActivity::removePublisher(RosPublisher* pub) {
      os::MutexLock lock(publishers_lock);
      publishers.erase(pub);
    }

    RosPublishActivity::~RosPublishActivity() {
      Logger::In in("RosPublishActivity");
      log(Info) << "RosPublishActivity cleans up: no more work."<<endlog();
      stop();
    }

}
