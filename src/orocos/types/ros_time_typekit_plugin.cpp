/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:19:02 CET 2010  ros_time_typekit_plugin.cpp

                        ros_time_typekit_plugin.cpp -  description
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


#include <ros/time.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>

namespace ros_integration {
  using namespace RTT;
   
    /**
     * This interface defines the types of the realTime package.
     */
    class ROStimeTypekitPlugin
      : public types::TypekitPlugin
    {
    public:
      virtual std::string getName(){
	return std::string("ros-")+"time";
      }

      virtual bool loadTypes() {
	RTT::types::Types()->addType( new types::TemplateTypeInfo<ros::Time>("time") );
	return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROStimeTypekitPlugin )

