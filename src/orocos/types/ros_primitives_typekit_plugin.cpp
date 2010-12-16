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
#include <rtt/types/SequenceTypeInfo.hpp>

namespace ros_integration {
  using namespace RTT;
   
  /**
   * This interface defines the primitive ROS msg types to the realTime package.
   */
  class ROSPrimitivesTypekitPlugin
    : public types::TypekitPlugin
  {
    public:
      virtual std::string getName(){
	      return std::string("ros-")+"primitives";
      }
 
      virtual bool loadTypes() {
	      RTT::types::Types()->addType( new types::TemplateTypeInfo<ros::Time>("time") );
        RTT::types::Types()->addType( new types::StdTypeInfo<uint8_t>("uint8") );
        RTT::types::Types()->addType( new types::StdTypeInfo<int8_t>("int8") );
        RTT::types::Types()->addType( new types::StdTypeInfo<int16_t>("int16") );
        RTT::types::Types()->addType( new types::StdTypeInfo<uint16_t>("uint16") );
        RTT::types::Types()->addType( new types::StdTypeInfo<int32_t>("int32") );
        RTT::types::Types()->addType( new types::StdTypeInfo<uint32_t>("uint32") );
        RTT::types::Types()->addType( new types::StdTypeInfo<int64_t>("int64") );
        RTT::types::Types()->addType( new types::StdTypeInfo<uint64_t>("uint64") );
        RTT::types::Types()->addType( new types::StdTypeInfo<float>("float32") );
        RTT::types::Types()->addType( new types::StdTypeInfo<double>("float64") );
        RTT::types::Types()->addType( new types::StdStringTypeInfo("string") );
	      return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSPrimitivesTypekitPlugin )

