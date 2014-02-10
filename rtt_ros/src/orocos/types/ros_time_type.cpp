/******************************************************************************
*                 This file is part of the OROCOS toolchain ROS project       *
*                                                                             *
*        (C) 2010 Steven Bellens, steven.bellens@mech.kuleuven.be             *
*                    Department of Mechanical Engineering,                    *
*                   Katholieke Universiteit Leuven, Belgium.                  *
*                                                                             *
*       You may redistribute this software and/or modify it under either the  *
*       terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
*       <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
*       discretion) of the Modified BSD License:                              *
*       Redistribution and use in source and binary forms, with or without    *
*       modification, are permitted provided that the following conditions    *
*       are met:                                                              *
*       1. Redistributions of source code must retain the above copyright     *
*       notice, this list of conditions and the following disclaimer.         *
*       2. Redistributions in binary form must reproduce the above copyright  *
*       notice, this list of conditions and the following disclaimer in the   *
*       documentation and/or other materials provided with the distribution.  *
*       3. The name of the author may not be used to endorse or promote       *
*       products derived from this software without specific prior written    *
*       permission.                                                           *
*       THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
*       IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
*       WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
*       ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
*       INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
*       (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
*       OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
*       HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
*       STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
*       IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
*       POSSIBILITY OF SUCH DAMAGE.                                           *
*                                                                             *
*******************************************************************************/

#include "ros_primitives_typekit_plugin.hpp"

namespace ros_integration{
    using namespace RTT;
    using namespace RTT::types;

    // This class works around the ROS time representation.
    class RosTimeTypeInfo : public types::PrimitiveTypeInfo<ros::Time,false>
    {
    public:
        RosTimeTypeInfo() : types::PrimitiveTypeInfo<ros::Time,false>("time") {}

        virtual std::ostream& write( std::ostream& os, base::DataSourceBase::shared_ptr in ) const {
            internal::DataSource<ros::Time>::shared_ptr d = boost::dynamic_pointer_cast< internal::DataSource<ros::Time> >( in );
            if ( d ) {
                double tm = d->rvalue().sec + double(d->rvalue().nsec)/1000000000.0;
                os << tm;
            } else {
                std::string output = std::string("(")+ in->getTypeName() +")";
                os << output;
            }
            return os;
        }
    };

    // This class works around the ROS time representation.
    class RosDurationTypeInfo : public types::PrimitiveTypeInfo<ros::Duration,false>
    {
    public:
        RosDurationTypeInfo() : types::PrimitiveTypeInfo<ros::Duration,false>("duration") {}

        virtual std::ostream& write( std::ostream& os, base::DataSourceBase::shared_ptr in ) const {
            internal::DataSource<ros::Duration>::shared_ptr d = boost::dynamic_pointer_cast< internal::DataSource<ros::Duration> >( in );
            if ( d ) {
                double tm = d->rvalue().sec + double(d->rvalue().nsec)/1000000000.0;
                os << tm;
            } else {
                std::string output = std::string("(")+ in->getTypeName() +")";
                os << output;
            }
            return os;
        }
    };

  void loadTimeTypes(){
	     RTT::types::Types()->addType( new RosTimeTypeInfo );
	     RTT::types::Types()->addType( new RosDurationTypeInfo );
  }
}
