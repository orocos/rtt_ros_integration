/***************************************************************************
  tag: Sergio Portoles  Thu Aug 06 14:30:00 CEST 2020  RosParamDataSource.cpp

                        RosParamDataSource.cpp -  description
                           -------------------
    begin                : Thu Aug 06 2020
    copyright            : (C) 2020 Peter Soetens
    email                : peter.soetens@fmtc.be

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU General Public                   *
 *   License as published by the Free Software Foundation;                 *
 *   version 2 of the License.                                             *
 *                                                                         *
 *   As a special exception, you may use this file as part of a free       *
 *   software library without restriction.  Specifically, if other files   *
 *   instantiate templates or use macros or inline functions from this     *
 *   file, or you compile this file and link it with other files to        *
 *   produce an executable, this file does not by itself cause the         *
 *   resulting executable to be covered by the GNU General Public          *
 *   License.  This exception does not however invalidate any other        *
 *   reasons why the executable file might be covered by the GNU General   *
 *   Public License.                                                       *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public             *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/


#include "rtt_rosparam/RosParamDataSource.hpp"

namespace RTT {
  namespace rosparam { 

  // /**
  //  * Specialisation for std::string to keep capacity when set( ... ) is called.
  //  */
  // template<>
  // void RosParamDataSource<std::string>::set( AssignableDataSource<std::string>::param_t t )
  // {
  //     mdata = t.c_str();
  // }

  // /**
  //  * Specialisation for std::string to keep capacity when clone() is called.
  //  */
  // template<>
  // RosParamDataSource<std::string>::RosParamDataSource( std::string t )
  //     : mdata( t.c_str() )
  // {
  // }

  } // namespace rosparam
} // namespace RTT

/*
 * Explicit template instantiation for core data source types
 */
template class RTT::rosparam::RosParamDataSource< bool >;
template class RTT::rosparam::RosParamDataSource< std::string >;


// A possible application: new addRosParamProperty()
//
// /**
//  * Adds a ROS parameter of any type as a property to this bag.
//  * A Property is created which causes contents of the
//  * property always to be in sync
//  * with the contents of the ROS parameter.
//  * @param name The name of this property
//  * @return the Property object by reference, which you can further query or document.
//  */
// template<class T>
// Property<T>& addRosParamProperty( const std::string& name) {
//   typename internal::AssignableDataSource<T>::shared_ptr datasource( new RTT::rosparam::RosParamDataSource<T>(name) );
//     Property<T>* p = new Property<T>(name,"", datasource);
//     this->ownProperty( p );
//     return *p;
// }
