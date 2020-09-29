/*
 *tag: Sergio Portoles  Thu Aug 06 14:30:00 CEST 2020  ros_param_data_source.cpp
 *
 *        This file is part of the OROCOS toolchain ROS project
 *
 *                  (C) 2020, Intermodalics BVBA
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "rtt_rosparam/ros_param_data_source.hpp"

namespace rtt_rosparam {

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

} // namespace rtt_rosparam

/*
 * Explicit template instantiation for core data source types
 */
template class rtt_rosparam::RosParamDataSource< bool >;
template class rtt_rosparam::RosParamDataSource< std::string >;


// A possible application: new addRosParamProperty()
//
// /**
//  * Adds a ROS parameter of types supported by ROS param as a property to this
//  * bag.
//  * A Property is created which causes contents of the property always to be
//  * in sync with the contents of the ROS parameter.
//  * Supported ROS parameter types:
//  * bool, int, std::string, double, float and its respective vector types.
//  * @param name The name of this property
//  * @return the Property object by reference, which you can further query or document.
//  */
// template<class T>
// Property<T>& rtt_rosparam::addRosParamProperty( const std::string& name) {
//   typename internal::AssignableDataSource<T>::shared_ptr datasource( new RTT::rosparam::RosParamDataSource<T>(name) );
//     Property<T>* p = new Property<T>(name,"", datasource);
//     this->ownProperty( p );
//     return *p;
// }
