/*
 * (C) 2020, Intermodalics BVBA
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

#ifndef __RTT_ROSPARAM__ROS_PARAM_DATA_SOURCE_HPP
#define __RTT_ROSPARAM__ROS_PARAM_DATA_SOURCE_HPP

#include "rtt/internal/DataSource.hpp"
#include "rtt/Logger.hpp"

#include "ros/param.h"

using namespace RTT;

namespace rtt_rosparam {

/**
 * A DataSource which checks the ROS parameter server and it
 * returns the parameter value in its get() method.
 * It sets the parameter value in its set() method.
 * The constructor receives the full qualified property name attached
 * to the DataSource.
 * <br><b>Warning</b>
 * The properties with a RosParamDataSource<T> are not real-time safe
 * and therefore this DataSource shouldn't be used in the updateHook()
 * of a real-time component.
 */
template<typename T>
class RosParamDataSource
    : public internal::AssignableDataSource<T>
{
  /**
   * Stores the full-qualified property name of the ROS parameter
   */
  std::string mparam_name_;
  mutable typename internal::DataSource<T>::value_t mcached_data_;

public:

  typedef boost::intrusive_ptr< RosParamDataSource<T> > shared_ptr;

  ~RosParamDataSource()
  {
  }

  RosParamDataSource(std::string param_name_id)
      : mparam_name_(param_name_id),
        mcached_data_()
  {
  }

  typename internal::DataSource<T>::result_t get() const
  {
    (void) evaluate();
    return value();
  }

  bool evaluate() const
  {
    if (!ros::param::getCached(mparam_name_, mcached_data_)) {
      RTT::log(RTT::Error) << "The value of parameter " + mparam_name_ + " could not be fetched." << RTT::endlog();
      return false;
    }
    return true;
  }

  typename internal::DataSource<T>::result_t value() const
  {
    return mcached_data_;
  }

  // There is not referred element, no allocation exists for this data source and it is not an alias.
  // But it needs to return something, because otherwise the .write() calls that require a rvalue()
  // would fail. So we use an internal cache
  typename internal::DataSource<T>::const_reference_t rvalue() const
  {
    return mcached_data_;
  }

  void set( typename internal::AssignableDataSource<T>::param_t t )
  {
    set() = t;
    updated();
  }

  // The referenced element is a cache, since the source of the data is external.
  // After working with the reference, if something was assigned, the data
  // source may need a call to updated() after set()
  typename internal::AssignableDataSource<T>::reference_t set()
  {
    return mcached_data_;
  }

  void updated()
  {
    ros::param::set(mparam_name_, mcached_data_);
  }

  virtual RosParamDataSource<T>* clone() const
  {
    return new RosParamDataSource<T>(this->mparam_name_);
  }

  virtual RosParamDataSource<T>* copy( std::map<const base::DataSourceBase*, base::DataSourceBase*>& already_cloned ) const
  {
    // Just like in ValueDataSource
    if (already_cloned[this] != 0) {
      assert(dynamic_cast<RosParamDataSource<T>*>(already_cloned[this]) == static_cast<RosParamDataSource<T>*>(already_cloned[this]));
      return static_cast<RosParamDataSource<T>*>(already_cloned[this]);
    }
    already_cloned[this] = const_cast<RosParamDataSource<T>*>(this);
    return const_cast<RosParamDataSource<T>*>(this);
  }

}; // class RosParamDataSource

} // namespace rtt_rosparam

#endif // __RTT_ROSPARAM__ROS_PARAM_DATA_SOURCE_HPP
