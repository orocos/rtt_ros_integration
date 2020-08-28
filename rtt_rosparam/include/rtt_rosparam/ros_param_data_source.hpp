/***************************************************************************
  tag: Sergio Portoles  Thu Aug 06 09:10:00 CEST 2020  ros_param_data_source.hpp

                        ros_param_data_source.hpp -  description
                           -------------------
    begin                : Thu Aug 06 2020
    copyright            : (C) 2020 Intermodalics, BVBA
    email                : orocos-dev@orocos.org

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


#ifndef __RTT_ROSPARAM__ROS_PARAM_DATA_SOURCE_HPP
#define __RTT_ROSPARAM__ROS_PARAM_DATA_SOURCE_HPP

// #include "mystd.hpp"
#include "rtt/internal/DataSource.hpp"
#include "rtt/internal/DataSourceTypeInfo.hpp"
#include "rtt/internal/Reference.hpp"

#include "ros/ros.h"
#include "ros/param.h"
#include "ros/names.h"

#include <boost/function.hpp>
#include <exception>

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
  /**
   * Use shared_ptr.
   */
  typedef boost::intrusive_ptr< RosParamDataSource<T> > shared_ptr;

  ~RosParamDataSource()
  {
  }

  RosParamDataSource(std::string param_name_id)     
      : mparam_name_(param_name_id),
        mcached_data_()
  {
  }

  typename internal::DataSource<T>::result_t get() const {
    (void) evaluate();
    return value();
  }

  bool evaluate() const
  {
    T return_value;
    if (!ros::param::getCached(mparam_name_, return_value)) {
      throw std::out_of_range("The parameter " + mparam_name_ + " cannot be fetched.");
      return false;
    }
    mcached_data_ = return_value;
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
    ros::param::set(mparam_name_, t);
  }

  // There is not referred element, no allocation exists for this data source and it is not an alias.
  typename internal::AssignableDataSource<T>::reference_t set()
  {
    return mcached_data_;
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
