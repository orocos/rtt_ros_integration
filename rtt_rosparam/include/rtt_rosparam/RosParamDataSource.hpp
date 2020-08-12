/***************************************************************************
  tag: Sergio Portoles  Thu Aug 06 09:10:00 CEST 2020  RosParamDataSource.gpp

                        RosParamDataSource.gpp -  description
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


#ifndef __RTT_ROSPARAM__ROSPARAMDATASOURCE_HPP
#define __RTT_ROSPARAM__ROSPARAMDATASOURCE_HPP

// #include "mystd.hpp"
#include "rtt/internal/DataSource.hpp"
#include "rtt/internal/DataSourceTypeInfo.hpp"
#include "rtt/internal/Reference.hpp"

#include "ros/ros.h"
#include "ros/param.h"
#include "ros/names.h"

#include <boost/function.hpp>
#include <exception>

namespace RTT
{
  
  namespace rosparam {

  /**
   * A DataSource which checks the ROS parameter server and it
   * returns the parameter value in its get() method.
   * It sets the parameter value in its set() method.
   * The constructor receives the full qualified property name attached
   * to the DataSource.
   */
  template<typename T>
  class RosParamDataSource
      : public internal::AssignableDataSource<T>
  {
    /**
     * Stores the full-qualified property name of the ROS parameter
     */
    std::string mparam_name_;
    bool muse_cached_;
    mutable typename internal::DataSource<T>::value_t mcached_data_;

  public:
    /**
     * Use shared_ptr.
     */
    ~RosParamDataSource();

    typedef boost::intrusive_ptr< RosParamDataSource<T> > shared_ptr;

    RosParamDataSource(std::string param_name_id, bool use_cached = true);

    typename internal::DataSource<T>::result_t get() const
    {
      T return_value;
      if (muse_cached_) {
        if (!ros::param::getCached(mparam_name_, return_value)) {
          // std::cerr << "The parameter " << mparam_name_ << " cannot be fetched." << std::endl;
          throw(std::range_error("The parameter " + mparam_name_ + " cannot be fetched."));
          return T();
        }
      } else {
        if (!ros::param::get(mparam_name_, return_value)) {
          // std::cerr << "The parameter " << mparam_name_ << " cannot be fetched." << std::endl;
          throw(std::range_error("The parameter " + mparam_name_ + " cannot be fetched."));
          return T();
        }
      }
      return mcached_data_ = return_value;
    }

    typename internal::DataSource<T>::result_t value() const
    {
      return get();
    }

    // There is not referred element, no allocation exists for this data source and it is not an alias.
    // But it needs to return something, because otherwise the .write() calls that require a rvalue()
    // would fail. So we use an internal cache
    typename internal::DataSource<T>::const_reference_t rvalue() const
    {
      return mcached_data_;
    }

    void set( typename internal::AssignableDataSource<T>::param_t t );

    // There is not referred element, no allocation exists for this data source and it is not an alias.
    typename internal::AssignableDataSource<T>::reference_t set()
    {
      return mcached_data_;
    }

    virtual RosParamDataSource<T>* clone() const;

    virtual RosParamDataSource<T>* copy( std::map<const base::DataSourceBase*, base::DataSourceBase*>& alreadyCloned ) const;

  }; // class RosParamDataSource


  // template<>
  // internal::DataSource<std::string>::result_t RosParamDataSource<std::string>::get() const
  // {
  //   std::string return_value;
  //   if (muse_cached_) {
  //     if (!ros::param::getCached(mparam_name_, return_value)) {
  //       throw(std::range_error("The parameter " + mparam_name_ + " cannot be fetched."));
  //       return std::string();
  //     }
  //   } else {
  //     if (!ros::param::get(mparam_name_, return_value)) {
  //       throw(std::range_error("The parameter " + mparam_name_ + " cannot be fetched."));
  //       return std::string();
  //     }
  //   }
  //   base::DataSourceBase::shared_ptr mobj = this->clone();
  //   if (nullptr == mobj) {
  //     std::cerr << "Couldn't cast the ds ptr into a base::DataSourceBase::shared_ptr" << std::endl;
  //   }
  //   // From DataSource.cpp:48
  //   // std::ostream mystream("stream");
  //   // mobj->getTypeInfo()->write(std::cout, mobj);
  //   std::cout << "DataSource typed: " << mobj->getTypeInfo()->getTypeIdName() << std::endl;
  //   return return_value;
  // }
  
  // template<>
  // internal::DataSource<double>::result_t RosParamDataSource<double>::get() const
  // {
  //   double return_value;
  //   std::cout << " It is a double ! " << std::endl;
  //   if (muse_cached_) {
  //     if (!ros::param::getCached(mparam_name_, return_value)) {
  //       throw(std::range_error("The parameter " + mparam_name_ + " cannot be fetched."));
  //       return 0.0;
  //     }
  //   } else {
  //     if (!ros::param::get(mparam_name_, return_value)) {
  //       throw(std::range_error("The parameter " + mparam_name_ + " cannot be fetched."));
  //       return 0.0;
  //     }
  //   }
  //   return return_value;
  // }

  template<typename T>
  RosParamDataSource<T>::~RosParamDataSource() {}

  template<typename T>
  RosParamDataSource<T>::RosParamDataSource( std::string param_name, bool use_cached)
      : mparam_name_(param_name),
        muse_cached_(use_cached),
        mcached_data_(T())
  {
  }
  template<typename T>
  void RosParamDataSource<T>::set( typename internal::AssignableDataSource<T>::param_t t )
  {
    ros::param::set(mparam_name_, t);
  }

  // template <typename T>
  // typename internal::AssignableDataSource<T>::reference_t RosParamDataSource<T>::set() {
  //   // return mdummy_;
  // }

  template<typename T>
  RosParamDataSource<T>* RosParamDataSource<T>::clone() const
  {
      return new RosParamDataSource<T>(this->mparam_name_);
  }

  template<typename T>
  RosParamDataSource<T>* RosParamDataSource<T>::copy( std::map<const base::DataSourceBase*, base::DataSourceBase*>& alreadyCloned ) const {
      return const_cast<RosParamDataSource<T>*>(this); // no copy needed, data is outside.
  }

  } // namespace rosparam
} //namespace RTT

// /*
//  * Extern template declarations for core data source types
//  * (instantiated in DataSources.cpp)
// */
RTT_EXT_IMPL template class RTT::rosparam::RosParamDataSource< bool >;
RTT_EXT_IMPL template class RTT::rosparam::RosParamDataSource< std::string >;

#endif // __RTT_ROSPARAM__ROSPARAMDATASOURCE_HPP
