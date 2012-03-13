/*
 * (C) 2010 Ruben Smits, ruben.smits@mech.kuleuven.be, Department of Mechanical
 Engineering, Katholieke Universiteit Leuven, Belgium.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

 Alternatively, the contents of this file may be used under the terms of
 either of the New BSD License
 */
#ifndef UBLAS_TOOLKIT_HPP
#define UBLAS_TOOLKIT_HPP

#include <rtt/types/TypekitPlugin.hpp>
#include <Eigen/Core>

// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN
namespace Eigen {

class EigenToolkitPlugin: public RTT::types::TypekitPlugin {
public:
	virtual std::string getName();

	virtual bool loadTypes();
	virtual bool loadConstructors();
	virtual bool loadOperators();
};
}
#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSourceTypeInfo< Eigen::VectorXd >;
    extern template class RTT::internal::DataSource< Eigen::VectorXd >;
    extern template class RTT::internal::AssignableDataSource< Eigen::VectorXd >;
    extern template class RTT::internal::AssignCommand< Eigen::VectorXd >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< Eigen::VectorXd >;
    extern template class RTT::internal::ConstantDataSource< Eigen::VectorXd >;
    extern template class RTT::internal::ReferenceDataSource< Eigen::VectorXd >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::OutputPort< Eigen::VectorXd >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::InputPort< Eigen::VectorXd >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< Eigen::VectorXd >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< Eigen::VectorXd >;
    extern template class RTT::Constant< Eigen::VectorXd >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSourceTypeInfo< Eigen::MatrixXd >;
    extern template class RTT::internal::DataSource< Eigen::MatrixXd >;
    extern template class RTT::internal::AssignableDataSource< Eigen::MatrixXd >;
    extern template class RTT::internal::AssignCommand< Eigen::MatrixXd >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< Eigen::MatrixXd >;
    extern template class RTT::internal::ConstantDataSource< Eigen::MatrixXd >;
    extern template class RTT::internal::ReferenceDataSource< Eigen::MatrixXd >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::OutputPort< Eigen::MatrixXd >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::InputPort< Eigen::MatrixXd >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< Eigen::MatrixXd >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< Eigen::MatrixXd >;
    extern template class RTT::Constant< Eigen::MatrixXd >;
#endif

#endif

