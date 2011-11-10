// Copyright  (C)  2008  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "eigen_toolkit.hpp"
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/types/Operators.hpp>
#include <rtt/types/OperatorTypes.hpp>
#include <rtt/types/TemplateConstructor.hpp>
#include <rtt/types/Types.hpp>
#include <rtt/Logger.hpp>
#include <rtt/internal/DataSources.hpp>
#include <rtt/internal/mystd.hpp>
#include <rtt/os/StartStopManager.hpp>
#include <rtt/types/TypekitRepository.hpp>


#include <Eigen/Core>
namespace Eigen{

    using namespace RTT;
    using namespace RTT::detail;

    std::istream& operator>>(std::istream &is,MatrixXd& v){
      return is;
    }
    std::istream& operator>>(std::istream &is,VectorXd& v){
      return is;
    }



    struct VectorTypeInfo : public types::TemplateTypeInfo<VectorXd,true>
    {
        VectorTypeInfo():TemplateTypeInfo<VectorXd, true >("eigen_vector")
        {
        };

      virtual bool decomposeTypeImpl(const VectorXd& vec, PropertyBag& targetbag) const
        {
            targetbag.setType("eigen_vector");
            int dimension = vec.rows();
            std::string str;

            if(!targetbag.empty())
                return false;

            for ( int i=0; i < dimension ; i++){
                std::stringstream out;
                out << i+1;
                str = out.str();
                targetbag.add( new Property<double>(str, str +"th element of vector",vec(i)) ); // Put variables in the bag
            }

            return true;
        };

      virtual bool composeTypeImpl(const PropertyBag& bag, VectorXd& result) const{

            if ( bag.getType() == "eigen_vector" ) {
                int dimension = bag.size();
                result.resize( dimension );

                // Get values
                for (int i = 0; i < dimension ; i++) {
                    std::stringstream out;
                    out << i+1;
                    Property<double> elem = bag.getProperty(out.str());
                    if(elem.ready())
                        result(i) = elem.get();
                    else{
                        log(Error)<<"Could not read element "<<i+1<<endlog();
                        return false;
                    }
                }
            }else{
                log(Error) << "Composing Property< VectorXd > :"
                           << " type mismatch, got type '"<< bag.getType()
                           << "', expected type "<<"eigen_vector."<<endlog();
                return false;
            }
            return true;
        };
    };

    struct MatrixTypeInfo : public types::TemplateTypeInfo<MatrixXd,true>{
        MatrixTypeInfo():TemplateTypeInfo<MatrixXd, true >("eigen_matrix"){
        };


        bool decomposeTypeImpl(const MatrixXd& mat, PropertyBag& targetbag) const{
            targetbag.setType("eigen_matrix");
            unsigned int dimension = mat.rows();
            if(!targetbag.empty())
                return false;

            for ( unsigned int i=0; i < dimension ; i++){
                std::stringstream out;
                out << i+1;
                targetbag.add( new Property<VectorXd >(out.str(), out.str() +"th row of matrix",mat.row(i) )); // Put variables in the bag
            }

            return true;
        };

        bool composeTypeImpl(const PropertyBag& bag, MatrixXd& result) const{
            if ( bag.getType() == "eigen_matrix" ) {
                unsigned int rows = bag.size();
                unsigned int cols = 0;
                // Get values
                for (unsigned int i = 0; i < rows ; i++) {
                    std::stringstream out;
                    out << i+1;
                    Property<PropertyBag> row_bag =  bag.getProperty(out.str());
                    if(!row_bag.ready()){
                        log(Error)<<"Could not read row "<<i+1<<endlog();
                        return false;
                    }
                    Property<VectorXd > row_p(row_bag.getName(),row_bag.getDescription());
                    if(!(row_p.compose(row_bag))){
                        log(Error)<<"Could not compose row "<<i+1<<endlog();
                        return false;
                    }
                    if(row_p.ready()){
                        if(i==0){
                            cols = row_p.get().size();
                            result.resize(rows,cols);
                        } else
                            if(row_p.get().rows()!=(int)cols){
                                log(Error)<<"Row "<<i+1<<" size does not match matrix columns"<<endlog();
                                return false;
                            }
                        result.row(i)=row_p.get();
                    }else{
                        log(Error)<<"Property of Row "<<i+1<<"was not ready for use"<<endlog();
                        return false;
                    }
                }
            }else {
                log(Error) << "Composing Property< MatrixXd > :"
                           << " type mismatch, got type '"<< bag.getType()
                           << "', expected type "<<"ublas_matrix."<<endlog();
                return false;
            }
            return true;
        };
    };

    struct vector_index
        : public std::binary_function<const VectorXd&, int, double>
    {
        double operator()(const VectorXd& v, int index) const
        {
            if ( index >= (int)(v.size()) || index < 0)
                return 0.0;
            return v(index);
        }
    };

    struct get_size
        : public std::unary_function<const VectorXd&, int>
    {
        int operator()(const VectorXd& cont ) const
        {
            return cont.rows();
        }
    };

    struct vector_index_value_constructor
        : public std::binary_function<int,double,const VectorXd&>
    {
        typedef const VectorXd& (Signature)( int, double );
        mutable boost::shared_ptr< VectorXd > ptr;
        vector_index_value_constructor() :
            ptr( new VectorXd ){}
        const VectorXd& operator()(int size,double value ) const
        {
            ptr->resize(size);
            (*ptr)=Eigen::VectorXd::Constant(size,value);
            return *(ptr);
        }
    };

    struct vector_index_array_constructor
        : public std::unary_function<std::vector<double >,const VectorXd&>
    {
        typedef const VectorXd& (Signature)( std::vector<double > );
        mutable boost::shared_ptr< VectorXd > ptr;
        vector_index_array_constructor() :
            ptr( new VectorXd ){}
        const VectorXd& operator()(std::vector<double > values) const
        {
            (*ptr)=VectorXd::Map(&values[0],values.size());
            return *(ptr);
        }
    };

    struct vector_index_constructor
        : public std::unary_function<int,const VectorXd&>
    {
        typedef const VectorXd& (Signature)( int );
        mutable boost::shared_ptr< VectorXd > ptr;
        vector_index_constructor() :
            ptr( new VectorXd() ){}
        const VectorXd& operator()(int size ) const
        {
            ptr->resize(size);
            return *(ptr);
        }
    };

    struct matrix_index
        : public std::ternary_function<const MatrixXd&, int, int, double>
    {
        double operator()(const MatrixXd& m, int i, int j) const{
            if ( i >= (int)(m.rows()) || i < 0 || j<0 || j>= (int)(m.cols()))
                return 0.0;
            return m(i,j);
        }
    };

    struct matrix_i_j_constructor
        : public std::binary_function<int,int,const MatrixXd&>
    {
        typedef const MatrixXd& (Signature)( int, int );
        mutable boost::shared_ptr< MatrixXd > ptr;
        matrix_i_j_constructor() :
            ptr( new MatrixXd() ){}
        const MatrixXd& operator()(int size1,int size2) const
        {
            ptr->resize(size1,size2);
            return *(ptr);
        }
    };


    std::string EigenToolkitPlugin::getName()
    {
        return "Eigen";
    }

    bool EigenToolkitPlugin::loadTypes()
    {
        RTT::types::TypeInfoRepository::Instance()->addType( new VectorTypeInfo() );
        RTT::types::TypeInfoRepository::Instance()->addType( new MatrixTypeInfo() );
        return true;
    }

    bool EigenToolkitPlugin::loadConstructors()
    {
    	RTT::types::Types()->type("eigen_vector")->addConstructor(types::newConstructor(vector_index_constructor()));
        RTT::types::Types()->type("eigen_vector")->addConstructor(types::newConstructor(vector_index_value_constructor()));
        RTT::types::Types()->type("eigen_vector")->addConstructor(types::newConstructor(vector_index_array_constructor()));
        RTT::types::Types()->type("eigen_matrix")->addConstructor(types::newConstructor(matrix_i_j_constructor()));
        return true;
    }

    bool EigenToolkitPlugin::loadOperators()
    {
        RTT::types::OperatorRepository::Instance()->add( newBinaryOperator( "[]", vector_index() ) );
        //RTT::types::OperatorRepository::Instance()->add( newDotOperator( "size", get_size() ) );
        //RTT::types::OperatorRepository::Instance()->add( newTernaryOperator( "[,]", matrix_index() ) );
        return true;
    }
}

ORO_TYPEKIT_PLUGIN(Eigen::EigenToolkitPlugin)
