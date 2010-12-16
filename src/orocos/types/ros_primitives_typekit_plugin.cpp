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
#include <rtt/typekit/StdTypeInfo.hpp>
#include <rtt/typekit/StdStringTypeInfo.hpp>
#include <rtt/types/TemplateConstructor.hpp>

namespace ros_integration {
    using namespace RTT;
    using namespace RTT::types;

    double float_to_double( float val ) {return double(val);}
    float double_to_float( double val ) {return float(val);}
    int32_t float_to_int(float f) { return int32_t(f); }
    float int_to_float(int i) { return float(i); }
    int32_t double_to_int(double f) { return int32_t(f); }
    double int_to_double(int32_t i) { return double(i); }
    uint32_t int_to_uint(int32_t i) { return (uint32_t)(i); }
    int uint_to_int(uint32_t ui) { return int32_t(ui); }
    bool int_to_bool(int32_t i) { return bool(i); }

    template<class T,class R>
    R a_to_b( T t ) { return R(t); }

    struct string_ctor
        : public std::unary_function<int, const std::string&>
    {
        mutable boost::shared_ptr< std::string > ptr;
        typedef const std::string& (Signature)( int );
        string_ctor()
            : ptr( new std::string() ) {}
        const std::string& operator()( int size ) const
        {
            ptr->resize( size );
            return *(ptr);
        }
    };


   
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
        RTT::types::Types()->addType( new types::StdStringTypeInfo() );
	      return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { 
          types::TypeInfoRepository::shared_ptr ti = types::TypeInfoRepository::Instance();
          // x to float64
          ti->type("float64")->addConstructor( newConstructor( &float_to_double, true ));
          ti->type("float64")->addConstructor( newConstructor( &int_to_double, true ));

          // x to float
          ti->type("float")->addConstructor( newConstructor( &int_to_float, true ));
          ti->type("float")->addConstructor( newConstructor( &double_to_float, true ));

          // x to int
          ti->type("int32")->addConstructor( newConstructor( &float_to_int, false ));
          ti->type("int32")->addConstructor( newConstructor( &double_to_int, false ));

          // we certainly need int32/uint32 to 8/16 since the RTT parser only knows 32bit wide ints.

          // x to uint8_t (ROS' bool)
          ti->type("uint8")->addConstructor( newConstructor( &a_to_b<int8_t,uint8_t>, false ));
          ti->type("uint8")->addConstructor( newConstructor( &a_to_b<int16_t,uint8_t>, false ));
          ti->type("uint8")->addConstructor( newConstructor( &a_to_b<int32_t,uint8_t>, false ));
          ti->type("uint8")->addConstructor( newConstructor( &a_to_b<uint16_t,uint8_t>, false ));
          ti->type("uint8")->addConstructor( newConstructor( &a_to_b<uint32_t,uint8_t>, false ));

          ti->type("int8")->addConstructor( newConstructor( &a_to_b<uint8_t,int8_t>, false ));
          ti->type("int8")->addConstructor( newConstructor( &a_to_b<uint16_t,int8_t>, false ));
          ti->type("int8")->addConstructor( newConstructor( &a_to_b<uint32_t,int8_t>, false ));
          ti->type("int8")->addConstructor( newConstructor( &a_to_b<int16_t,int8_t>, false ));
          ti->type("int8")->addConstructor( newConstructor( &a_to_b<int32_t,int8_t>, false ));

          // x to uint16_t
          ti->type("uint16")->addConstructor( newConstructor( &a_to_b<int8_t,uint16_t>, false ));
          ti->type("uint16")->addConstructor( newConstructor( &a_to_b<int16_t,uint16_t>, false ));
          ti->type("uint16")->addConstructor( newConstructor( &a_to_b<int32_t,uint16_t>, false ));
          ti->type("uint16")->addConstructor( newConstructor( &a_to_b<uint8_t,uint16_t>, true ));
          ti->type("uint16")->addConstructor( newConstructor( &a_to_b<uint32_t,uint16_t>, false ));

          ti->type("int16")->addConstructor( newConstructor( &a_to_b<uint8_t,int16_t>, true ));
          ti->type("int16")->addConstructor( newConstructor( &a_to_b<uint16_t,int16_t>, false ));
          ti->type("int16")->addConstructor( newConstructor( &a_to_b<uint32_t,int16_t>, false ));
          ti->type("int16")->addConstructor( newConstructor( &a_to_b<int8_t,int16_t>, true ));
          ti->type("int16")->addConstructor( newConstructor( &a_to_b<int32_t,int16_t>, false ));

          // x to uint32_t
          ti->type("uint32")->addConstructor( newConstructor( &a_to_b<int8_t,uint32_t>, false ));
          ti->type("uint32")->addConstructor( newConstructor( &a_to_b<int16_t,uint32_t>, false ));
          ti->type("uint32")->addConstructor( newConstructor( &a_to_b<int32_t,uint32_t>, true ));
          ti->type("uint32")->addConstructor( newConstructor( &a_to_b<uint8_t,uint32_t>, true ));
          ti->type("uint32")->addConstructor( newConstructor( &a_to_b<uint16_t,uint32_t>, true ));

          ti->type("int32")->addConstructor( newConstructor( &a_to_b<uint8_t,int32_t>, true ));
          ti->type("int32")->addConstructor( newConstructor( &a_to_b<uint16_t,int32_t>, true ));
          ti->type("int32")->addConstructor( newConstructor( &a_to_b<uint32_t,int32_t>, true ));
          ti->type("int32")->addConstructor( newConstructor( &a_to_b<int8_t,int32_t>, true ));
          ti->type("int32")->addConstructor( newConstructor( &a_to_b<int16_t,int32_t>, true ));

          ti->type("string")->addConstructor( newConstructor( string_ctor() ) );
          return true; 
      }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSPrimitivesTypekitPlugin )

