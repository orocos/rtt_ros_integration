/******************************************************************************
*                 This file is part of the OROCOS toolchain ROS project       *
*                                                                             *
*        (C) 2010 Ruben Smits, ruben.smits@mech.kuleuven.be                   *
*                 Steven Bellens, steven.bellens@mech.kuleuven.be             *
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

namespace ros_integration {

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

    void loadTimeTypes();

    void loadUInt8Types();
    void loadInt8Types();

    void loadUInt16Types();
    void loadInt16Types();

    void loadUInt32Types();
    void loadInt32Types();

    void loadUInt64Types();
    void loadInt64Types();

    void loadFloat32Types();
    void loadFloat64Types();

    void loadStringTypes();

    std::string ROSPrimitivesTypekitPlugin::getName(){
	    return std::string("ros-")+"primitives";
    }
 
    bool ROSPrimitivesTypekitPlugin::loadTypes() {
      loadTimeTypes();

      loadInt8Types();
      loadUInt8Types();

      loadInt16Types();
      loadUInt16Types();

      loadInt32Types();
      loadUInt32Types();

      loadInt64Types();
      loadUInt64Types();

      loadFloat32Types();
      loadFloat64Types();

      loadStringTypes();

	    return true;
    }
    bool ROSPrimitivesTypekitPlugin::loadOperators() { return true; }
    bool ROSPrimitivesTypekitPlugin::loadConstructors() { 
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
        ti->type("duration")->addConstructor( newConstructor ( &a_to_b<double,ros::Duration>, true));
        ti->type("time")->addConstructor( newConstructor ( &a_to_b<double,ros::Time>, true));
        return true; 
    }
};

ORO_TYPEKIT_PLUGIN( ros_integration::ROSPrimitivesTypekitPlugin )

