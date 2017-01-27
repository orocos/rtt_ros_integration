/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Intermodalics BVBA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of of the copyright holders nor the names of
 *     any contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef STD_MSGS_VECTOR_MULTI_ARRAY_ADAPTER_H
#define STD_MSGS_VECTOR_MULTI_ARRAY_ADAPTER_H

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>

#include <ros/assert.h>
#include <ros/message_traits.h>
#include <ros/serialization.h>

namespace std_msgs {

// container for a std::vector<T, ContainerAllocator> with pointer semantics
template <typename T, class ContainerAllocator = std::allocator<T> >
class VectorMultiArrayAdapter
{
public:
  typedef std::vector<T, ContainerAllocator> VectorType;

  VectorMultiArrayAdapter()
    : vector_(&owned_vector_) {}
  VectorMultiArrayAdapter(VectorType &v)
    : vector_(&v) {}
  VectorMultiArrayAdapter(const VectorType &v)
    : vector_(const_cast<VectorType*>(&v)) {}

  VectorType *operator->() { return vector_; }
  const VectorType *operator->() const { return vector_; }
  VectorType &operator*() { return *vector_; }
  const VectorType &operator*() const { return *vector_; }

private:
  VectorType owned_vector_;
  VectorType* vector_;
};

} // namespace std_msgs


#define STD_MSGS_DEFINE_MULTIARRAY_TRAITS(value_type, msg, static_md5sum1, static_md5sum2) \
  namespace ros \
  { \
  namespace message_traits \
  { \
    \
    template <class ContainerAllocator> struct MD5Sum<std_msgs::VectorMultiArrayAdapter<value_type, ContainerAllocator> > \
    { \
      static const char* value() \
      { \
        ROS_STATIC_ASSERT(MD5Sum<std_msgs::msg>::static_value1 == static_md5sum1); \
        ROS_STATIC_ASSERT(MD5Sum<std_msgs::msg>::static_value2 == static_md5sum2); \
        return MD5Sum<std_msgs::msg>::value(); \
      } \
      \
      static const char* value(const std_msgs::VectorMultiArrayAdapter<value_type, ContainerAllocator> &) \
      { \
        return value(); \
      } \
    }; \
    \
    template <class ContainerAllocator> struct DataType<std_msgs::VectorMultiArrayAdapter<value_type, ContainerAllocator> > \
    { \
      static const char* value() \
      { \
        return DataType<std_msgs::msg>::value(); \
      } \
     \
      static const char* value(const std_msgs::VectorMultiArrayAdapter<value_type, ContainerAllocator> &) \
      { \
        return value(); \
      } \
    }; \
    \
    template <class ContainerAllocator> struct Definition<std_msgs::VectorMultiArrayAdapter<value_type, ContainerAllocator> > \
    { \
      static const char* value() \
      { \
        return Definition<std_msgs::msg>::value(); \
      } \
      \
      static const char* value(const std_msgs::VectorMultiArrayAdapter<value_type, ContainerAllocator> &) \
      { \
        return value(); \
      } \
    }; \
    \
  } \
  }

STD_MSGS_DEFINE_MULTIARRAY_TRAITS(float, Float32MultiArray, 0x6a40e0ffa6a17a50ULL, 0x3ac3f8616991b1f6ULL)
STD_MSGS_DEFINE_MULTIARRAY_TRAITS(double, Float64MultiArray, 0x4b7d974086d4060eULL, 0x7db4613a7e6c3ba4ULL)
STD_MSGS_DEFINE_MULTIARRAY_TRAITS(int16_t, Int16MultiArray, 0xd9338d7f523fcb69ULL, 0x2fae9d0a0e9f067cULL)
STD_MSGS_DEFINE_MULTIARRAY_TRAITS(int32_t, Int32MultiArray, 0x1d99f79f8b325b44ULL, 0xfee908053e9c945bULL)
STD_MSGS_DEFINE_MULTIARRAY_TRAITS(int64_t, Int64MultiArray, 0x54865aa6c65be044ULL, 0x8113a2afc6a49270ULL)
STD_MSGS_DEFINE_MULTIARRAY_TRAITS(int8_t, Int8MultiArray, 0xd7c1af35a1b4781bULL, 0xbe79e03dd94b7c13ULL)
STD_MSGS_DEFINE_MULTIARRAY_TRAITS(uint16_t, UInt16MultiArray, 0x52f264f1c973c4b7ULL, 0x3790d384c6cb4484ULL)
STD_MSGS_DEFINE_MULTIARRAY_TRAITS(uint32_t, UInt32MultiArray, 0x4d6a180abc9be191ULL, 0xb96a7eda6c8a233dULL)
STD_MSGS_DEFINE_MULTIARRAY_TRAITS(uint64_t, UInt64MultiArray, 0x6088f127afb1d6c7ULL, 0x2927aa1247e945afULL)
STD_MSGS_DEFINE_MULTIARRAY_TRAITS(uint8_t, UInt8MultiArray, 0x82373f1612381bb6ULL, 0xee473b5cd6f5d89cULL)

namespace ros {
namespace serialization {

template <typename T, class ContainerAllocator>
struct Serializer<std_msgs::VectorMultiArrayAdapter<T, ContainerAllocator> >
{
  typedef std_msgs::VectorMultiArrayAdapter<T, ContainerAllocator> AdaptedType;

  template<typename Stream>
  inline static void write(Stream& stream, const AdaptedType& v)
  {
    // Note: We mimic serialization of a std_msgs/MultiArrayLayout here because
    // we cannot use a static const instance here due to the variable field dim[0].size.
    stream.next((uint32_t)1);         // layout.dim.size()
    stream.next(std::string());       // layout.dim[0].label
    stream.next((uint32_t)v->size()); // layout.dim[0].size
    stream.next((uint32_t)1);         // layout.dim[0].stride
    stream.next((uint32_t)0);         // layout.data_offset
    stream.next(*v);
  }

  template<typename Stream>
  inline static void read(Stream& stream, AdaptedType& v)
  {
    std_msgs::MultiArrayLayout_<ContainerAllocator> layout;
    stream.next(layout); // layout is ignored on read!
    stream.next(*v);
  }

  inline static uint32_t serializedLength(const AdaptedType& v)
  {
    return 20 + serializationLength(*v);
  }
};

} // namespace serialization
} // namespace ros

#endif // STD_MSGS_VECTOR_MULTI_ARRAY_ADAPTER_H
