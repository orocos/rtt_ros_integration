/*********************************************************************
 *
 *  Copyright (c) 2015, Intermodalics BVBA
 *  All rights reserved.
 *
 *********************************************************************/

#ifndef RTT_DYNAMIC_RECONFIGURE_TESTS_TEST_COMPONENT_HPP
#define RTT_DYNAMIC_RECONFIGURE_TESTS_TEST_COMPONENT_HPP

#include <rtt/TaskContext.hpp>
#include <geometry_msgs/Vector3.h>

struct Properties {
    Properties()
    : int_param()
    , double_param()
    , str_param()
    , bool_param()
    , float_param()
    , uint_param()
    , bag_param()
    , str_param_in_bag()
    , vector3_param()
    {}

    int int_param;
    double double_param;
    std::string str_param;
    bool bool_param;

    // other property types
    float float_param;
    unsigned int uint_param;

    // PropertyBag properties
    RTT::PropertyBag bag_param;
    std::string str_param_in_bag;

    // composable properties
    geometry_msgs::Vector3 vector3_param;
};

class DynamicReconfigureTestComponent : public RTT::TaskContext
{
public:
    Properties props;

    // types directly supported by dynamic_reconfigure
    DynamicReconfigureTestComponent(const std::string &name = "component")
        : RTT::TaskContext(name)
    {
        this->addProperty("int_param", props.int_param);
        this->addProperty("double_param", props.double_param);
        this->addProperty("str_param", props.str_param);
        this->addProperty("bool_param", props.bool_param);

        this->addProperty("float_param", props.float_param);
        this->addProperty("uint_param", props.uint_param);

        props.bag_param.addProperty("str_param", props.str_param_in_bag);
        this->addProperty("bag_param", props.bag_param);

        props.vector3_param.x = 1.0;
        props.vector3_param.y = 2.0;
        props.vector3_param.z = 3.0;
        this->addProperty("vector3_param", props.vector3_param);
    }
};

#endif // RTT_DYNAMIC_RECONFIGURE_TESTS_TEST_COMPONENT_HPP
