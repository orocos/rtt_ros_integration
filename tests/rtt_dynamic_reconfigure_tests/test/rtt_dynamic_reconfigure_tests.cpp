/*********************************************************************
 *
 *  Copyright (c) 2014, Intermodalics BVBA
 *  All rights reserved.
 *
 *********************************************************************/

#define RTT_STATIC

#include <rtt_dynamic_reconfigure/server.h>
#include <rtt_dynamic_reconfigure/auto_config.h>
#include <gtest/gtest.h>

#include <rtt/plugin/PluginLoader.hpp>
#include <rtt/Logger.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/os/main.h>

#include <rtt/internal/GlobalService.hpp>
#include <rtt/OperationCaller.hpp>

#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/config_tools.h>

#include <rtt_dynamic_reconfigure_tests/TestConfig.h>

using namespace rtt_dynamic_reconfigure;
using namespace rtt_dynamic_reconfigure_tests;

class DynamicReconfigureTest : public ::testing::Test
{
public:
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
    };

    class Component : public RTT::TaskContext
    {
    public:
        Properties props;

        // types directly supported by dynamic_reconfigure
        Component()
            : RTT::TaskContext("component")
        {
            this->addProperty("int_param", props.int_param);
            this->addProperty("double_param", props.double_param);
            this->addProperty("str_param", props.str_param);
            this->addProperty("bool_param", props.bool_param);

            this->addProperty("float_param", props.float_param);
            this->addProperty("uint_param", props.uint_param);

            props.bag_param.addProperty("str_param", props.str_param_in_bag);
            this->addProperty("bag", props.bag_param);
        }
    } tc;

    // virtual void SetUp() {}
    // virtual void TearDown() {}
};

static const dynamic_reconfigure::Group *getGroup(const dynamic_reconfigure::ConfigDescription::_groups_type *groups, const std::string &name)
{
    if (!groups) return 0;
    for(dynamic_reconfigure::ConfigDescription::_groups_type::const_iterator it = groups->begin(); it != groups->end(); ++it) {
        if (it->name == name) return &(*it);
    }
    return 0;
}

static const dynamic_reconfigure::ParamDescription *getParamDescription(const dynamic_reconfigure::Group *group, const std::string &name)
{
    if (!group) return 0;
    for(dynamic_reconfigure::Group::_parameters_type::const_iterator it = group->parameters.begin(); it != group->parameters.end(); ++it) {
        if (it->name == name) return &(*it);
    }
    return 0;
}

TEST_F(DynamicReconfigureTest, ConfigDescription)
{
    // load test_reconfigure service
    ASSERT_TRUE(tc.loadService("test_reconfigure"));
    ASSERT_TRUE(tc.provides()->hasService("test_reconfigure"));

    // properties should still have their original default value
    EXPECT_EQ(0,     tc.props.int_param);
    EXPECT_EQ(0,     tc.props.double_param);
    EXPECT_EQ("",    tc.props.str_param);
    EXPECT_FALSE(tc.props.bool_param);

    // non-existent properties should have been created with the default value
    ASSERT_TRUE(tc.properties()->getPropertyType<double>("non_existent"));
    EXPECT_EQ(5.0, *(tc.properties()->getPropertyType<double>("non_existent")));

    // get a pointer to the reconfigure service
    boost::shared_ptr<Server<TestConfig> > server = boost::shared_dynamic_cast<Server<TestConfig> >(tc.provides("test_reconfigure"));
    ASSERT_TRUE(server.get());

    // check ConfigDescription groups
    dynamic_reconfigure::ConfigDescriptionPtr description = server->getDescriptionMessage();
    EXPECT_TRUE(getParamDescription(getGroup(&(description->groups), "Default"), "int_param"));
    EXPECT_TRUE(getParamDescription(getGroup(&(description->groups), "Default"), "double_param"));
    EXPECT_TRUE(getParamDescription(getGroup(&(description->groups), "Default"), "str_param"));
    EXPECT_TRUE(getParamDescription(getGroup(&(description->groups), "Default"), "bool_param"));
    EXPECT_TRUE(getParamDescription(getGroup(&(description->groups), "Default"), "non_existent"));
    //EXPECT_TRUE(getGroup(&(description->groups), "a_group"));
    //EXPECT_TRUE(getParamDescription(getGroup(&(description->groups), "a_group"), "group_param"));

    // check ConfigDescription dflt/min/max values
    struct {
        int int_param;
        double double_param;
        std::string str_param;
        bool bool_param;
        std::string group_param;
    } temp;
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "int_param", temp.int_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "double_param", temp.double_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "str_param", temp.str_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "bool_param", temp.bool_param));
    //EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "group_param", temp.group_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "int_param", temp.int_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "double_param", temp.double_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "str_param", temp.str_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "bool_param", temp.bool_param));
    //EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "group_param", temp.group_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "int_param", temp.int_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "double_param", temp.double_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "str_param", temp.str_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "bool_param", temp.bool_param));
    //EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "group_param", temp.group_param));
}

TEST_F(DynamicReconfigureTest, MinMaxDefault)
{
    // load test_reconfigure service
    ASSERT_TRUE(tc.loadService("test_reconfigure"));
    ASSERT_TRUE(tc.provides()->hasService("test_reconfigure"));

    // check minimum values
    const RTT::PropertyBag &min = tc.provides("test_reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("min")->rvalue();
    EXPECT_EQ(0,    *(min.getPropertyType<int>("int_param")));
    EXPECT_EQ(0.0,  *(min.getPropertyType<double>("double_param")));
    EXPECT_EQ(0.0,  *(min.getPropertyType<double>("non_existent")));

    // check maximum values
    const RTT::PropertyBag &max = tc.provides("test_reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("max")->rvalue();
    EXPECT_EQ(100,  *(max.getPropertyType<int>("int_param")));
    EXPECT_EQ(1.0,  *(max.getPropertyType<double>("double_param")));
    EXPECT_EQ(10.0, *(max.getPropertyType<double>("non_existent")));

    // check default values
    const RTT::PropertyBag &dflt = tc.provides("test_reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("default")->rvalue();
    EXPECT_EQ(50,   *(dflt.getPropertyType<int>("int_param")));
    EXPECT_EQ(.5,   *(dflt.getPropertyType<double>("double_param")));
    EXPECT_EQ("Hello World", dflt.getPropertyType<std::string>("str_param")->rvalue());
    EXPECT_TRUE(*(dflt.getPropertyType<bool>("bool_param")));
    EXPECT_EQ(5.0,  *(dflt.getPropertyType<double>("non_existent")));
}

TEST_F(DynamicReconfigureTest, AutoConfig)
{
    // load test_reconfigure service
    ASSERT_TRUE(tc.loadService("reconfigure"));
    ASSERT_TRUE(tc.provides()->hasService("reconfigure"));

    // get a pointer to the reconfigure service
    boost::shared_ptr<Server<AutoConfig> > server = boost::shared_dynamic_cast<Server<AutoConfig> >(tc.provides("reconfigure"));
    ASSERT_TRUE(server.get());

    // min/max/default property bag should exist with the same number of properties
    ASSERT_TRUE(tc.provides("reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("min"));
    ASSERT_TRUE(tc.provides("reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("max"));
    ASSERT_TRUE(tc.provides("reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("default"));
    ASSERT_EQ(tc.properties()->size(), tc.provides("reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("min")->rvalue().size());
    ASSERT_EQ(tc.properties()->size(), tc.provides("reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("max")->rvalue().size());
    ASSERT_EQ(tc.properties()->size(), tc.provides("reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("default")->rvalue().size());

    // properties should still have their original default value
    EXPECT_EQ(0,     tc.props.int_param);
    EXPECT_EQ(0.0,   tc.props.double_param);
    EXPECT_EQ("",    tc.props.str_param);
    EXPECT_FALSE(tc.props.bool_param);
    EXPECT_EQ(0.0,   tc.props.float_param);
    EXPECT_EQ(0,     tc.props.uint_param);
    EXPECT_TRUE(tc.props.bag_param.size() == 1);
    EXPECT_EQ("",    tc.props.str_param_in_bag);

    // default value properties should exists with the initial value of the properties
    const RTT::PropertyBag &dflt = tc.provides("reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("default")->rvalue();
    ASSERT_TRUE(dflt.getPropertyType<int>("int_param"));
    EXPECT_EQ(tc.props.int_param,    *(dflt.getPropertyType<int>("int_param")));
    ASSERT_TRUE(dflt.getPropertyType<double>("double_param"));
    EXPECT_EQ(tc.props.double_param, *(dflt.getPropertyType<double>("double_param")));
    ASSERT_TRUE(dflt.getPropertyType<std::string>("str_param"));
    EXPECT_EQ(tc.props.str_param,    dflt.getPropertyType<std::string>("str_param")->rvalue());
    ASSERT_TRUE(dflt.getPropertyType<bool>("bool_param"));
    EXPECT_EQ(tc.props.bool_param,   *(dflt.getPropertyType<bool>("bool_param")));
    ASSERT_TRUE(dflt.getPropertyType<float>("float_param"));
    EXPECT_EQ(tc.props.float_param,   *(dflt.getPropertyType<float>("float_param")));
    ASSERT_TRUE(dflt.getPropertyType<unsigned int>("uint_param"));
    EXPECT_EQ(tc.props.uint_param,   *(dflt.getPropertyType<unsigned int>("uint_param")));

    ASSERT_TRUE(dflt.getPropertyType<RTT::PropertyBag>("bag"));
    const RTT::PropertyBag &dflt_bag = *(dflt.getPropertyType<RTT::PropertyBag>("bag"));
    EXPECT_TRUE(dflt_bag.size() == 1);
    ASSERT_TRUE(dflt_bag.getPropertyType<std::string>("str_param"));
    EXPECT_EQ(tc.props.str_param_in_bag, dflt_bag.getPropertyType<std::string>("str_param")->rvalue());

    // check ConfigDescription
    dynamic_reconfigure::ConfigDescriptionPtr description = server->getDescriptionMessage();
    ASSERT_TRUE(description->groups.size() == 2);
    EXPECT_EQ("Default", description->groups[0].name);
    EXPECT_EQ("", description->groups[0].type);
    EXPECT_EQ(6, description->groups[0].parameters.size());
    EXPECT_EQ(0, description->groups[0].parent);
    EXPECT_EQ(0, description->groups[0].id);
    EXPECT_EQ("bag", description->groups[1].name);
    EXPECT_EQ("", description->groups[1].type);
    EXPECT_EQ(1, description->groups[1].parameters.size());
    EXPECT_EQ(0, description->groups[1].parent);
    EXPECT_EQ(1, description->groups[1].id);

    // check default/minimum/maximum values in description message
    struct {
        int int_param;
        double double_param;
        std::string str_param;
        bool bool_param;
        double float_param;
        int uint_param;
        std::string str_param_in_bag;
    } temp;
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "int_param", temp.int_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "double_param", temp.double_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "str_param", temp.str_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "bool_param", temp.bool_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "float_param", temp.float_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "uint_param", temp.uint_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "bag__str_param", temp.str_param_in_bag));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "int_param", temp.int_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "double_param", temp.double_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "str_param", temp.str_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "bool_param", temp.bool_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "float_param", temp.float_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "uint_param", temp.uint_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "bag__str_param", temp.str_param_in_bag));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "int_param", temp.int_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "double_param", temp.double_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "str_param", temp.str_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "bool_param", temp.bool_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "float_param", temp.float_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "uint_param", temp.uint_param));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "bag__str_param", temp.str_param_in_bag));
}

TEST_F(DynamicReconfigureTest, AutoConfigAddPropertiesAndRefresh)
{
    // load test_reconfigure service
    ASSERT_TRUE(tc.loadService("reconfigure"));
    ASSERT_TRUE(tc.provides()->hasService("reconfigure"));

    // get a pointer to the reconfigure service
    boost::shared_ptr<Server<AutoConfig> > server = boost::shared_dynamic_cast<Server<AutoConfig> >(tc.provides("reconfigure"));
    ASSERT_TRUE(server.get());

    // add a property to the TaskContext after having loaded the reconfigure service
    tc.properties()->ownProperty(new RTT::Property<std::string>("new_param"));

    // refresh dynamic reconfigure service
    server->refresh();

    // check ConfigDescription
    dynamic_reconfigure::ConfigDescriptionPtr description = server->getDescriptionMessage();
    std::string str;
    EXPECT_TRUE(getParamDescription(getGroup(&(description->groups), "Default"), "new_param"));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->dflt, "new_param", str));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->min, "new_param", str));
    EXPECT_TRUE(dynamic_reconfigure::ConfigTools::getParameter(description->max, "new_param", str));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // Initialize Orocos
    __os_init(argc, argv);

    RTT::Logger::log().setStdStream(std::cerr);
    RTT::Logger::log().mayLogStdOut(true);
    RTT::Logger::log().setLogLevel(RTT::Logger::Debug);

    RTT::ComponentLoader::Instance()->import("rtt_ros", std::string());
    RTT::OperationCaller<bool(std::string)> import = RTT::internal::GlobalService::Instance()->provides("ros")->getOperation("import");
    import("rtt_dynamic_reconfigure_tests");

    return RUN_ALL_TESTS();
}
