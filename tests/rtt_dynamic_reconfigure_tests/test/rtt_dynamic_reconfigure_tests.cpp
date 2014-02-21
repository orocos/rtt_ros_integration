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

using namespace rtt_dynamic_reconfigure;

class DynamicReconfigureTest : public ::testing::Test
{
protected:
    RTT::TaskContext tc;

    // types directly supported by dynamic_reconfigure
    int int_property;
    double double_property;
    std::string str_property;
    bool bool_property;

    // other property types
    float float_property;
    unsigned int uint_property;

    // PropertyBag properties
    RTT::PropertyBag bag_property;
    std::string str_property_in_bag;

public:
    DynamicReconfigureTest()
        : tc("taskcontext")
        , int_property()
        , double_property()
        , str_property()
        , bool_property()
        , float_property()
        , uint_property()
        , bag_property()
        , str_property_in_bag()
    {
        tc.addProperty("int_param", int_property);
        tc.addProperty("double_param", double_property);
        tc.addProperty("str_param", str_property);
        tc.addProperty("bool_param", bool_property);

        tc.addProperty("float_param", float_property);
        tc.addProperty("uint_param", uint_property);

        bag_property.addProperty("str_param", str_property_in_bag);
        tc.addProperty("bag", bag_property);
    }

    // virtual void SetUp() {}
    // virtual void TearDown() {}
};

TEST_F(DynamicReconfigureTest, LoadService)
{
    // load test_reconfigure service
    ASSERT_TRUE(tc.loadService("test_reconfigure"));
    ASSERT_TRUE(tc.provides()->hasService("test_reconfigure"));

    // properties should still have their original default value
    EXPECT_EQ(0,     int_property);
    EXPECT_EQ(0,     double_property);
    EXPECT_EQ("",    str_property);
    EXPECT_FALSE(bool_property);

    // non-existent properties should have been created with the default value
    ASSERT_TRUE(tc.properties()->getPropertyType<double>("non_existent"));
    EXPECT_EQ(5.0, *(tc.properties()->getPropertyType<double>("non_existent")));
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

    // properties should still have their original default value
    EXPECT_EQ(0,     int_property);
    EXPECT_EQ(0.0,   double_property);
    EXPECT_EQ("",    str_property);
    EXPECT_FALSE(bool_property);
    EXPECT_EQ(0.0,   float_property);
    EXPECT_EQ(0,     uint_property);
    EXPECT_TRUE(bag_property.size() == 1);
    EXPECT_EQ("",    str_property_in_bag);

    // default value properties should exists with the initial value of the properties
    const RTT::PropertyBag &dflt = tc.provides("reconfigure")->properties()->getPropertyType<RTT::PropertyBag>("default")->rvalue();
    ASSERT_TRUE(dflt.getPropertyType<int>("int_param"));
    EXPECT_EQ(int_property,    *(dflt.getPropertyType<int>("int_param")));
    ASSERT_TRUE(dflt.getPropertyType<double>("double_param"));
    EXPECT_EQ(double_property, *(dflt.getPropertyType<double>("double_param")));
    ASSERT_TRUE(dflt.getPropertyType<std::string>("str_param"));
    EXPECT_EQ(str_property,    dflt.getPropertyType<std::string>("str_param")->rvalue());
    ASSERT_TRUE(dflt.getPropertyType<bool>("bool_param"));
    EXPECT_EQ(bool_property,   *(dflt.getPropertyType<bool>("bool_param")));
    ASSERT_TRUE(dflt.getPropertyType<float>("float_param"));
    EXPECT_EQ(bool_property,   *(dflt.getPropertyType<float>("float_param")));
    ASSERT_TRUE(dflt.getPropertyType<unsigned int>("uint_param"));
    EXPECT_EQ(bool_property,   *(dflt.getPropertyType<unsigned int>("uint_param")));

    ASSERT_TRUE(dflt.getPropertyType<RTT::PropertyBag>("bag"));
    EXPECT_TRUE(dflt.getPropertyType<RTT::PropertyBag>("bag")->rvalue().size() == 1);
    ASSERT_TRUE(dflt.getPropertyType<RTT::PropertyBag>("bag")->rvalue().getPropertyType<std::string>("str_param"));
    EXPECT_EQ(str_property_in_bag, dflt.getPropertyType<RTT::PropertyBag>("bag")->rvalue().getPropertyType<std::string>("str_param")->rvalue());
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
    // ...

    // refresh dynamic reconfigure service
    server->refresh();

    // check ConfigDescription
    dynamic_reconfigure::ConfigDescriptionPtr description = server->getConfigDefault().__getDescriptionMessage__(server.get());
    // ...
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
