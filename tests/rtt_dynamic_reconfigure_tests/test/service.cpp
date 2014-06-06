/*********************************************************************
 *
 *  Copyright (c) 2014, Intermodalics BVBA
 *  All rights reserved.
 *
 *********************************************************************/

#include <rtt_dynamic_reconfigure/server.h>
#include <rtt_dynamic_reconfigure_tests/TestConfig.h>

using namespace rtt_dynamic_reconfigure_tests;

namespace rtt_dynamic_reconfigure {

template <>
struct Updater<TestConfig> {
  static bool propertiesFromConfig(TestConfig &config, uint32_t level, RTT::PropertyBag &bag) {
    setProperty<int>("int_param", bag, config.int_param);
    setProperty<double>("double_param", bag, config.double_param);
    setProperty<std::string>("str_param", bag, config.str_param);
    setProperty<bool>("bool_param", bag, config.bool_param);
    setProperty<double>("non_existent", bag, config.non_existent);
    return true;
  }
  static bool configFromProperties(TestConfig &config, const RTT::PropertyBag &bag) {
    getProperty<int>("int_param", bag, config.int_param);
    getProperty<double>("double_param", bag, config.double_param);
    getProperty<std::string>("str_param", bag, config.str_param);
    getProperty<bool>("bool_param", bag, config.bool_param);
    getProperty<double>("non_existent", bag, config.non_existent);
    return true;
  }
};

} // namespace rtt_dynamic_reconfigure

RTT_DYNAMIC_RECONFIGURE_SERVICE_PLUGIN(TestConfig, "test_reconfigure")
