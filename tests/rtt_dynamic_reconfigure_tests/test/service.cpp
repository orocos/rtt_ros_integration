/*
 * (C) 2014, Intermodalics BVBA
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
