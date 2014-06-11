/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Intermodalics BVBA
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
*   * Neither the name of the Intermodalics BVBA nor the names of its
*     contributors may be used to endorse or promote products derived
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

#ifndef RTT_DYNAMIC_RECONFIGURE_AUTO_CONFIG_H
#define RTT_DYNAMIC_RECONFIGURE_AUTO_CONFIG_H

#include <rtt_dynamic_reconfigure/server.h>
#include <boost/thread/shared_mutex.hpp>

namespace rtt_dynamic_reconfigure {

/**
 * The AutoConfig class serves as a generic config type for rtt_dynamic_reconfigure::Server that dynamically creates the Config and ConfigDescription from the owner TaskContext's properties.
 */
class AutoConfig : public RTT::PropertyBag
{
public:
    typedef Server<AutoConfig> ServerType;

    std::string prefix_;
    std::string name;
    std::string type;
    int parent;
    int id;
    bool state;

    AutoConfig();
    AutoConfig(const RTT::PropertyBag &bag);
    ~AutoConfig();

    bool __fromMessage__(dynamic_reconfigure::Config &msg, const AutoConfig &sample);
    static bool __fromMessage__(AutoConfig &config, dynamic_reconfigure::Config &msg, const AutoConfig &sample);
    void __toMessage__(dynamic_reconfigure::Config &msg) const;
    static void __toMessage__(const AutoConfig &config, dynamic_reconfigure::Config &msg);

    void __toServer__(const ros::NodeHandle &nh) const;
    void __fromServer__(const ros::NodeHandle &nh);
    void __clamp__(const ServerType *server);
    uint32_t __level__(const AutoConfig &config) const;

    static dynamic_reconfigure::ConfigDescriptionPtr __getDescriptionMessage__(const ServerType *server);
    static const AutoConfig& __getDefault__(const ServerType *server);
    static const AutoConfig& __getMax__(const ServerType *server);
    static const AutoConfig& __getMin__(const ServerType *server);

    static void __refreshDescription__(const ServerType *server);

private:
    struct Cache;
    typedef boost::shared_ptr<Cache> CachePtr;
    static std::map<const ServerType *, CachePtr> cache_;
    static boost::shared_mutex cache_mutex_;
    static void buildCache(const ServerType *server, RTT::TaskContext *owner);
};

struct AutoConfig::Cache {
    dynamic_reconfigure::ConfigDescriptionPtr description_message_;
    AutoConfig default_;
    AutoConfig max_;
    AutoConfig min_;
};

} // namespace rtt_dynamic_reconfigure


#include "server.h"

namespace rtt_dynamic_reconfigure {

template <>
struct Updater<AutoConfig> {
    static bool propertiesFromConfig(AutoConfig &config, uint32_t, RTT::PropertyBag &bag) { RTT::refreshProperties(bag, config); }
    static bool configFromProperties(AutoConfig &config, const RTT::PropertyBag &bag)     { RTT::updateProperties(config, bag); }
};

template <>
struct dynamic_reconfigure_traits<AutoConfig> {
    typedef Server<AutoConfig> ServerType;

    static void getMin(AutoConfig &config, const ServerType *server)           { config = AutoConfig::__getMin__(server); }
    static void getMax(AutoConfig &config, const ServerType *server)           { config = AutoConfig::__getMax__(server); }
    static void getDefault(AutoConfig &config, const ServerType *server)       { config = AutoConfig::__getDefault__(server); }
    static dynamic_reconfigure::ConfigDescriptionPtr getDescriptionMessage(const ServerType *server) { return AutoConfig::__getDescriptionMessage__(server); }

    static const bool canRefresh = true;
    static void refreshDescription(const ServerType *server) { AutoConfig::__refreshDescription__(server); }

    static void toMessage(AutoConfig &config, dynamic_reconfigure::Config &message, const ServerType *server) { config.__toMessage__(message); }
    static void fromMessage(AutoConfig &config, dynamic_reconfigure::Config &message, const ServerType *server) { config.__fromMessage__(message, config); }
    static void clamp(AutoConfig &config, const ServerType *server) { config.__clamp__(server); }

    static RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr toPropertyBag(AutoConfig &config, const ServerType *) {
        return RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr(new RTT::internal::ReferenceDataSource<RTT::PropertyBag>(config));
    }
};

} // namespace rtt_dynamic_reconfigure

#endif // RTT_DYNAMIC_RECONFIGURE_AUTO_CONFIG_H
