/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009-2010, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#ifndef RTT_DYNAMIC_RECONFIGURE_SERVER_H
#define RTT_DYNAMIC_RECONFIGURE_SERVER_H

#include <rtt/Service.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/Logger.hpp>

#include <rtt/internal/DataSources.hpp>

#include <ros/ros.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>

namespace rtt_dynamic_reconfigure {

template <class ConfigType> class Server;

template <class ConfigType>
struct Updater {
    static bool propertiesFromConfig(ConfigType &config, uint32_t level, RTT::PropertyBag &) { return false; }
    static bool configFromProperties(ConfigType &config, const RTT::PropertyBag &) { return false; }
};

template <class ConfigType>
struct dynamic_reconfigure_traits {
    typedef Server<ConfigType> ServerType;

    static void getMin(ConfigType &config, const ServerType *)           { config = ConfigType::__getMin__(); }
    static void getMax(ConfigType &config, const ServerType *)           { config = ConfigType::__getMax__(); }
    static void getDefault(ConfigType &config, const ServerType *)       { config = ConfigType::__getDefault__(); }
    static dynamic_reconfigure::ConfigDescriptionPtr getDescriptionMessage(const ServerType *) { return dynamic_reconfigure::ConfigDescriptionPtr(new dynamic_reconfigure::ConfigDescription(ConfigType::__getDescriptionMessage__())); }

    static const bool canRefresh = false;
    static void refreshDescription(const ServerType *) {}

    static void toMessage(ConfigType &config, dynamic_reconfigure::Config &message, const ServerType *) { config.__toMessage__(message); }
    static void fromMessage(ConfigType &config, dynamic_reconfigure::Config &message, const ServerType *) { config.__fromMessage__(message); }
    static void clamp(ConfigType &config, const ServerType *) { config.__clamp__(); }

    static RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr toPropertyBag(ConfigType &config, const ServerType *) {
        RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr ds(new RTT::internal::ValueDataSource<RTT::PropertyBag>());
        if (!Updater<ConfigType>::propertiesFromConfig(config, ~0, ds->set()))
            ds.reset();
        return ds;
    }
};

template <class ConfigType>
class Server : public RTT::Service
{
private:
    typedef Updater<ConfigType> updater;
    typedef dynamic_reconfigure_traits<ConfigType> traits;

    RTT::os::MutexRecursive mutex_;
    ros::NodeHandle *node_handle_;
    ros::ServiceServer set_service_;
    ros::Publisher update_pub_;
    ros::Publisher descr_pub_;

    ConfigType config_;
    ConfigType min_;
    ConfigType max_;
    ConfigType default_;

public:
    Server(const std::string &name, RTT::TaskContext *owner)
        : RTT::Service(name, owner)
        , node_handle_(0)
    {
        construct();
    }

    Server(RTT::TaskContext *owner)
        : RTT::Service("reconfigure", owner)
        , node_handle_(0)
    {
        construct();
    }

    virtual ~Server() {
        shutdown();
    }

    void updateConfig(const ConfigType &config)
    {
        updateConfigInternal(config);
    }

    void getConfigMax(ConfigType &config) const
    {
        config = max_;
    }
    const ConfigType &getConfigMax() const { return max_; }
    ConfigType &getConfigMax() { return max_; }

    void getConfigMin(ConfigType &config) const
    {
        config = min_;
    }
    const ConfigType &getConfigMin() const { return min_; }
    ConfigType &getConfigMin() { return min_; }

    void getConfigDefault(ConfigType &config) const
    {
        config = default_;
    }
    const ConfigType &getConfigDefault() const { return default_; }
    ConfigType &getConfigDefault() { return default_; }

    void setConfigMax(const ConfigType &config)
    {
        max_ = config;
        PublishDescription();
    }

    void setConfigMin(const ConfigType &config)
    {
        min_ = config;
        PublishDescription();
    }

    void setConfigDefault(const ConfigType &config)
    {
        default_ = config;
        PublishDescription();
    }

    dynamic_reconfigure::ConfigDescriptionPtr getDescriptionMessage() {
        dynamic_reconfigure::ConfigDescriptionPtr description_message = traits::getDescriptionMessage(this);

        // Copy over min_ max_ default_
        traits::toMessage(max_, description_message->max, this);
        traits::toMessage(min_, description_message->min, this);
        traits::toMessage(default_, description_message->dflt, this);

        return description_message;
    }

    void advertise(std::string ns = std::string())
    {
        // set default namespace
        if (ns.empty()) {
            if (getOwner()->getName() == "Deployer")
                ns = "~";
            else
                ns = "~" + getOwner()->getName();
        }

        // create NodeHandle
        if (node_handle_) delete node_handle_;
        node_handle_ = new ros::NodeHandle(ns);

        // advertise service server and publishers
        set_service_ = node_handle_->advertiseService("set_parameters", &Server<ConfigType>::setConfigCallback, this);
        descr_pub_ = node_handle_->advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
        update_pub_ = node_handle_->advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);

        // publish update once
        PublishDescription();
        updateConfigInternal(config_);
    }

    void shutdown()
    {
        if (node_handle_) {
            node_handle_->shutdown();
            delete node_handle_;
            node_handle_ = 0;
        }
    }

    bool updated()
    {
        ConfigType new_config = config_;
        if (!updater::configFromProperties(new_config, *(getOwner()->properties()))) return false;
        updateConfig(new_config);
        return true;
    }

    void refresh()
    {
        RTT::os::MutexLock lock(mutex_);

        // Refresh config description (no-op unless for AutoConfig)
        traits::refreshDescription(this);

        // Grab copys of the data from the config files.  These are declared in the generated config file.
        traits::getMin(min_, this);
        traits::getMax(max_, this);
        traits::getDefault(default_, this);

        // Publish the description
        PublishDescription();

        // Add/replace min/max/default properties
        this->properties()->remove(this->properties()->getProperty("min"));
        this->properties()->remove(this->properties()->getProperty("max"));
        this->properties()->remove(this->properties()->getProperty("default"));
        this->properties()->ownProperty(new RTT::Property<RTT::PropertyBag>("min", "Minimum values as published to dynamic_reconfigure clients", traits::toPropertyBag(min_, this)));
        this->properties()->ownProperty(new RTT::Property<RTT::PropertyBag>("max", "Maximum values as published to dynamic_reconfigure clients", traits::toPropertyBag(max_, this)));
        this->properties()->ownProperty(new RTT::Property<RTT::PropertyBag>("default", "Default values as published to dynamic_reconfigure clients", traits::toPropertyBag(default_, this)));

        // Get initial values from current property settings
        config_ = ConfigType();
        traits::getDefault(config_, this);
        updater::configFromProperties(config_, *(getOwner()->properties()));
        if (node_handle_)
            config_.__fromServer__(*node_handle_);
        traits::clamp(config_, this);

        // At startup we need to load the configuration with all level bits set. (Everything has changed.)
        RTT::PropertyBag init_config;
        updater::propertiesFromConfig(config_, ~0, init_config);
        RTT::updateProperties(*(getOwner()->properties()), init_config);

        updateConfigInternal(config_);
   }

private:
    void construct()
    {
        this->addOperation("advertise", &Server<ConfigType>::advertise, this)
                .doc("Advertise this dynamic_reconfigure server at the master.")
                .arg("namespace", "The namespace this server should be advertised in. Defaults to ~component.");
        this->addOperation("shutdown", &Server<ConfigType>::shutdown, this)
                .doc("Shutdown this dynamic_reconfigure server.");
        this->addOperation("updated", &Server<ConfigType>::updated, this)
                .doc("Notify the dynamic_reconfigure server that properties have been updated. This will update the GUI.");

        if (traits::canRefresh)
            this->addOperation("refresh", &Server<ConfigType>::refresh, this)
                .doc("Rediscover the owner's properties. Call this operation after having added properties.");

        // refresh once
        refresh();
    }

    void PublishDescription() {
        if (!descr_pub_) return;
        descr_pub_.publish(getDescriptionMessage());
    }

    bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
                           dynamic_reconfigure::Reconfigure::Response &rsp)
    {
        RTT::os::MutexLock lock(mutex_);

        ConfigType new_config = config_;
        traits::fromMessage(new_config, req.config, this);
        traits::clamp(new_config, this);
        uint32_t level = config_.__level__(new_config);

        if (!updater::propertiesFromConfig(new_config, level, *(getOwner()->properties()))) return false;

        updateConfigInternal(new_config);
        new_config.__toMessage__(rsp.config);
        return true;
    }

    void updateConfigInternal(const ConfigType &config)
    {
        RTT::os::MutexLock lock(mutex_);
        config_ = config;
        if (node_handle_)
            config_.__toServer__(*node_handle_);
        dynamic_reconfigure::Config msg;
        config_.__toMessage__(msg);

        if (update_pub_)
            update_pub_.publish(msg);
    }
};

template <typename T>
bool setProperty(const std::string &name, RTT::PropertyBag &bag, T &value)
{
    if (bag.getProperty(name)) {
        RTT::Property<T> *prop = bag.getPropertyType<T>(name);
        if (!prop) {
            RTT::log(RTT::Error) << "Could not assign property '" << name << "': Property exists with a different type." << RTT::endlog();
        } else {
            prop->set() = value;
        }
    } else {
        /* bag.ownProperty(new RTT::Property<TYPE>(name, std::string(), value)); */
        bag.addProperty(name, value);
    }
}

template <typename T>
bool getProperty(const std::string &name, const RTT::PropertyBag &bag, T &value)
{
    RTT::Property<T> *prop = bag.getPropertyType<T>(name);
    if (!prop) {
        RTT::log(RTT::Error) << "Could not get property '" << name << "': No such property in the bag." << RTT::endlog();
    } else {
        value = prop->rvalue();
    }
}

} // namespace rtt_dynamic_reconfigure

#include <rtt/plugin/ServicePlugin.hpp>
//#define RTT_DYNAMIC_RECONFIGURE_SERVICE_PLUGIN(CONFIG, NAME) \
//    ORO_SERVICE_NAMED_PLUGIN(rtt_dynamic_reconfigure::Server<CONFIG>, NAME)

#define RTT_DYNAMIC_RECONFIGURE_SERVICE_PLUGIN(CONFIG, NAME) \
    extern "C" {\
        RTT_EXPORT bool loadRTTPlugin(RTT::TaskContext* tc);  \
        bool loadRTTPlugin(RTT::TaskContext* tc) {    \
            if (tc == 0) return true; \
            RTT::Service::shared_ptr sp( new rtt_dynamic_reconfigure::Server<CONFIG>(NAME, tc ) ); \
            return tc->provides()->addService( sp ); \
        } \
        RTT_EXPORT RTT::Service::shared_ptr createService();  \
        RTT::Service::shared_ptr createService() {    \
            RTT::Service::shared_ptr sp( new rtt_dynamic_reconfigure::Server<CONFIG>( 0 ) ); \
            return sp; \
        } \
        RTT_EXPORT std::string getRTTPluginName(); \
        std::string getRTTPluginName() { \
            return NAME; \
        } \
        RTT_EXPORT std::string getRTTTargetName(); \
        std::string getRTTTargetName() { \
            return OROCOS_TARGET_NAME; \
        } \
    }

#endif // RTT_DYNAMIC_RECONFIGURE_SERVER_H
