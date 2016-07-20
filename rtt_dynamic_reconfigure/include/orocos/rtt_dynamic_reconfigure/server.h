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
#include <rtt/Operation.hpp>
#include <rtt/OperationCaller.hpp>

#include <rtt/internal/DataSources.hpp>
#include <rtt/internal/GlobalEngine.hpp>

#include <ros/ros.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>

#include <rtt/rtt-config.h>
#if !defined(RTT_VERSION_GTE)
    #define RTT_VERSION_GTE(major,minor,patch) \
        ((RTT_VERSION_MAJOR > major) || (RTT_VERSION_MAJOR == major && \
         (RTT_VERSION_MINOR > minor) || (RTT_VERSION_MINOR == minor && \
         (RTT_VERSION_PATCH >= patch))))
#endif

namespace rtt_dynamic_reconfigure {

template <class ConfigType> class Server;
typedef bool (UpdateCallbackSignature)(const RTT::PropertyBag &source, uint32_t level);
typedef void (NotifyCallbackSignature)(uint32_t level);

/**
 * This class converts between a dynamic_reconfigure Config class and an RTT::PropertyBag.
 * The user should specialize this class for custom config types or inherit from it and reimplement
 * the propertiesFromConfig() and configFromProperties() member functions in a child class.
 */
template <class ConfigType>
struct Updater {
    virtual bool propertiesFromConfig(ConfigType &config, uint32_t level, RTT::PropertyBag &) { return false; }
    virtual bool configFromProperties(ConfigType &config, const RTT::PropertyBag &) { return false; }
};

template <class ConfigType>
struct dynamic_reconfigure_traits {
    typedef Server<ConfigType> ServerType;

    /**
     * Get the minimum values of config ConfigType.
     *
     * \param config reference to the config instance to be filled with the minimum values
     * \param server pointer to the rtt_dynamic_reconfigure server instance (only used for AutoConfig)
     */
    static void getMin(ConfigType &min, const ServerType *) { min = ConfigType::__getMin__(); }

    /**
     * Get the maximum values of config ConfigType.
     *
     * \param config reference to the config instance to be filled with the maximum values
     * \param server pointer to the rtt_dynamic_reconfigure server instance (only used for AutoConfig)
     */
    static void getMax(ConfigType &max, const ServerType *) { max = ConfigType::__getMax__(); }

    /**
     * Get the default values of config ConfigType.
     *
     * \param config reference to the config instance to be filled with the default values
     * \param server pointer to the rtt_dynamic_reconfigure server instance (only used for AutoConfig)
     */
    static void getDefault(ConfigType &dflt, const ServerType *) { dflt = ConfigType::__getDefault__(); }

    /**
     * Get the ConfigDescription of config ConfigType.
     *
     * \param server pointer to the rtt_dynamic_reconfigure server instance (only used for AutoConfig)
     * \return a shared_ptr to the dynamic_reconfigure::ConfigDescription for ConfigType
     */
    static dynamic_reconfigure::ConfigDescriptionPtr getDescriptionMessage(const ServerType *) { return dynamic_reconfigure::ConfigDescriptionPtr(new dynamic_reconfigure::ConfigDescription(ConfigType::__getDescriptionMessage__())); }

    /**
     * If true, the description of ConfigType is dynamic and can be changed during run-time (e.g. by adding/removing properties).
     */
    static const bool canRefresh = false;

    /**
     * Refresh the ConfigDescription from the properties of the owning TaskContext (no-op unless for ConfigType AutoConfig)
     *
     * \param server pointer to the rtt_dynamic_reconfigure server instance
     */
    static void refreshDescription(const ServerType *) {}

    /**
     * Convert an instance of ConfigType to a dynamic_reconfigure::Config message
     *
     * \param config referencte to the ConfigType instance to be read
     * \param message reference to the Config message to be filled
     * \param server pointer to the rtt_dynamic_reconfigure server instance (only used for AutoConfig)
     */
    static void toMessage(ConfigType &config, dynamic_reconfigure::Config &message, const ServerType *) { config.__toMessage__(message); }

    /**
     * Convert a dynamic_reconfigure::Config message to an instance of ConfigType
     *
     * \param config referencte to the ConfigType instance to be filled
     * \param message reference to the Config message to be read
     * \param server pointer to the rtt_dynamic_reconfigure server instance (only used for AutoConfig)
     */
    static void fromMessage(ConfigType &config, dynamic_reconfigure::Config &message, const ServerType *) { config.__fromMessage__(message); }

    /**
     * Clamp the values in the ConfigType instance to the limits given in the config description/properties
     *
     * \param config referencte to the ConfigType instance to be clamped
     * \param server pointer to the rtt_dynamic_reconfigure server instance (only used for AutoConfig)
     */
    static void clamp(ConfigType &config, const ServerType *) { config.__clamp__(); }

    /**
     * Creates a new RTT::internal::AssignableDataSource<RTT::PropertyBag> filled with properties from a ConfigType instance.
     *
     * \param config referencte to the ConfigType instance to be read
     * \param server pointer to the rtt_dynamic_reconfigure server instance whose updater is going to be used
     */
    static RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr getDataSource(ConfigType &config, const ServerType *server) {
        RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr ds(new RTT::internal::ValueDataSource<RTT::PropertyBag>());
        if (!server->updater()->propertiesFromConfig(config, ~0, ds->set()))
            ds.reset();
        return ds;
    }
};

namespace {
  struct null_deleter {
      void operator()(const void *) const {}
  };
}

/**
 * The Server<ConfigType> class implements a dynamic_reconfigure server as an RTT service.
 * It provides a similar API than the pure cpp dynamic_reconfigure server implemented in the <a href="http://wiki.ros.org/dynamic_reconfigure">dynamic_reconfigure</a> package.
 */
template <class ConfigType>
class Server : public RTT::Service
{
private:
    typedef Updater<ConfigType> UpdaterType;
    typedef dynamic_reconfigure_traits<ConfigType> traits;
    typedef boost::shared_ptr< Server<ConfigType> > shared_ptr;

    RTT::os::MutexRecursive mutex_;
    ros::NodeHandle *node_handle_;
    ros::ServiceServer set_service_;
    ros::Publisher update_pub_;
    ros::Publisher descr_pub_;

    ConfigType config_;
    ConfigType min_;
    ConfigType max_;
    ConfigType default_;

    mutable boost::shared_ptr<UpdaterType> updater_;
    bool initialized_;

    RTT::OperationCaller<UpdateCallbackSignature> update_callback_;
    RTT::OperationCaller<NotifyCallbackSignature> notify_callback_;
    RTT::Operation<UpdateCallbackSignature> update_callback_default_impl_;

public:
    /**
     * Construct a named rtt_dynamic_reconfigure server.
     *
     * \param name name of the service instance
     * \param owner pointer to the TaskContext instance owning this service/to be configured
     */
    Server(const std::string &name, RTT::TaskContext *owner)
        : RTT::Service(name, owner)
        , node_handle_(0)
        , update_callback_default_impl_("updateProperties", &Server<ConfigType>::updatePropertiesDefaultImpl, this, RTT::OwnThread, owner->engine())
    {
        construct();
    }

    /**
     * Construct an rtt_dynamic_reconfigure server named "reconfigure".
     *
     * \param owner pointer to the TaskContext instance owning this service/to be configured
     */
    Server(RTT::TaskContext *owner)
        : RTT::Service("reconfigure", owner)
        , node_handle_(0)
        , update_callback_default_impl_("updateProperties", &Server<ConfigType>::updatePropertiesDefaultImpl, this, RTT::OwnThread, owner->engine())
    {
        construct();
    }

    /**
     * Destruct the rtt_dynamic_reconfigure server.
     * This unadvertises the dynamic_reconfigure topics and services.
     */
    virtual ~Server() {
        shutdown();
    }

    /**
     * Update the config from an instance of ConfigType
     *
     * \param config the ConfigType instance
     */
    void updateConfig(const ConfigType &config)
    {
        updateConfigInternal(config);
    }

    /**
     * Get the maximum values of all configuration parameters
     *
     * \param config the ConfigType instance to be filled
     */
    void getConfigMax(ConfigType &max) const
    {
        max = max_;
    }

    /**
     * Get the maximum values of all configuration parameters
     *
     * \return a const reference to the ConfigType instance holding the maximum values
     */
    const ConfigType &getConfigMax() const { return max_; }

    /**
     * Get the maximum values of all configuration parameters
     *
     * \return a reference to the ConfigType instance holding the maximum values
     */
    ConfigType &getConfigMax() { return max_; }

    /**
     * Get the minimum values of all configuration parameters
     *
     * \param config the ConfigType instance to be filled
     */
    void getConfigMin(ConfigType &min) const
    {
        min = min_;
    }

    /**
     * Get the minimum values of all configuration parameters
     *
     * \return a const reference to the ConfigType instance holding the minimum values
     */
    const ConfigType &getConfigMin() const { return min_; }

    /**
     * Get the minimum values of all configuration parameters
     *
     * \return a reference to the ConfigType instance holding the minimum values
     */
    ConfigType &getConfigMin() { return min_; }

    /**
     * Get the default values of all configuration parameters
     *
     * \param config the ConfigType instance to be filled
     */
    void getConfigDefault(ConfigType &config) const
    {
        config = default_;
    }

    /**
     * Get the default values of all configuration parameters
     *
     * \return a const reference to the ConfigType instance holding the default values
     */
    const ConfigType &getConfigDefault() const { return default_; }

    /**
     * Get the default values of all configuration parameters
     *
     * \return a reference to the ConfigType instance holding the default values
     */
    ConfigType &getConfigDefault() { return default_; }

    /**
     * Set the maximum values of all configuration parameters and republish the description
     *
     * \return a const reference to the ConfigType instance holding the maximum values
     */
    void setConfigMax(const ConfigType &config)
    {
        max_ = config;
        PublishDescription();
    }

    /**
     * Set the minimum values of all configuration parameters and republish the description
     *
     * \return a const reference to the ConfigType instance holding the minimum values
     */
    void setConfigMin(const ConfigType &config)
    {
        min_ = config;
        PublishDescription();
    }

    /**
     * Set the default values of all configuration parameters and republish the description
     *
     * \return a const reference to the ConfigType instance holding the default values
     */
    void setConfigDefault(const ConfigType &config)
    {
        default_ = config;
        PublishDescription();
    }

    /**
     * Construct a new instance of the description message including the current values of the minimum, maximum and default values.
     *
     * \return a dynamic_reconfigure::ConfigDescriptionPtr instance holding the decription message
     */
    dynamic_reconfigure::ConfigDescriptionPtr getDescriptionMessage() {
        dynamic_reconfigure::ConfigDescriptionPtr description_message = traits::getDescriptionMessage(this);

        // Copy over min_ max_ default_
        traits::toMessage(max_, description_message->max, this);
        traits::toMessage(min_, description_message->min, this);
        traits::toMessage(default_, description_message->dflt, this);

        return description_message;
    }

    /**
     * Advertise the dynamic_reconfigure topics and services at the master.
     * Needs to be called explicitly after construction, e.g from the owner's configureHook().
     *
     * \param ns The ROS namespace this dynamic_reconfigure server should be advertised in. If empty, defaults to the name of the owning TaskContext.
     */
    void advertise(std::string ns = std::string())
    {
        // shutdown publishers/service servers from previous runs
        shutdown();

        // set default namespace
        if (ns.empty()) {
            if (getOwner()->getName() == "Deployer")
                ns = "~";
            else
                ns = "~" + getOwner()->getName();
        }

        // create NodeHandle
        node_handle_ = new ros::NodeHandle(ns);

        // advertise service server and publishers
        set_service_ = node_handle_->advertiseService("set_parameters", &Server<ConfigType>::setConfigCallback, this);
        descr_pub_ = node_handle_->advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
        update_pub_ = node_handle_->advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);

        // publish update once
        PublishDescription();
        updateConfigInternal(config_);
    }

    /**
     * Unadvertise the dynamic_reconfigure topics and services at the master.
     * This is the contrary of the advertise() member function.
     */
    void shutdown()
    {
        if (node_handle_) {
            node_handle_->shutdown();
            delete node_handle_;
            node_handle_ = 0;
        }
    }

    /**
     * Inform the server that some properties have been updated.
     *
     * This will republish the parameter values and updates the UI.
     */
    bool updated()
    {
        ConfigType new_config = config_;
        if (!updater()->configFromProperties(new_config, *(getOwner()->properties()))) return false;
        updateConfig(new_config);
        return true;
    }

    /**
     * Refresh the config description, minimum and maximum values, get and publish the current config from the default values and the TaskContext's properties.
     * Call this function whenever properties have been added or removed or the minimum, maximum or default values have been changed.
     */
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
        this->properties()->ownProperty(new RTT::Property<RTT::PropertyBag>("min", "Minimum values as published to dynamic_reconfigure clients", traits::getDataSource(min_, this)));
        this->properties()->ownProperty(new RTT::Property<RTT::PropertyBag>("max", "Maximum values as published to dynamic_reconfigure clients", traits::getDataSource(max_, this)));
        this->properties()->ownProperty(new RTT::Property<RTT::PropertyBag>("default", "Default values as published to dynamic_reconfigure clients", traits::getDataSource(default_, this)));

        // Get initial values from current property settings
        config_ = ConfigType();
        traits::getDefault(config_, this);
        updater()->configFromProperties(config_, *(getOwner()->properties()));
        if (node_handle_)
            config_.__fromServer__(*node_handle_);
        traits::clamp(config_, this);

        // At startup we need to load the configuration with all level bits set (everything has changed).
        RTT::PropertyBag init_config;
        updater()->propertiesFromConfig(config_, ~0, init_config);

        // Invoke update and notification callback
#if !RTT_VERSION_GTE(2,8,99)
        // Additional check for RTT < 2.9:
        // ===============================
        // We do not know for sure which thread is calling this method/operation, but we can check if the current
        // thread is the same as the thread that will process the update/notify operation. If yes, we clone the
        // underlying OperationCaller implementation and set the caller to the processing engine. In this case
        // RTT < 2.9 should always call the operation directly as if it would be a ClientThread operation:
        // https://github.com/orocos-toolchain/rtt/blob/toolchain-2.8/rtt/base/OperationCallerInterface.hpp#L79
        //
        // RTT 2.9 and above already checks the caller thread internally and therefore does not require this hack.
        //
        RTT::base::OperationCallerBase<UpdateCallbackSignature>::shared_ptr update_callback_impl = update_callback_.getOperationCallerImpl();
        if (update_callback_impl && update_callback_impl->isSend()) {
            RTT::ExecutionEngine *engine = update_callback_impl->getMessageProcessor();
            if (engine && engine->getThread() && engine->getThread()->isSelf()) {
                RTT::Logger::In in(this->getOwner()->getName() + "." + this->getName());
                RTT::log(RTT::Debug) << "calling my own updateProperties operation from refresh()" << RTT::endlog();
                update_callback_impl.reset(update_callback_impl->cloneI(engine));
                update_callback_impl->call(init_config, ~0);
            } else {
                update_callback_(init_config, ~0);
            }
        }
        RTT::base::OperationCallerBase<NotifyCallbackSignature>::shared_ptr notify_callback_impl = notify_callback_.getOperationCallerImpl();
        if (notify_callback_impl && notify_callback_impl->isSend()) {
            RTT::ExecutionEngine *engine = notify_callback_impl->getMessageProcessor();
            if (engine && engine->getThread() && engine->getThread()->isSelf()) {
                RTT::Logger::In in(this->getOwner()->getName() + "." + this->getName());
                RTT::log(RTT::Debug) << "calling my own notifyPropertiesUpdate operation from refresh()" << RTT::endlog();
                notify_callback_impl.reset(notify_callback_impl->cloneI(engine));
                notify_callback_impl->call(~0);
            } else {
                notify_callback_(~0);
            }
        }
#else
        update_callback_(init_config, ~0);
        if (notify_callback_.ready()) notify_callback_(~0);
#endif

        updateConfigInternal(config_);
    }

    /**
     * Retrieve/construct the Updater instance of this rtt_dynamic_reconfigure server.
     *
     * \return a pointer to an instance of Updater<ConfigType>
     */
    UpdaterType *updater() const
    {
        if (!updater_) updater_.reset(new UpdaterType());
        return updater_.get();
    }

    /**
     * Sets the Updater instance to use to update config from properties or vice-versa.
     *
     * \param updater a pointer to an instance of Updater<ConfigType>
     */
    void setUpdater(UpdaterType *updater)
    {
        updater_.reset(updater, null_deleter());
    }

    /**
     * Sets the property update callback (defaults to RTT::updateProperties(*(getOwner()->properties()), ...))
     *
     * \param impl an Operation with the signature bool(const RTT::PropertyBag &)
     */
    void setUpdateCallback(RTT::OperationInterfacePart *impl)
    {
        update_callback_ = impl;
    }

    /**
     * Sets the notification callback that notifies a TaskContext that its properties have been updated
     *
     * \param impl an Operation with the signature void()
     */
    void setNotificationCallback(RTT::OperationInterfacePart *impl)
    {
        notify_callback_ = impl;
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

        this->addOperation("refresh", &Server<ConfigType>::refresh, this)
            .doc("Rediscover the owner's properties or update advertised min/max/default values. Call this operation after having added properties.");

        // check if owner implements the Updater interface
        UpdaterType *updater = dynamic_cast<UpdaterType *>(getOwner());
        if (updater) setUpdater(updater);

        // check if owner provides the updateProperties operation
        if (getOwner() && getOwner()->provides()->hasOperation("updateProperties")) {
            update_callback_ = getOwner()->provides()->getLocalOperation("updateProperties");
        } else {
            update_callback_ = update_callback_default_impl_.getOperationCaller();
        }

        // check if owner provides the notifyPropertiesUpdate operation
        if (getOwner() && getOwner()->provides()->hasOperation("notifyPropertiesUpdate")) {
            notify_callback_ = getOwner()->provides()->getLocalOperation("notifyPropertiesUpdate");
        }

        // update_callback_ and notify_callback_ are called from the ROS spinner thread -> set GlobalEngine as caller engine
        update_callback_.setCaller(RTT::internal::GlobalEngine::Instance());
        notify_callback_.setCaller(RTT::internal::GlobalEngine::Instance());

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

        RTT::PropertyBag bag;
        if (!updater()->propertiesFromConfig(new_config, level, bag)) return false;
        if (!update_callback_.ready() || !update_callback_(bag, level)) return false;
        if (notify_callback_.ready()) notify_callback_(level);

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

    bool updatePropertiesDefaultImpl(const RTT::PropertyBag &source, uint32_t)
    {
        return RTT::updateProperties(*(getOwner()->properties()), source);
    }
};

/**
 * Sets a named property in a PropertyBag by its name.
 * If the property does not exists, it is created as a reference to the given value if the type matches or as a copy if the type is different.
 *
 * Use this function in Updater<ConfigType>::propertiesFromConfig() to fill the PropertyBag from config values.
 *
 * \param name the name of the property
 * \param bag the PropertyBag in which the property should be set/added
 * \param value the value of the property
 */
template <typename T, typename ValueType>
bool setProperty(const std::string &name, RTT::PropertyBag &bag, ValueType &value)
{
    if (bag.getProperty(name)) {
        RTT::Property<T> *prop = bag.getPropertyType<T>(name);
        if (!prop) {
            RTT::log(RTT::Error) << "Could not assign property '" << name << "': Property exists with a different type." << RTT::endlog();
            return false;
        }

        prop->set() = value;

    } else {
        if (boost::is_same<T,ValueType>::value) {
            bag.addProperty(name, value);
        } else {
            bag.ownProperty(new RTT::Property<T>(name, std::string(), value));
        }
    }

    return true;
}

/**
 * Gets a named property in a PropertyBag by its name.
 * If the property does not exists, it is created as a reference to the given value if the type matches or as a copy if the type is different.
 *
 * Use this function in Updater<ConfigType>::configFromProperties() to fill the ConfigType struct with the values in the PropertyBag.
 *
 * \param name the name of the property
 * \param bag the PropertyBag to be searched for the named property
 * \param value the value of the property
 */
template <typename T, typename ValueType>
bool getProperty(const std::string &name, const RTT::PropertyBag &bag, ValueType &value)
{
    RTT::Property<T> *prop = bag.getPropertyType<T>(name);
    if (!prop) {
        RTT::log(RTT::Error) << "Could not get property '" << name << "': No such property in the bag." << RTT::endlog();
        return false;
    }

    value = prop->rvalue();
    return true;
}

} // namespace rtt_dynamic_reconfigure

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
