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

#include <rtt_dynamic_reconfigure/auto_config.h>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSources.hpp>
#include <rtt/types/PropertyComposition.hpp>

#include <cassert>
#include <climits>
#include <cfloat>

#include <boost/thread/locks.hpp>

#include <dynamic_reconfigure/config_tools.h>

namespace rtt_dynamic_reconfigure {

using namespace dynamic_reconfigure;

/**
 * A special datasource that holds an instance of an AutoConfig description.
 *
 * The AutoConfigDataSource is used internally to store AutoConfig instances in a Property<RTT::PropertyBag>.
 */
class AutoConfigDataSource
    : public RTT::internal::AssignableDataSource<RTT::PropertyBag>
{
protected:
    AutoConfig mdata;

public:
    ~AutoConfigDataSource() {}

    typedef boost::intrusive_ptr<AutoConfigDataSource> shared_ptr;

    AutoConfigDataSource(const AutoConfig &data) : mdata(data) {}
    AutoConfigDataSource() {}

    RTT::internal::DataSource<RTT::PropertyBag>::result_t get() const { return mdata; }
    RTT::internal::DataSource<RTT::PropertyBag>::result_t value() const { return mdata; }

    void set( typename AssignableDataSource<RTT::PropertyBag>::param_t t ) { mdata = t; }
    AutoConfig& set() { return mdata; }
    const AutoConfig& rvalue() const { return mdata; }

    virtual AutoConfigDataSource* clone() const { return new AutoConfigDataSource(mdata); }

    /* copied from ValueDataSource<T>::copy() in DataSources.inl */
    virtual AutoConfigDataSource* copy( std::map<const RTT::base::DataSourceBase*, RTT::base::DataSourceBase*>& replace ) const
    {
        // if somehow a copy exists, return the copy, otherwise return this (see Attribute copy)
        if ( replace[this] != 0 ) {
            assert ( dynamic_cast<AutoConfigDataSource*>( replace[this] ) == static_cast<AutoConfigDataSource*>( replace[this] ) );
            return static_cast<AutoConfigDataSource*>( replace[this] );
        }
        // Other pieces in the code rely on insertion in the map :
        replace[this] = const_cast<AutoConfigDataSource*>(this);
        // return this instead of a copy.
        return const_cast<AutoConfigDataSource*>(this);
    }

    /**
     * This method narrows a base::DataSourceBase to a AutoConfigDataSource,
     * possibly returning a new object.
     */
    static AutoConfigDataSource* narrow(RTT::base::DataSourceBase* dsb) {
        AutoConfigDataSource* ret = dynamic_cast< AutoConfigDataSource* >( dsb );
        return ret;
    }
};

AutoConfig::AutoConfig()
    : parent(), id(), state()
{
}

AutoConfig::AutoConfig(const RTT::PropertyBag &bag)
    : parent(), id(), state()
{
    this->fromProperties(bag);
}

AutoConfig::~AutoConfig()
{
}

// default type
template <typename T> struct PropertyTypeInfo {
    typedef std::string dynamic_reconfigure_type;
    static std::string getType() { return "str"; }
    static bool hasLimits() { return false; }
    static T getMin() { return std::numeric_limits<T>::lowest(); }
    static T getMax() { return std::numeric_limits<T>::max(); }
};

template <> struct PropertyTypeInfo<bool>
{
    typedef bool dynamic_reconfigure_type;
    static std::string getType() { return "bool"; }
    static bool hasLimits() { return false; }
    static bool getMin() { return false; }
    static bool getMax() { return true; }
};

template <> struct PropertyTypeInfo<int>
{
    typedef int dynamic_reconfigure_type;
    static std::string getType() { return "int"; }
    static bool hasLimits() { return true; }
    static int getMin() { return INT_MIN; }
    static int getMax() { return INT_MAX; }
};

template <> struct PropertyTypeInfo<unsigned int>
{
    typedef int dynamic_reconfigure_type;
    static std::string getType() { return "int"; }
    static bool hasLimits() { return true; }
    static int getMin() { return 0; }
    static int getMax() { return INT_MAX; }
};

template <> struct PropertyTypeInfo<std::string>
{
    typedef std::string dynamic_reconfigure_type;
    static std::string getType() { return "str"; }
    static bool hasLimits() { return false; }
    static std::string getMin() { return ""; }
    static std::string getMax() { return ""; }
};

template <> struct PropertyTypeInfo<double>
{
    typedef double dynamic_reconfigure_type;
    static std::string getType() { return "double"; }
    static bool hasLimits() { return true; }
    static double getMin() { return -DBL_MAX; }
    static double getMax() { return  DBL_MAX; }
};

template <> struct PropertyTypeInfo<float>
{
    typedef double dynamic_reconfigure_type;
    static std::string getType() { return "double"; }
    static bool hasLimits() { return true; }
    static double getMin() { return -FLT_MAX; }
    static double getMax() { return  FLT_MAX; }
};

static AutoConfig *getAutoConfigFromProperty(const RTT::base::PropertyBase *pb)
{
    const RTT::Property<RTT::PropertyBag> *prop = dynamic_cast<const RTT::Property<RTT::PropertyBag> *>(pb);
    if (!prop) return 0;
    AutoConfigDataSource *ds = AutoConfigDataSource::narrow(prop->getDataSource().get());
    if (!ds) return 0;
    return &(ds->set());
}

template <typename T>
static bool propertyFromMessage(AutoConfig &config, Config &msg, const RTT::base::PropertyBase *sample, const std::string &param_name)
{
    const RTT::Property<T> *sample_prop = dynamic_cast<const RTT::Property<T> *>(sample);
    if (!sample_prop) return false;

    typename PropertyTypeInfo<T>::dynamic_reconfigure_type value;
    if (!ConfigTools::getParameter(msg, param_name, value)) return false;

    RTT::Property<T> *prop = config.getPropertyType<T>(sample->getName());
    if (!prop) {
        prop = sample_prop->create();
        config.ownProperty(prop);
    }
    prop->set(value);
    return true;
}

bool AutoConfig::__fromMessage__(Config &msg, const AutoConfig &sample)
{
    return __fromMessage__(*this, msg, sample);
}

bool AutoConfig::__fromMessage__(AutoConfig &config, Config &msg, const AutoConfig &sample)
{
    // get group state
    config.prefix_ = sample.prefix_;
    config.name = sample.name;
    config.id = sample.id;
    config.parent = sample.parent;
    dynamic_reconfigure::ConfigTools::getGroupState(msg, config.name, config);

    // iterate over all properties in sample
    bool result = true;
    for(RTT::PropertyBag::const_iterator i = sample.begin(); i != sample.end(); ++i) {
        std::string param_name = config.prefix_ + (*i)->getName();

        // For sub groups, add a sub config to *this and recurse...
        const AutoConfig *sample_sub = getAutoConfigFromProperty(*i);
        if (sample_sub) {
            RTT::Property<RTT::PropertyBag> *sub = config.getPropertyType<RTT::PropertyBag>((*i)->getName());
            AutoConfigDataSource *ds;
            if (sub) {
                ds = AutoConfigDataSource::narrow(sub->getDataSource().get());
                assert(ds->rvalue().getType() == sample_sub->getType());
            } else {
                ds = new AutoConfigDataSource();
                ds->set().setType(sample_sub->getType());
            }

            if (ds && __fromMessage__(ds->set(), msg, *sample_sub)) {
                if (!sub) {
                    // new AutoConfigDataSource
                    if (!ds->rvalue().empty()) {
                        sub = new RTT::Property<RTT::PropertyBag>((*i)->getName(), (*i)->getDescription(), AutoConfigDataSource::shared_ptr(ds));
                        config.ownProperty(sub);
                    } else {
                        delete ds;
                    }
                }
                continue;
            }
        }

        // search parameter in Config message
        bool param_found = false;
        for(Config::_bools_type::const_iterator n = msg.bools.begin(); n != msg.bools.end(); ++n) {
            if (n->name == param_name) param_found = true;
        }
        for(Config::_ints_type::const_iterator n = msg.ints.begin(); n != msg.ints.end(); ++n) {
            if (n->name == param_name) param_found = true;
        }
        for(Config::_strs_type::const_iterator n = msg.strs.begin(); n != msg.strs.end(); ++n) {
            if (n->name == param_name) param_found = true;
        }
        for(Config::_doubles_type::const_iterator n = msg.doubles.begin(); n != msg.doubles.end(); ++n) {
            if (n->name == param_name) param_found = true;
        }
        if (!param_found) continue;

        // get parameter value from Config message
        if (
            propertyFromMessage<bool>(config, msg, *i, param_name) ||
            propertyFromMessage<int>(config, msg, *i, param_name) ||
            propertyFromMessage<unsigned int>(config, msg, *i, param_name) ||
            propertyFromMessage<std::string>(config, msg, *i, param_name) ||
            propertyFromMessage<double>(config, msg, *i, param_name) ||
            propertyFromMessage<float>(config, msg, *i, param_name)
           ) continue;

        result = false;
    }

    return result;
}

template <typename T>
static bool propertyToMessage(Config &msg, const RTT::base::PropertyBase *pb, const std::string &_prefix)
{
    const RTT::Property<T> *prop = dynamic_cast<const RTT::Property<T> *>(pb);
    if (!prop) return false;

    typename PropertyTypeInfo<T>::dynamic_reconfigure_type value = prop->get();
    ConfigTools::appendParameter(msg, _prefix + pb->getName(), value);
    return true;
}

void AutoConfig::__toMessage__(Config &msg) const
{
    __toMessage__(*this, msg);
}

void AutoConfig::__toMessage__(const AutoConfig &config, Config &msg)
{
    // add group state
    dynamic_reconfigure::ConfigTools::appendGroup(msg, config.name, config.id, config.parent, config);

    // iterate over all properties
    bool result = true;
    for(RTT::PropertyBag::const_iterator i = config.begin(); i != config.end(); ++i) {
        if (propertyToMessage<bool>(msg, *i, config.prefix_) ||
            propertyToMessage<int>(msg, *i, config.prefix_) ||
            propertyToMessage<unsigned int>(msg, *i, config.prefix_) ||
            propertyToMessage<std::string>(msg, *i, config.prefix_) ||
            propertyToMessage<double>(msg, *i, config.prefix_) ||
            propertyToMessage<float>(msg, *i, config.prefix_)
           ) continue;

        // test if *i has type AutoConfig
        const AutoConfig *sub = getAutoConfigFromProperty(*i);
        if (sub) {
            __toMessage__(*sub, msg);
            continue;
        }

        result = false;
    }
}

void AutoConfig::__toServer__(const ros::NodeHandle &nh) const
{

}

void AutoConfig::__fromServer__(const ros::NodeHandle &nh)
{

}

void AutoConfig::__clamp__(const ServerType *server)
{
    const AutoConfig &min = server->getConfigMin();
    const AutoConfig &max = server->getConfigMax();

    // TODO: clamp values
}

uint32_t AutoConfig::__level__(const AutoConfig &config) const
{
    return 0;
}

bool AutoConfig::updateProperties(RTT::PropertyBag &target) const
{
    RTT::PropertyBag composed;
    if (!RTT::types::composePropertyBag(*this, composed)) return false;
    return RTT::updateProperties(target, composed);
}

bool AutoConfig::fromProperties(const RTT::PropertyBag &source)
{
    RTT::PropertyBag decomposed;
    if (!RTT::types::decomposePropertyBag(source, decomposed)) return false;

    for(RTT::PropertyBag::const_iterator i = decomposed.begin(); i != decomposed.end(); ++i) {
        RTT::base::PropertyBase *pb = this->getProperty((*i)->getName());
        if (pb) {
            pb->update(*i);
            continue;
        }

        RTT::Property<RTT::PropertyBag> *sub = dynamic_cast<RTT::Property<RTT::PropertyBag> *>(*i);
        if (sub) {
            AutoConfigDataSource *ds = new AutoConfigDataSource(sub->rvalue());
            ds->set().setType(sub->rvalue().getType());
            this->ownProperty(new RTT::Property<RTT::PropertyBag>(sub->getName(), sub->getDescription(), ds));
            continue;
        } else {
            this->ownProperty((*i)->clone());
        }
    }

    return true;
}

template <typename T>
static bool buildParamDescription(const RTT::base::PropertyBase *pb, const std::string &prefix, Group::_parameters_type& params, AutoConfig& dflt, AutoConfig& min, AutoConfig& max)
{
    const RTT::Property<T> *prop = dynamic_cast<const RTT::Property<T> *>(pb);
    if (!prop) return false;

    ParamDescription param;
    param.name = prefix + pb->getName();
    param.type = PropertyTypeInfo<T>::getType();
    param.description = pb->getDescription();
    params.push_back(param);

    // get current value as default
    if (!dflt.getProperty(pb->getName())) {
        RTT::Property<T> *dflt_prop = prop->create();
        dflt_prop->set(prop->get());
        dflt.ownProperty(dflt_prop);
    }

    // get minimum/maximum value
    if (!min.getProperty(pb->getName())) {
        RTT::Property<T> *min_prop = prop->create();
        min_prop->set(PropertyTypeInfo<T>::getMin());
        min.ownProperty(min_prop);
    }
    if (!max.getProperty(pb->getName())) {
        RTT::Property<T> *max_prop = prop->create();
        max_prop->set(PropertyTypeInfo<T>::getMax());
        max.ownProperty(max_prop);
    }

    return true;
}

static void buildGroupDescription(RTT::TaskContext *owner, const RTT::PropertyBag &bag, ConfigDescription& config_description, AutoConfig& dflt, AutoConfig& min, AutoConfig& max, const std::string &prefix, const std::string &name, const std::string &type, int32_t parent, int32_t id)
{
    std::size_t group_index = config_description.groups.size();
    config_description.groups.push_back(Group());

    Group &group = config_description.groups[group_index];
    group.name = name.empty() ? "Default" : name;
    group.type = type;
    group.parent = parent;
    group.id = id;

    dflt.prefix_ = prefix;
    dflt.name = group.name;
    dflt.type = group.type;
    dflt.parent = group.parent;
    dflt.id = group.id;
    dflt.state = true;

    min.prefix_ = prefix;
    min.name = group.name;
    min.type = group.type;
    min.parent = group.parent;
    min.id = group.id;
    min.state = true;

    max.prefix_ = prefix;
    max.name = group.name;
    max.type = group.type;
    max.parent = group.parent;
    max.id = group.id;
    max.state = true;

    // for loop might invalidate group reference -> use index group_index instead
    for(RTT::PropertyBag::const_iterator i = bag.begin(); i != bag.end(); ++i) {
        if (buildParamDescription<bool>(*i, prefix, config_description.groups[group_index].parameters, dflt, min, max) ||
            buildParamDescription<int>(*i, prefix, config_description.groups[group_index].parameters, dflt, min, max) ||
            buildParamDescription<unsigned int>(*i, prefix, config_description.groups[group_index].parameters, dflt, min, max) ||
            buildParamDescription<std::string>(*i, prefix, config_description.groups[group_index].parameters, dflt, min, max) ||
            buildParamDescription<double>(*i, prefix, config_description.groups[group_index].parameters, dflt, min, max) ||
            buildParamDescription<float>(*i, prefix, config_description.groups[group_index].parameters, dflt, min, max)
           ) continue;

        const RTT::Property<RTT::PropertyBag> *sub = dynamic_cast<RTT::Property<RTT::PropertyBag> *>(*i);
        if (sub) {
            AutoConfig *sub_dflt = getAutoConfigFromProperty(dflt.getProperty(sub->getName()));
            if (!sub_dflt) {
                AutoConfigDataSource *ds = new AutoConfigDataSource();
                sub_dflt = &(ds->set());
                sub_dflt->setType(sub->rvalue().getType());
                dflt.ownProperty(new RTT::Property<RTT::PropertyBag>(sub->getName(), sub->getDescription(), ds));
            }

            AutoConfig *sub_min = getAutoConfigFromProperty(min.getProperty(sub->getName()));
            if (!sub_min) {
                AutoConfigDataSource *ds = new AutoConfigDataSource();
                sub_min = &(ds->set());
                sub_min->setType(sub->rvalue().getType());
                min.ownProperty(new RTT::Property<RTT::PropertyBag>(sub->getName(), sub->getDescription(), ds));
            }

            AutoConfig *sub_max = getAutoConfigFromProperty(max.getProperty(sub->getName()));
            if (!sub_max) {
                AutoConfigDataSource *ds = new AutoConfigDataSource();
                sub_max = &(ds->set());
                sub_max->setType(sub->rvalue().getType());
                max.ownProperty(new RTT::Property<RTT::PropertyBag>(sub->getName(), sub->getDescription(), ds));
            }

            buildGroupDescription(owner, sub->rvalue(), config_description, *sub_dflt, *sub_min, *sub_max, prefix + sub->getName() + "__", prefix + sub->getName(), "", config_description.groups[group_index].id, ++id);
        }
    }
}

std::map<const AutoConfig::ServerType *, AutoConfig::CachePtr> AutoConfig::cache_;
boost::shared_mutex AutoConfig::cache_mutex_;

void AutoConfig::buildCache(const ServerType *server, RTT::TaskContext *owner)
{
    RTT::PropertyBag decomposed;
    if (!RTT::types::decomposePropertyBag(*(owner->properties()), decomposed)) {
        RTT::log(RTT::Error) << "Failed to decompose properties of '" << owner->getName() << "' for dynamic_reconfigure. Properties with custom types will not be available for reconfiguration." << RTT::endlog();
        decomposed = *(owner->properties());
    }

    boost::upgrade_lock<boost::shared_mutex> upgrade_lock(cache_mutex_);
    if (upgrade_lock.owns_lock())
    {
        boost::upgrade_to_unique_lock<boost::shared_mutex> unique_lock(upgrade_lock);
        CachePtr& cache = cache_[server];
        if (!cache) cache.reset(new Cache());
        cache->description_message_.reset(new ConfigDescription);
        buildGroupDescription(owner, decomposed, *(cache->description_message_), cache->default_, cache->min_, cache->max_, "", "", "", 0, 0);
    }
}

dynamic_reconfigure::ConfigDescriptionPtr AutoConfig::__getDescriptionMessage__(const ServerType *server)
{
    boost::shared_lock<boost::shared_mutex> lock(cache_mutex_);
    if (!cache_.count(server)) buildCache(server, server->getOwner());
    return cache_.at(server)->description_message_;
}

const AutoConfig &AutoConfig::__getDefault__(const ServerType *server)
{
    boost::shared_lock<boost::shared_mutex> lock(cache_mutex_);
    if (!cache_.count(server)) buildCache(server, server->getOwner());
    return cache_.at(server)->default_;
}

const AutoConfig &AutoConfig::__getMax__(const ServerType *server)
{
    boost::shared_lock<boost::shared_mutex> lock(cache_mutex_);
    if (!cache_.count(server)) buildCache(server, server->getOwner());
    return cache_.at(server)->max_;
}

const AutoConfig &AutoConfig::__getMin__(const ServerType *server)
{
    boost::shared_lock<boost::shared_mutex> lock(cache_mutex_);
    if (!cache_.count(server)) buildCache(server, server->getOwner());
    return cache_.at(server)->min_;
}

void AutoConfig::__refreshDescription__(const ServerType *server)
{
    buildCache(server, server->getOwner());
}

} // namespace rtt_dynamic_reconfigure
