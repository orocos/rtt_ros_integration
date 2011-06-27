#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/marsh/PropertyBagIntrospector.hpp>

#include <ros/ros.h>
#include <stack>

using namespace RTT;
using namespace std;

class RosParam: public RTT::Service
{
public:
    RosParam(TaskContext* owner) :
        Service("rosparam", owner)
    {
        addOperation("storeProperties", &RosParam::storeProperties, this) .doc(
                "Stores all properties of this component to the ros param server");
        addOperation("refreshProperties", &RosParam::refreshProperties, this).doc(
                "Refreshes all properties of this component from the ros param server");
        addOperation("storeProperty", &RosParam::storeProperty, this) .doc(
                "Stores one property of this component to the ros param server").arg(
                "param_name", "Name of the property.").arg("private",
                "true if parameter should be put in private namespace").arg(
                "relative",
                "true if parameter should be put in the relative (component name) namespace");
        addOperation("refreshProperty", &RosParam::refreshProperty, this) .doc(
                "Refreshes one property of this component from the ros param server").arg(
                "param_name", "Name of the property.").arg("private",
                "true if parameter should be found the private namespace").arg(
                "relative",
                "true if parameter should be found in the relative (component name) namespace");
        this->doc("Store component properties on the ROS parameter server or refresh them using values on the ROS parameter server");
    }
private:

    std::stack<XmlRpc::XmlRpcValue> value_stack;

    bool storeProperties()
    {
        Property<PropertyBag> bag(getOwner()->getName(), "");
        // decompose prop into primitive property types.
        marsh::PropertyBagIntrospector pbi(bag.value());
        pbi.introspect(*this->getOwner()->properties());
        if (PropertyToXmlRpcValue(&bag))
        {
            bool retval = true;
            assert(!value_stack.empty());
            try
            {
                ros::param::set(string("~") + bag.getName(), value_stack.top());
            } catch (ros::InvalidNameException ex)
            {
                log(Error) << ex.what() << endlog();
                retval = false;
            }
            value_stack.pop();
            assert(value_stack.empty());
            return retval;
        }
        return false;
    }
    bool storeProperty(const string& prop_name, bool priv, bool rel)
    {
        base::PropertyBase* base = this->getOwner()->properties()->getProperty(
                prop_name);
        if (base == NULL)
        {
            log(Error) << this->getOwner()->getName()
                    << " does not have a Property with name " << prop_name
                    << endlog();
            return false;
        }
        string param_name;
        if (priv)
            param_name = "~";
        if (rel)
            param_name += this->getOwner()->getName() + string("/");
        param_name += base->getName();
        if (PropertyToXmlRpcValue(base))
        {
            bool retval = true;
            assert(!value_stack.empty());
            try
            {
                ros::param::set(param_name, value_stack.top());
            } catch (ros::InvalidNameException ex)
            {
                log(Error) << ex.what() << endlog();
                retval = false;
            }
            value_stack.pop();
            assert(value_stack.empty());
            return retval;

        }
        else
        {
            //check if base can be decomposed into a PropertyBag, if so we need to decompose it.
            PropertyBag bag;
            bag.add(base);
            // decompose prop into primitive property types.
            PropertyBag decomposed_bag;
            marsh::PropertyBagIntrospector pbi(decomposed_bag);
            pbi.introspect(bag);
            if (PropertyToXmlRpcValue(decomposed_bag.getProperty(
                    base->getName())))
            {
                bool retval = true;
                assert(!value_stack.empty());
                try
                {
                    ros::param::set(param_name, value_stack.top());
                } catch (ros::InvalidNameException ex)
                {
                    log(Error) << ex.what() << endlog();
                    retval = false;
                }
                value_stack.pop();
                assert(value_stack.empty());
                return retval;
            }
            return false;
        }
        return false;
    }

    bool PropertyToXmlRpcValue(base::PropertyBase* prop)
    {
        return PropertyToXmlRpcValue<bool> (
                dynamic_cast<Property<bool>*> (prop)) || PropertyToXmlRpcValue<
                int> (dynamic_cast<Property<int>*> (prop))
                || PropertyToXmlRpcValue<double> (
                        dynamic_cast<Property<double>*> (prop))
                || PropertyToXmlRpcValue<string> (dynamic_cast<Property<
                        std::string>*> (prop)) || PropertyToXmlRpcValue(
                dynamic_cast<Property<PropertyBag>*> (prop));
    }

    template<class T>
    bool PropertyToXmlRpcValue(Property<T>* prop)
    {
        if (!prop)
            return false;

        value_stack.push(XmlRpc::XmlRpcValue(prop->rvalue()));
        return true;
    }

    bool PropertyToXmlRpcValue(Property<PropertyBag>* prop)
    {
        if (!prop)
            return false;
        PropertyBag& bag = prop->value();
        XmlRpc::XmlRpcValue dict;
        for (size_t i = 0; i < bag.size(); i++)
        {
            if (PropertyToXmlRpcValue(bag.getItem(i)))
            {
                if (!bag.getItem(i)->getName().empty() && bag.getType()
                        != "array")
                    dict[bag.getItem(i)->getName()] = value_stack.top();
                else
                    dict[i] = value_stack.top();
                value_stack.pop();
            }
        }
        if (bag.size() == 0)
            log(Warning) << "Exporting empty property bag " << prop->getName()
                    << endlog();
        value_stack.push(dict);
        return true;
    }

    bool refreshProperties()
    {
        XmlRpc::XmlRpcValue rpcval;
        try
        {
            if (!ros::param::get(string("~") + this->getOwner()->getName(),
                    rpcval))
            {
                log(Error)
                        << "The parameter server does not have a Property with name "
                        << this->getOwner()->getName() << endlog();
                return false;
            }
        } catch (ros::InvalidNameException ex)
        {
            log(Error) << ex.what() << endlog();
            return false;
        }
        Property<PropertyBag> bag(getOwner()->getName(), "");
        // decompose prop into primitive property types.
        marsh::PropertyBagIntrospector pbi(bag.value());
        pbi.introspect(*this->getOwner()->properties());
        for (PropertyBag::iterator it = bag.value().begin(); it
                != bag.value().end(); ++it)
        {
            if (rpcval.hasMember((*it)->getName()))
            {
                if (!XmlRpcValueToProperty(rpcval[(*it)->getName()], (*it)))
                    log(Warning) << "Could not update Property "
                            << (*it)->getName() << endlog();
                base::PropertyBase* base =
                        this->getOwner()->properties()->getProperty(
                                (*it)->getName());
                if (!base->getTypeInfo()->composeType((*it)->getDataSource(),
                        base->getDataSource()))
                    log(Warning)<<"Could not compose "<<base->getName()<<endlog();
            }
            else
                log(Warning) << "Could not find Property " << (*it)->getName()
                        << endlog();
        }
        return true;
    }

    bool refreshProperty(const string& prop_name, bool priv, bool rel)
    {
        base::PropertyBase* base = this->getOwner()->properties()->getProperty(
                prop_name);
        if (base == NULL)
        {
            log(Error) << this->getOwner()->getName()
                    << " does not have a Property with name " << prop_name
                    << endlog();
            return false;
        }
        string param_name;
        if (priv)
            param_name = "~";
        if (rel)
            param_name += this->getOwner()->getName() + string("/");
        param_name += base->getName();
        XmlRpc::XmlRpcValue rpcval;
        try
        {
            if (!ros::param::get(param_name, rpcval))
            {
                log(Error)
                        << "The parameter server does not have a Property with name "
                        << param_name << endlog();
                return false;
            }
        } catch (ros::InvalidNameException ex)
        {
            log(Error) << ex.what() << endlog();
            return false;
        }
        PropertyBag bag;
        bag.add(base);
        // decompose prop into primitive property types.
        PropertyBag decomposed_bag;
        marsh::PropertyBagIntrospector pbi(decomposed_bag);
        pbi.introspect(bag);
        if (!XmlRpcValueToProperty(rpcval, decomposed_bag.getProperty(
                base->getName())))
            return false;
        if (base->getTypeInfo()->composeType(decomposed_bag.getProperty(
                base->getName())->getDataSource(), base->getDataSource()))
            return true;
        return false;
    }

    bool XmlRpcValueToProperty(XmlRpc::XmlRpcValue &val,
            base::PropertyBase* prop)
    {
        switch (val.getType())
        {
        case XmlRpc::XmlRpcValue::TypeBoolean:
        {
            Property<bool> tmp("");
            tmp.set(val);
            if (prop->refresh(&tmp))
                return true;
            return false;
        }
        case XmlRpc::XmlRpcValue::TypeDouble:
        {
            Property<double> tmp("");
            tmp.set(val);
            if (prop->refresh(&tmp))
                return true;
            return false;
        }
        case XmlRpc::XmlRpcValue::TypeInt:
        {
            Property<int> tmp("");
            tmp.set(val);
            if (prop->refresh(&tmp))
                return true;
            return false;
        }
        case XmlRpc::XmlRpcValue::TypeString:
        {
            Property<std::string> tmp("");
            tmp.set(val);
            if (prop->refresh(&tmp))
                return true;
            return false;
        }
        case XmlRpc::XmlRpcValue::TypeArray:
        {
            Property<PropertyBag> bag("");
            bag = prop;
            if (!bag.ready())
                return false;
            if (val.size() != (int) bag.value().size())
                return false;
            for (int i = 0; i < val.size(); i++)
            {
                if (!XmlRpcValueToProperty(val[i], bag.value().getItem(i)))
                    return false;
            }
            //is this necessary?
            if (prop->getTypeInfo()->composeType(bag.getDataSource(),
                    prop->getDataSource()))
                return true;
            return false;
        }
        case XmlRpc::XmlRpcValue::TypeStruct:
        {
            Property<PropertyBag> bag("");
            bag = prop;
            if (!bag.ready())
                return false;
            if (bag.value().empty())
            {
                log(Warning) << "Could not update " << prop->getName()
                        << endlog();
                return false;
            }
            for (PropertyBag::iterator it = bag.value().begin(); it
                    != bag.value().end(); ++it)
            {
                if (val.hasMember((*it)->getName()))
                {
                    if (!XmlRpcValueToProperty(val[(*it)->getName()],
                            bag.value().getProperty((*it)->getName())))
                        log(Warning) << "Could not convert "
                                << (*it)->getName() << " from XmlRpcValue"
                                << endlog();
                }
                else
                    log(Warning) << "Could not find " << (*it)->getName()
                            << " in " << prop->getName() << endlog();
            }
            //is this necessary?
            if (prop->getTypeInfo()->composeType(bag.getDataSource(),
                    prop->getDataSource()))
                return true;
            return false;
        }
        default:
        {
            log(Warning) << "Cannot handle the type of " << prop->getName()
                    << endlog();
            return false;
        }
        }
    }

};


ORO_SERVICE_NAMED_PLUGIN(RosParam, "rosparam")
