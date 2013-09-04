#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/marsh/PropertyBagIntrospector.hpp>

#include <ros/ros.h>
#include <stack>

using namespace RTT;
using namespace std;

class ROSParamService: public RTT::Service
{
public:
  ROSParamService(TaskContext* owner) :
    Service("rosparam", owner)
  {
    this->doc("RTT Service for synchronizing ROS parameters with the properties of a corresponding RTT component");

    this->addOperation("storeProperties", &ROSParamService::storeProperties, this) 
      .doc("Stores all properties of this component to the ros param server");
    this->addOperation("refreshProperties", &ROSParamService::refreshProperties, this)
      .doc("Refreshes all properties of this component from the ros param server");
    this->addOperation("storeProperty", &ROSParamService::storeProperty, this) 
      .doc("Stores one property of this component to the ros param server")
      .arg("param_name", "Name of the property.")
      .arg("private", "true if parameter should be put in private namespace")
      .arg("relative", "true if parameter should be put in the relative (component name) namespace");
    this->addOperation("refreshProperty", &ROSParamService::refreshProperty, this) 
      .doc("Refreshes one property of this component from the ros param server")
      .arg("param_name", "Name of the property.")
      .arg("private", "true if parameter should be found the private namespace")
      .arg("relative", "true if parameter should be found in the relative (component name) namespace");

    /*
     *      this->addOperation("setNamespace", &ROSParamService::setNamespace, this) 
     *        .doc("Sets the default namespace for parameters retrieved by this service.")
     *        .arg("namespace", "ROS graph namespace. Relative to the namespace of this node unless prefixed with '/' (absolute) or '~' (private) to this node.");
     *
     *      this->addOperation("get", &ROSParamService::getParam, this) 
     *        .doc("Gets one property of this component from the ROS param server based on this service's namespace. (see setNamespace)")
     *        .arg("name", "Name of the property / parameter.");
     *
     *      this->addOperation("set", &ROSParamService::setParam, this) 
     *        .doc("Sets one parameter on the ROS parameter server based on the similarly-named property of this component based on this service's namespace. (see setNamespace)")
     *        .arg("name", "Name of the property / parameter.");
     */
  }
private:

  std::stack<XmlRpc::XmlRpcValue> xml_value_stack_;

  bool storeProperties();

  bool storeProperty(const string& prop_name, bool priv, bool rel);

  bool PropertyToXmlRpcValue(base::PropertyBase* prop);

  template<class T>
    bool PropertyToXmlRpcValue(Property<T>* prop);

  bool PropertyToXmlRpcValue(Property<PropertyBag>* prop);

  bool refreshProperties();

  bool refreshProperty(const string& prop_name, bool priv, bool rel);

  bool XmlRpcValueToProperty(XmlRpc::XmlRpcValue &val,
      base::PropertyBase* prop);

};

bool ROSParamService::storeProperties()
{
  using namespace RTT;

  // Create a root property bag for all the properties in this component
  Property<PropertyBag> bag(this->getOwner()->getName(), "");

  // Decompose this task's properties into primitive property types.
  marsh::PropertyBagIntrospector pbi(bag.value());
  pbi.introspect(*this->getOwner()->properties());

  bool retval = false;

  if(PropertyToXmlRpcValue(&bag)) {

    assert(!xml_value_stack_.empty());

    try {
      ros::param::set(string("~") + bag.getName(), xml_value_stack_.top());
      retval = true;
    } catch (ros::InvalidNameException ex) {
      log(Error) << ex.what() << endlog();
    }

    xml_value_stack_.pop();
    assert(xml_value_stack_.empty());
  }

  return retval;
}

bool ROSParamService::storeProperty(const string& prop_name, bool priv, bool rel)
{
  base::PropertyBase* property_base = this->getOwner()->properties()->getProperty(prop_name);

  if (property_base == NULL) {
    log(Error) << this->getOwner()->getName() << " does not have a Property with name " << prop_name << endlog();
    return false;
  }

  string param_name;

  if (priv) {
    param_name = "~";
  }

  if (rel) {
    param_name += this->getOwner()->getName() + string("/");
  }

  param_name += property_base->getName();

  if (PropertyToXmlRpcValue(property_base)) {
    bool retval = true;
    assert(!xml_value_stack_.empty());
    try {
      ros::param::set(param_name, xml_value_stack_.top());
    } catch (ros::InvalidNameException ex) {
      log(Error) << ex.what() << endlog();
      retval = false;
    }
    xml_value_stack_.pop();
    assert(xml_value_stack_.empty());
    return retval;

  } else  {
    // Check if property_base can be decomposed into a PropertyBag, if so we need to decompose it.
    PropertyBag bag;
    bag.add(property_base);

    // Decompose the property into primitive property types.
    PropertyBag decomposed_bag;
    marsh::PropertyBagIntrospector pbi(decomposed_bag);
    pbi.introspect(bag);

    if (PropertyToXmlRpcValue(decomposed_bag.getProperty(property_base->getName()))) {
      bool retval = true;
      assert(!xml_value_stack_.empty());
      try {
        ros::param::set(param_name, xml_value_stack_.top());
      } catch (ros::InvalidNameException ex) {
        log(Error) << ex.what() << endlog();
        retval = false;
      }
      xml_value_stack_.pop();
      assert(xml_value_stack_.empty());
      return retval;
    }
    return false;
  }
  return false;
}

bool ROSParamService::PropertyToXmlRpcValue(base::PropertyBase* prop)
{
  // Try to convert the property via dynamic_cast
  return 
    // Handle primitive properties
    PropertyToXmlRpcValue<bool>(dynamic_cast<Property<bool>*>(prop)) ||
    PropertyToXmlRpcValue<int>(dynamic_cast<Property<int>*>(prop)) ||
    PropertyToXmlRpcValue<double>(dynamic_cast<Property<double>*>(prop)) ||
    PropertyToXmlRpcValue<std::string>(dynamic_cast<Property<std::string>*>(prop)) ||
    // Handle composite properties
    PropertyToXmlRpcValue(dynamic_cast<Property<PropertyBag>*>(prop));
}

  template<class T>
bool ROSParamService::PropertyToXmlRpcValue(Property<T>* prop)
{
  if (!prop) {
    return false;
  }

  // Store the primitive XmlRpc value
  xml_value_stack_.push(XmlRpc::XmlRpcValue(prop->rvalue()));

  return true;
}

bool ROSParamService::PropertyToXmlRpcValue(Property<PropertyBag>* prop)
{
  if (!prop) {
    return false;
  }

  // Get the property bag for this composite property
  PropertyBag& bag = prop->value();
  XmlRpc::XmlRpcValue xml_dict;

  for (size_t i = 0; i < bag.size(); i++) {
    if (PropertyToXmlRpcValue(bag.getItem(i))) {
      if (!bag.getItem(i)->getName().empty() && bag.getType() != "array") {
        xml_dict[bag.getItem(i)->getName()] = xml_value_stack_.top();
      } else {
        xml_dict[i] = xml_value_stack_.top();
      }
      xml_value_stack_.pop();
    }
  }

  if (bag.size() == 0) {
    log(Warning) << "Exporting empty property bag " << prop->getName() << endlog();
  }

  // Store the composite XmlRpc value
  xml_value_stack_.push(xml_dict);

  return true;
}

bool ROSParamService::refreshProperties()
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
      base::PropertyBase* property_base =
        this->getOwner()->properties()->getProperty(
            (*it)->getName());
      if (!property_base->getTypeInfo()->composeType((*it)->getDataSource(),
            property_base->getDataSource()))
        log(Warning)<<"Could not compose "<<property_base->getName()<<endlog();
    }
    else
      log(Warning) << "Could not find Property " << (*it)->getName()
        << endlog();
  }
  return true;
}

bool ROSParamService::refreshProperty(const string& prop_name, bool priv, bool rel)
{
  base::PropertyBase* property_base = this->getOwner()->properties()->getProperty(
      prop_name);
  if (property_base == NULL)
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
  param_name += property_base->getName();
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
  bag.add(property_base);
  // decompose prop into primitive property types.
  PropertyBag decomposed_bag;
  marsh::PropertyBagIntrospector pbi(decomposed_bag);
  pbi.introspect(bag);
  if (!XmlRpcValueToProperty(rpcval, decomposed_bag.getProperty(
          property_base->getName())))
    return false;
  if (property_base->getTypeInfo()->composeType(decomposed_bag.getProperty(
          property_base->getName())->getDataSource(), property_base->getDataSource()))
    return true;
  return false;
}

bool ROSParamService::XmlRpcValueToProperty(XmlRpc::XmlRpcValue &val,
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

ORO_SERVICE_NAMED_PLUGIN(ROSParamService, "rosparam")
