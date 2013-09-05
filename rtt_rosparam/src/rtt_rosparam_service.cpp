#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
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

#if 0
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
#endif

   
    this->addOperation("setNamespace", &ROSParamService::setNamespace, this) 
      .doc("Sets the default namespace for parameters retrieved by this service.")
      .arg("namespace", "ROS graph namespace. Relative to the namespace of this node unless prefixed with '/' (absolute) or '~' (private) to this node.");

    this->addOperation("get", &ROSParamService::getParam, this) 
      .doc("Gets one property of this component from the ROS param server based on this service's namespace. (see setNamespace)")
      .arg("name", "Name of the property / parameter.");

    this->addOperation("set", &ROSParamService::setParam, this) 
      .doc("Sets one parameter on the ROS parameter server based on the similarly-named property of this component based on this service's namespace. (see setNamespace)")
      .arg("name", "Name of the property / parameter.");
  
  }
private:

  void setNamespace(const std::string &name);
  bool getParam(const std::string &name);
  bool setParam(const std::string &name);

#if 0
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
#endif

};

template<class T>
bool castable(const RTT::base::PropertyBase *prop);
template<class T>
XmlRpc::XmlRpcValue rttPropertyToXmlParam(const T &prop);
template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<float>(const float &prop);
template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<RTT::PropertyBag>(const RTT::PropertyBag &bag);
template<class T>
XmlRpc::XmlRpcValue rttPropertyToXmlParam(const std::vector<T> &vec);
XmlRpc::XmlRpcValue rttPropertyBaseToXmlParam(const RTT::base::PropertyBase *prop);

template<class T>
bool castable(const RTT::base::PropertyBase *prop) 
{
  return dynamic_cast<const Property<T>*>(prop);
}

template<class T>
XmlRpc::XmlRpcValue rttPropertyToXmlParam(const T &prop)
{
  return XmlRpc::XmlRpcValue(prop);
}

template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<float>(const float &prop)
{
  return XmlRpc::XmlRpcValue(static_cast<const double &>(prop));
}

template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<RTT::PropertyBag>(const RTT::PropertyBag &bag)
{
  // Make the xml value a struct
  XmlRpc::XmlRpcValue xml_struct;
  const XmlRpc::XmlRpcValue::ValueStruct &xml_map = (const XmlRpc::XmlRpcValue::ValueStruct &)(xml_struct);

  // Get the properties
  const RTT::PropertyBag::Properties &properties = bag.getProperties();

  for(RTT::PropertyBag::Properties::const_iterator it = properties.begin();
      it != properties.end();
      ++it)
  {
    xml_struct[(*it)->getName()] = rttPropertyBaseToXmlParam(*it);
  }

  return xml_struct;
}

template<class T>
XmlRpc::XmlRpcValue rttPropertyToXmlParam(const std::vector<T> &vec)
{
  XmlRpc::XmlRpcValue xml_array;
  xml_array.setSize(vec.size());

  for(unsigned i=0; i<vec.size(); i++) {
    xml_array[i] = rttPropertyToXmlParam<T>(vec.at(i));
  }

  return xml_array;
}

#define RETURN_RTT_PROPERTY_TO_XML_PARAM(type,prop)\
  if(castable< type >(prop)) { return rttPropertyToXmlParam< type >(static_cast<const RTT::Property< type >*>(prop)->rvalue()); }

#define RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(type,elem_type,prop)\
  if(castable< type >(prop)) { return rttPropertyToXmlParam< elem_type >(static_cast<const RTT::Property< type >*>(prop)->rvalue()); }

XmlRpc::XmlRpcValue rttPropertyBaseToXmlParam(const RTT::base::PropertyBase *prop)
{
  // Primitive parameters
  RETURN_RTT_PROPERTY_TO_XML_PARAM(std::string,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(double,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(float,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(int,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(bool,prop);

  // Composite parameters
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<std::string>, std::string, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<double>, double, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<float>, float, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<int>, int, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<bool>, bool, prop);

  RETURN_RTT_PROPERTY_TO_XML_PARAM(RTT::PropertyBag,prop);

  // Add more types here //

  return XmlRpc::XmlRpcValue();
}


bool ROSParamService::setParam(const std::string &name)
{
  XmlRpc::XmlRpcValue xml_value;
  xml_value = rttPropertyBaseToXmlParam(this->getOwner()->getProperty(name));
  ros::param::set(name, xml_value);
  return true;
}

// Declarations
template <class T>
bool xmlParamToProp(const XmlRpc::XmlRpcValue &xml_value, RTT::Property<T>* prop);
template <class T>
bool xmlParamToProp(const XmlRpc::XmlRpcValue &xml_value, RTT::Property<std::vector<T> >* prop);
template <> 
bool xmlParamToProp<RTT::PropertyBag>(const XmlRpc::XmlRpcValue &xml_value, RTT::Property<RTT::PropertyBag>* prop);
bool xmlParamToProp( const XmlRpc::XmlRpcValue &xml_value, RTT::base::PropertyBase* prop_base);

template <class T>
bool xmlParamToProp(
    const XmlRpc::XmlRpcValue &xml_value,
    RTT::Property<T>* prop)
{
  // Check if the property value is the requested type T
  if(!prop) {
    return false;
  }

  // Set the value
  prop->set((const T&)xml_value);

  return true;
}

template <class T>
bool xmlParamToProp(
    const XmlRpc::XmlRpcValue &xml_value,
    RTT::Property<std::vector<T> >* prop)
{
  // Check if the property value is the requested type T
  if(!prop) {
    return false;
  }

  // Make sure it's an array 
  if(xml_value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }

  // Copy the data into the vector property
  std::vector<T> &vec = prop->value();
  vec.resize(xml_value.size());
  for(size_t i=0; i<vec.size(); i++) {
    vec[i] = (const T&)xml_value[i];
  }

  return true;
}

template <>
bool xmlParamToProp<RTT::PropertyBag>(
    const XmlRpc::XmlRpcValue &xml_value,
    RTT::Property<RTT::PropertyBag>* prop)
{
  // Check if the property value is the requested type T
  if(!prop) {
    return false;
  }

  // Make sure it's an array 
  if(xml_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    return false;
  }

  bool success = true;
  typedef const XmlRpc::XmlRpcValue::ValueStruct & ConstStruct;
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = ((ConstStruct)xml_value).begin();
      it != ((ConstStruct)xml_value).end();
      ++it)
  {
    RTT::base::PropertyBase *sub_prop_base = prop->value().getProperty(it->first);
    success &= xmlParamToProp(it->second, sub_prop_base);
  }

  return success;
}

bool xmlParamToProp(
    const XmlRpc::XmlRpcValue &xml_value,
    RTT::base::PropertyBase* prop_base)
{
  // Switch based on the type of XmlRpcValue 
  switch(xml_value.getType()) {
    case XmlRpc::XmlRpcValue::TypeString:
      return 
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::string>*>(prop_base)); 
    case XmlRpc::XmlRpcValue::TypeDouble:
      return 
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<double>*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<float>*>(prop_base)); 
    case XmlRpc::XmlRpcValue::TypeInt:
      return 
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<int>*>(prop_base)); 
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return 
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<bool>*>(prop_base)); 
    case XmlRpc::XmlRpcValue::TypeArray:
      return 
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<std::string> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<double> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<float> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<int> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<bool> >*>(prop_base));
    case XmlRpc::XmlRpcValue::TypeStruct:
      return
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<RTT::PropertyBag>*>(prop_base));
  };

  RTT::log(RTT::Debug) << "No appropriate conversion for property \"" << prop_base->getName() << "\"" << RTT::endlog();

  return false;
}

bool ROSParamService::getParam(const std::string &name)
{
  // Get the parameter
  XmlRpc::XmlRpcValue xml_value;
  if(!ros::param::get(name, xml_value)) {
    RTT::log(RTT::Debug) << "ROS Parameter \"" << name << "\" not found on the parameter server!" << RTT::endlog();
    return false;
  }

  // Try to get the property if it exists
  RTT::base::PropertyBase *prop_base = this->getOwner()->getProperty(name);

  if(!prop_base) {
    RTT::log(RTT::Debug) << "RTT component does not have a property named \"" << name << "\"" << RTT::endlog();
    return false;
  }

  // Deal with the xml value
  return xmlParamToProp(xml_value, prop_base);
}
void ROSParamService::setNamespace(const std::string &ns)
{

}

#if 0

bool ROSParamService::storeProperties()
{
  using namespace RTT;

  // Create a root property bag for all the properties in this component
  Property<PropertyBag> bag(this->getOwner()->getName(), "");

  // Decompose this task's properties into 
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

#endif

ORO_SERVICE_NAMED_PLUGIN(ROSParamService, "rosparam")
