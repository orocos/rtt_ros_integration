#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/types/PropertyDecomposition.hpp>

#include <ros/ros.h>

using namespace RTT;
using namespace std;

class ROSParamService: public RTT::Service
{
public:

  enum ResolutionPolicy {
    RELATIVE, //! Relative resolution:  "name" -> "name"
    ABSOLUTE, //! Absolute resolution:  "name" -> "/name"
    PRIVATE,  //! Private resolution:   "name" -> "~name"
    COMPONENT //! Component resolution: "name" -> "~COMPONENT_NAME/name"
  };

  ROSParamService(TaskContext* owner) :
    Service("rosparam", owner)
  {
    this->doc("RTT Service for synchronizing ROS parameters with the properties of a corresponding RTT component");

    this->addConstant("RELATIVE",RELATIVE);
    this->addConstant("ABSOLUTE",ABSOLUTE);
    this->addConstant("PRIVATE",PRIVATE);
    this->addConstant("COMPONENT",COMPONENT);

    this->addOperation("getAllRelative", &ROSParamService::getParamsRelative, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the relative namespace.");
    this->addOperation("getAllAbsolute", &ROSParamService::getParamsAbsolute, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the absolute namespace.");
    this->addOperation("getAllPrivate", &ROSParamService::getParamsPrivate, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the node's private namespace.");
    this->addOperation("getAllComponentPrivate", &ROSParamService::getParamsComponentPrivate, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the component's private namespace.");
    this->addOperation("getAll", &ROSParamService::getParamsComponentPrivate, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the component's private namespace. This is an alias for getAllComponentPrivate().");

    this->addOperation("setAllRelative", &ROSParamService::setParamsRelative, this)
      .doc("Stores all properties of this component (and its sub-services) on the ROS param server from the similarly-named property in the relative namespace.");
    this->addOperation("setAllAbsolute", &ROSParamService::setParamsAbsolute, this)
      .doc("Stores all properties of this component (and its sub-services) on the ROS param server from the similarly-named property in the absolute namespace.");
    this->addOperation("setAllPrivate", &ROSParamService::setParamsPrivate, this)
      .doc("Stores all properties of this component (and its sub-services) on the ROS param server from the similarly-named property in the node's private namespace.");
    this->addOperation("setAllComponentPrivate", &ROSParamService::setParamsComponentPrivate, this)
      .doc("Stores all properties of this component (and its sub-services) on the ROS param server from the similarly-named property in the component's private namespace.");
    this->addOperation("setAll", &ROSParamService::setParamsComponentPrivate, this)
      .doc("Stores all properties of this component (and its sub-services) on the ROS param server from the similarly-named property in the component's private namespace. This is an alias for setAllComponentPrivate().");

    this->addOperation("get", &ROSParamService::getParam, this) 
      .doc("Gets one property of this component (or populates the properties of a named RTT sub-service) from the ROS param server based on the given resolution policy.")
      .arg("name", "Name of the property / service / parameter.")
      .arg("policy", "ROS parameter namespace resolution policy.");

    this->addOperation("getRelative", &ROSParamService::getParamRelative, this) 
      .doc("Gets one property of this component (or populates the properties of a named RTT sub-service) from the ROS param server in the relative namespace.")
      .arg("name", "Name of the property / service / parameter.");
    this->addOperation("getAbsolute", &ROSParamService::getParamAbsolute, this) 
      .doc("Gets one property of this component (or populates the properties of a named RTT sub-service) from the ROS param server in the absolute namespace.")
      .arg("name", "Name of the property / service / parameter.");
    this->addOperation("getPrivate", &ROSParamService::getParamPrivate, this) 
      .doc("Gets one property of this component (or populates the properties of a named RTT sub-service) from the ROS param server in the node's private namespace.")
      .arg("name", "Name of the property / service / parameter.");
    this->addOperation("getComponentPrivate", &ROSParamService::getParamComponentPrivate, this) 
      .doc("Gets one property of this component (or populates the properties of a named RTT sub-service) from the ROS param server in the component's private namespace.")
      .arg("name", "Name of the property / service / parameter.");

    this->addOperation("set", &ROSParamService::setParam, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component (or stores the properties of a named RTT sub-service) in the ROS parameter namespace based on the given resolution policy.")
      .arg("name", "Name of the property / service / parameter.")
      .arg("policy", "ROS parameter namespace resolution policy.");

    this->addOperation("setRelative", &ROSParamService::setParamRelative, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component (or stores the properties of a named RTT sub-service) in the relative namespace.")
      .arg("name", "Name of the property / service / parameter.");
    this->addOperation("setAbsolute", &ROSParamService::setParamAbsolute, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component (or stores the properties of a named RTT sub-service) in the absolute namespace.")
      .arg("name", "Name of the property / service / parameter.");
    this->addOperation("setPrivate", &ROSParamService::setParamPrivate, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component (or stores the properties of a named RTT sub-service) in the node's private namespace.")
      .arg("name", "Name of the property / service / parameter.");
    this->addOperation("setComponentPrivate", &ROSParamService::setParamComponentPrivate, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component (or stores the properties of a named RTT sub-service) in the component's private namespace.")
      .arg("name", "Name of the property / service / parameter.");
  
  }
private:

  //! Resolve a parameter name based on the given \ref ResolutionPolicy
  const std::string resolvedName(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy);

  bool getParams(RTT::Service::shared_ptr service, const std::string& ns, const ROSParamService::ResolutionPolicy policy);
  bool getParams(const ROSParamService::ResolutionPolicy policy);
  bool getParamsRelative() { return getParams(RELATIVE); }
  bool getParamsAbsolute() { return getParams(ABSOLUTE); }
  bool getParamsPrivate() { return getParams(PRIVATE); }
  bool getParamsComponentPrivate() { return getParams(COMPONENT); }

  bool getParam(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy = ROSParamService::COMPONENT);
  bool getParamRelative(const std::string &name) { return getParam(name, RELATIVE); }
  bool getParamAbsolute(const std::string &name) { return getParam(name, ABSOLUTE); }
  bool getParamPrivate(const std::string &name) { return getParam(name, PRIVATE); }
  bool getParamComponentPrivate(const std::string &name) { return getParam(name, COMPONENT); }

  bool setParams(RTT::Service::shared_ptr service, const std::string& ns, const ROSParamService::ResolutionPolicy policy);
  bool setParams(const ROSParamService::ResolutionPolicy policy);
  bool setParamsRelative() { return setParams(RELATIVE); }
  bool setParamsAbsolute() { return setParams(ABSOLUTE); }
  bool setParamsPrivate() { return setParams(PRIVATE); }
  bool setParamsComponentPrivate() { return setParams(COMPONENT); }

  bool setParam(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy = ROSParamService::COMPONENT);
  bool setParamRelative(const std::string &name) { return setParam(name, RELATIVE); }
  bool setParamAbsolute(const std::string &name) { return setParam(name, ABSOLUTE); }
  bool setParamPrivate(const std::string &name) { return setParam(name, PRIVATE); }
  bool setParamComponentPrivate(const std::string &name) { return setParam(name, COMPONENT); }
};

const std::string ROSParamService::resolvedName(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy)
{
  std::string resolved_name = param_name;
  switch(policy) {
    case ROSParamService::RELATIVE:
      return param_name;
    case ROSParamService::ABSOLUTE:
      return std::string("/") + param_name;
    case ROSParamService::PRIVATE:
      return std::string("~") + param_name;
    case ROSParamService::COMPONENT:
      return std::string("~") + this->getOwner()->getName() + "/" + param_name;
  };

  // Relative by default
  return param_name;
}



//! Determine if the RTT property can be casted into an RTT::Property<T>
template<class T>
bool castable(const RTT::base::PropertyBase *prop);
//! Convert a value to an XmlRpc value
template<class T>
XmlRpc::XmlRpcValue rttPropertyToXmlParam(const T &prop);
//! Convert a PropertyBag to an XmlRpc struct value
template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<RTT::PropertyBag>(const RTT::PropertyBag &bag);
//! Convert a std::vector<T> to an XmlRpc array value
template<class T>
XmlRpc::XmlRpcValue rttPropertyToXmlParam(const std::vector<T> &vec);
//! Convert an abstract RTT PropertyBase to an XmlRpc value
XmlRpc::XmlRpcValue rttPropertyBaseToXmlParam(RTT::base::PropertyBase *prop);

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
  return XmlRpc::XmlRpcValue(static_cast<double>(prop));
}

template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<unsigned int>(const unsigned int &prop)
{
  return XmlRpc::XmlRpcValue(static_cast<int>(prop));
}

template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<char>(const char &prop)
{
  return XmlRpc::XmlRpcValue(static_cast<int>(prop));
}

template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<unsigned char>(const unsigned char &prop)
{
  return XmlRpc::XmlRpcValue(static_cast<int>(prop));
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

// These just save typing
#define RETURN_RTT_PROPERTY_TO_XML_PARAM(type,prop)\
  if(castable< type >(prop)) { return rttPropertyToXmlParam< type >(static_cast<const RTT::Property< type >*>(prop)->rvalue()); }

#define RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(type,elem_type,prop)\
  if(castable< type >(prop)) { return rttPropertyToXmlParam< elem_type >(static_cast<const RTT::Property< type >*>(prop)->rvalue()); }

XmlRpc::XmlRpcValue rttPropertyBaseToXmlParam(RTT::base::PropertyBase *prop)
{
  // Primitive parameters
  RETURN_RTT_PROPERTY_TO_XML_PARAM(std::string,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(double,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(float,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(int,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(unsigned int,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(char,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(unsigned char,prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(bool,prop);

  // Vector parameters
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<std::string>, std::string, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<double>, double, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<float>, float, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<int>, int, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<unsigned int>, unsigned int, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<char>, char, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<unsigned char>, unsigned char, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<bool>, bool, prop);

  // Struct parameters
  RETURN_RTT_PROPERTY_TO_XML_PARAM(RTT::PropertyBag,prop);

  // Try to decompose property into a property bag
  RTT::PropertyBag bag;
  if (RTT::types::propertyDecomposition(prop, bag)) {
    return rttPropertyToXmlParam(bag);
  }

  return XmlRpc::XmlRpcValue();
}


bool ROSParamService::setParam(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy)
{
  XmlRpc::XmlRpcValue xml_value;

  // Try to find a property named param_name
  RTT::base::PropertyBase *property = this->getOwner()->getProperty(param_name);
  if (property) {
    xml_value = rttPropertyBaseToXmlParam(this->getOwner()->getProperty(param_name));
    ros::param::set(resolvedName(param_name,policy), xml_value);
    return true;
  }

  // Try to find a sub-service named param_name
  RTT::Service::shared_ptr service = this->getOwner()->provides()->getService(param_name);
  if (service) {
    // Set all parameters of the sub-service
    return setParams(service, service->getName(), policy);
  }

  RTT::log(RTT::Debug) << "RTT component does not have a property or service named \"" << param_name << "\"" << RTT::endlog();
  return false;
}

bool ROSParamService::setParams(const ROSParamService::ResolutionPolicy policy)
{
  return setParams(this->getOwner()->provides(), std::string(), policy);
}

bool ROSParamService::setParams(RTT::Service::shared_ptr service, const std::string& ns, const ROSParamService::ResolutionPolicy policy) {
  XmlRpc::XmlRpcValue xml_value;
  xml_value = rttPropertyToXmlParam(*(service->properties()));
  ros::param::set(resolvedName(ns, policy), xml_value);

  // Recurse into sub-services
  RTT::Service::ProviderNames names = service->getProviderNames();
  for (RTT::Service::ProviderNames::const_iterator it = names.begin(); it != names.end(); ++it)
  {
    RTT::Service::shared_ptr sub(service->getService(*it));
    if (sub) {
      if (!setParams(sub, ns + "/" + sub->getName(), policy)) return false;
    }
  }

  return true;
}

// Declarations
//! Convert an XmlRpc value into an RTT property
template <class T>
bool xmlParamToProp(const XmlRpc::XmlRpcValue &xml_value, RTT::Property<T>* prop);
//! Convert an XmlRpc array value into an RTT std::vector property
template <class T>
bool xmlParamToProp(const XmlRpc::XmlRpcValue &xml_value, RTT::Property<std::vector<T> >* prop);
//! Convert an XmlRpc structure value into an RTT PropertyBag property
template <> 
bool xmlParamToProp<RTT::PropertyBag>(const XmlRpc::XmlRpcValue &xml_value, RTT::Property<RTT::PropertyBag>* prop);
//! Convert an XmlRpc structure value into an abstract RTT PropertyBase
bool xmlParamToProp( const XmlRpc::XmlRpcValue &xml_value, RTT::base::PropertyBase* prop_base);

//! Convert an XmlRpc value to type T
template <class T> void xmlParamToValue(const XmlRpc::XmlRpcValue &xml_value, T &value) {
  value = static_cast<const T&>(const_cast<XmlRpc::XmlRpcValue &>(xml_value));
}

template <> void xmlParamToValue<float>(const XmlRpc::XmlRpcValue &xml_value, float &value) {
  value = static_cast<const double&>(const_cast<XmlRpc::XmlRpcValue &>(xml_value));
}

template <> void xmlParamToValue<unsigned int>(const XmlRpc::XmlRpcValue &xml_value, unsigned int &value) {
  value = static_cast<const int&>(const_cast<XmlRpc::XmlRpcValue &>(xml_value));
}

template <> void xmlParamToValue<char>(const XmlRpc::XmlRpcValue &xml_value, char &value) {
  value = static_cast<const int&>(const_cast<XmlRpc::XmlRpcValue &>(xml_value));
}

template <> void xmlParamToValue<unsigned char>(const XmlRpc::XmlRpcValue &xml_value, unsigned char &value) {
  value = static_cast<const int&>(const_cast<XmlRpc::XmlRpcValue &>(xml_value));
}

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
  xmlParamToValue(xml_value, prop->set());

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
    xmlParamToValue(xml_value[i], vec[i]);
  }

  return true;
}

template <>
bool xmlParamToProp<bool>(
    const XmlRpc::XmlRpcValue &xml_value,
    RTT::Property<std::vector<bool> >* prop)
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
  std::vector<bool> &vec = prop->value();
  vec.resize(xml_value.size());
  for(size_t i=0; i<vec.size(); i++) {
    bool temp;
    xmlParamToValue(xml_value[i], temp);
    vec[i] = temp;
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

  // Make sure it's a struct
  if(xml_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    return false;
  }

  // Copy the properties
  bool success = true;
  // We need to copy the struct because XmlRpc++ doesn't have const operations for this
  XmlRpc::XmlRpcValue xml_value_struct(xml_value);
  for(XmlRpc::XmlRpcValue::ValueStruct::iterator it = xml_value_struct.begin();
      it != xml_value_struct.end();
      ++it)
  {
    RTT::base::PropertyBase *sub_prop_base = prop->value().getProperty(it->first);
    if(sub_prop_base) {
      success &= xmlParamToProp(it->second, sub_prop_base);
    }
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
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<int>*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<unsigned int>*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<char>*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<unsigned char>*>(prop_base));
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return 
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<bool>*>(prop_base)); 
    case XmlRpc::XmlRpcValue::TypeArray:
      return 
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<std::string> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<double> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<float> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<int> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<unsigned int> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<char> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<unsigned char> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<bool> >*>(prop_base));
    case XmlRpc::XmlRpcValue::TypeStruct:
      if (xmlParamToProp(xml_value, dynamic_cast<RTT::Property<RTT::PropertyBag>*>(prop_base))) return true;
      // try to decompose the property in a property bag:
      {
         RTT::Property<RTT::PropertyBag> bag(prop_base->getName());
         return RTT::types::propertyDecomposition(prop_base, bag.set()) && xmlParamToProp(xml_value, &bag);
      }
  };

  RTT::log(RTT::Debug) << "No appropriate conversion for property \"" << prop_base->getName() << "\"" << RTT::endlog();

  return false;
}

bool ROSParamService::getParam(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy)
{
  RTT::Logger::In in("ROSParamService::getParam");

  // Get the parameter
  XmlRpc::XmlRpcValue xml_value;

  const std::string resolved_name = resolvedName(param_name,policy);
  if(!ros::param::get(resolved_name, xml_value)) {
    RTT::log(RTT::Debug) << "ROS Parameter \"" << resolved_name << "\" not found on the parameter server!" << RTT::endlog();
    return false;
  }

  // Try to get the property if it exists
  RTT::base::PropertyBase *prop_base = this->getOwner()->getProperty(param_name);
  if(prop_base) {
    // Deal with the xml value
    return xmlParamToProp(xml_value, prop_base);
  }

  // Try to get the properties of a sub-service if it exists
  RTT::Service::shared_ptr service = this->getOwner()->provides()->getService(param_name);
  if(service) {
    // Get all parameters of the sub-service
    return getParams(service, service->getName(), policy);
  }

  RTT::log(RTT::Debug) << "RTT component does not have a property or service named \"" << param_name << "\"" << RTT::endlog();
  return false;
}

bool ROSParamService::getParams(const ROSParamService::ResolutionPolicy policy)
{
  return getParams(this->getOwner()->provides(), std::string(), policy);
}

bool ROSParamService::getParams(RTT::Service::shared_ptr service, const std::string& ns, const ROSParamService::ResolutionPolicy policy)
{
  RTT::Logger::In in("ROSParamService::getParams");

  // Get the parameter
  XmlRpc::XmlRpcValue xml_value;

  const std::string resolved_name = resolvedName(ns, policy);
  if(!ros::param::get(resolved_name, xml_value)) {
    RTT::log(RTT::Debug) << "ROS Parameter \"" << resolved_name << "\" not found on the parameter server!" << RTT::endlog();
    return false;
  }

  // Create a Property<> wrapper around the propertybag
  RTT::PropertyBag *properties = service->properties();
  RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr datasource(new RTT::internal::ReferenceDataSource<RTT::PropertyBag>(*properties));
  RTT::Property<RTT::PropertyBag> prop(this->getOwner()->getName(),"",datasource);

  // Deal with the xml value
  if (!xmlParamToProp(xml_value, &prop)) return false;

  // Recurse into sub-services
  RTT::Service::ProviderNames names = service->getProviderNames();
  for (RTT::Service::ProviderNames::const_iterator it = names.begin(); it != names.end(); ++it)
  {
    RTT::Service::shared_ptr sub(service->getService(*it));
    if (sub) {
      std::string sub_ns = sub->getName();
      if (!ns.empty()) sub_ns = ns + "/" + sub_ns;
      getParams(sub, sub_ns, policy);
    }
  }

  return true;
}

ORO_SERVICE_NAMED_PLUGIN(ROSParamService, "rosparam")
