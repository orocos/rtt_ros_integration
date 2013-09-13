#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

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

    this->addOperation("getAll", &ROSParamService::getParams, this) 
      .doc("Gets all properties of this component from the ROS param server relative to the component namespace.");

    this->addOperation("setAll", &ROSParamService::setParams, this) 
      .doc("Gets all properties of this component from the ROS param server relative to the component namespace.");

    this->addOperation("get", &ROSParamService::getParam, this) 
      .doc("Gets one property of this component from the ROS param server based on the given resolution policy.")
      .arg("name", "Name of the property / parameter.")
      .arg("policy", "ROS parameter namespace resolution policy.");

    this->addOperation("getRelative", &ROSParamService::getParamRelative, this) 
      .doc("Gets one property of this component from the ROS param server in the relative namespace.")
      .arg("name", "Name of the property / parameter.");
    this->addOperation("getAbsolute", &ROSParamService::getParamAbsolute, this) 
      .doc("Gets one property of this component from the ROS param server in the absolute namespace.")
      .arg("name", "Name of the property / parameter.");
    this->addOperation("getPrivate", &ROSParamService::getParamPrivate, this) 
      .doc("Gets one property of this component from the ROS param server in the node's private namespace.")
      .arg("name", "Name of the property / parameter.");
    this->addOperation("getComponentPrivate", &ROSParamService::getParamComponentPrivate, this) 
      .doc("Gets one property of this component from the ROS param server in the component's private namespace.")
      .arg("name", "Name of the property / parameter.");

    this->addOperation("set", &ROSParamService::setParam, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component based on the given resolution policy.")
      .arg("name", "Name of the property / parameter.")
      .arg("policy", "ROS parameter namespace resolution policy.");

    this->addOperation("setRelative", &ROSParamService::setParamRelative, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component in the relative namespace.")
      .arg("name", "Name of the property / parameter.");
    this->addOperation("setAbsolute", &ROSParamService::setParamAbsolute, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component in the absolute namespace.")
      .arg("name", "Name of the property / parameter.");
    this->addOperation("setPrivate", &ROSParamService::setParamPrivate, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component in the node's private namespace.")
      .arg("name", "Name of the property / parameter.");
    this->addOperation("setComponentPrivate", &ROSParamService::setParamComponentPrivate, this) 
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component in the component's private namespace.")
      .arg("name", "Name of the property / parameter.");
  
  }
private:

  //! Resolve a parameter name based on the given \ref ResolutionPolicy
  const std::string resolvedName(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy);

  bool getParams();
  bool getParam(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy = ROSParamService::COMPONENT);
  bool getParamRelative(const std::string &name) { return getParam(name, RELATIVE); }
  bool getParamAbsolute(const std::string &name) { return getParam(name, ABSOLUTE); }
  bool getParamPrivate(const std::string &name) { return getParam(name, PRIVATE); }
  bool getParamComponentPrivate(const std::string &name) { return getParam(name, COMPONENT); }

  bool setParams();
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
//! Convert a float value to an XmlRpc double value
template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<float>(const float &prop);
//! Convert a PropertyBag to an XmlRpc struct value
template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam<RTT::PropertyBag>(const RTT::PropertyBag &bag);
//! Convert a std::vector<T> to an XmlRpc array value
template<class T>
XmlRpc::XmlRpcValue rttPropertyToXmlParam(const std::vector<T> &vec);
//! Convert an abstract RTT PropertyBase to an XmlRpc value
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

// These just save typing
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

  // Vector parameters
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<std::string>, std::string, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<double>, double, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<float>, float, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<int>, int, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<bool>, bool, prop);

  // Struct parameters
  RETURN_RTT_PROPERTY_TO_XML_PARAM(RTT::PropertyBag,prop);

  // Add more types here Eigen types... KDL types... etc //

  return XmlRpc::XmlRpcValue();
}


bool ROSParamService::setParam(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy)
{
  XmlRpc::XmlRpcValue xml_value;
  xml_value = rttPropertyBaseToXmlParam(this->getOwner()->getProperty(param_name));
  ros::param::set(resolvedName(param_name,policy), xml_value);
  return true;
}

bool ROSParamService::setParams()
{
  XmlRpc::XmlRpcValue xml_value;
  xml_value = rttPropertyToXmlParam(this->getOwner()->properties());
  ros::param::set(std::string("~") + this->getOwner()->getName(), xml_value);
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

  // Copy the properties
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
        // Add more types here Eigen types... KDL types... etc //
    case XmlRpc::XmlRpcValue::TypeStruct:
      return
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<RTT::PropertyBag>*>(prop_base));
  };

  RTT::log(RTT::Debug) << "No appropriate conversion for property \"" << prop_base->getName() << "\"" << RTT::endlog();

  return false;
}

bool ROSParamService::getParam(
    const std::string &param_name, 
    const ROSParamService::ResolutionPolicy policy)
{
  // Get the parameter
  XmlRpc::XmlRpcValue xml_value;

  const std::string resolved_name = resolvedName(param_name,policy);
  if(!ros::param::get(resolved_name, xml_value)) {
    RTT::log(RTT::Debug) << "ROS Parameter \"" << resolved_name << "\" not found on the parameter server!" << RTT::endlog();
    return false;
  }

  // Try to get the property if it exists
  RTT::base::PropertyBase *prop_base = this->getOwner()->getProperty(param_name);

  if(!prop_base) {
    RTT::log(RTT::Debug) << "RTT component does not have a property named \"" << param_name << "\"" << RTT::endlog();
    return false;
  }

  // Deal with the xml value
  return xmlParamToProp(xml_value, prop_base);
}

bool ROSParamService::getParams()
{
  // Get the parameter
  XmlRpc::XmlRpcValue xml_value;

  const std::string resolved_name = std::string("~") + this->getOwner()->getName();
  if(!ros::param::get(resolved_name, xml_value)) {
    RTT::log(RTT::Debug) << "ROS Parameter \"" << resolved_name << "\" not found on the parameter server!" << RTT::endlog();
    return false;
  }

  // Create a Property<> wrapper around the propertybag
  RTT::PropertyBag *properties = this->getOwner()->properties();
  RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr datasource(new RTT::internal::ReferenceDataSource<RTT::PropertyBag>(*properties));

  RTT::Property<RTT::PropertyBag> prop(this->getOwner()->getName(),"",datasource);

  // Deal with the xml value
  return xmlParamToProp(xml_value, &prop);
}

ORO_SERVICE_NAMED_PLUGIN(ROSParamService, "rosparam")
