#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/types/PropertyDecomposition.hpp>

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_convertible.hpp>

#include <XmlRpcException.h>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <rtt_rosparam/rosparam.h>

#ifndef ADD_ROSPARAM_OPERATION
#define ADD_ROSPARAM_OPERATION(return_type_str, return_type, func) \
  this->addOperation("get"#return_type_str, &ROSParamService::get##func< return_type , RELATIVE >, this).doc("Get a " #return_type " from rosparam"); \
  this->addOperation("set"#return_type_str, &ROSParamService::set##func< return_type , RELATIVE >, this).doc("Set a " #return_type " in rosparam"); \
  this->addOperation("get"#return_type_str"Relative", &ROSParamService::get##func< return_type , RELATIVE >, this).doc("Get a " #return_type " from rosparam using the relative resolution policy : `relative/param`"); \
  this->addOperation("set"#return_type_str"Relative", &ROSParamService::set##func< return_type , RELATIVE >, this).doc("Set a " #return_type " in rosparam using the relative resolution policy : `relative/param`"); \
  this->addOperation("get"#return_type_str"Absolute", &ROSParamService::get##func< return_type , ABSOLUTE >, this).doc("Get a " #return_type " from rosparam using the absolute resolution policy : `/global/param`"); \
  this->addOperation("set"#return_type_str"Absolute", &ROSParamService::set##func< return_type , ABSOLUTE >, this).doc("Set a " #return_type " in rosparam using the absolute resolution policy : `/global/param`"); \
  this->addOperation("get"#return_type_str"Private", &ROSParamService::get##func< return_type , PRIVATE >, this).doc("Get a " #return_type " from rosparam using the private resolution policy : `~private/param`"); \
  this->addOperation("set"#return_type_str"Private", &ROSParamService::set##func< return_type , PRIVATE >, this).doc("Set a " #return_type " in rosparam using the private resolution policy : `~private/param`"); \
  this->addOperation("get"#return_type_str"ComponentPrivate", &ROSParamService::get##func< return_type , COMPONENT_PRIVATE >, this).doc("Get a " #return_type " from rosparam using the following resolution policy : `~component_name/param`"); \
  this->addOperation("set"#return_type_str"ComponentPrivate", &ROSParamService::set##func< return_type , COMPONENT_PRIVATE >, this).doc("Set a " #return_type " in rosparam using the following resolution policy : `~component_name/param`"); \
  this->addOperation("get"#return_type_str"ComponentRelative", &ROSParamService::get##func< return_type , COMPONENT_RELATIVE >, this).doc("Get a " #return_type " from rosparam using the following resolution policy : `component_name/param`"); \
  this->addOperation("set"#return_type_str"ComponentRelative", &ROSParamService::set##func< return_type , COMPONENT_RELATIVE >, this).doc("Set a " #return_type " in rosparam using the following resolution policy : `component_name/param`"); \
  this->addOperation("get"#return_type_str"ComponentAbsolute", &ROSParamService::get##func< return_type , COMPONENT_ABSOLUTE >, this).doc("Get a " #return_type " from rosparam using the following resolution policy : `/component_name/param`"); \
  this->addOperation("set"#return_type_str"ComponentAbsolute", &ROSParamService::set##func< return_type , COMPONENT_ABSOLUTE >, this).doc("Set a " #return_type " in rosparam using the following resolution policy : `/component_name/param`");
#endif


using namespace RTT;
using namespace std;

using namespace rtt_rosparam;

class ROSParamService: public RTT::Service
{
public:

  ROSParamService(TaskContext* owner) :
    Service("rosparam", owner)
  {
    this->doc("RTT Service for synchronizing ROS parameters with the properties of a corresponding RTT component");

    this->setValue(new Constant<int>("RELATIVE", static_cast<int>(RELATIVE)));
    this->setValue(new Constant<int>("ABSOLUTE", static_cast<int>(ABSOLUTE)));
    this->setValue(new Constant<int>("PRIVATE", static_cast<int>(PRIVATE)));
    this->setValue(new Constant<int>("COMPONENT", static_cast<int>(COMPONENT)));
    this->setValue(new Constant<int>("COMPONENT_PRIVATE", static_cast<int>(COMPONENT_PRIVATE)));
    this->setValue(new Constant<int>("COMPONENT_RELATIVE", static_cast<int>(COMPONENT_RELATIVE)));
    this->setValue(new Constant<int>("COMPONENT_ABSOLUTE", static_cast<int>(COMPONENT_ABSOLUTE)));

    this->addOperation("getAllRelative", &ROSParamService::getParamsRelative, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the relative namespace.");
    this->addOperation("getAllAbsolute", &ROSParamService::getParamsAbsolute, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the absolute namespace.");
    this->addOperation("getAllPrivate", &ROSParamService::getParamsPrivate, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the node's private namespace.");
    this->addOperation("getAllComponentPrivate", &ROSParamService::getParamsComponentPrivate, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the component's private namespace.");
    this->addOperation("getAllComponentRelative", &ROSParamService::getParamsComponentRelative, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the component's relative namespace.");
    this->addOperation("getAllComponentAbsolute", &ROSParamService::getParamsComponentAbsolute, this)
      .doc("Gets all properties of this component (and its sub-services) from the ROS param server in the component's absolute namespace.");
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
    this->addOperation("setAllComponentRelative", &ROSParamService::setParamsComponentRelative, this)
      .doc("Stores all properties of this component (and its sub-services) on the ROS param server from the similarly-named property in the component's relative namespace.");
    this->addOperation("setAllComponentAbsolute", &ROSParamService::setParamsComponentAbsolute, this)
      .doc("Stores all properties of this component (and its sub-services) on the ROS param server from the similarly-named property in the component's absolute namespace.");
    this->addOperation("setAll", &ROSParamService::setParamsComponentPrivate, this)
      .doc("Stores all properties of this component (and its sub-services) on the ROS param server from the similarly-named property in the component's private namespace. This is an alias for setAllComponentPrivate().");

    this->addOperation("get", &ROSParamService::get, this)
      .doc("Gets one property of this component (or populates the properties of a named RTT sub-service) from the ROS param server based on the given resolution policy.")
      .arg("name", "Name of the property / service / parameter.")
      .arg("policy", "ROS parameter namespace resolution policy.");
    this->addOperation("getParam", &ROSParamService::getParam, this)
      .doc("Gets one property of this component (or populates the properties of a named RTT sub-service) from the ROS param server based on the given ROS parameter name.")
      .arg("param_name", "Name of the ROS parameter. Use '~' and '/' leaders for private or absolute resolution.")
      .arg("name", "Name of the RTT property or service.");

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
    this->addOperation("getComponentRelative", &ROSParamService::getParamComponentRelative, this)
      .doc("Gets one property of this component (or populates the properties of a named RTT sub-service) from the ROS param server in the component's relative namespace.")
      .arg("name", "Name of the property / service / parameter.");
    this->addOperation("getComponentAbsolute", &ROSParamService::getParamComponentAbsolute, this)
      .doc("Gets one property of this component (or populates the properties of a named RTT sub-service) from the ROS param server in the component's absolute namespace.")
      .arg("name", "Name of the property / service / parameter.");

    this->addOperation("set", &ROSParamService::set, this)
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component (or stores the properties of a named RTT sub-service) in the ROS parameter namespace based on the given resolution policy.")
      .arg("name", "Name of the property / service / parameter.")
      .arg("policy", "ROS parameter namespace resolution policy.");
    this->addOperation("setParam", &ROSParamService::setParam, this)
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component (or stores the properties of a named RTT sub-service) in the ROS parameter namespace based on the given ROS parameter name.")
      .arg("param_name", "Name of the ROS parameter. Use '~' and '/' leaders for private or absolute resolution.")
      .arg("name", "Name of the RTT property or service.");

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
    this->addOperation("setComponentRelative", &ROSParamService::setParamComponentRelative, this)
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component (or stores the properties of a named RTT sub-service) in the component's relative namespace.")
      .arg("name", "Name of the property / service / parameter.");
    this->addOperation("setComponentAbsolute", &ROSParamService::setParamComponentAbsolute, this)
      .doc("Sets one parameter on the ROS param server from the similarly-named property of this component (or stores the properties of a named RTT sub-service) in the component's absolute namespace.")
      .arg("name", "Name of the property / service / parameter.");

    ADD_ROSPARAM_OPERATION(String, std::string, ParamImpl)
    ADD_ROSPARAM_OPERATION(Double, double, ParamImpl)
    ADD_ROSPARAM_OPERATION(Float, float, ParamImpl)
    ADD_ROSPARAM_OPERATION(Int, int, ParamImpl)
    ADD_ROSPARAM_OPERATION(Bool, bool, ParamImpl)

    // Vector parameters
    ADD_ROSPARAM_OPERATION(VectorOfString, std::vector<std::string>, ParamImpl)
    ADD_ROSPARAM_OPERATION(VectorOfDouble, std::vector<double>, ParamImpl)
    ADD_ROSPARAM_OPERATION(VectorOfFloat, std::vector<float>, ParamImpl)
    ADD_ROSPARAM_OPERATION(VectorOfInt, std::vector<int>, ParamImpl)
    ADD_ROSPARAM_OPERATION(VectorOfBool, std::vector<bool>, ParamImpl)

    ADD_ROSPARAM_OPERATION(EigenVectorXd, double, EigenVectorParamImpl)
    ADD_ROSPARAM_OPERATION(EigenVectorXf, float, EigenVectorParamImpl)

  }

private:

  template <typename T, ResolutionPolicy P> bool getParamImpl(const std::string& ros_param_name, T& value)
  {
    if (!ros::param::get(resolvedName(ros_param_name,P), value)) {
      RTT::log(RTT::Debug) << "ROS Parameter \"" << ros_param_name << "\" not found on the parameter server!" << RTT::endlog();
      return false;
    }
    return true;
  }

  template <typename T, ResolutionPolicy P> void setParamImpl(const std::string& ros_param_name, const T& value)
  {
    ros::param::set(resolvedName(ros_param_name,P), value);
  }

  template <typename T, ResolutionPolicy P> bool getEigenVectorParamImpl(const std::string& ros_param_name, Eigen::Matrix<T,Eigen::Dynamic,1>& eigen_vector)
  {
    std::vector<T> value;
    if (!getParamImpl< std::vector<T> , P >(ros_param_name,value)) {
      return false;
    }
    eigen_vector = Eigen::Matrix<T,Eigen::Dynamic,1>::Map(value.data(),value.size());
    return true;
  }

  template <typename T, ResolutionPolicy P> void setEigenVectorParamImpl(const std::string& ros_param_name, const Eigen::Matrix<T,Eigen::Dynamic,1>& eigen_vector)
  {
    std::vector<T> value(eigen_vector.data(),eigen_vector.data() + eigen_vector.size() );
    setParamImpl< std::vector<T> , P >(ros_param_name,value);
  }

  //! Resolve a parameter name based on the given \ref ResolutionPolicy
  const std::string resolvedName(
    const std::string &param_name,
    const ResolutionPolicy policy);

  bool getParams(RTT::Service::shared_ptr service, const std::string& ns);
  bool getParams(const ResolutionPolicy policy);
  bool getParamsRelative() { return getParams(RELATIVE); }
  bool getParamsAbsolute() { return getParams(ABSOLUTE); }
  bool getParamsPrivate() { return getParams(PRIVATE); }
  bool getParamsComponentPrivate() { return getParams(COMPONENT_PRIVATE); }
  bool getParamsComponentRelative() { return getParams(COMPONENT_RELATIVE); }
  bool getParamsComponentAbsolute() { return getParams(COMPONENT_ABSOLUTE); }

  bool get(
    const std::string &param_name,
    const unsigned int policy = (unsigned int) COMPONENT_PRIVATE);
  bool getParam(
    const std::string &ros_name,
    const std::string &rtt_name);
  bool getParamRelative(const std::string &name) { return get(name, RELATIVE); }
  bool getParamAbsolute(const std::string &name) { return get(name, ABSOLUTE); }
  bool getParamPrivate(const std::string &name) { return get(name, PRIVATE); }
  bool getParamComponentPrivate(const std::string &name) { return get(name, COMPONENT_PRIVATE); }
  bool getParamComponentRelative(const std::string &name) { return get(name, COMPONENT_RELATIVE); }
  bool getParamComponentAbsolute(const std::string &name) { return get(name, COMPONENT_ABSOLUTE); }

  bool setParams(RTT::Service::shared_ptr service, const std::string& ns);
  bool setParams(const ResolutionPolicy policy);
  bool setParamsRelative() { return setParams(RELATIVE); }
  bool setParamsAbsolute() { return setParams(ABSOLUTE); }
  bool setParamsPrivate() { return setParams(PRIVATE); }
  bool setParamsComponentPrivate() { return setParams(COMPONENT_PRIVATE); }
  bool setParamsComponentRelative() { return setParams(COMPONENT_RELATIVE); }
  bool setParamsComponentAbsolute() { return setParams(COMPONENT_ABSOLUTE); }

  bool set(
    const std::string &param_name,
    const unsigned int policy = (unsigned int) COMPONENT_PRIVATE);
  bool setParam(
    const std::string &ros_name,
    const std::string &rtt_name);
  bool setParamRelative(const std::string &name) { return set(name, RELATIVE); }
  bool setParamAbsolute(const std::string &name) { return set(name, ABSOLUTE); }
  bool setParamPrivate(const std::string &name) { return set(name, PRIVATE); }
  bool setParamComponentPrivate(const std::string &name) { return set(name, COMPONENT_PRIVATE); }
  bool setParamComponentRelative(const std::string &name) { return set(name, COMPONENT_RELATIVE); }
  bool setParamComponentAbsolute(const std::string &name) { return set(name, COMPONENT_ABSOLUTE); }
};

const std::string ROSParamService::resolvedName(
    const std::string &param_name,
    const ResolutionPolicy policy)
{
  std::string leader = "";
  std::string resolved_name = "";

  if(param_name.length() > 0) {
    leader = param_name[0];
  }

  switch(policy) {
    case RELATIVE:
      resolved_name = param_name;
      break;
    case ABSOLUTE:
      resolved_name = (leader == "/") ? param_name : std::string("/") + param_name;
      break;
    case PRIVATE:
      resolved_name = (leader == "~") ? param_name : std::string("~") + param_name;
      break;
    case COMPONENT_PRIVATE:
      resolved_name = std::string("~") + ros::names::append(this->getOwner()->getName(),param_name);
      break;
    case COMPONENT_RELATIVE:
      resolved_name = ros::names::append(this->getOwner()->getName(),param_name);
      break;
    case COMPONENT_ABSOLUTE:
      resolved_name = std::string("/") +ros::names::append(this->getOwner()->getName(),param_name);
      break;
  };

  RTT::log(RTT::Debug) << "["<<this->getOwner()->getName()<<"] Resolving ROS param \""<<param_name<<"\" to \""<<resolved_name<<"\"" << RTT::endlog();

  return resolved_name;
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

template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam(const Eigen::VectorXd &vec)
{
  XmlRpc::XmlRpcValue xml_array;
  xml_array.setSize(vec.size());

  for(unsigned i=0; i<vec.size(); i++) {
    xml_array[i] = rttPropertyToXmlParam<double>(vec(i));
  }

  return xml_array;
}

template<>
XmlRpc::XmlRpcValue rttPropertyToXmlParam(const Eigen::VectorXf &vec)
{
  XmlRpc::XmlRpcValue xml_array;
  xml_array.setSize(vec.size());

  for(unsigned i=0; i<vec.size(); i++) {
    xml_array[i] = rttPropertyToXmlParam<double>(vec(i));
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
  RETURN_RTT_PROPERTY_TO_XML_PARAM(std::string, prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(double, prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(float, prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(int, prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(unsigned int, prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(char, prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(unsigned char, prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(bool, prop);

  // Vector parameters
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<std::string>, std::string, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<double>, double, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<float>, float, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<int>, int, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<unsigned int>, unsigned int, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<char>, char, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<unsigned char>, unsigned char, prop);
  RETURN_RTT_PROPERTY_CONTAINER_TO_XML_PARAM(std::vector<bool>, bool, prop);

  RETURN_RTT_PROPERTY_TO_XML_PARAM(Eigen::VectorXd, prop);
  RETURN_RTT_PROPERTY_TO_XML_PARAM(Eigen::VectorXf, prop);

  // Struct parameters
  RETURN_RTT_PROPERTY_TO_XML_PARAM(RTT::PropertyBag, prop);

  // Try to decompose property into a property bag
  RTT::PropertyBag bag;
  if (RTT::types::propertyDecomposition(prop, bag)) {
    return rttPropertyToXmlParam(bag);
  }

  return XmlRpc::XmlRpcValue();
}


bool ROSParamService::set(
    const std::string &param_name,
    const unsigned int policy)
{
  RTT::Logger::In in("ROSParamService::set");

  const std::string resolved_name = resolvedName(param_name,ResolutionPolicy(policy));

  return this->setParam(resolved_name, param_name);
}

bool ROSParamService::setParam(
    const std::string &ros_name,
    const std::string &rtt_name)
{
  RTT::Logger::In in("ROSParamService::setParam");

  XmlRpc::XmlRpcValue xml_value;

  // Try to find a property named rtt_name
  RTT::base::PropertyBase *property = this->getOwner()->getProperty(rtt_name);
  if (property) {
    xml_value = rttPropertyBaseToXmlParam(this->getOwner()->getProperty(rtt_name));
    ros::param::set(ros_name, xml_value);
    return true;
  }

  // Try to find a sub-service named rtt_name
  RTT::Service::shared_ptr service = this->getOwner()->provides()->getService(rtt_name);
  if (service) {
    // Set all parameters of the sub-service
    return setParams(service, ros_name);
  }

  RTT::log(RTT::Debug) << "RTT component does not have a property or service named \"" << rtt_name << "\"" << RTT::endlog();
  return false;
}

bool ROSParamService::setParams(const ResolutionPolicy policy)
{
  return setParams(this->getOwner()->provides(), resolvedName(std::string(), policy));
}

bool ROSParamService::setParams(
    RTT::Service::shared_ptr service,
    const std::string& ns) {
  XmlRpc::XmlRpcValue xml_value;
  xml_value = rttPropertyToXmlParam(*(service->properties()));
  ros::param::set(ns, xml_value);

  // Recurse into sub-services
  RTT::Service::ProviderNames names = service->getProviderNames();
  for (RTT::Service::ProviderNames::const_iterator it = names.begin(); it != names.end(); ++it)
  {
    RTT::Service::shared_ptr sub(service->getService(*it));
    if (sub) {
      if (!setParams(sub, ros::names::append(ns,sub->getName()))) return false;
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
//! Convert an XmlRpc array value into an RTT Eigen::VectorXd property
template <>
bool xmlParamToProp(const XmlRpc::XmlRpcValue &xml_value, RTT::Property<Eigen::VectorXd>* prop);
template <>
bool xmlParamToProp(const XmlRpc::XmlRpcValue &xml_value, RTT::Property<Eigen::VectorXf>* prop);
//! Convert an XmlRpc structure value into an RTT PropertyBag property
template <>
bool xmlParamToProp<RTT::PropertyBag>(const XmlRpc::XmlRpcValue &xml_value, RTT::Property<RTT::PropertyBag>* prop);
//! Convert an XmlRpc structure value into an abstract RTT PropertyBase
bool xmlParamToProp( const XmlRpc::XmlRpcValue &xml_value, RTT::base::PropertyBase* prop_base);

template <class XMLRPCType,class PropertyType, class enabled=void> struct XmlParamToValue{
    static bool assign(const XMLRPCType& xml_value, PropertyType &prop_value)
    {
        return false;
    }
};

template <class XMLRPCType, class PropertyType >
struct XmlParamToValue <XMLRPCType, PropertyType, typename boost::enable_if<boost::is_convertible<XMLRPCType,PropertyType> >::type >{
    static bool assign(const XMLRPCType& xml_value, PropertyType &prop_value)
    {
        prop_value = xml_value;
        return true;
    }
};

//! Convert an XmlRpc value to type T
template <class PropertyType> bool xmlParamToValue(const XmlRpc::XmlRpcValue &xml_value, PropertyType &value) {
    switch(xml_value.getType()) {
    case XmlRpc::XmlRpcValue::TypeString:
        return XmlParamToValue<std::string, PropertyType>::assign(static_cast<const std::string&>(const_cast<XmlRpc::XmlRpcValue &>(xml_value)),value);
    case XmlRpc::XmlRpcValue::TypeDouble:
        return XmlParamToValue<double, PropertyType>::assign(static_cast<double>(const_cast<XmlRpc::XmlRpcValue &>(xml_value)),value);
    case XmlRpc::XmlRpcValue::TypeInt:
        return XmlParamToValue<int, PropertyType>::assign(static_cast<int>(const_cast<XmlRpc::XmlRpcValue &>(xml_value)),value);
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return XmlParamToValue<bool, PropertyType>::assign(static_cast<bool>(const_cast<XmlRpc::XmlRpcValue &>(xml_value)),value);
    }
    return false;
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
  return xmlParamToValue(xml_value, prop->set());

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
  bool result = true;
  for(size_t i=0; i<vec.size(); i++) {
    result &= xmlParamToValue(xml_value[i], vec[i]);
  }

  return result;
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
  bool result = true;
  for(size_t i=0; i<vec.size(); i++) {
    bool temp;
    result &= xmlParamToValue(xml_value[i], temp);
    vec[i] = temp;
  }

  return result;
}

template <>
bool xmlParamToProp(
    const XmlRpc::XmlRpcValue &xml_value,
    RTT::Property<Eigen::VectorXd >* prop)
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
  Eigen::VectorXd &vec = prop->value();
  vec.resize(xml_value.size());
  bool result = true;
  for(size_t i=0; i<vec.size(); i++) {
    double temp;
    result &= xmlParamToValue(xml_value[i], temp);
    vec[i] = temp;
  }

  return result;
}

template <>
bool xmlParamToProp(
    const XmlRpc::XmlRpcValue &xml_value,
    RTT::Property<Eigen::VectorXf >* prop)
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
  Eigen::VectorXf &vec = prop->value();
  vec.resize(xml_value.size());
  bool result = true;
  for(size_t i=0; i<vec.size(); i++) {
    double temp;
    result &= xmlParamToValue(xml_value[i], temp);
    vec[i] = temp;
  }

  return result;
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
    if (sub_prop_base) {
      success &= xmlParamToProp(it->second, sub_prop_base);
    } else {
      // create property based on XmlRpc type?
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
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<double>*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<float>*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<int>*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<unsigned int>*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<char>*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<unsigned char>*>(prop_base));

    case XmlRpc::XmlRpcValue::TypeBoolean:
      return
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<bool>*>(prop_base));

    case XmlRpc::XmlRpcValue::TypeArray:
      if (
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<std::string> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<double> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<float> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<int> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<unsigned int> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<char> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<unsigned char> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<std::vector<bool> >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<Eigen::VectorXd >*>(prop_base)) ||
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<Eigen::VectorXf >*>(prop_base)) )
      {
        // Return true if it gets parsed into an array structure
        return true;
      }
      break;

    case XmlRpc::XmlRpcValue::TypeStruct:
      if (
        xmlParamToProp(xml_value, dynamic_cast<RTT::Property<RTT::PropertyBag>*>(prop_base)) )
      {
        // Return true if it gets parsed into a PropertyBag structure
        return true;
      }
      break;
  };

  // try property bag decomposition
  {
    RTT::Property<RTT::PropertyBag> bag(prop_base->getName());
    if (RTT::types::propertyDecomposition(prop_base, bag.set()) && xmlParamToProp(xml_value, &bag)) {
      return true;
    } else {
      RTT::log(RTT::Debug) << "Could not decompose property bag for property type \"" << prop_base->getName() << "\"" << RTT::endlog();
      return false;
    }
  }

  RTT::log(RTT::Debug) << "No appropriate conversion for property \"" << prop_base->getName() << "\"" << RTT::endlog();
  return false;
}

bool ROSParamService::get(
    const std::string &param_name,
    const unsigned int policy)
{
  RTT::Logger::In in("ROSParamService::get");

  const std::string resolved_name = resolvedName(
      param_name,ResolutionPolicy(policy));

  return this->getParam(resolved_name, param_name);
}

bool ROSParamService::getParam(
    const std::string &ros_name,
    const std::string &rtt_name)
{
  RTT::Logger::In in("ROSParamService::getParam");

  try {
    // Get the parameter
    XmlRpc::XmlRpcValue xml_value;

    if(!ros::param::get(ros_name, xml_value)) {
      RTT::log(RTT::Debug) << "ROS Parameter \"" << ros_name << "\" not found on the parameter server!" << RTT::endlog();
      return false;
    }

    // Try to get the property if it exists
    RTT::base::PropertyBase *prop_base = this->getOwner()->getProperty(rtt_name);
    if(prop_base) {
      // Deal with the xml value
      bool ret = xmlParamToProp(xml_value, prop_base);
      if(!ret) {
        RTT::log(RTT::Warning) << "Could not convert \"" << ros_name << "\" from an XMLRPC value to an RTT property." << RTT::endlog();
      }
      return ret;
    }

    // Try to get the properties of a sub-service if it exists
    RTT::Service::shared_ptr service = this->getOwner()->provides()->getService(rtt_name);
    if(service) {
      // Get all parameters of the sub-service
      return getParams(service, ros_name);
    }

    RTT::log(RTT::Debug) << "RTT component does not have a property or service named \"" << rtt_name << "\"" << RTT::endlog();

  } catch(XmlRpc::XmlRpcException &err) {
    RTT::log(RTT::Error) << "XmlRpcException when getting ROS parameter \""<<ros_name<<"\": " << err.getMessage() << RTT::endlog();
    RTT::log(RTT::Debug) << " -- Make sure your parameters are the right primitive type." << RTT::endlog();
  }

  return false;
}


bool ROSParamService::getParams(const ResolutionPolicy policy)
{
  return getParams(this->getOwner()->provides(), resolvedName(std::string(), policy));
}

bool ROSParamService::getParams(
    RTT::Service::shared_ptr service,
    const std::string& ns)
{
  RTT::Logger::In in("ROSParamService::getParams");

  // Get the parameter
  XmlRpc::XmlRpcValue xml_value;

  if(!ros::param::get(ns, xml_value)) {
    RTT::log(RTT::Debug) << "ROS Parameter namespace \"" << ns << "\" not found on the parameter server!" << RTT::endlog();
    return false;
  }

  // Create a Property<> wrapper around the propertybag
  RTT::PropertyBag *properties = service->properties();
  RTT::internal::AssignableDataSource<RTT::PropertyBag>::shared_ptr datasource(new RTT::internal::ReferenceDataSource<RTT::PropertyBag>(*properties));
  RTT::Property<RTT::PropertyBag> prop(this->getOwner()->getName(),"",datasource);

  // Deal with the xml value
  bool ret = xmlParamToProp(xml_value, &prop);
  if (!ret) {
    RTT::log(RTT::Warning) << "Could not convert \"" << ns << "\" from an XMLRPC value to an RTT property." << RTT::endlog();
    return false;
  }

  // Recurse into sub-services
  RTT::Service::ProviderNames names = service->getProviderNames();
  for (RTT::Service::ProviderNames::const_iterator it = names.begin(); it != names.end(); ++it)
  {
    RTT::Service::shared_ptr sub(service->getService(*it));
    if (sub) {
      std::string sub_ns = sub->getName();
      if (!ns.empty()) sub_ns = ros::names::append(ns,sub_ns);
      getParams(sub, sub_ns);
    }
  }

  return true;
}

ORO_SERVICE_NAMED_PLUGIN(ROSParamService, "rosparam")
