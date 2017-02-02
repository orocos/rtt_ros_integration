#include <rtt/os/startstop.h>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <rtt_rosparam/rosparam.h>

#include <geometry_msgs/typekit/Vector3.h>

#include <ros/init.h>
#include <ros/names.h>
#include <ros/param.h>
#include <ros/this_node.h>

#include <gtest/gtest.h>

using namespace RTT;
using namespace rtt_rosparam;

// helper function to compare PropertyBags
namespace RTT {
  bool operator==(const PropertyBag &left, const PropertyBag &right) {
    std::set<base::PropertyBase *> right_seen;

    for(PropertyBag::const_iterator left_it = left.begin(); left_it != left.end(); ++left_it) {
      base::PropertyBase *left_prop = *left_it;
      base::PropertyBase *right_prop = right.getProperty(left_prop->getName());
      if (right_prop) {
        right_seen.insert(right_prop);
        if (left_prop->getType() != right_prop->getType()) return false;

        // Recurse if both properties are PropertyBags
        Property<PropertyBag> left_bag;
        left_bag = left_prop;
        Property<PropertyBag> right_bag;
        right_bag = right_prop;
        if (left_bag.ready() && right_bag.ready()) {
          if (!(left_bag == right_bag)) return false;
          continue;
        }

        // Otherwise,...
        // ... we believe that the values are the same for now (TODO)
      }
    }

    // check that all values of the right bag have been visited
    if (right_seen.size() != right.size()) return false;

    return true;
  }
}

class ParamTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Import packages
    ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_rosparam", "" ));
    ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_geometry_msgs", "" ));

    tc = new TaskContext("rtt_rosparam_component");

    // Primitive parameters
    tc->addProperty("string", props.string_);
    tc->addProperty("double", props.double_);
    tc->addProperty("float", props.float_);
    tc->addProperty("int", props.int_);
    tc->addProperty("uint", props.uint_);
    tc->addProperty("char", props.char_);
    tc->addProperty("uchar", props.uchar_);
    tc->addProperty("bool", props.bool_);

    // Vector parameters
    tc->addProperty("vstring", props.v_string_);
    tc->addProperty("vdouble", props.v_double_);
    tc->addProperty("vfloat", props.v_float_);
    tc->addProperty("vint", props.v_int_);
    tc->addProperty("vuint", props.v_uint_);
    tc->addProperty("vchar", props.v_char_);
    tc->addProperty("vuchar", props.v_uchar_);
    tc->addProperty("vbool", props.v_bool_);

    tc->addProperty("eigenvectordouble", props.eigenvector_double_);
    tc->addProperty("eigenvectorfloat", props.eigenvector_float_);

    // Struct parameters
    tc->addProperty("bag", props.bag_);
    tc->addProperty("vector3", props.vector3_);

    // Delete parameters that might be left from previous executions
    deleteParameters();

    // Load rosparam service and get pointers
    ASSERT_TRUE(tc->loadService("rosparam"));
    ASSERT_TRUE((rosparam = tc->getProvider<ROSParam>("rosparam")).get());
    ASSERT_TRUE((service = tc->provides()->getService("rosparam")).get());
  }

  virtual void TearDown() {
    deleteParameters();
    delete tc;
  }

  struct Values {
    // Primitive parameters
    std::string string_;
    double double_;
    float float_;
    int int_;
    unsigned int uint_;
    char char_;
    unsigned char uchar_;
    bool bool_;

    // Vector parameters_
    std::vector<std::string> v_string_;
    std::vector<double> v_double_;
    std::vector<float> v_float_;
    std::vector<int> v_int_;
    std::vector<unsigned int> v_uint_;
    std::vector<char> v_char_;
    std::vector<unsigned char> v_uchar_;
    std::vector<bool> v_bool_;

    Eigen::VectorXd eigenvector_double_;
    Eigen::VectorXf eigenvector_float_;

    // Struct parameters
    RTT::PropertyBag bag_;
    geometry_msgs::Vector3 vector3_;

    Values() {
      reset();
    }

    void initialize() {
      // Primitive parameters
      string_ = "foobar";
      double_ = 5.5;
      float_ = 6.5f;
      int_ = -7;
      uint_ = 8u;
      char_ = 'c';
      uchar_ = 255u;
      bool_ = true;

      // Vector parameters
      v_string_.push_back(string_);
      v_double_.push_back(double_);
      v_float_.push_back(float_);
      v_int_.push_back(int_);
      v_uint_.push_back(uint_);
      v_char_.push_back(char_);
      v_uchar_.push_back(uchar_);
      v_bool_.push_back(bool_);

      eigenvector_double_ = Eigen::Vector3d(5.0, 6.0, 7.0);
      eigenvector_float_ = Eigen::Vector3f(8.0f, 9.0f, 10.0f);

      // Struct parameters
      bag_.clear();
      bag_.ownProperty(new RTT::Property<std::string>("string", "", "yet another value"));
      bag_.ownProperty(new RTT::Property<int>("int", "", 20));

      vector3_.x = 20.0;
      vector3_.y = 21.0;
      vector3_.z = 22.0;
    }

    void reset() {
      string_.clear();
      double_ = 0.0;
      float_ = 0.0f;
      int_ = 0;
      uint_ = 0u;
      char_ = 0;
      uchar_ = 0u;
      bool_ = false;

      v_string_.clear();
      v_double_.clear();
      v_float_.clear();
      v_int_.clear();
      v_uint_.clear();
      v_char_.clear();
      v_uchar_.clear();
      v_bool_.clear();

      eigenvector_double_.setZero(0);
      eigenvector_float_.setZero(0);

      bag_.clear();
      bag_.ownProperty(new RTT::Property<std::string>("string", "", std::string()));
      bag_.ownProperty(new RTT::Property<int>("int", "", 0));

      vector3_= geometry_msgs::Vector3();
    }
  } props, params;


  void getParameters(const std::string &prefix, Values &data, bool only_ros_types = false) {
    EXPECT_TRUE(ros::param::get(prefix + "string", data.string_));
    EXPECT_TRUE(ros::param::get(prefix + "double", data.double_));
    EXPECT_TRUE(ros::param::get(prefix + "float", data.float_));
    EXPECT_TRUE(ros::param::get(prefix + "int", data.int_));
    if (!only_ros_types) {
      int temp;
      EXPECT_TRUE(ros::param::get(prefix + "uint", temp));
      data.uint_ = static_cast<unsigned int>(temp);
      EXPECT_TRUE(ros::param::get(prefix + "char", temp));
      data.char_ = static_cast<char>(temp);
      EXPECT_TRUE(ros::param::get(prefix + "uchar", temp));
      data.uchar_ = static_cast<unsigned char>(temp);
    }
    EXPECT_TRUE(ros::param::get(prefix + "bool", data.bool_));

    EXPECT_TRUE(ros::param::get(prefix + "vstring", data.v_string_));
    EXPECT_TRUE(ros::param::get(prefix + "vdouble", data.v_double_));
    EXPECT_TRUE(ros::param::get(prefix + "vfloat", data.v_float_));
    EXPECT_TRUE(ros::param::get(prefix + "vint", data.v_int_));
    if (!only_ros_types) {
      std::vector<int> vtemp;
      EXPECT_TRUE(ros::param::get(prefix + "vuint", vtemp));
      data.v_uint_.assign(vtemp.begin(), vtemp.end());
      EXPECT_TRUE(ros::param::get(prefix + "vchar", vtemp));
      data.v_char_.assign(vtemp.begin(), vtemp.end());
      EXPECT_TRUE(ros::param::get(prefix + "vuchar", vtemp));
      data.v_uchar_.assign(vtemp.begin(), vtemp.end());
    }
    EXPECT_TRUE(ros::param::get(prefix + "vbool", data.v_bool_));

    std::vector<double> v_temp_double;
    EXPECT_TRUE(ros::param::get(prefix + "eigenvectordouble", v_temp_double));
    data.eigenvector_double_ = Eigen::Map<Eigen::VectorXd>(v_temp_double.data(), v_temp_double.size());
    std::vector<float> v_temp_float;
    EXPECT_TRUE(ros::param::get(prefix + "eigenvectorfloat", v_temp_float));
    data.eigenvector_float_ = Eigen::Map<Eigen::VectorXf>(v_temp_float.data(), v_temp_float.size());

    if (!only_ros_types) {
      data.bag_.clear();
      XmlRpc::XmlRpcValue bag_xmlrpc;
      ros::param::get(prefix + "bag", bag_xmlrpc);
      if (bag_xmlrpc.valid() && bag_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for(XmlRpc::XmlRpcValue::iterator it = bag_xmlrpc.begin(); it != bag_xmlrpc.end(); ++it) {
          switch(it->second.getType()) {
            case XmlRpc::XmlRpcValue::TypeString:
              data.bag_.ownProperty(new RTT::Property<std::string>(it->first, "", it->second));
              break;
            case XmlRpc::XmlRpcValue::TypeInt:
              data.bag_.ownProperty(new RTT::Property<int>(it->first, "", it->second));
              break;
            default:
              break;
          }
        }
      }

      EXPECT_TRUE(ros::param::get(prefix + "vector3/x", data.vector3_.x));
      EXPECT_TRUE(ros::param::get(prefix + "vector3/y", data.vector3_.y));
      EXPECT_TRUE(ros::param::get(prefix + "vector3/z", data.vector3_.z));
    }
  }

  void setParameters(const std::string &prefix, const Values &data) {
    ros::param::set(prefix + "string", data.string_);
    ros::param::set(prefix + "double", data.double_);
    ros::param::set(prefix + "float", data.float_);
    ros::param::set(prefix + "int", data.int_);
    int temp;
    ros::param::set(prefix + "uint", temp = data.uint_);
    ros::param::set(prefix + "char", temp = data.char_);
    ros::param::set(prefix + "uchar", temp = data.uchar_);
    ros::param::set(prefix + "bool", data.bool_);

    ros::param::set(prefix + "vstring", data.v_string_);
    ros::param::set(prefix + "vdouble", data.v_double_);
    ros::param::set(prefix + "vfloat", data.v_float_);
    ros::param::set(prefix + "vint", data.v_int_);
    std::vector<int> vtemp;
    vtemp.assign(data.v_uint_.begin(), data.v_uint_.end());
    ros::param::set(prefix + "vuint", vtemp);
    vtemp.assign(data.v_char_.begin(), data.v_char_.end());
    ros::param::set(prefix + "vchar", vtemp);
    vtemp.assign(data.v_uchar_.begin(), data.v_uchar_.end());
    ros::param::set(prefix + "vuchar", vtemp);
    ros::param::set(prefix + "vbool", data.v_bool_);

    std::vector<double> v_temp_double(data.eigenvector_double_.data(), data.eigenvector_double_.data() + data.eigenvector_double_.size());
    ros::param::set(prefix + "eigenvectordouble", v_temp_double);
    std::vector<float> v_temp_float(data.eigenvector_float_.data(), data.eigenvector_float_.data() + data.eigenvector_float_.size());
    ros::param::set(prefix + "eigenvectorfloat", v_temp_float);

    XmlRpc::XmlRpcValue bag_xmlrpc;
    (void) bag_xmlrpc.begin(); // force struct type
    RTT::Property<std::string> bag_string = data.bag_.getProperty("string");
    if (bag_string.ready()) bag_xmlrpc["string"] = bag_string.rvalue();
    RTT::Property<int> bag_int = data.bag_.getProperty("int");
    if (bag_string.ready()) bag_xmlrpc["int"] = bag_int.rvalue();
    ros::param::set(prefix + "bag", bag_xmlrpc);

    ros::param::set(prefix + "vector3/x", data.vector3_.x);
    ros::param::set(prefix + "vector3/y", data.vector3_.y);
    ros::param::set(prefix + "vector3/z", data.vector3_.z);
  }

  void deleteParameters() {
    for(RTT::PropertyBag::const_iterator prop_it = tc->properties()->begin();
        prop_it != tc->properties()->end(); ++prop_it)
    {
      RTT::base::PropertyBase *prop = *prop_it;
      ros::param::del(prop->getName()); // RELATIVE
      ros::param::del(std::string("/") + prop->getName()); // ABSOLUTE
//      ros::param::del(std::string("~") + prop->getName()); // PRIVATE
      ros::param::del(tc->getName() + "/" + prop->getName()); // COMPONENT_RELATIVE
      ros::param::del(std::string("/") + tc->getName() + "/" + prop->getName()); // COMPONENT_ABSOLUTE
//      ros::param::del(std::string("~") + tc->getName() + "/" + prop->getName()); // COMPONENT_PRIVATE
    }
    ros::param::del("~");
  }

  void compareValues(const Values &expected, const Values &actual, const std::string &test_name, bool only_ros_types = false) {
    EXPECT_EQ(expected.string_, actual.string_) << "string values do not match in " << test_name;
    EXPECT_EQ(expected.double_, actual.double_) << "double values do not match in " << test_name;
    EXPECT_EQ(expected.float_, actual.float_) << "float values do not match in " << test_name;
    EXPECT_EQ(expected.int_, actual.int_) << "int values do not match in " << test_name;
    if (!only_ros_types) {
      EXPECT_EQ(expected.uint_, actual.uint_) << "unsigned int values do not match in " << test_name;
      EXPECT_EQ(expected.char_, actual.char_) << "char values do not match in " << test_name;
      EXPECT_EQ(expected.uchar_, actual.uchar_) << "unsigned char values do not match in " << test_name;
    }
    EXPECT_EQ(expected.bool_, actual.bool_) << "bool values do not match in " << test_name;
    EXPECT_EQ(expected.v_string_, actual.v_string_) << "string vectors do not match in " << test_name;
    EXPECT_EQ(expected.v_double_, actual.v_double_) << "double vectors do not match in " << test_name;
    EXPECT_EQ(expected.v_float_, actual.v_float_) << "float vectors do not match in " << test_name;
    EXPECT_EQ(expected.v_int_, actual.v_int_) << "int vectors does do match in " << test_name;
    if (!only_ros_types) {
      EXPECT_EQ(expected.v_uint_, actual.v_uint_) << "unsigned int vectors do not match in " << test_name;
      EXPECT_EQ(expected.v_char_, actual.v_char_) << "char vectors do not match in " << test_name;
      EXPECT_EQ(expected.v_uchar_, actual.v_uchar_) << "unsigned char vectors do not match in " << test_name;
    }
    EXPECT_EQ(expected.v_bool_, actual.v_bool_) << "bool vectors do not match in " << test_name;

    EXPECT_EQ(expected.eigenvector_double_, actual.eigenvector_double_) << "Eigen::VectorXd values do not match in " << test_name;
    EXPECT_EQ(expected.eigenvector_float_, actual.eigenvector_float_) << "Eigen::VectorXf values do not match in " << test_name;
    if (!only_ros_types) {
      EXPECT_EQ(expected.bag_, actual.bag_) << "PropertyBag contents do not match in " << test_name;
      EXPECT_EQ(expected.vector3_.x, actual.vector3_.x) << "geometry_msgs/Vector3 values do not match in " << test_name;
      EXPECT_EQ(expected.vector3_.y, actual.vector3_.y) << "geometry_msgs/Vector3 values do not match in " << test_name;
      EXPECT_EQ(expected.vector3_.z, actual.vector3_.z) << "geometry_msgs/Vector3 values do not match in " << test_name;
    }
  }

  TaskContext *tc;
  boost::shared_ptr<ROSParam> rosparam;
  boost::shared_ptr<Service> service;
};

TEST_F(ParamTest, Constants)
{
  EXPECT_EQ(RELATIVE, Constant<int>(service->getValue("RELATIVE")).get());
  EXPECT_EQ(ABSOLUTE, Constant<int>(service->getValue("ABSOLUTE")).get());
  EXPECT_EQ(PRIVATE, Constant<int>(service->getValue("PRIVATE")).get());
  EXPECT_EQ(COMPONENT_PRIVATE, Constant<int>(service->getValue("COMPONENT_PRIVATE")).get());
  EXPECT_EQ(COMPONENT_RELATIVE, Constant<int>(service->getValue("COMPONENT_RELATIVE")).get());
  EXPECT_EQ(COMPONENT_ABSOLUTE, Constant<int>(service->getValue("COMPONENT_ABSOLUTE")).get());
  EXPECT_EQ(COMPONENT, Constant<int>(service->getValue("COMPONENT")).get());
}

TEST_F(ParamTest, SetAllPropertiesOnParameterServer)
{
  // initialize properties to some values
  props.initialize();

  // setAllRelative()
  EXPECT_TRUE(rosparam->setAllRelative());
  getParameters("", params);
  compareValues(props, params, "setAllRelative()");
  params.reset();

//  // setAllAbsolute()
//  EXPECT_TRUE(rosparam->setAllAbsolute());
//  getParameters("/", params);
//  compareValues(props, params, "setAllAbsolute()");
//  params.reset();

  // setAllPrivate()
  EXPECT_TRUE(rosparam->setAllPrivate());
  getParameters("~", params);
  compareValues(props, params, "setAllPrivate()");
  params.reset();

  // setAllComponentRelative()
  EXPECT_TRUE(rosparam->setAllComponentRelative());
  getParameters(tc->getName() + "/", params);
  compareValues(props, params, "setAllComponentRelative()");
  params.reset();

  // setAllComponentAbsolute()
  EXPECT_TRUE(rosparam->setAllComponentAbsolute());
  getParameters("/" + tc->getName() + "/", params);
  compareValues(props, params, "setAllComponentAbsolute()");
  params.reset();

  // setAllComponentPrivate()
  EXPECT_TRUE(rosparam->setAllComponentPrivate());
  getParameters("~" + tc->getName() + "/", params);
  compareValues(props, params, "setAllComponentPrivate()");
  params.reset();
}

TEST_F(ParamTest, GetAllPropertiesFromParameterServer)
{
  // initialize parameters to some values
  params.initialize();

  // getAllRelative()
  setParameters("", params);
  props.reset();
  EXPECT_TRUE(rosparam->getAllRelative());
  compareValues(params, props, "getAllRelative()");

  // getAllAbsolute()
  setParameters("/", params);
  props.reset();
  EXPECT_TRUE(rosparam->getAllAbsolute());
  compareValues(params, props, "getAllAbsolute()");

  // getAllPrivate()
  setParameters("~", params);
  props.reset();
  EXPECT_TRUE(rosparam->getAllPrivate());
  compareValues(params, props, "getAllPrivate()");

  // getAllComponentRelative()
  setParameters("" + tc->getName() + "/", params);
  props.reset();
  EXPECT_TRUE(rosparam->getAllComponentRelative());
  compareValues(params, props, "getAllComponentRelative()");

  // getAllComponentAbsolute()
  setParameters("/" + tc->getName() + "/", params);
  props.reset();
  EXPECT_TRUE(rosparam->getAllComponentAbsolute());
  compareValues(params, props, "getAllComponentAbsolute()");

  // getAllComponentPrivate()
  setParameters("~" + tc->getName() + "/", params);
  props.reset();
  EXPECT_TRUE(rosparam->getAllComponentPrivate());
  compareValues(params, props, "getAllComponentPrivate()");
}

TEST_F(ParamTest, SetSinglePropertyOnParameterServer)
{
  // initialize properties to some values
  props.initialize();

  // setRelative()
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->setRelative((*prop_it)->getName()));
  }
  getParameters("", params);
  compareValues(props, params, "setRelative()");
  params.reset();

//  // setAbsolute()
//  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
//       prop_it != tc->properties()->end(); ++prop_it) {
//    EXPECT_TRUE(rosparam->setAbsolute((*prop_it)->getName()));
//  }
//  getParameters("/", params);
//  compareValues(props, params, "setAbsolute()");
//  params.reset();

  // setPrivate()
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->setPrivate((*prop_it)->getName()));
  }
  getParameters("~", params);
  compareValues(props, params, "setPrivate()");
  params.reset();

  // setComponentRelative()
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->setComponentRelative((*prop_it)->getName()));
  }
  getParameters(tc->getName() + "/", params);
  compareValues(props, params, "setComponentRelative()");
  params.reset();

  // setComponentAbsolute()
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->setComponentAbsolute((*prop_it)->getName()));
  }
  getParameters("/" + tc->getName() + "/", params);
  compareValues(props, params, "setComponentAbsolute()");
  params.reset();

  // setComponentPrivate()
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->setComponentPrivate((*prop_it)->getName()));
  }
  getParameters("~" + tc->getName() + "/", params);
  compareValues(props, params, "setComponentPrivate()");
  params.reset();
}

TEST_F(ParamTest, GetSinglePropertyFromParameterServer)
{
  // initialize parameters to some values
  params.initialize();

  // getRelative()
  setParameters("", params);
  props.reset();
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->getRelative((*prop_it)->getName()));
  }
  compareValues(params, props, "getRelative()");

  // getAbsolute()
  setParameters("/", params);
  props.reset();
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->getAbsolute((*prop_it)->getName()));
  }
  compareValues(params, props, "getAbsolute()");

  // getPrivate()
  setParameters("~", params);
  props.reset();
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->getPrivate((*prop_it)->getName()));
  }
  compareValues(params, props, "getPrivate()");

  // getComponentRelative()
  setParameters("" + tc->getName() + "/", params);
  props.reset();
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->getComponentRelative((*prop_it)->getName()));
  }
  compareValues(params, props, "getComponentRelative()");

  // getComponentAbsolute()
  setParameters("/" + tc->getName() + "/", params);
  props.reset();
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->getComponentAbsolute((*prop_it)->getName()));
  }
  compareValues(params, props, "getComponentAbsolute()");

  // getComponentPrivate()
  setParameters("~" + tc->getName() + "/", params);
  props.reset();
  for (PropertyBag::const_iterator prop_it = tc->properties()->begin();
       prop_it != tc->properties()->end(); ++prop_it) {
    EXPECT_TRUE(rosparam->getComponentPrivate((*prop_it)->getName()));
  }
  compareValues(params, props, "getComponentPrivate()");
}

TEST_F(ParamTest, SetValueOnParameterServer)
{
  // initialize properties to some values
  props.initialize();

  // fetch parameters one by one
  rosparam->setStringComponentPrivate("string", props.string_);
  rosparam->setDoubleComponentPrivate("double", props.double_);
  rosparam->setFloatComponentPrivate("float", props.float_);
  rosparam->setIntComponentPrivate("int", props.int_);
  rosparam->setBoolComponentPrivate("bool", props.bool_);
  rosparam->setVectorOfStringComponentPrivate("vstring", props.v_string_);
  rosparam->setVectorOfDoubleComponentPrivate("vdouble", props.v_double_);
  rosparam->setVectorOfFloatComponentPrivate("vfloat", props.v_float_);
  rosparam->setVectorOfIntComponentPrivate("vint", props.v_int_);
  rosparam->setVectorOfBoolComponentPrivate("vbool", props.v_bool_);
  rosparam->setEigenVectorXdComponentPrivate("eigenvectordouble", props.eigenvector_double_);
  rosparam->setEigenVectorXfComponentPrivate("eigenvectorfloat", props.eigenvector_float_);
  getParameters("~" + tc->getName() + "/", params, /* only_ros_types = */ true);
  compareValues(props, params, "set[Type]ComponentPrivate()", /* only_ros_types = */ true);
}

TEST_F(ParamTest, GetValueFromParameterServer)
{
  // initialize parameters to some values
  params.initialize();
  setParameters("~" + tc->getName() + "/", params);

  // fetch parameters one by one
  props.reset();
  EXPECT_TRUE(rosparam->getStringComponentPrivate("string", props.string_));
  EXPECT_TRUE(rosparam->getDoubleComponentPrivate("double", props.double_));
  EXPECT_TRUE(rosparam->getFloatComponentPrivate("float", props.float_));
  EXPECT_TRUE(rosparam->getIntComponentPrivate("int", props.int_));
  EXPECT_TRUE(rosparam->getBoolComponentPrivate("bool", props.bool_));
  EXPECT_TRUE(rosparam->getVectorOfStringComponentPrivate("vstring", props.v_string_));
  EXPECT_TRUE(rosparam->getVectorOfDoubleComponentPrivate("vdouble", props.v_double_));
  EXPECT_TRUE(rosparam->getVectorOfFloatComponentPrivate("vfloat", props.v_float_));
  EXPECT_TRUE(rosparam->getVectorOfIntComponentPrivate("vint", props.v_int_));
  EXPECT_TRUE(rosparam->getVectorOfBoolComponentPrivate("vbool", props.v_bool_));
  EXPECT_TRUE(rosparam->getEigenVectorXdComponentPrivate("eigenvectordouble", props.eigenvector_double_));
  EXPECT_TRUE(rosparam->getEigenVectorXfComponentPrivate("eigenvectorfloat", props.eigenvector_float_));
  compareValues(params, props, "get[Type]ComponentPrivate()", /* only_ros_types = */ true);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Start ROS node
  ros::M_string remappings;
  remappings["__ns"] = "rtt_rosparam_namespace";
  ros::init(remappings, "rtt_rosparam_tests_node", ros::init_options::AnonymousName);

  // Initialize Orocos
  __os_init(argc, argv);

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
//  RTT::Logger::log().setLogLevel(RTT::Logger::Debug);

  return RUN_ALL_TESTS();
}

