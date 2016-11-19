#ifndef __RTT_ROSPARAM_ROSPARAM_H
#define __RTT_ROSPARAM_ROSPARAM_H

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>

#define RTT_ROSPARAM_EIGEN_SUPPORT
#ifdef RTT_ROSPARAM_EIGEN_SUPPORT
  #include <Eigen/Dense>
  #define eigen_matrix_xd Eigen::Matrix<double,Eigen::Dynamic,1>
  #define eigen_matrix_xf Eigen::Matrix<float,Eigen::Dynamic,1>
#endif

#ifndef ADD_ROSPARAM_SERVICE_CONSTRUCTOR
#define ADD_ROSPARAM_SERVICE_CONSTRUCTOR(return_type_str) \
  ,get##return_type_str("get"#return_type_str) \
  ,set##return_type_str("set"#return_type_str) \
  ,get##return_type_str##Relative ("get"#return_type_str"Relative") \
  ,set##return_type_str##Relative ("set"#return_type_str"Relative") \
  ,get##return_type_str##Absolute ("get"#return_type_str"Absolute") \
  ,set##return_type_str##Absolute ("set"#return_type_str"Absolute") \
  ,get##return_type_str##Private ("get"#return_type_str"Private") \
  ,set##return_type_str##Private ("set"#return_type_str"Private") \
  ,get##return_type_str##ComponentPrivate ("get"#return_type_str"ComponentPrivate") \
  ,set##return_type_str##ComponentPrivate ("set"#return_type_str"ComponentPrivate") \
  ,get##return_type_str##ComponentRelative ("get"#return_type_str"ComponentRelative") \
  ,set##return_type_str##ComponentRelative ("set"#return_type_str"ComponentRelative") \
  ,get##return_type_str##ComponentAbsolute ("get"#return_type_str"ComponentAbsolute") \
  ,set##return_type_str##ComponentAbsolute ("set"#return_type_str"ComponentAbsolute")
#endif

#ifndef ADD_ROSPARAM_OPERATION_CALLER
#define ADD_ROSPARAM_OPERATION_CALLER(return_type_str) \
  this->addOperationCaller(get##return_type_str); \
  this->addOperationCaller(set##return_type_str); \
  this->addOperationCaller(get##return_type_str##Relative); \
  this->addOperationCaller(set##return_type_str##Relative); \
  this->addOperationCaller(get##return_type_str##Absolute); \
  this->addOperationCaller(set##return_type_str##Absolute); \
  this->addOperationCaller(get##return_type_str##Private); \
  this->addOperationCaller(set##return_type_str##Private); \
  this->addOperationCaller(get##return_type_str##ComponentPrivate); \
  this->addOperationCaller(set##return_type_str##ComponentPrivate); \
  this->addOperationCaller(get##return_type_str##ComponentRelative); \
  this->addOperationCaller(set##return_type_str##ComponentRelative); \
  this->addOperationCaller(get##return_type_str##ComponentAbsolute); \
  this->addOperationCaller(set##return_type_str##ComponentAbsolute);
#endif

#ifndef DECLARE_ROSPARAM_OPERATION_CALLER
#define DECLARE_ROSPARAM_OPERATION_CALLER(return_type_str, return_type) \
  RTT::OperationCaller<bool(const std::string &, return_type &)>        get##return_type_str; \
  RTT::OperationCaller<void(const std::string &, const return_type &)>  set##return_type_str; \
  RTT::OperationCaller<bool(const std::string &, return_type &)>        get##return_type_str##Relative; \
  RTT::OperationCaller<void(const std::string &, const return_type &)>  set##return_type_str##Relative; \
  RTT::OperationCaller<bool(const std::string &, return_type &)>        get##return_type_str##Absolute; \
  RTT::OperationCaller<void(const std::string &, const return_type &)>  set##return_type_str##Absolute; \
  RTT::OperationCaller<bool(const std::string &, return_type &)>        get##return_type_str##Private; \
  RTT::OperationCaller<void(const std::string &, const return_type &)>  set##return_type_str##Private; \
  RTT::OperationCaller<bool(const std::string &, return_type &)>        get##return_type_str##ComponentPrivate; \
  RTT::OperationCaller<void(const std::string &, const return_type &)>  set##return_type_str##ComponentPrivate; \
  RTT::OperationCaller<bool(const std::string &, return_type &)>        get##return_type_str##ComponentRelative; \
  RTT::OperationCaller<void(const std::string &, const return_type &)>  set##return_type_str##ComponentRelative; \
  RTT::OperationCaller<bool(const std::string &, return_type &)>        get##return_type_str##ComponentAbsolute; \
  RTT::OperationCaller<void(const std::string &, const return_type &)>  set##return_type_str##ComponentAbsolute;
#endif

namespace rtt_rosparam {

  typedef enum  {
    RELATIVE, //! Relative resolution:  "name" -> "name"
    ABSOLUTE, //! Absolute resolution:  "name" -> "/name"
    PRIVATE,  //! Private resolution:   "name" -> "~name"
    COMPONENT_PRIVATE, //! Component resolution: "name" -> "~COMPONENT_NAME/name"
    COMPONENT_RELATIVE, //! Component resolution: "name" -> "COMPONENT_NAME/name"
    COMPONENT_ABSOLUTE, //! Component resolution: "name" -> "/COMPONENT_NAME/name"
    COMPONENT = COMPONENT_PRIVATE //! For backwards compatibility, component resolution: COMPONENT_PRIVATE
  } ResolutionPolicy;

  class ROSParam : public RTT::ServiceRequester
  {

  public:
    static const int RELATIVE           = rtt_rosparam::RELATIVE; //! For backwards compatibility
    static const int ABSOLUTE           = rtt_rosparam::ABSOLUTE; //! For backwards compatibility
    static const int PRIVATE            = rtt_rosparam::PRIVATE; //! For backwards compatibility
    static const int COMPONENT_PRIVATE  = rtt_rosparam::COMPONENT_PRIVATE; //! For backwards compatibility
    static const int COMPONENT_RELATIVE = rtt_rosparam::COMPONENT_RELATIVE; //! For backwards compatibility
    static const int COMPONENT_ABSOLUTE = rtt_rosparam::COMPONENT_ABSOLUTE; //! For backwards compatibility
    static const int COMPONENT          = rtt_rosparam::COMPONENT_PRIVATE; //! For backwards compatibility, component resolution: COMPONENT_PRIVATE

    ROSParam(RTT::TaskContext *owner) :
      RTT::ServiceRequester("rosparam",owner),
      getAllRelative("getAllRelative"),
      getAllAbsolute("getAllAbsolute"),
      getAllPrivate("getAllPrivate"),
      getAllComponentPrivate("getAllComponentPrivate"),
      getAllComponentRelative("getAllComponentRelative"),
      getAllComponentAbsolute("getAllComponentAbsolute"),
      getAll("getAll"),
      setAllRelative("setAllRelative"),
      setAllAbsolute("setAllAbsolute"),
      setAllPrivate("setAllPrivate"),
      setAllComponentPrivate("setAllComponentPrivate"),
      setAllComponentRelative("setAllComponentRelative"),
      setAllComponentAbsolute("setAllComponentAbsolute"),
      setAll("setAll"),
      get("get"),
      getParam("getParam"),
      getRelative("getRelative"),
      getAbsolute("getAbsolute"),
      getPrivate("getPrivate"),
      getComponentPrivate("getComponentPrivate"),
      getComponentRelative("getComponentRelative"),
      getComponentAbsolute("getComponentAbsolute"),
      set("set"),
      setParam("setParam"),
      setRelative("setRelative"),
      setAbsolute("setAbsolute"),
      setPrivate("setPrivate"),
      setComponentPrivate("setComponentPrivate"),
      setComponentRelative("setComponentRelative"),
      setComponentAbsolute("setComponentAbsolute")

      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(String)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(Double)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(Float)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(Int)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(Bool)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(VectorOfString)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(VectorOfDouble)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(VectorOfFloat)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(VectorOfInt)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(VectorOfBool)

#ifdef RTT_ROSPARAM_EIGEN_SUPPORT
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(EigenVectorXd)
      ADD_ROSPARAM_SERVICE_CONSTRUCTOR(EigenVectorXf)
#endif

    {
      this->addOperationCaller(getAllRelative);
      this->addOperationCaller(getAllAbsolute);
      this->addOperationCaller(getAllPrivate);
      this->addOperationCaller(getAllComponentPrivate);
      this->addOperationCaller(getAllComponentRelative);
      this->addOperationCaller(getAllComponentAbsolute);
      this->addOperationCaller(getAll);

      this->addOperationCaller(setAllRelative);
      this->addOperationCaller(setAllAbsolute);
      this->addOperationCaller(setAllPrivate);
      this->addOperationCaller(setAllComponentPrivate);
      this->addOperationCaller(setAllComponentRelative);
      this->addOperationCaller(setAllComponentAbsolute);
      this->addOperationCaller(setAll);

      this->addOperationCaller(get);
      this->addOperationCaller(getParam);
      this->addOperationCaller(getRelative);
      this->addOperationCaller(getAbsolute);
      this->addOperationCaller(getPrivate);
      this->addOperationCaller(getComponentPrivate);
      this->addOperationCaller(getComponentRelative);
      this->addOperationCaller(getComponentAbsolute);

      this->addOperationCaller(set);
      this->addOperationCaller(setParam);
      this->addOperationCaller(setRelative);
      this->addOperationCaller(setAbsolute);
      this->addOperationCaller(setPrivate);
      this->addOperationCaller(setComponentPrivate);
      this->addOperationCaller(setComponentRelative);
      this->addOperationCaller(setComponentAbsolute);

      ADD_ROSPARAM_OPERATION_CALLER(String)
      ADD_ROSPARAM_OPERATION_CALLER(Double)
      ADD_ROSPARAM_OPERATION_CALLER(Float)
      ADD_ROSPARAM_OPERATION_CALLER(Int)
      ADD_ROSPARAM_OPERATION_CALLER(Bool)
      ADD_ROSPARAM_OPERATION_CALLER(VectorOfString)
      ADD_ROSPARAM_OPERATION_CALLER(VectorOfDouble)
      ADD_ROSPARAM_OPERATION_CALLER(VectorOfFloat)
      ADD_ROSPARAM_OPERATION_CALLER(VectorOfInt)
      ADD_ROSPARAM_OPERATION_CALLER(VectorOfBool)

#ifdef RTT_ROSPARAM_EIGEN_SUPPORT
      ADD_ROSPARAM_OPERATION_CALLER(EigenVectorXd)
      ADD_ROSPARAM_OPERATION_CALLER(EigenVectorXf)
#endif
    }

    RTT::OperationCaller<bool(void)> getAllRelative;
    RTT::OperationCaller<bool(void)> getAllAbsolute;
    RTT::OperationCaller<bool(void)> getAllPrivate;
    RTT::OperationCaller<bool(void)> getAllComponentPrivate;
    RTT::OperationCaller<bool(void)> getAllComponentRelative;
    RTT::OperationCaller<bool(void)> getAllComponentAbsolute;
    RTT::OperationCaller<bool(void)> getAll;
    RTT::OperationCaller<bool(void)> setAllRelative;
    RTT::OperationCaller<bool(void)> setAllAbsolute;
    RTT::OperationCaller<bool(void)> setAllPrivate;
    RTT::OperationCaller<bool(void)> setAllComponentPrivate;
    RTT::OperationCaller<bool(void)> setAllComponentRelative;
    RTT::OperationCaller<bool(void)> setAllComponentAbsolute;
    RTT::OperationCaller<bool(void)> setAll;

    RTT::OperationCaller<bool(const std::string &, const unsigned int)> get;
    RTT::OperationCaller<bool(const std::string &, const std::string &)> getParam;
    RTT::OperationCaller<bool(const std::string &)> getRelative;
    RTT::OperationCaller<bool(const std::string &)> getAbsolute;
    RTT::OperationCaller<bool(const std::string &)> getPrivate;
    RTT::OperationCaller<bool(const std::string &)> getComponentPrivate;
    RTT::OperationCaller<bool(const std::string &)> getComponentRelative;
    RTT::OperationCaller<bool(const std::string &)> getComponentAbsolute;

    RTT::OperationCaller<bool(const std::string &, const unsigned int)> set;
    RTT::OperationCaller<bool(const std::string &, const std::string &)> setParam;
    RTT::OperationCaller<bool(const std::string &)> setRelative;
    RTT::OperationCaller<bool(const std::string &)> setAbsolute;
    RTT::OperationCaller<bool(const std::string &)> setPrivate;
    RTT::OperationCaller<bool(const std::string &)> setComponentPrivate;
    RTT::OperationCaller<bool(const std::string &)> setComponentRelative;
    RTT::OperationCaller<bool(const std::string &)> setComponentAbsolute;

    DECLARE_ROSPARAM_OPERATION_CALLER(String, std::string)
    DECLARE_ROSPARAM_OPERATION_CALLER(Double, double)
    DECLARE_ROSPARAM_OPERATION_CALLER(Float, float)
    DECLARE_ROSPARAM_OPERATION_CALLER(Int, int)
    DECLARE_ROSPARAM_OPERATION_CALLER(Bool, bool)
    DECLARE_ROSPARAM_OPERATION_CALLER(VectorOfString, std::vector<std::string>)
    DECLARE_ROSPARAM_OPERATION_CALLER(VectorOfDouble, std::vector<double>)
    DECLARE_ROSPARAM_OPERATION_CALLER(VectorOfFloat, std::vector<float>)
    DECLARE_ROSPARAM_OPERATION_CALLER(VectorOfInt, std::vector<int>)
    DECLARE_ROSPARAM_OPERATION_CALLER(VectorOfBool, std::vector<bool>)

#ifdef RTT_ROSPARAM_EIGEN_SUPPORT
    DECLARE_ROSPARAM_OPERATION_CALLER(EigenVectorXd, eigen_matrix_xd)
    DECLARE_ROSPARAM_OPERATION_CALLER(EigenVectorXf, eigen_matrix_xf)
#endif

  };
}

#endif // ifndef __RTT_ROSPARAM_ROSPARAM_H
