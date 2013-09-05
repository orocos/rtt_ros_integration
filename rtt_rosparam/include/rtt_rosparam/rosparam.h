#ifndef __RTT_ROSPARAM_ROSPARAM_H
#define __RTT_ROSPARAM_ROSPARAM_H

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>

namespace rtt_rosparam {

  class ROSParam : public RTT::ServiceRequester
  {

  public:
    ROSParam(RTT::TaskContext *owner);

    enum ResolutionPolicy {
      RELATIVE, //! Relative resolution:  "name" -> "name"
      ABSOLUTE, //! Absolute resolution:  "name" -> "/name"
      PRIVATE,  //! Private resolution:   "name" -> "~name"
      COMPONENT //! Component resolution: "name" -> "~COMPONENT_NAME/name"
    };

    RTT::OperationCaller<bool(void)> getAll;
    RTT::OperationCaller<bool(void)> setAll;

    RTT::OperationCaller<bool(const std::string &, const int)> get;
    RTT::OperationCaller<bool(const std::string &)> getRelative;
    RTT::OperationCaller<bool(const std::string &)> getAbsolute;
    RTT::OperationCaller<bool(const std::string &)> getPrivate;
    RTT::OperationCaller<bool(const std::string &)> getComponentPrivate;

    RTT::OperationCaller<bool(const std::string &, const int)> set;
    RTT::OperationCaller<bool(const std::string &)> setRelative;
    RTT::OperationCaller<bool(const std::string &)> setAbsolute;
    RTT::OperationCaller<bool(const std::string &)> setPrivate;
    RTT::OperationCaller<bool(const std::string &)> setComponentPrivate;
  };

  ROSParam::ROSParam(RTT::TaskContext *owner) :
    RTT::ServiceRequester("rosparam",owner),
    getAll("getAll"),
    setAll("setAll"),
    get("get"),
    getRelative("getRelative"),
    getAbsolute("getAbsolute"),
    getPrivate("getPrivate"),
    getComponentPrivate("getComponentPrivate"),
    set("set"),
    setRelative("setRelative"),
    setAbsolute("setAbsolute"),
    setPrivate("setPrivate"),
    setComponentPrivate("setComponentPrivate")
  {
    this->addOperationCaller(getAll);
    this->addOperationCaller(setAll);

    this->addOperationCaller(get);
    this->addOperationCaller(getRelative);
    this->addOperationCaller(getAbsolute);
    this->addOperationCaller(getPrivate);
    this->addOperationCaller(getComponentPrivate);

    this->addOperationCaller(set);
    this->addOperationCaller(setRelative);
    this->addOperationCaller(setAbsolute);
    this->addOperationCaller(setPrivate);
    this->addOperationCaller(setComponentPrivate);
  }

}

#endif // ifndef __RTT_ROSPARAM_ROSPARAM_H
