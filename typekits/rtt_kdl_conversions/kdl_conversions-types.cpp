#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <kdl_conversions/kdl_msg.h>

namespace KDL
{
  /**
   * KDL RTT bindings
   */
  class KDLConversionTypekitPlugin
    : public RTT::types::TypekitPlugin
  {
  public:
      std::string getName(){return "KDLConversions";};
      bool loadTypes(){return true;};
      bool loadConstructors(){return true;};
      bool loadOperators()
      {
          RTT::Service::shared_ptr gs = RTT::internal::GlobalService::Instance();
          gs->provides("KDL")->addOperation("pointMsgToKDL",&tf::pointMsgToKDL);
          gs->provides("KDL")->addOperation("pointKDLToMsg",&tf::pointKDLToMsg);
          gs->provides("KDL")->addOperation("poseMsgToKDL",&tf::poseMsgToKDL);
          gs->provides("KDL")->addOperation("poseKDLToMsg",&tf::poseKDLToMsg);
          gs->provides("KDL")->addOperation("quaternionMsgToKDL",&tf::quaternionMsgToKDL);
          gs->provides("KDL")->addOperation("quaternionKDLToMsg",&tf::quaternionKDLToMsg);
          gs->provides("KDL")->addOperation("transformMsgToKDL",&tf::transformMsgToKDL);
          gs->provides("KDL")->addOperation("transformKDLToMsg",&tf::transformKDLToMsg);
          gs->provides("KDL")->addOperation("twistMsgToKDL",&tf::twistMsgToKDL);
          gs->provides("KDL")->addOperation("twistKDLToMsg",&tf::twistKDLToMsg);
          gs->provides("KDL")->addOperation("vectorMsgToKDL",&tf::vectorMsgToKDL);
          gs->provides("KDL")->addOperation("vectorKDLToMsg",&tf::vectorKDLToMsg);
          gs->provides("KDL")->addOperation("wrenchMsgToKDL",&tf::wrenchMsgToKDL);
          gs->provides("KDL")->addOperation("wrenchKDLToMsg",&tf::wrenchKDLToMsg);
          return true;
      }
  };
    /**
     * The single global instance of the KDL Typekit.
     */
    extern KDLConversionTypekitPlugin KDLTypekit;
}

ORO_TYPEKIT_PLUGIN(KDL::KDLConversionTypekitPlugin)
