#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <tf_conversions/tf_kdl.h>

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
          gs->provides("KDL")->addOperation("TwistToMsg",&tf::TwistKDLToMsg);
          gs->provides("KDL")->addOperation("MsgToTwist",&tf::TwistMsgToKDL);
          gs->provides("KDL")->addOperation("FrameToMsg",&tf::PoseKDLToMsg);
          gs->provides("KDL")->addOperation("MsgToFrame",&tf::PoseKDLToMsg);
          return true;
      }
  };
    /**
     * The single global instance of the KDL Typekit.
     */
    extern KDLConversionTypekitPlugin KDLTypekit;
}

ORO_TYPEKIT_PLUGIN(KDL::KDLConversionTypekitPlugin)
