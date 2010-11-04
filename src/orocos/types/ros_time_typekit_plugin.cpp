#include <ros/time.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>

namespace ros_integration {
  using namespace RTT;
   
    /**
     * This interface defines the types of the realTime package.
     */
    class ROStimeTypekitPlugin
      : public types::TypekitPlugin
    {
    public:
      virtual std::string getName(){
	return std::string("ros-")+"time";
      }

      virtual bool loadTypes() {
	RTT::types::Types()->addType( new types::TemplateTypeInfo<ros::Time>("time") );
	return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROStimeTypekitPlugin )

