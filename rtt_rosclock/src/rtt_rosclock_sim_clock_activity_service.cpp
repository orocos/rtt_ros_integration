
#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/types/PropertyDecomposition.hpp>

#include <rtt_rosclock/rtt_rosclock.h>

using namespace RTT;
using namespace std;

class SimClockActivityService: public RTT::Service
{
public:

  SimClockActivityService(TaskContext* owner) :
    Service("sim_clock_activity", owner)
  {
    this->doc("RTT Service for synchronizing ROS parameters with the properties of a corresponding RTT component");

    // TODO: load rosclock global service
    rtt_rosclock::set_sim_clock_activity(owner);
  }

};

ORO_SERVICE_NAMED_PLUGIN(SimClockActivityService, "sim_clock_activity")
