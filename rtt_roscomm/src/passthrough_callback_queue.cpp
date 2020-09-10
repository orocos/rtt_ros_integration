
#include "rtt_roscomm/passthrough_callback_queue.hpp"

namespace rtt_roscomm {

PassthroughCallbackQueue::PassthroughCallbackQueue()
        {}

void PassthroughCallbackQueue::addCallback(
    const ros::CallbackInterfacePtr &callback,
    uint64_t owner_id)
{
  // Call CallbackInterface::CallResult SubscriptionQueue::call()
  if (ros::CallbackInterface::TryAgain == callback->call()) {
    ros::getGlobalCallbackQueue()->addCallback(callback, owner_id);
  }
}

void PassthroughCallbackQueue::removeByID(uint64_t owner_id)
{
}

} // namespace rtt_roscomm

