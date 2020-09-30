#include "rtt_roscomm/passthrough_callback_queue.hpp"

namespace rtt_roscomm {

void PassthroughCallbackQueue::addCallback(
    const ros::CallbackInterfacePtr &callback,
    uint64_t owner_id)
{
  ros::CallbackInterface::CallResult result = callback->call();
  if (ros::CallbackInterface::TryAgain == result) {
    ros::getGlobalCallbackQueue()->addCallback(callback, owner_id);
  }
}

} // namespace rtt_roscomm
