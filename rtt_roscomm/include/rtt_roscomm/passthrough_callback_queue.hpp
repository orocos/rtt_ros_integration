#ifndef RTT_ROSCOMM__PASSTHROUGH_CALLBACK_QUEUE_HPP_
#define RTT_ROSCOMM__PASSTHROUGH_CALLBACK_QUEUE_HPP_

#include <ros/ros.h>
#include <ros/callback_queue_interface.h>
#include <ros/subscription_queue.h>
#include <ros/callback_queue.h>

namespace rtt_roscomm {
class PassthroughCallbackQueue: public ros::CallbackQueueInterface
{
  public:
    /** 
     * Implementation of ros::CallbackQueueInterface::addCallback()
     * 
     * This method is executing the callback received instead of adding
     * it to a queue. In this way, the queue is bypassed and immediately
     * executed.
     * 
     * @param callback callback to execute, instead of queueing
     * @param owner_id Owner of the callback, in other implementations it might
     *                 be used to remove the callback. In this implementation
     *                 it has no effect.
     * 
     * @return void
     */
    virtual void addCallback(
        const ros::CallbackInterfacePtr &callback,
        uint64_t owner_id=0);

    /** 
     * Implementation of ros::CallbackQueueInterface::removeByID()
     * 
     * No-op
     * 
     * @param owner_id Not used.
     * 
     * @return void
     */
    virtual void removeByID(uint64_t owner_id) {}

}; // class PassthroughCallbackQueue

} // namespace rtt_roscomm

#endif // RTT_ROSCOMM__PASSTHROUGH_CALLBACK_QUEUE_HPP_
