/*
 * (C) 2020, Intermodalics BVBA
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
     * it to a queue. In this way, the queue is bypassed and the callback is
     * immediately executed.
     * 
     * @param callback callback to execute, instead of queueing
     * @param owner_id Not used
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
     */
    virtual void removeByID(uint64_t owner_id) {}

}; // class PassthroughCallbackQueue

} // namespace rtt_roscomm

#endif // RTT_ROSCOMM__PASSTHROUGH_CALLBACK_QUEUE_HPP_
