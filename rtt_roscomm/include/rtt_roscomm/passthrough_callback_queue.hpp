
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
      PassthroughCallbackQueue();

      /** 
       * Implementation of ros::CallbackQueueInterface::addCallback()
       * 
       * @param callback aaa
       * @param owner_id aaa
       * 
       * @return void
       */
      virtual void addCallback(
          const ros::CallbackInterfacePtr &callback,
          uint64_t owner_id=0);

      /** 
       * Implementation of ros::CallbackQueueInterface::removeByID()
       * 
       * @param owner_id aaa
       * 
       * @return void
       */
      virtual void removeByID(uint64_t owner_id);

    private:

      /**
       * Port where to write the message
       */
      // base::PortInterface* output_port_ptr_;

  }; // class PassthroughCallbackQueue

  /**
   * Helpers to generate subscribers with a custom ros::CallbackQueueInterface
   */

      // template<class M, class T>
    // ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size, 
    //                     void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj,
    //                     ros::CallbackQueueInterface* cq, ros::NodeHandle& nh,
    //                     const TransportHints& transport_hints = ros::TransportHints())
    // {
    //   ros::SubscribeOptions ops;
    //   ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
    //   ops.transport_hints = transport_hints;
    //   ops.callback_queue = cq;
    //   return nh.subscribe(ops);
    // }

  template<class M, class T>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj, 
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }

  /// and the const version
  template<class M, class T>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size, void(T::*fp)(M) const, T* obj, 
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }
  template<class M, class T>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size, 
      void(T::*fp)(const boost::shared_ptr<M const>&), T* obj, 
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }
  template<class M, class T>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size, 
      void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj, 
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, boost::bind(fp, obj, _1));
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }
  template<class M, class T>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size, void(T::*fp)(M), 
      const boost::shared_ptr<T>& obj,
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
    ops.tracked_object = obj;
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }

  template<class M, class T>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size, void(T::*fp)(M) const, 
      const boost::shared_ptr<T>& obj,
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
    ops.tracked_object = obj;
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }
  template<class M, class T>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size, 
      void(T::*fp)(const boost::shared_ptr<M const>&), 
      const boost::shared_ptr<T>& obj,
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
    ops.tracked_object = obj;
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }
  template<class M, class T>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size, 
      void(T::*fp)(const boost::shared_ptr<M const>&) const, 
      const boost::shared_ptr<T>& obj,
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, boost::bind(fp, obj.get(), _1));
    ops.tracked_object = obj;
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }
  template<class M>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size, void(*fp)(M),
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template initByFullCallbackType<M>(topic, queue_size, fp);
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }
  template<class M>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size,
      void(*fp)(const boost::shared_ptr<M const>&),
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, fp);
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }
  template<class M>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size,
      const boost::function<void (const boost::shared_ptr<M const>&)>& callback,
      const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template init<M>(topic, queue_size, callback);
    ops.tracked_object = tracked_object;
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }
  template<class M, class C>
  ros::Subscriber subscribe(
      ros::NodeHandle& nh, ros::CallbackQueueInterface* cq,
      const std::string& topic, uint32_t queue_size,
      const boost::function<void (C)>& callback,
      const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(),
      const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    ros::SubscribeOptions ops;
    ops.template initByFullCallbackType<C>(topic, queue_size, callback);
    ops.tracked_object = tracked_object;
    ops.transport_hints = transport_hints;
    ops.callback_queue = cq;
    return nh.subscribe(ops);
  }

} // namespace rtt_roscomm

#endif // RTT_ROSCOMM__PASSTHROUGH_CALLBACK_QUEUE_HPP_
