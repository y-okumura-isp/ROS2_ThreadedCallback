#ifndef THREADED_SUBSCRIBER_HPP
#define THREADED_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include "threaded_callback.hpp"

/// Subscription which does subscription callback by thread.
template<typename MsgPtr, typename Msg>
class ThreadedSubscription : public ThreadedCallback
{
public:
  ThreadedSubscription(size_t sched_priority=0, int policy=SCHED_OTHER, size_t core_id=1):
      ThreadedCallback(sched_priority, policy, core_id)
  {}

  auto create_subscription(rclcpp::Node *node,
                           const std::string & topic,
                           const rclcpp::QoS & qos)
  {
    auto callback =
        [this, topic](MsgPtr msg) -> void
        {
          if(working_) {
            std::cout << "sorry I'm busy" << std::endl;
            return;
          }

          {
            std::lock_guard<std::mutex> lk(mtx_);
            ready_ = true;

            // TODO: Is it deep copy? Anyway we want to avoid data copy.
            // But to avoid it, we should consider what happens when new topic is received under task_thread working.
            // In task_thread, msg may change at begining and end even if msg is updated in atomic way.
            msg_ = *(msg.get());
          }
          cv_.notify_one();
        };

    sub_ = node->create_subscription<Msg>(topic, qos, callback);
    return sub_;
  }

  /// create_subscription with overrun handler
  /**
   * create subscription with overrun handler.
   * on_overun() is fire if overrun_period time passes and  subscription callback still running.
   *
   * \param[in] node node pointer
   * \param[in] topic topic name
   * \param[in] qos topic QoS
   * \param[in] overrun_period overrun period
   */
  template<typename DurationRepT = int64_t, typename DurationT = std::milli>
  auto create_subscription(rclcpp::Node *node,  // TODO use smart pointer
                           const std::string & topic,
                           const rclcpp::QoS & qos,
                           std::chrono::duration<DurationRepT, DurationT> overrun_period)
  {
    create_overrun_handler(node, overrun_period);
    return create_subscription(node, topic, qos);
  }

protected:
  /// you can read topic by this variable
  Msg msg_;
  typename rclcpp::Subscription<Msg>::SharedPtr sub_;
};

#endif  // THREADED_SUBSCRIBER_HPP
