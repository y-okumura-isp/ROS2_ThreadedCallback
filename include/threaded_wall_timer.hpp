#ifndef THREADED_WALL_TIMER_HPP
#define THREADED_WALL_TIMER_HPP

#include "threaded_callback.hpp"

class ThreadedWallTimer : public ThreadedCallback
{
public:
  ThreadedWallTimer(size_t sched_priority=0, int policy=SCHED_OTHER, size_t core_id=1):
      ThreadedCallback(sched_priority, policy, core_id)
  {}

  template<typename DurationRepT = int64_t, typename DurationT = std::milli>
  auto create_wall_timer(rclcpp::Node *node,
                         std::chrono::duration<DurationRepT, DurationT> period)
  {
    auto callback =
        [this, period]() -> void
        {
          if(working_) {
            std::cout << "sorry I'm busy" << std::endl;
            return;
          }

          {
            std::lock_guard<std::mutex> lk(mtx_);
            ready_ = true;
          }
          cv_.notify_one();
       };

    return node->create_wall_timer(period, callback);
  }

  template<typename DurationRepT = int64_t, typename DurationT = std::milli>
  auto create_wall_timer(rclcpp::Node *node,
                         std::chrono::duration<DurationRepT, DurationT> period,
                         std::chrono::duration<DurationRepT, DurationT> overrun_period)
  {
    create_overrun_handler(node, overrun_period);
    return create_subscription(node, period);
  }
};

#endif  // THREADED_WALL_TIMER_HPP
