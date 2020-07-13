#ifndef THREADED_CALLBACK_HPP
#define THREADED_CALLBACK_HPP

#include <condition_variable>
#include <thread>
#include <mutex>

#include <pthread.h>

class ThreadedCallback
{
public:
  /// Constructor
  /**
   * \param[in] sched_priority task thread priority.
   * \param[in] policy specify SCHED_OTHER, SCHED_RR, SCHED_FIFO etc, see pthread_setschedparam(3).
   */
  ThreadedCallback(size_t sched_priority=0, int policy=SCHED_OTHER, size_t core_id=1):
      ready_(false), working_(false), fin_(false)
  {
    auto task_thread_func =
        [this]() -> void
        {
          for(;;) {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this]{ return ready_; });
            ready_ = false;
            if(fin_) break;
            working_ = true;

            // do staffs
            if(overrun_timer_.get()) {
              overrun_timer_->reset();
            }
            on_callback();
            if(overrun_timer_.get()) {
              overrun_timer_->cancel();
            }

            working_ = false;
          }
        };

    t_ = std::move(std::thread(task_thread_func));

    // set scheduler
    struct sched_param param;
    param.sched_priority = sched_priority;
    int ret = pthread_setschedparam(t_.native_handle(), policy, &param);
    if(0 != ret) {
      // TODO error handling
      std::cout << "cannot set task thread scheduler, retval = " << ret
                << " policy = " << policy
                << " priority = " << sched_priority
                << std::endl;
    }

    // set affinity
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);  // TODO core_id is valid
    int rc = pthread_setaffinity_np(t_.native_handle(),
                                    sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
      std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
    }
  }

  virtual ~ThreadedCallback()
  {
    {
      std::unique_lock<std::mutex> lk(mtx_);
      ready_ = true;
      fin_ = true;
    }
    cv_.notify_one();
    t_.join();
  }

protected:
  /// subscription callback
  /**
   * Subscription callback function.
   * This is executed as task thread.
   * Task thread is created per create_subscription.
   * For example, if there are 3 concrete class of ThreadedSubscription,
   * 3 task thread are created.
   *
   * Use `msg` variable to read topic.
   *
   * TODO: give msg as argument, i.e. change signiture as on_subscription(T msg).
   */
  virtual void on_callback() = 0;

  /// overrun handler
  /**
   * Overrun handler.
   */
  virtual void on_overrun() = 0;

  /**
   * Implement orverrun handler by oneshot timer.
   * timer instance is hold by this class and caller can hold (as you like).
   * If caller set timer manually, I don't know what happends.
   *
   * TODO: move it to private function.
   * Don't call this, but use create_subscription with oeverun_period.
   */
  template<typename DurationRepT = int64_t, typename DurationT = std::milli>
  auto create_overrun_handler(
      rclcpp::Node *node,
      std::chrono::duration<DurationRepT, DurationT> overrun_period)
  {
    auto callback =
        [this]() -> void
        {
          on_overrun();
          overrun_timer_->cancel();
        };

    std::cout << "overrun timer " << std::chrono::duration_cast<std::chrono::milliseconds>(overrun_period).count() << " [ms]" << std::endl;
    overrun_timer_ = node->create_wall_timer(overrun_period, callback);
    overrun_timer_->cancel();

    return overrun_timer_;
  }

  std::condition_variable cv_;
  std::mutex mtx_;
  bool ready_;
  bool working_;
  bool fin_;

private:
  std::thread t_;
  rclcpp::TimerBase::SharedPtr overrun_timer_;
};

#endif // THREADED_CALLBACK_HPP
