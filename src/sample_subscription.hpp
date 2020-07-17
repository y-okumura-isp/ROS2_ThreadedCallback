#ifndef SAMPLE_SUBSCRIPTION_HPP
#define SAMPLE_SUBSCRIPTION_HPP

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "threaded_subscriber.hpp"

using namespace std::chrono_literals;

std::chrono::system_clock::time_point time_now()
{
  return std::chrono::system_clock::now();
}

long time_diff(const std::chrono::system_clock::time_point &from)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(time_now() - from).count();
}

typedef std_msgs::msg::String MyMsg;

/// Dummy callback with time consuming task.
/**
 * This callback sum length LEN array TIMES times.
 */
template<int LEN, int TIMES>
class SampleThreadedSubscription : public ThreadedSubscription<MyMsg::UniquePtr, MyMsg>
{
public:
  SampleThreadedSubscription(const std::string &name,
                             size_t sched_priority,
                             int policy,
                             size_t core_id):
      ThreadedSubscription<MyMsg::UniquePtr, MyMsg>(sched_priority, policy, core_id),
      name_(name), count_(0), st_(time_now())
  {}

protected:
  void on_callback() override
  {
    auto t = time_now();
    // Dump msg just after called
    std::cout << name_ << " start " << msg_.data.c_str()
              << " at t = " << time_diff(st_)
              << " count_ = " << count_
              << std::endl;

    // dummy procedure which takes long time,
    // this calcuration has no meaning.
    for(int i=0; i<LEN; i++) {
      data_[i] = count_;
    }
    for(int j=0; j<TIMES; j++) {
      std::cout << name_ << " loop " << j << std::endl;
      for(int i=0; i<LEN; i++) {
        data_[i] += std::sin(i+j);
      }
    }
    auto v = std::accumulate(data_.begin(), data_.end(), 0.0);

    // dump msg at end of function
    std::cout << name_ << " end   " << msg_.data.c_str()
              << " at t = " << time_diff(st_)
              << " count_ = " << count_
              << " v = " << v
              << " tdiff = " << time_diff(t)
              << std::endl;

    count_++;
  }

  void on_overrun() override
  {
    std::cout << name_
              << " overrun at t = " << time_diff(st_)
              << std::endl;
  }

private:
  std::string name_;
  uint64_t count_;
  std::chrono::system_clock::time_point st_;
  std::array<double, LEN> data_;
};


/// SubA. the fastest procedure, the highest priority.
template<int LEN, int TIMES>
class SampleNode : public rclcpp::Node
{
public:
  /// constructor
  /**
   * \param[in] name node name
   * \param[in] ns namespace
   * \param[in] topic_name topic name
   * \param[in] qos topic qos
   */
  SampleNode(const std::string & name,
             const std::string & ns,
             const std::string & topic_name,
             const rclcpp::QoS & qos,
             size_t sched_priority,
             int policy,
             size_t core_id):
      Node(name, ns),
      helper_(new SampleThreadedSubscription<LEN, TIMES>(name, sched_priority, policy, core_id))
  {
    subscription_ = helper_->create_subscription(
        this,
        topic_name,
        qos);
  }

  /// Constructor with overrun handler
  template<typename DurationRepT = int64_t, typename DurationT = std::milli>
  SampleNode(const std::string & name,
             const std::string & ns,
             const std::string & topic_name,
             const rclcpp::QoS & qos,
             size_t sched_priority,
             int policy,
             size_t core_id,
             std::chrono::duration<DurationRepT, DurationT> overrun_period):
      Node(name, ns),
      helper_(new SampleThreadedSubscription<LEN, TIMES>(name, sched_priority, policy, core_id))
  {
    subscription_ = helper_->create_subscription(
        this,
        topic_name,
        qos,
        overrun_period);
  }

private:
  std::unique_ptr<SampleThreadedSubscription<LEN, TIMES>> helper_;

  rclcpp::Subscription<MyMsg>::SharedPtr subscription_;
};

#endif  // SAMPLE_SUBSCRIPTION_HPP
