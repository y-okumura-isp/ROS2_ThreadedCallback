#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "threaded_wall_timer.hpp"

typedef std_msgs::msg::String MyMsg;
using namespace std::chrono_literals;

#define _NOW (std::chrono::system_clock::now())
#define NOW (std::chrono::duration_cast<std::chrono::milliseconds>(_NOW - st_).count())

class SampleSenderByTimer : public ThreadedWallTimer
{
public:
  SampleSenderByTimer(
      rclcpp::Node *node, const std::string &topic, const rclcpp::QoS qos,
      size_t sched_priority=0, int policy=SCHED_OTHER, size_t core_id=1)
      : ThreadedWallTimer(sched_priority, policy, core_id),
        count_(0), topic_(topic),
        st_(_NOW)
  {
    pub_ = node->create_publisher<MyMsg>(topic, qos);
  }

protected:
  void on_callback() override;

  void on_overrun() override;

private:
  rclcpp::Publisher<MyMsg>::SharedPtr pub_;
  uint64_t count_;
  const std::string topic_;
  MyMsg msg_;

  std::chrono::system_clock::time_point st_;
};

void SampleSenderByTimer::on_callback()
{
  std::cout << topic_ << " t = " << NOW << std::endl;
  msg_.data = "HelloWorld" + std::to_string(count_);
  pub_->publish(msg_);
  count_++;
}

void SampleSenderByTimer::on_overrun()
{
}

class PubNode : public rclcpp::Node
{
public:
  PubNode(const std::string & name,
          const std::string & ns,
          const rclcpp::QoS & qos,
          uint64_t period_ms,
          uint64_t gap_ms,
          bool runs_A, bool runs_B, bool runs_C):
      Node(name, ns)
  {
    // run timers 100ms gap
    if(runs_C) {
      std::cout << "runs_C" << std::endl;
      helper_c_ = std::make_unique<SampleSenderByTimer>(this, "topic_c", qos);
      timerC_ = helper_c_->create_wall_timer(this, std::chrono::milliseconds(period_ms));
      std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
    }

    if(runs_B) {
      std::cout << "runs_C" << std::endl;
      helper_b_ = std::make_unique<SampleSenderByTimer>(this, "topic_c", qos);
      timerB_ = helper_b_->create_wall_timer(this, std::chrono::milliseconds(period_ms));
      std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
    }

    if(runs_A) {
      std::cout << "runs_A" << std::endl;
      helper_a_ = std::make_unique<SampleSenderByTimer>(this, "topic_a", qos);
      timerA_ = helper_a_->create_wall_timer(this, std::chrono::milliseconds(period_ms));
      std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timerA_, timerB_, timerC_;

  std::unique_ptr<SampleSenderByTimer> helper_a_, helper_b_, helper_c_;
};

int main(int argc, char *argv[]) {
  auto qos = 1;
  bool runs_A = false;
  bool runs_B = false;
  bool runs_C = false;

  /* option
   *   -a: send pubA
   *   -b: send pubB
   *   -c: send pubC
   */
  const std::string optstring = "-abc";
  int c = 0;
  while((c = getopt(argc, argv, optstring.c_str())) != -1) {
    switch(c)
    {
      case('a'): {
        runs_A = true;
        break;
      }
      case('b'): {
        runs_B = true;
        break;
      }
      case('c'): {
        runs_C = true;
        break;
      }
      default:
        break;
    }
  }

  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto pub_node = std::make_shared<PubNode>("Pub", "ns", qos, 1000, 100,
                                            runs_A, runs_B, runs_C);
  exec->add_node(pub_node);

  exec->spin();
  exec->remove_node(pub_node);

  rclcpp::shutdown();
  return 0;
}
