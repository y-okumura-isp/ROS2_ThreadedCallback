#include <chrono>
#include <memory>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

typedef std_msgs::msg::String MyMsg;
using namespace std::chrono_literals;

#define _NOW (std::chrono::system_clock::now())
#define NOW (std::chrono::duration_cast<std::chrono::milliseconds>(_NOW - st_).count())

class PubNode : public rclcpp::Node
{
public:
  PubNode(const std::string & name,
          const std::string & ns,
          const rclcpp::QoS & qos,
          uint64_t period_ms,
          uint64_t gap_ms,
          bool runs_A, bool runs_B, bool runs_C):
      Node(name, ns), count_a_(0), count_b_(0), count_c_(0), st_(_NOW)
  {
    pubA_ = this->create_publisher<MyMsg>("topic_a", qos);
    pubB_ = this->create_publisher<MyMsg>("topic_b", qos);
    pubC_ = this->create_publisher<MyMsg>("topic_c", qos);

    auto callback_a =
        [this]() -> void
        {
          std::cout << "pubA t = " << NOW << std::endl;
          MyMsg msg;
          msg.data = "HelloWorld" + std::to_string(count_a_);
          pubA_->publish(msg);
          count_a_++;
        };
    auto callback_b =
        [this]() -> void
        {
          std::cout << "pubB t = " << NOW << std::endl;
          MyMsg msg;
          msg.data = "HelloWorld" + std::to_string(count_b_);
          pubB_->publish(msg);
          count_b_++;
        };
    auto callback_c =
        [this]() -> void
        {
          std::cout << "pubC t = " << NOW << std::endl;
          MyMsg msg;
          msg.data = "HelloWorld" + std::to_string(count_c_);
          pubC_->publish(msg);
          count_c_++;
        };


    // run timers 100ms gap
    if(runs_C) {
      timerC_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                        callback_c);
      std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
    }

    if(runs_B) {
      timerB_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                        callback_b);
      std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
    }

    if(runs_A) {
      timerA_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                          callback_a);
      std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timerA_, timerB_, timerC_;
  rclcpp::Publisher<MyMsg>::SharedPtr pubA_, pubB_, pubC_;
  uint64_t count_a_, count_b_, count_c_;
  std::chrono::system_clock::time_point st_;
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
