#include <iostream>
#include <memory>

#include <pthread.h>

#include "sample_subscription.hpp"

using namespace std::chrono_literals;

// see rttest_set_sched_priority
void set_sched_priority(size_t sched_priority, int policy)
{
  struct sched_param param;

  param.sched_priority = sched_priority;

  // note that sched_setscheduler can set the priority of an arbitrary process
  int ret = sched_setscheduler(0, policy, &param);
  if(ret < 0) {
    std::cout << "sched_setscheduler of main thread failed ret = " << ret << std::endl;
  }
}

/// main
int main(int argc, char *argv[]) {
  auto qos = 1;

  // Thread policy:
  //   main thread = 98, DDS child thread 97, callback is more lower
  // Threads are created internaly when calling rclcpp::init(), add_node().
  // So, set priority = 97 initially and set 98 just before spin.
  set_sched_priority(97, SCHED_RR);

  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  size_t core_id = 2;

  auto sub_a_node = std::make_shared<SampleNode<640*480, 1>>("SubA", "ns", "topic_a", qos,
                                                             90, SCHED_RR, core_id);
  exec->add_node(sub_a_node);
  auto sub_b_node = std::make_shared<SampleNode<640*480, 10>>("SubB", "ns", "topic_b", qos,
                                                              80, SCHED_RR, core_id);
  exec->add_node(sub_b_node);
  auto sub_c_node = std::make_shared<SampleNode<640*480, 100>>("SubC", "ns", "topic_c", qos,
                                                               70, SCHED_RR, core_id,
                                                               70ms);
  exec->add_node(sub_c_node);

  // finally set main thread scheduling policy, affinity
  set_sched_priority(98, SCHED_RR);
  // TODO set affinity

  exec->spin();
  exec->remove_node(sub_c_node);
  exec->remove_node(sub_b_node);
  exec->remove_node(sub_a_node);

  rclcpp::shutdown();
  return 0;
}

