## What's this?
- Do callbacks as different thread from main thread with specific CPU core and scheduling policy, and detect overrun.
  It's a PoC (proof of concept) implementation so I implement in user-land, not in ROS 2 layer.
  Now only subscriber callback is implemented.
- `ThreadedSubscription` in `include/threaded_subscriber.hpp` is a helper class to create such a callback.
- In `include/sample_subscription.hpp`, you can see how to inherit `SampleThreadedSubscription` and how to use it from Node in `SampleNode`.
  The `main` function is in `src/main_sub.cpp`, it's an implementation of the PoC scenario.

## Motivation
- Callback as thread
  - By using ROS 2 rclcpp SingleThreadedExecutor, callbacks such as subscriber callback or timer callback are called in main thread.
  - Predictability is important for Real-Time, but running all threads makes things difficult.
  - For example, if executor has many exectable callbacks at the same time(how many? it depends on the timing),
    it's difficult to predict when specific callback will be done.
  - Of course as thread creation shoud be avoided in runtime, callback thread should be re-used.

- Scheduling: thread priority and policy
  - In [ROS2 generated child thread scheduling policy affects timers](https://discourse.ros.org/t/ros2-generated-child-thread-scheduling-policy-affects-timers/14212), DDS child process affects sleep jitter if only one priority is used.
  - And even if callbacks are done in other thread than main thread, scheduler falls into RR(Round-Robin) or FIFO if only one priority is used.
  - So it's good for programmer to specify priority or scheduling policy.

- CPU Core
  - Callbacks in different CPU core don't affect each other.
  - Pottentially running too many thread in one core makes more overhead, and core migration makes more cache-miss .
  - Simply way, but it's also good to specify CPU core i.e. thread affiniy.

- Detect overrun (deadline)
  - It's good to detect deadline for error handling.


## PoC scenario
- Consider
  - There is only one CPU core.
  - There are 3 tasks(callbacks) with different priority, namely TaskA, TaskB, and TaskC.
  - Tasks are fired by topic, namely there are 3 paris, PubA-SubA, PubB-SubB, PubC-SubC.
    Tasks are fired by SubX as thread.
  - TaskA has the highest priority and shortest task.
    TaskB has middle priority and middle task.
    TaskC has the lowest priority and longest task.
    Namely TaskA shoud run even if TaskB or TaskC is running. 

To illustrate this, see figure below.
  - TaskC fires by topic C. `-` means TaskC is running.
  - When topic B comes, TaskB fires and TaskC is stopped. `O` means TackC is stop
    The same is true when topic A comes.
  - When TaskA is finished(`X` means this), TaskB runs because TaskB has higer priority than TaskC.
    The same When TaskB finished.

(In a nutshell preemption.)

```
priority
   ^
   | TaskA                        ---X
   | 
   | TaskB                --------O    ------X
   | 
   | TaskC   ------------O                    ------X
   |         ^           ^        ^
   |         |           |        |
   | Topic   C           B        A
   |
   +----------------------------------------------------------> time
```

## Environment
- I build & run:
  - Ubuntu 18.04 64bit + ROS 2 Foxy
  - Raspberry Pi 32bit + ROS 2 Eloquent

## Build 

```
colcon build --symlink-install
```

## Run scenario
Use 2 terminals. To understand the scenario see below.

```
# first terminal (must be root)
./build/realistic_threaded_callback/main_sub

# second terminal
./build/realistic_threaded_callback/main_pub -a -b -c
```

`main_sub` runs SubA, SubB and SubC.
Subscribers have dummy task: they has a large array, and manipulate the array with many loops.
`SubC loop 0` etc in below stdout is outputed in the loop.
Callback code image:

```cpp
for(int i=0; i<LOOP; i++) {
  std::cout << "SubC loop " << i << std::endl;
  for(int j=i; j<LEN; j++) {
    data[j] +=  do_something();
  }
}
```

`main_pub` runs PubA if `-a` option is specified. `-b` and `-c` mean PubB, PubC in order.
Topics for SubC, SubB and SubA are published periodically with a delay of 100 ms.


You can see following log in stdout.

```
SubC start HelloWorld0 at t = 7246 count_ = 0     // SubC starts at t = 7246
SubC loop 0
SubB start HelloWorld0 at t = 7378 count_ = 0     // TopicB comes at t = 7348 and SubB starts.
SubB loop 0                                       // SubC stops, and SubB runs.
SubA start HelloWorld0 at t = 7509 count_ = 0     // TopicB comes at t = 7509 and SubA starts.
SubA loop 0                                       // SubB stops, and SubA runs.
SubA end   HelloWorld0 at t = 7627 count_ = 0 v = 1.353 tdiff = 118 // SubA finishes.
SubB loop 1                                       // SubB resumes.
SubB loop 2

// snip

SubB loop 9
SubB end   HelloWorld0 at t = 8386 count_ = 0 v = 3.39885 tdiff = 1008  // SubB finished.
SubC loop 1                                       // SubC resumes

// snip

SubC loop 19
SubC end   HelloWorld0 at t = 10017 count_ = 0 v = 1.65432 tdiff = 2771 // SubC finished.
```

If tasks are done too fast, change SampleNode loop count(2nd argument for template) such as `SampleNode<640*480, 10>` to `SampleNode<640*480, 100>`.

## How to use ThreadedSubscription
I think ThreadedSubscription is only PoC class, so we should implement a similar mechanism in ROS layer, and provide more sophisticated API.
But to clarify requests for threaded callback, I write how to use ThreadedSubscription.
Please see `include/sample_subscription.hpp` for concrete example.

(1) To create helper class, inherit ThreadedSubscription and define 2 callbacks: `on_subscription()`, `on_overrun()`.
    `on_subscription` is for subscription callback. You can read topic via msg_ variable.
    `on_overrun` is overrun handler, a callback for deadline miss.
    In constructor you can specify sched_priority and policy `ThreadedSubscription(size_t sched_priority=0, int policy=SCHED_OTHER)`
(2) Use created helper in Node class
  Use `ThreadedSubscription::create_subscription(rclcpp::Node *node, const std::string & topic, const rclcpp::QoS & qos)` when you don't nedd overrun handler.
  Use  `ThreadedSubscription::create_subscription(rclcpp::Node *node, const std::string & topic, const rclcpp::QoS & qos, std::chrono::duration<DurationRepT, DurationT> overrun_period)` when you need overrun handler.

TODO
  - Unify hepler class into Node
    It's more intuitive to specify subscription callback and overrun handler in Node::create_subscription.
  - Consider data lifecycle, how to hold data
    As Data writer(executor in this situation) and data reader(subscription callback) runs in parallel, we need to prevent data from changing in the middle of the callback.
    In my implementation, instance variable `msg_` is used to store topic for callback thread.
    So extra data copy is done.
    It seems good for me to call `rcl_take` in callback thread.

## Future work
- See TODO in above chapter.
- If the concept "callback as thread with specific CPU core and scheduling policy and overun handler" is acceptable, I want to implement similar mechanics in rcl or rclcpp. But as rcl does not have executor mechanism, it may be easy to start with rclcpp.
- Decide what to do for new topic when the callback is already running. Drop? Delay?
  To pevent from waiting lock, ThreadedSubscription knows the callback thread is running or not.
  So when the callback is already running, ThreadedSubscription only receives new topics and does nothing.
  - If we select delay, we may need to consider Executor `get_next_ready_executable` and `wait_for_work` relation.
    We should clear event flag, but execute subscription lazily.
