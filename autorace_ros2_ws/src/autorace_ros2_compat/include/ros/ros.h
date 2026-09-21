#pragma once

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <typeinfo>
#include <type_traits>

namespace ros {

inline rclcpp::Node::SharedPtr & global_node_storage()
{
  static rclcpp::Node::SharedPtr node;
  return node;
}

inline rclcpp::Node::SharedPtr node()
{
  auto n = global_node_storage();
  if (!n) {
    throw std::runtime_error("ros2 compat: ros::init() must be called before NodeHandle use");
  }
  return n;
}

inline std::string normalize_parameter_name(std::string name)
{
  while (!name.empty() && name.front() == '/') name.erase(name.begin());
  for (char &c : name) {
    if (c == '/') c = '.';
  }
  if (name.empty()) name = "unnamed_parameter";
  return name;
}

inline void init(int argc, char ** argv, const std::string & name)
{
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  global_node_storage() = std::make_shared<rclcpp::Node>(name, options);
}

inline bool ok() { return rclcpp::ok(); }
inline void spinOnce() { rclcpp::spin_some(node()); }
inline void spin() { rclcpp::spin(node()); }
inline void shutdown() { rclcpp::shutdown(); }

class Rate {
public:
  explicit Rate(double hz) : rate_(hz) {}
  void sleep() { rate_.sleep(); }
private:
  rclcpp::Rate rate_;
};

class Duration {
public:
  explicit Duration(double seconds = 0.0) : seconds_(seconds) {}
  void sleep() const {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds_));
  }
  double toSec() const { return seconds_; }
private:
  double seconds_;
};

struct TimerEvent {};

struct Time {
  static void init() {}
  static builtin_interfaces::msg::Time now() {
    const auto t = node()->now().nanoseconds();
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<int32_t>(t / 1000000000LL);
    msg.nanosec = static_cast<uint32_t>(t % 1000000000LL);
    return msg;
  }
};

class Publisher {
  struct Concept { virtual ~Concept() = default; };
  template<typename MsgT>
  struct Model : Concept {
    explicit Model(typename rclcpp::Publisher<MsgT>::SharedPtr p) : pub(std::move(p)) {}
    typename rclcpp::Publisher<MsgT>::SharedPtr pub;
  };
public:
  Publisher() = default;

  template<typename MsgT>
  static Publisher make(typename rclcpp::Publisher<MsgT>::SharedPtr p)
  {
    Publisher out;
    out.impl_ = std::make_shared<Model<MsgT>>(std::move(p));
    return out;
  }

  template<typename MsgT>
  explicit Publisher(typename rclcpp::Publisher<MsgT>::SharedPtr p)
  : impl_(std::make_shared<Model<MsgT>>(std::move(p))) {}

  template<typename MsgT>
  void publish(const MsgT & msg) const {
    auto p = std::dynamic_pointer_cast<Model<MsgT>>(impl_);
    if (!p) {
      throw std::runtime_error(std::string("ros2 compat publisher type mismatch: ") + typeid(MsgT).name());
    }
    p->pub->publish(msg);
  }

  template<typename MsgT>
  void publish(const std::shared_ptr<MsgT> & msg) const { publish(*msg); }

  explicit operator bool() const { return static_cast<bool>(impl_); }
private:
  std::shared_ptr<Concept> impl_;
};

class Subscriber {
public:
  Subscriber() = default;
  explicit Subscriber(rclcpp::SubscriptionBase::SharedPtr sub) : sub_(std::move(sub)) {}
private:
  rclcpp::SubscriptionBase::SharedPtr sub_;
};

class Timer {
public:
  Timer() = default;
  explicit Timer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}
private:
  rclcpp::TimerBase::SharedPtr timer_;
};

class ServiceClient {};

class NodeHandle {
public:
  NodeHandle() : node_(ros::node()), private_(false) {}
  explicit NodeHandle(const std::string & ns) : node_(ros::node()), private_(ns == "~"), ns_(ns == "~" ? "" : ns) {}

  template<typename MsgT>
  Publisher advertise(const std::string & topic, size_t queue_size, bool latch = false)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(queue_size == 0 ? 1 : queue_size));
    if (latch) qos.transient_local().reliable();
    return Publisher::make<MsgT>(node_->create_publisher<MsgT>(resolve(topic), qos));
  }

  template<typename ObjT, typename MsgT>
  Subscriber subscribe(
    const std::string & topic, size_t queue_size,
    void (ObjT::* cb)(const std::shared_ptr<const MsgT> &), ObjT * obj)
  {
    auto sub = node_->create_subscription<MsgT>(
      resolve(topic), subscription_qos(queue_size),
      [obj, cb](typename MsgT::ConstSharedPtr msg) { (obj->*cb)(msg); });
    return Subscriber(sub);
  }

  template<typename ObjT, typename MsgT>
  Subscriber subscribe(
    const std::string & topic, size_t queue_size,
    void (ObjT::* cb)(std::shared_ptr<const MsgT>), ObjT * obj)
  {
    auto sub = node_->create_subscription<MsgT>(
      resolve(topic), subscription_qos(queue_size),
      [obj, cb](typename MsgT::ConstSharedPtr msg) { (obj->*cb)(msg); });
    return Subscriber(sub);
  }

  template<typename ObjT, typename MsgT>
  Subscriber subscribe(
    const std::string & topic, size_t queue_size,
    void (ObjT::* cb)(const MsgT &), ObjT * obj)
  {
    auto sub = node_->create_subscription<MsgT>(
      resolve(topic), subscription_qos(queue_size),
      [obj, cb](typename MsgT::ConstSharedPtr msg) { (obj->*cb)(*msg); });
    return Subscriber(sub);
  }

  template<typename MsgT>
  Subscriber subscribe(const std::string & topic, size_t queue_size, void (*cb)(MsgT))
  {
    auto sub = node_->create_subscription<MsgT>(
      resolve(topic), subscription_qos(queue_size),
      [cb](typename MsgT::ConstSharedPtr msg) { cb(*msg); });
    return Subscriber(sub);
  }

  template<typename MsgT>
  Subscriber subscribe(
    const std::string & topic, size_t queue_size,
    void (*cb)(const std::shared_ptr<const MsgT> &))
  {
    auto sub = node_->create_subscription<MsgT>(
      resolve(topic), subscription_qos(queue_size),
      [cb](typename MsgT::ConstSharedPtr msg) { cb(msg); });
    return Subscriber(sub);
  }

  template<typename ObjT>
  Timer createTimer(const Duration & duration, void (ObjT::* cb)(const TimerEvent &), ObjT * obj)
  {
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(duration.toSec()));
    auto timer = node_->create_wall_timer(period, [obj, cb]() {
      TimerEvent ev;
      (obj->*cb)(ev);
    });
    return Timer(timer);
  }

  template<typename T>
  bool getParam(const std::string & raw_name, T & value) const
  {
    const auto name = resolve_parameter(raw_name);
    if (!node_->has_parameter(name)) {
      return false;
    }
    try {
      const rclcpp::Parameter p = node_->get_parameter(name);
      if constexpr (std::is_same_v<T, bool>) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
          value = p.as_bool();
          return true;
        }
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          value = (p.as_int() != 0);
          return true;
        }
      } else if constexpr (std::is_integral_v<T>) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          value = static_cast<T>(p.as_int());
          return true;
        }
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          value = static_cast<T>(p.as_double());
          return true;
        }
      } else if constexpr (std::is_floating_point_v<T>) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          value = static_cast<T>(p.as_double());
          return true;
        }
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          value = static_cast<T>(p.as_int());
          return true;
        }
      } else if constexpr (std::is_same_v<T, std::string>) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          value = p.as_string();
          return true;
        }
      } else {
        // Fallback for a ROS2-native parameter type not used by the migrated stack.
        return node_->get_parameter(name, value);
      }
    } catch (const std::exception &) {
      return false;
    }
    return false;
  }

  template<typename T>
  void param(const std::string & raw_name, T & value, const T & default_value) const
  {
    const auto name = resolve_parameter(raw_name);
    if (!node_->has_parameter(name)) {
      if constexpr (std::is_same_v<T, bool>) {
        node_->declare_parameter(name, static_cast<bool>(default_value));
      } else if constexpr (std::is_integral_v<T>) {
        node_->declare_parameter(name, static_cast<int64_t>(default_value));
      } else if constexpr (std::is_floating_point_v<T>) {
        node_->declare_parameter(name, static_cast<double>(default_value));
      } else if constexpr (std::is_same_v<T, std::string>) {
        node_->declare_parameter(name, default_value);
      } else {
        node_->declare_parameter<T>(name, default_value);
      }
    }
    if (!getParam(raw_name, value)) value = default_value;
  }

  rclcpp::Node::SharedPtr get_node() const { return node_; }

private:
  static rclcpp::QoS subscription_qos(size_t queue_size)
  {
    // Best-effort subscriptions remain compatible with both ROS2 sensor-data
    // publishers (camera/LiDAR are commonly best-effort) and reliable local
    // publishers. This avoids a common ROS1->ROS2 "topic exists but no data"
    // failure during the first migration pass.
    rclcpp::QoS qos(rclcpp::KeepLast(queue_size == 0 ? 1 : queue_size));
    qos.best_effort().durability_volatile();
    return qos;
  }

  std::string resolve_parameter(const std::string & raw_name) const
  {
    // ROS1 code in this workspace frequently uses absolute private parameter
    // names such as /waypoint_follower_node/is_parking_dist.  In ROS2 the
    // parameter belongs to the node and is named simply is_parking_dist.
    std::string name = raw_name;
    const std::string node_prefix = std::string("/") + node_->get_name() + "/";
    if (name.rfind(node_prefix, 0) == 0) {
      name.erase(0, node_prefix.size());
    }
    return normalize_parameter_name(name);
  }

  std::string resolve(const std::string & topic) const
  {
    if (topic.empty() || topic.front() == '/') return topic;
    if (private_) {
      return std::string("/") + node_->get_name() + "/" + topic;
    }
    if (!ns_.empty()) {
      std::string prefix = ns_;
      if (prefix.front() != '/') prefix = "/" + prefix;
      if (prefix.back() != '/') prefix += "/";
      return prefix + topic;
    }
    return topic;
  }

  rclcpp::Node::SharedPtr node_;
  bool private_;
  std::string ns_;
};

namespace param {
template<typename T>
inline bool get(const std::string & name, T & value)
{
  NodeHandle nh;
  return nh.getParam(name, value);
}
}  // namespace param

}  // namespace ros

#define ROS_INFO(...) RCLCPP_INFO(ros::node()->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(ros::node()->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(ros::node()->get_logger(), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(ros::node()->get_logger(), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(ros::node()->get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(ros::node()->get_logger(), x)
#define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(ros::node()->get_logger(), x)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(ros::node()->get_logger(), x)
#define ROS_DEBUG_STREAM(x) RCLCPP_DEBUG_STREAM(ros::node()->get_logger(), x)
#define ROS_INFO_THROTTLE(period, ...) \
  RCLCPP_INFO_THROTTLE(ros::node()->get_logger(), *ros::node()->get_clock(), static_cast<int64_t>((period) * 1000.0), __VA_ARGS__)
