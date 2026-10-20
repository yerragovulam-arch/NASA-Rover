// Minimal, deterministic stand-in for rclcpp, just enough to compile and run
// the *unmodified* Psyche node sources without ROS 2 installed.
//
// What it provides:
//   * a simulated clock (rclcpp::World::advance) -- no wall time anywhere
//   * a message bus: publish() queues, World::spin() delivers FIFO
//   * wall timers fired in order by World::advance()
//   * parameters: defaults, programmatic overrides, or a params.yaml file
//   * logging captured per level (so tests can assert on warnings)
//   * a log of every message published on every topic
//
// This is NOT a ROS replacement: no QoS, no executors, no threads, no services.
#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <vector>

namespace rclcpp
{

struct Duration
{
  int64_t ns{0};
  double seconds() const {return static_cast<double>(ns) * 1e-9;}
};

struct Time
{
  int64_t ns{0};
  double seconds() const {return static_cast<double>(ns) * 1e-9;}
  Duration operator-(const Time & o) const {return Duration{ns - o.ns};}
  Time operator+(const Duration & d) const {return Time{ns + d.ns};}
};

struct Logger {};
struct QoS {QoS(int = 10) {}};
struct SensorDataQoS : QoS {};

class Clock
{
public:
  Time now() const;
};

struct LogLine {int level; std::string text; double t;};
struct PVal
{
  enum Kind {NUM, BOOL, STR, VEC} kind{NUM};
  double d{0};
  bool b{false};
  std::string s;
  std::vector<double> v;
};

class World
{
public:
  struct Sub
  {
    std::string topic;
    std::type_index type{typeid(void)};
    std::function<void(const std::shared_ptr<void> &)> cb;
    bool alive{true};
    const void * owner{nullptr};
  };
  struct Timer
  {
    int64_t period{0}, next{0};
    std::function<void()> cb;
    bool alive{true};
    const void * owner{nullptr};
  };
  struct Pending {std::string topic; std::type_index type; std::shared_ptr<void> msg;};
  struct Logged {double t; std::type_index type; std::shared_ptr<void> msg;};

  int64_t now_ns{0};
  bool record{true};            // keep a log of every published message
  size_t record_cap{2000000};
  std::vector<std::shared_ptr<Sub>> subs;
  std::vector<std::shared_ptr<Timer>> timers;
  std::deque<Pending> queue;
  std::map<std::string, std::vector<Logged>> log;
  std::vector<LogLine> logs;
  size_t level_count[3]{0, 0, 0};
  size_t log_cap{200000};
  std::map<std::string, PVal> params;  // key "node/param"
  bool echo_logs{false};

  void reset()
  {
    now_ns = 0; subs.clear(); timers.clear(); queue.clear(); log.clear(); logs.clear();
    level_count[0] = level_count[1] = level_count[2] = 0;
    params.clear(); record = true;
  }

  // ---- delivery ----
  void spin()
  {
    int guard = 0;
    while (!queue.empty()) {
      if (++guard > 5000000) {throw std::runtime_error("message storm: spin() did not settle");}
      Pending p = std::move(queue.front());
      queue.pop_front();
      for (size_t i = 0; i < subs.size(); ++i) {
        auto s = subs[i];
        if (s->alive && s->topic == p.topic && s->type == p.type) {s->cb(p.msg);}
      }
    }
  }

  // Advance simulated time, firing wall timers in order and delivering messages.
  void advance(double seconds)
  {
    const int64_t target = now_ns + static_cast<int64_t>(std::llround(seconds * 1e9));
    advance_to_ns(target);
  }
  void advance_to_ns(int64_t target)
  {
    spin();
    for (;;) {
      std::shared_ptr<Timer> best;
      for (auto & t : timers) {
        if (t->alive && t->next <= target && (!best || t->next < best->next)) {best = t;}
      }
      if (!best) {break;}
      now_ns = std::max(now_ns, best->next);
      best->next += best->period;
      best->cb();
      spin();
    }
    now_ns = std::max(now_ns, target);
    spin();
  }

  // ---- publishing / inspection ----
  template<class M>
  void emit(const std::string & topic, const M & m)
  {
    auto sp = std::make_shared<M>(m);
    queue.push_back({topic, typeid(M), sp});
    if (record) {
      auto & v = log[topic];
      if (v.size() < record_cap) {v.push_back({now_ns * 1e-9, typeid(M), sp});}
    }
  }
  template<class M>
  void inject(const std::string & topic, const M & m) {emit(topic, m);}

  template<class M>
  std::vector<std::pair<double, M>> msgs(const std::string & topic) const
  {
    std::vector<std::pair<double, M>> out;
    auto it = log.find(topic);
    if (it == log.end()) {return out;}
    for (auto & l : it->second) {
      if (l.type == typeid(M)) {out.emplace_back(l.t, *std::static_pointer_cast<M>(l.msg));}
    }
    return out;
  }
  size_t count(const std::string & topic) const
  {
    auto it = log.find(topic);
    return it == log.end() ? 0 : it->second.size();
  }
  void clear_log() {log.clear(); logs.clear(); level_count[0] = level_count[1] = level_count[2] = 0;}
  size_t count_logs(int level, const std::string & contains = "") const
  {
    if (contains.empty() && level >= 0 && level < 3) {return level_count[level];}
    size_t n = 0;
    for (auto & l : logs) {
      if (l.level == level && (contains.empty() || l.text.find(contains) != std::string::npos)) {++n;}
    }
    return n;
  }

  // ---- parameters ----
  void set_param(const std::string & node, const std::string & name, double v)
  {
    PVal p; p.kind = PVal::NUM; p.d = v; params[node + "/" + name] = p;
  }
  void set_param(const std::string & node, const std::string & name, bool v)
  {
    PVal p; p.kind = PVal::BOOL; p.b = v; params[node + "/" + name] = p;
  }
  void set_param(const std::string & node, const std::string & name, const std::string & v)
  {
    PVal p; p.kind = PVal::STR; p.s = v; params[node + "/" + name] = p;
  }
  void set_param_vec(const std::string & node, const std::string & name, const std::vector<double> & v)
  {
    PVal p; p.kind = PVal::VEC; p.v = v; params[node + "/" + name] = p;
  }

  // Load a ROS 2 params.yaml (the simple flat style used in this workspace).
  void load_params_file(const std::string & path)
  {
    std::ifstream f(path);
    if (!f) {throw std::runtime_error("cannot open params file " + path);}
    std::string line, node, pending_key, pending_val;
    bool in_list = false;
    auto trim = [](std::string s) {
        size_t a = s.find_first_not_of(" \t\r");
        size_t b = s.find_last_not_of(" \t\r");
        return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
      };
    auto store = [&](const std::string & key, std::string val) {
        val = trim(val);
        PVal p;
        if (!val.empty() && val[0] == '[') {
          p.kind = PVal::VEC;
          std::string inner = val.substr(1, val.find(']') - 1);
          std::stringstream ss(inner);
          std::string tok;
          while (std::getline(ss, tok, ',')) {
            tok = trim(tok);
            if (!tok.empty()) {p.v.push_back(std::stod(tok));}
          }
        } else if (val == "true" || val == "false") {
          p.kind = PVal::BOOL; p.b = (val == "true");
        } else if (!val.empty() && (val[0] == '"' || val[0] == '\'')) {
          p.kind = PVal::STR; p.s = val.substr(1, val.size() - 2);
        } else {
          try {p.kind = PVal::NUM; p.d = std::stod(val);} catch (...) {p.kind = PVal::STR; p.s = val;}
        }
        params[node + "/" + key] = p;
      };
    while (std::getline(f, line)) {
      size_t hash = line.find('#');
      std::string body = trim(hash == std::string::npos ? line : line.substr(0, hash));
      if (body.empty()) {continue;}
      if (in_list) {
        pending_val += " " + body;
        if (body.find(']') != std::string::npos) {in_list = false; store(pending_key, pending_val);}
        continue;
      }
      const bool top = line[0] != ' ' && line[0] != '\t';
      size_t colon = body.find(':');
      if (colon == std::string::npos) {continue;}
      std::string key = trim(body.substr(0, colon)), val = trim(body.substr(colon + 1));
      if (top && val.empty()) {node = key; continue;}
      if (key == "ros__parameters") {continue;}
      if (!val.empty() && val[0] == '[' && val.find(']') == std::string::npos) {
        in_list = true; pending_key = key; pending_val = val; continue;
      }
      store(key, val);
    }
  }
};

inline World & world()
{
  static World w;
  return w;
}

inline Time Clock::now() const {return Time{world().now_ns};}

namespace detail
{
inline void log(int level, const char * f, ...) __attribute__((format(printf, 2, 3)));
inline void log(int level, const char * f, ...)
{
  char buf[1024];
  va_list a; va_start(a, f); vsnprintf(buf, sizeof buf, f, a); va_end(a);
  if (level >= 0 && level < 3) {world().level_count[level]++;}
  if (world().logs.size() < world().log_cap) {world().logs.push_back({level, buf, world().now_ns * 1e-9});}
  if (world().echo_logs) {std::fprintf(stderr, "[%d %.2f] %s\n", level, world().now_ns * 1e-9, buf);}
}
}  // namespace detail

// Log levels: 0 info, 1 warn, 2 error
#define RCLCPP_INFO(logger, ...) ::rclcpp::detail::log(0, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ::rclcpp::detail::log(1, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ::rclcpp::detail::log(2, __VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, ms, ...) ::rclcpp::detail::log(0, __VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) ::rclcpp::detail::log(1, __VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, ms, ...) ::rclcpp::detail::log(2, __VA_ARGS__)

template<class M>
class Publisher
{
public:
  using SharedPtr = std::shared_ptr<Publisher<M>>;
  explicit Publisher(std::string topic) : topic_(std::move(topic)) {}
  void publish(const M & m) {world().emit(topic_, m);}
  const std::string & topic() const {return topic_;}
private:
  std::string topic_;
};

template<class M>
class Subscription
{
public:
  using SharedPtr = std::shared_ptr<Subscription<M>>;
};

class TimerBase
{
public:
  using SharedPtr = std::shared_ptr<TimerBase>;
};

class Node
{
public:
  explicit Node(const std::string & name) : name_(name), clock_(std::make_shared<Clock>()) {}
  virtual ~Node()
  {
    for (auto & s : world().subs) {if (s->owner == this) {s->alive = false;}}
    for (auto & t : world().timers) {if (t->owner == this) {t->alive = false;}}
  }
  Node(const Node &) = delete;
  Node & operator=(const Node &) = delete;

  Logger get_logger() const {return Logger{};}
  std::shared_ptr<Clock> get_clock() const {return clock_;}
  Time now() const {return clock_->now();}
  const std::string & get_name() const {return name_;}

  template<class T>
  T declare_parameter(const std::string & name, const T & def)
  {
    auto it = world().params.find(name_ + "/" + name);
    if (it == world().params.end()) {return def;}
    const PVal & p = it->second;
    if constexpr (std::is_same_v<T, bool>) {
      return p.kind == PVal::BOOL ? p.b : (p.kind == PVal::NUM ? p.d != 0.0 : def);
    } else if constexpr (std::is_integral_v<T>) {
      return p.kind == PVal::NUM ? static_cast<T>(std::llround(p.d)) : def;
    } else if constexpr (std::is_floating_point_v<T>) {
      return p.kind == PVal::NUM ? static_cast<T>(p.d) : def;
    } else if constexpr (std::is_same_v<T, std::string>) {
      return p.kind == PVal::STR ? p.s : def;
    } else if constexpr (std::is_same_v<T, std::vector<double>>) {
      return p.kind == PVal::VEC ? p.v : def;
    } else {
      return def;
    }
  }

  template<class M, class Q>
  std::shared_ptr<Publisher<M>> create_publisher(const std::string & topic, const Q &)
  {
    return std::make_shared<Publisher<M>>(topic);
  }

  template<class M, class Q, class F>
  std::shared_ptr<Subscription<M>> create_subscription(const std::string & topic, const Q &, F cb)
  {
    auto s = std::make_shared<World::Sub>();
    s->topic = topic;
    s->type = typeid(M);
    s->owner = this;
    s->cb = [cb](const std::shared_ptr<void> & p) {cb(std::static_pointer_cast<M>(p));};
    world().subs.push_back(s);
    return std::make_shared<Subscription<M>>();
  }

  template<class Rep, class Per, class F>
  std::shared_ptr<TimerBase> create_wall_timer(std::chrono::duration<Rep, Per> period, F cb)
  {
    auto t = std::make_shared<World::Timer>();
    t->period = std::chrono::duration_cast<std::chrono::nanoseconds>(period).count();
    if (t->period <= 0) {throw std::runtime_error("timer period must be > 0");}
    t->next = world().now_ns + t->period;
    t->owner = this;
    t->cb = cb;
    world().timers.push_back(t);
    return std::make_shared<TimerBase>();
  }

private:
  std::string name_;
  std::shared_ptr<Clock> clock_;
};

// main() stubs (the node sources' main() functions are renamed, never run).
inline void init(int, char **) {}
inline void shutdown() {}
inline void spin(const std::shared_ptr<Node> &) {}

}  // namespace rclcpp
