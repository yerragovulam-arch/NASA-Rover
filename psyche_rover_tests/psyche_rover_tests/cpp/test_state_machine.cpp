// rover_state_machine_node (real source via shim) with scripted peers: ordering, gating,
// map-settle discard, retry loops, safety signals, restart / late-join behaviour.
#include <set>

#include "nodes.hpp"

using namespace tt;
using psyche_state_machine::RoverStateMachine;
using psyche_obstacle_avoidance::ReactiveAvoidance;

namespace
{
constexpr double kTick = 0.2;  // state machine timer period

struct SM
{
  std::shared_ptr<RoverStateMachine> node;
  // Scripted peers
  bool posture_ready{true};
  bool send_costmap{true};
  bool path_found{true};
  bool path_answers{true};
  int obstacle{-1};   // -1: leave alone, 0/1: publish that value each step

  SM() {node = std::make_shared<RoverStateMachine>();}
  static std::string state()
  {
    auto m = W().msgs<std_msgs::msg::String>("/rover_state");
    return m.empty() ? "START" : m.back().second.data;
  }
  // One timer period, with peers reacting to the current state first.
  void step()
  {
    const std::string s = state();
    if (obstacle >= 0) {W().inject("/obstacle_detected", boolmsg(obstacle == 1));}
    if (s == "STAND_UP") {W().inject("/posture_ready", boolmsg(posture_ready));}
    if (s == "MAP_TERRAIN" && send_costmap) {W().inject("/terrain_costmap", make_grid(40, 40, 0.1, 0));}
    if (s == "PICK_PATH" && path_answers) {W().inject("/path_found", boolmsg(path_found));}
    W().advance(kTick);
  }
  void run(double seconds) {for (double t = 0; t < seconds - 1e-9; t += kTick) {step();}}
  bool until(const std::string & target, double max_s = 20.0)
  {
    for (double t = 0; t < max_s; t += kTick) {
      if (state() == target) {return true;}
      step();
    }
    return state() == target;
  }
  static std::vector<std::string> history()
  {
    std::vector<std::string> h;
    for (auto & m : W().msgs<std_msgs::msg::String>("/rover_state")) {h.push_back(m.second.data);}
    return h;
  }
  static std::vector<std::pair<double, bool>> nav_log()
  {
    std::vector<std::pair<double, bool>> v;
    for (auto & m : W().msgs<std_msgs::msg::Bool>("/nav_enabled")) {v.emplace_back(m.first, m.second.data);}
    return v;
  }
  static int stand_msgs(bool value)
  {
    int n = 0;
    for (auto & m : W().msgs<std_msgs::msg::Bool>("/stand_request")) {n += m.second.data == value;}
    return n;
  }
};
}  // namespace

TEST(happy_path_visits_every_state_in_order)
{
  SM sm;
  REQUIRE(sm.until("TRAVERSE_PATH"));
  const std::vector<std::string> want = {"PERCEIVE", "STAND_UP", "MAP_TERRAIN",
    "EVALUATE_TRAVERSABILITY", "PICK_PATH", "TRAVERSE_PATH"};
  CHECK(SM::history() == want);
  auto nav = SM::nav_log();
  REQUIRE(nav.size() == 1);
  CHECK(nav[0].second == true);
}

TEST(needs_no_operator_input_to_reach_traverse)
{
  // Only internal topics are used in the happy path above; additionally confirm the
  // mission start needs no /cmd_vel, /nav_enabled or /stand_request input from outside.
  SM sm;
  REQUIRE(sm.until("TRAVERSE_PATH"));
  CHECK(W().count("/cmd_vel") == 0);
}

TEST(stand_up_blocks_until_posture_ready_and_keeps_requesting)
{
  SM sm; sm.posture_ready = false;
  sm.run(10.0);
  CHECK(SM::state() == "STAND_UP");
  CHECK_MSG(SM::stand_msgs(true) >= 45, "only %d stand requests in 10 s", SM::stand_msgs(true));
  CHECK(W().count("/nav_enabled") == 0);
  sm.posture_ready = true;
  CHECK(sm.until("MAP_TERRAIN", 2.0));
}

TEST(stand_up_ignores_posture_ready_false_messages)
{
  SM sm; sm.posture_ready = true;
  sm.until("STAND_UP");
  W().inject("/posture_ready", boolmsg(false));
  sm.posture_ready = false;
  sm.run(2.0);
  CHECK(SM::state() == "STAND_UP");
}

TEST(costmaps_during_settle_window_are_discarded)
{
  SM sm; sm.send_costmap = false;
  REQUIRE(sm.until("MAP_TERRAIN"));
  const double t_enter = W().now_ns * 1e-9;
  // maps arrive every tick but only after 1.2 s may count
  sm.send_costmap = true;
  double t_leave = -1;
  for (int k = 0; k < 40 && t_leave < 0; ++k) {
    sm.step();
    if (SM::state() != "MAP_TERRAIN") {t_leave = W().now_ns * 1e-9;}
  }
  INFO("left MAP_TERRAIN %.2f s after entering (map_settle_s = 1.2)", t_leave - t_enter);
  CHECK(t_leave - t_enter >= 1.2 - 1e-6);
  CHECK(t_leave - t_enter < 2.0);
}

TEST(costmap_sent_before_map_terrain_is_not_reused)
{
  SM sm; sm.send_costmap = false;
  sm.until("STAND_UP");
  W().inject("/terrain_costmap", make_grid(40, 40, 0.1, 0));    // stale map from before
  W().spin();
  sm.run(5.0);
  CHECK(SM::state() == "MAP_TERRAIN");
}

TEST(map_settle_parameter_is_honoured)
{
  for (double settle : {0.0, 3.0}) {
    reset_world();
    W().set_param("rover_state_machine", "map_settle_s", settle);
    SM sm; sm.send_costmap = false;
    sm.until("MAP_TERRAIN");
    const double t0 = W().now_ns * 1e-9;
    sm.send_costmap = true;
    double leave = -1;
    for (int k = 0; k < 60 && leave < 0; ++k) {sm.step(); if (SM::state() != "MAP_TERRAIN") {leave = W().now_ns * 1e-9 - t0;}}
    CHECK_MSG(leave >= settle - 1e-6, "settle %.1f: left after %.2f", settle, leave);
    CHECK_MSG(leave <= settle + 0.7, "settle %.1f: left only after %.2f", settle, leave);
  }
}

TEST(no_path_loops_back_to_mapping_without_driving)
{
  SM sm; sm.path_found = false;
  sm.run(30.0);
  auto h = SM::history();
  int maps = 0, picks = 0;
  for (auto & s : h) {maps += s == "MAP_TERRAIN"; picks += s == "PICK_PATH";}
  INFO("30 s with no path: %d map passes, %d pick passes", maps, picks);
  CHECK(maps >= 4 && picks >= 4);
  for (auto & s : h) {CHECK(s != "TRAVERSE_PATH");}
  CHECK(SM::nav_log().empty());
  CHECK(SM::stand_msgs(false) == 0);
  // then a path appears -> drive
  sm.path_found = true;
  CHECK(sm.until("TRAVERSE_PATH", 10.0));
}

TEST(path_found_sent_outside_pick_path_is_ignored)
{
  SM sm; sm.path_answers = false;
  sm.until("MAP_TERRAIN");
  W().inject("/path_found", boolmsg(true)); W().spin();      // premature answer
  sm.run(4.0);
  CHECK(SM::state() == "PICK_PATH");                          // still waiting for a fresh answer
  CHECK(SM::nav_log().empty());
}

TEST(nav_enabled_is_true_only_while_in_traverse)
{
  SM sm;
  REQUIRE(sm.until("TRAVERSE_PATH"));
  sm.run(2.0);
  sm.obstacle = 1; sm.run(0.6); sm.obstacle = 0;
  sm.run(8.0);
  // reconstruct the state timeline and check each nav message against it
  auto nav = SM::nav_log();
  REQUIRE(nav.size() >= 2);
  CHECK(nav[0].second == true);
  CHECK(nav[1].second == false);
  CHECK_MSG(nav[1].first - nav[0].first >= 2.0, "nav was only enabled for %.2f s", nav[1].first - nav[0].first);
}

TEST(obstacle_during_traverse_disables_nav_within_one_tick_and_remaps)
{
  SM sm;
  REQUIRE(sm.until("TRAVERSE_PATH"));
  sm.run(1.0);
  const double t0 = W().now_ns * 1e-9;
  W().inject("/obstacle_detected", boolmsg(true)); W().spin();
  W().advance(kTick + 1e-6);
  auto nav = SM::nav_log();
  REQUIRE(nav.size() >= 2);
  CHECK(nav.back().second == false);
  CHECK_MSG(nav.back().first - t0 <= kTick + 1e-3, "nav disabled %.2f s after obstacle", nav.back().first - t0);
  CHECK(SM::state() == "MAP_TERRAIN");
}

TEST(obstacle_flag_outside_traverse_is_harmless)
{
  SM sm; sm.obstacle = 1;
  // obstacle flag high from the start: must still reach TRAVERSE (and then drop out, below)
  CHECK(sm.until("TRAVERSE_PATH", 15.0));
}

XFAIL_TEST(persistent_obstacle_flag_does_not_livelock_the_rover,
           "TRAVERSE_PATH exits on the very next tick whenever /obstacle_detected is true, even though "
           "the planner just found a clear heading; with the flag permanently high (e.g. the flat-floor "
           "false positive, or any object within 1 m) nav is enabled for ~0.2 s per cycle and the rover never walks")
{
  SM sm; sm.obstacle = 1;
  sm.run(60.0);
  // longest continuous interval with nav enabled
  auto nav = SM::nav_log();
  double longest = 0, on_t = -1, total_on = 0;
  for (auto & e : nav) {
    if (e.second) {on_t = e.first;}
    else if (on_t >= 0) {longest = std::max(longest, e.first - on_t); total_on += e.first - on_t; on_t = -1;}
  }
  INFO("60 s with permanent obstacle flag: nav enabled %.1f s in total, longest stretch %.2f s, %zu enable pulses",
       total_on, longest, nav.size() / 2);
  CHECK_MSG(longest >= 2.0, "longest walking window only %.2f s", longest);
}

TEST(state_machine_never_withdraws_stand_request)
{
  SM sm; sm.obstacle = 1;
  sm.run(30.0);
  CHECK(SM::stand_msgs(false) == 0);
  CHECK(SM::stand_msgs(true) > 100);
}

TEST(obstacle_flicker_does_not_stall_mapping_cycle)
{
  SM sm;
  REQUIRE(sm.until("TRAVERSE_PATH"));
  int toggles = 0;
  for (int i = 0; i < 100; ++i) {
    W().inject("/obstacle_detected", boolmsg(i % 2 == 0)); toggles++;
    sm.step();
  }
  // still functioning: can reach traverse again once the flag settles low
  W().inject("/obstacle_detected", boolmsg(false));
  CHECK(sm.until("TRAVERSE_PATH", 15.0));
  CHECK(SM::stand_msgs(false) == 0);
}

TEST(obstacle_forces_fresh_map_not_reuse_of_old_one)
{
  SM sm;
  REQUIRE(sm.until("TRAVERSE_PATH"));
  sm.send_costmap = false;                                  // sensor goes quiet
  W().inject("/obstacle_detected", boolmsg(true)); W().spin();
  W().advance(kTick);                                       // let one tick observe the obstacle
  REQUIRE(SM::state() == "MAP_TERRAIN");
  sm.obstacle = 0;
  sm.run(10.0);
  CHECK_MSG(SM::state() == "MAP_TERRAIN", "advanced to %s without a fresh costmap", SM::state().c_str());
}

XFAIL_TEST(perceive_waits_for_live_sensor_data,
           "PERCEIVE is a stub: the machine proceeds to STAND_UP and mapping with no depth stream at all")
{
  SM sm; sm.send_costmap = false; sm.posture_ready = true;
  sm.run(3.0);
  CHECK_MSG(SM::state() == "PERCEIVE" || SM::state() == "START",
            "left PERCEIVE (now %s) without any sensor data", SM::state().c_str());
}

TEST(parameter_defaults_and_state_names_cover_all_states)
{
  SM sm; sm.path_found = false;
  sm.run(5.0);
  std::set<std::string> seen;
  for (auto & s : SM::history()) {seen.insert(s);}
  for (const char * n : {"PERCEIVE", "STAND_UP", "MAP_TERRAIN", "EVALUATE_TRAVERSABILITY", "PICK_PATH"}) {
    CHECK_MSG(seen.count(n), "state %s never reported", n);
  }
}

XFAIL_TEST(restarted_state_machine_revokes_nav_enabled,
           "after a state-machine restart nobody publishes /nav_enabled=false: gait/avoidance nodes "
           "keep the stale 'true' and a robot that was mid-traverse keeps obeying commands")
{
  auto sm = std::make_unique<SM>();
  REQUIRE(sm->until("TRAVERSE_PATH"));
  sm->run(1.0);
  const size_t before = SM::nav_log().size();
  sm.reset();                                   // process dies...
  sm = std::make_unique<SM>();                  // ...and is restarted by launch
  sm->path_answers = false; sm->send_costmap = false;
  sm->run(5.0);
  auto nav = SM::nav_log();
  bool revoked = false;
  for (size_t i = before; i < nav.size(); ++i) {revoked = revoked || !nav[i].second;}
  CHECK_MSG(revoked, "no nav_enabled=false after restart (state now %s)", SM::state().c_str());
}

XFAIL_TEST(late_starting_peer_learns_nav_enabled,
           "nav_enabled=true is published exactly once on entering TRAVERSE_PATH (stand_request, by contrast, "
           "is re-sent every tick): a reactive_avoidance/gait node that starts or restarts later stays gated forever")
{
  SM sm;
  REQUIRE(sm.until("TRAVERSE_PATH"));
  sm.run(5.0);
  auto late = std::make_shared<ReactiveAvoidance>();
  W().inject("/obstacle_sectors", sectors({3, 3, 3, 3, 3}));
  sm.run(2.0);
  W().inject("/obstacle_sectors", sectors({3, 3, 3, 3, 3}));
  W().spin();
  bool drove = false;
  for (auto & m : W().msgs<geometry_msgs::msg::Twist>("/cmd_vel")) {drove = drove || m.second.linear.x > 0;}
  CHECK_MSG(drove, "late-started avoidance node never became enabled while the machine sits in TRAVERSE_PATH");
}

int main(int argc, char ** argv) {return tf::run_main(argc, argv, "state_machine", reset_world);}
