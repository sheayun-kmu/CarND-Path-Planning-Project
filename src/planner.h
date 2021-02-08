#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "spline.h"

using std::vector;

struct Config {
public:
  static const constexpr int number_of_lanes = 3;
  static const constexpr double lane_width = 4.0;
  static const constexpr double car_width = 3.0;
  static const constexpr int wp_to_keep = 50;
  static const constexpr double dt = 1.0 / (double) wp_to_keep;
  static const constexpr double mps_to_mph = 3600.0 / 1000.0 / 1.609;
  static const constexpr double max_speed = 49.5;
  static const constexpr double front_time_gap = 1.0;
  static const constexpr double max_acceleration = 5.0;
  static const constexpr double max_deceleration = 9.0;
  static const constexpr double lane_change_time = 0.5;
  static const constexpr double lane_change_safety_margin = 10.0;
  static const constexpr double lookahead_dist = 50.0;
};

struct Agent {
public:
  int lane = -1;
  double s = 9999.0;
  double v = 2.0 * Config::max_speed / Config::mps_to_mph;
};

class Behaviour {
public:
  enum behaviour {
    keep_lane,
    lane_change_left,
    lane_change_right,
  };
private:
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  behaviour current_behaviour;
  int current_lane;
  int target_lane;
  double s;
  double predicted_s;
  double d;
  double predicted_d;
  double v;
  double target_vel;
  // vector<vector<double>> sensor_fusion;
  vector<vector<Agent>> agents_by_lane;
  int determined_timestep;
public:
  Behaviour(
    int lane,
    vector<double>& map_s, vector<double>& map_x, vector<double>& map_y
  );
  ~Behaviour();
  void set_ego_status(double ego_s, double ego_d, double ego_speed,
                      double ego_target_vel,
                      double ego_pred_s, double ego_pred_d,
                      vector<vector<double>>& sf, int timestep);
  void determine_next(void);
  vector<vector<double>> get_waypoints(void);
  inline behaviour get_current_behaviour(void) { return current_behaviour; }
  inline int get_current_lane(void) { return current_lane; }
  inline int get_target_lane(void) { return target_lane; }
  Agent check_ahead(int lane);
  Agent check_behind(int lane);
  bool can_change_lane(int lane);
  double lane_speed(int lane);
};

class Path {
private:
  int prev_size;
  double ref_x, ref_y, ref_prev_x, ref_prev_y;
  double ref_yaw;
  double ref_vel;
  vector<double> path_x;
  vector<double> path_y;
  tk::spline sp;
public:
  Path(vector<double>& prev_x, vector<double>& prev_y,
       vector<vector<double>>& wp,
       double car_x, double car_y, double car_yaw);
  ~Path();
  double motion_plan(double ego_pred_s, Agent agent_ahead,
                     double ref_vel, double target_vel);
  vector<double> get_path_x(void) { return path_x; }
  vector<double> get_path_y(void) { return path_y; }
};

Agent check_behind(vector<vector<double>>& sensor_fusion,
                   int lane, double car_s, int timestep);

#endif  // PLANNER_H
