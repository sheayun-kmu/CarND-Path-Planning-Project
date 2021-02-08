#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "spline.h"

using std::vector;

struct Config {
public:
  static const constexpr int number_of_lanes = 3;
  static const constexpr double lane_width = 4.0;
  static const constexpr double dt = 1.0 / 50.0;
  static const constexpr double max_speed = 89.5;
  static const constexpr double mps_to_mph = 3600.0 / 1000.0 / 1.609;
  static const constexpr double dist_threshold_2 = 40.0;
  static const constexpr double dist_threshold_1 = 0.5 * dist_threshold_2;
  static const constexpr double dist_overtake = 1.5 * dist_threshold_2;
  static const constexpr double overtake_margin1 = 0.3 * dist_overtake;
  static const constexpr double overtake_margin2 = 0.1 * dist_overtake;
  static const constexpr double acceleration = 0.15 * mps_to_mph;
};

struct Agent {
public:
  int lane = -1;
  double s = 0.0;
  double v = 0.0;
};

class Behaviour {
private:
  enum behaviour {
    keep_lane,
    lane_change_left,
    lane_change_right,
  };
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  behaviour current_behaviour;
  int current_lane;
  int target_lane;
  double s;
  double predicted_s;
  double d;
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
                      double ego_target_vel, double ego_pred_s,
                      vector<vector<double>>& sf, int timestep);
  void determine_next(void);
  vector<vector<double>> get_waypoints(void);
  inline int get_target_lane(void) { return target_lane; }
  Agent check_ahead(int lane);
  Agent check_behind(int lane);
  bool lane_change_desirable(int lane, double check_s, double check_v);
};

class Path {
private:
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
  double motion_plan(double car_s, Agent agent_ahead,
                     double ref_vel, double target_vel);
  vector<double> get_path_x(void) { return path_x; }
  vector<double> get_path_y(void) { return path_y; }
};

Agent check_behind(vector<vector<double>>& sensor_fusion,
                   int lane, double car_s, int timestep);

#endif  // PLANNER_H
