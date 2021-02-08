#include <vector>
#include <algorithm>
#include <iterator>
#include "planner.h"
#include "helpers.h"

using std::vector;

Behaviour::Behaviour(int lane,
                     vector<double>& map_s,
                     vector<double>& map_x,
                     vector<double>& map_y) {
  current_lane = lane;
  target_lane = lane;
  current_behaviour = keep_lane;
  std::copy(map_s.begin(), map_s.end(), std::back_inserter(map_waypoints_s));
  std::copy(map_x.begin(), map_x.end(), std::back_inserter(map_waypoints_x));
  std::copy(map_y.begin(), map_y.end(), std::back_inserter(map_waypoints_y));
  agents_by_lane.resize(3);
}

Behaviour::~Behaviour() {}

void Behaviour::set_ego_status(double ego_s, double ego_d, double ego_speed,
                               double ego_target_vel, double ego_pred_s,
                               vector<vector<double>>& sf, int timestep) {
  s = ego_s;
  d = ego_d;
  v = ego_speed;
  target_vel = ego_target_vel;
  determined_timestep = timestep;
  if (ego_pred_s > ego_s) {
    predicted_s = ego_pred_s;
  } else {
    predicted_s = ego_s;
  }
  for (int i = 0; i < 3; i++) {
    agents_by_lane[i].clear();
  }
  for (auto& st : sf) {
    Agent a;
    double agent_vx = st[3];
    double agent_vy = st[4];
    double agent_s = st[5];
    double agent_d = st[6];
    a.lane = (int) (agent_d / Config::lane_width);
    if (a.lane >= 0 && a.lane < Config::number_of_lanes) {
      double agent_v = sqrt(agent_vx * agent_vx + agent_vy * agent_vy);
      // predict where the agent is going to be
      // (when we reach the end of the previously generated path)
      double pred_s = agent_s + agent_v * Config::dt * (double) timestep;
      a.s = pred_s;
      a.v = agent_v;
      agents_by_lane[a.lane].push_back(a);
    }
  }
}

void Behaviour::determine_next(void) {
  Agent agent_ahead;
  behaviour next_behaviour;

  switch (current_behaviour) {
    // if we were keeping the lane, we check whether we want to overtake
    case keep_lane:
      agent_ahead = check_ahead(current_lane);
      next_behaviour = current_behaviour;
      bool lane_change;
      if (agent_ahead.lane >= 0) { // an agent found in the same lane
        // a slower vehicle in front that closer than a threshold
        if (agent_ahead.v < target_vel &&
            agent_ahead.s - s < Config::dist_overtake) {
          bool change_planned = false;
          if (!change_planned && current_lane - 1 >= 0) {
            if (lane_change_desirable(
                  current_lane - 1, agent_ahead.s, agent_ahead.v
            )) {
              target_lane = current_lane - 1;
              change_planned = true;
            }
          }
          if (!change_planned && current_lane + 1 < Config::number_of_lanes) {
            if (lane_change_desirable(
                  current_lane + 1, agent_ahead.s, agent_ahead.v
            )) {
              target_lane = current_lane + 1;
              change_planned = true;
            }
          }
        }
      }
      if (target_lane < current_lane) {
        next_behaviour = lane_change_left;
      } else if (target_lane > current_lane) {
        next_behaviour = lane_change_right;
      }
      break;
    // if we were in the middle of changing lanes, we check whether
    // the lane change is complete
    case lane_change_left:
    case lane_change_right:
      current_lane = (int) (d / Config::lane_width);
      if (current_lane == target_lane) {
        next_behaviour = keep_lane;
      } else {
        next_behaviour = current_behaviour;
      }
      break;

    default:
      break;
  }
  current_behaviour = next_behaviour;
}

vector<vector<double>> Behaviour::get_waypoints(void) {
  vector<vector<double>> wp;
  double target_d = Config::lane_width * ((double) target_lane + 0.5);
  double middle_d = (d + target_d) / 2.0;
  vector<double> next_wp0 = getXY(
    predicted_s + 30, target_d,
    map_waypoints_s, map_waypoints_x, map_waypoints_y
  );
  vector<double> next_wp1 = getXY(
    predicted_s + 60, target_d,
    map_waypoints_s, map_waypoints_x, map_waypoints_y
  );
  vector<double> next_wp2 = getXY(
    predicted_s + 90, target_d,
    map_waypoints_s, map_waypoints_x, map_waypoints_y
  );
  wp.push_back(next_wp0);
  wp.push_back(next_wp1);
  wp.push_back(next_wp2);
  return wp;
}

Agent Behaviour::check_ahead(int lane) {
  Agent nearest;
  double min_dist = -1.0;
  for (auto& a : agents_by_lane[lane]) {
    // check whether the agent is ahead of us
    if (a.s <= predicted_s) continue;
    // get the longitudinal distance from ego
    double dist = a.s - predicted_s;
    if (min_dist < 0 || dist < min_dist) {
      min_dist = dist;
      nearest = a;
    }
  }
  return nearest;
}

Agent Behaviour::check_behind(int lane) {
  Agent nearest;
  double min_dist = -1.0;
  for (auto& a : agents_by_lane[lane]) {
    // check whether the agent is behind us
    if (a.s >= predicted_s) continue;
    // get the longitudinal distance from ego
    double dist = predicted_s - a.s;
    if (min_dist < 0 || dist < min_dist) {
      min_dist = dist;
      nearest = a;
    }
  }
  return nearest;
}

bool Behaviour::lane_change_desirable(int lane,
                                      double check_s, double check_v) {
  Agent agent_behind = check_behind(lane);
  bool desirable = false;
  if (agent_behind.lane < 0 ||
      predicted_s - agent_behind.s >= Config::overtake_margin1 ||
      (predicted_s - agent_behind.s >= Config::overtake_margin2 &&
       agent_behind.v < v)
  ) {
    Agent agent_ahead = check_ahead(lane);
    if (agent_ahead.lane < 0 ||
        (agent_ahead.s > check_s && agent_ahead.v > check_v)) {
      desirable = true;
    }
  }
  return desirable;
}

Path::Path(vector<double>& prev_x, vector<double>& prev_y,
           vector<vector<double>>& wp,
           double car_x, double car_y, double car_yaw) {
  path_x.clear();
  path_y.clear();
  int prev_size = prev_x.size();
  // put waypoints in the previously generated (bot not yet traveled) path
  // into the set of current path being generated
  for (int i = 0; i < prev_size; i++) {
    path_x.push_back(prev_x[i]);
    path_y.push_back(prev_y[i]);
  }
  // get the last two points to form the starting section of
  // the spline to fit, and record the reference yaw,
  // according to which the path is going to be generated
  if (prev_size >= 2) {
    ref_x = prev_x[prev_size - 1];
    ref_y = prev_y[prev_size - 1];
    ref_prev_x = prev_x[prev_size - 2];
    ref_prev_y = prev_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);
  } else {
    // assume a straight line when no two previous points are available
    ref_x = car_x;
    ref_y = car_y;
    ref_yaw = deg2rad(car_yaw);
    ref_prev_x = ref_x - cos(car_yaw);
    ref_prev_y = ref_y - sin(car_yaw);
  }
  vector<double> ptsx;
  vector<double> ptsy;
  ptsx.push_back(ref_prev_x);
  ptsx.push_back(ref_x);
  ptsy.push_back(ref_prev_y);
  ptsy.push_back(ref_y);
  // push the waypoints to the points used by the spline
  for (int i = 0; i < wp.size(); i++) {
    ptsx.push_back(wp[i][0]);
    ptsy.push_back(wp[i][1]);
  }
  // transform the coordinates so that the points are represented
  // as X & Y coordinates with regard to the car position
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = shift_x * cos(0.0 - ref_yaw) - shift_y * sin(0.0 - ref_yaw);
    ptsy[i] = shift_x * sin(0.0 - ref_yaw) + shift_y * cos(0.0 - ref_yaw);
  }
  // setup spline
  sp.set_points(ptsx, ptsy);
}

Path::~Path() {}

double Path::motion_plan(double car_s, Agent agent_ahead,
                         double ref_vel, double target_vel) {
  if (agent_ahead.lane >= 0) {
    double dist = agent_ahead.s - car_s;
    if (dist < Config::dist_threshold_1) {
      // if the car in front is very near, take its half speed
      target_vel = 0.5 * Config::mps_to_mph * agent_ahead.v;
    } else if (dist < Config::dist_threshold_2) {
      // if the car in front is very near, take its half speed
      target_vel = Config::mps_to_mph * agent_ahead.v;
    }
  }
  // adjust reference velocity - whether to speed up or down
  if (ref_vel > target_vel) {
    ref_vel -= Config::acceleration;
  } else if (ref_vel < target_vel) {
    ref_vel += Config::acceleration;
  }
  // set target point to 30m ahead (only x axis)
  double target_x = 30.0;
  double target_y = sp(target_x);
  double target_dist = distance(0.0, 0.0, target_x, target_y);
  double x_add_on = 0.0;

  for (int i = 1; i <= 50 - path_x.size(); i++) {
    double N = (target_dist / (Config::dt * ref_vel / Config::mps_to_mph));
    double x_point = x_add_on + (target_x) / N;
    double y_point = sp(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    path_x.push_back(x_point);
    path_y.push_back(y_point);
  }
  return ref_vel;
}
