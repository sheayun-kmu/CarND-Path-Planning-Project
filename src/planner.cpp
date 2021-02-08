#include <vector>
#include <algorithm>
#include <iterator>
#include "planner.h"
#include "helpers.h"



#include <iostream>

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
  double overtake_dist = 1.2 * v * Config::front_time_gap;
  switch (current_behaviour) {
    // if we were keeping the lane, we check whether we want to overtake
    case keep_lane:
      agent_ahead = check_ahead(current_lane);
      next_behaviour = keep_lane;
      if (agent_ahead.lane >= 0) { // an agent found in the same lane
        if (agent_ahead.v < target_vel &&
            agent_ahead.s - predicted_s < overtake_dist) {
          // check the speed we can get by changing lanes
          double left_speed = lane_speed(current_lane - 1);
          double right_speed = lane_speed(current_lane + 1);
          // if both lane change is possible, take the more beneficial one
          if (left_speed >= 0.0 && right_speed >= 0.0) {
            if (left_speed < right_speed) {
              target_lane = current_lane + 1;
            } else {
              target_lane = current_lane - 1;
            }
          } else if (left_speed >= 0.0) { // only left change possible
            target_lane = current_lane - 1;
          } else if (right_speed >= 0.0) { // only right change possible
            target_lane = current_lane + 1;
          }
        } // end if overtaking maneuver
      } // end if blocked by another vehicle
      if (target_lane == current_lane && current_lane != 1) {
        // we prefer to keep the center lane because of its flexibility
        int other_side_lane = 2 - current_lane;
        Agent target_ahead = check_ahead(1);
        if (can_change_lane(1)) { // can change to middle lane
          if (target_ahead.lane < 0 ||
              target_ahead.s - predicted_s > overtake_dist) {
            // middle lane clear
            target_lane = 1;
          } else {
            // other side lane clear - consecutive lane change planned
            double other_speed = lane_speed(other_side_lane);
            if (other_speed > v + 1.0) {
              target_lane = 1;
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
  double delta_s;
  if (current_behaviour == keep_lane || v < 1.0) {
    delta_s = 30.0;
  } else {
    delta_s = v * Config::lane_change_time;
    if (delta_s < 30.0) {
      delta_s = 30.0;
    }
  }
  double dist1 = 1.0 * delta_s;
  double dist2 = 2.0 * delta_s;
  double dist3 = 3.0 * delta_s;
  vector<double> next_wp0 = getXY(
    predicted_s + dist1, target_d,
    map_waypoints_s, map_waypoints_x, map_waypoints_y
  );
  vector<double> next_wp1 = getXY(
    predicted_s + dist2, target_d,
    map_waypoints_s, map_waypoints_x, map_waypoints_y
  );
  vector<double> next_wp2 = getXY(
    predicted_s + dist3, target_d,
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

bool Behaviour::can_change_lane(int lane) {
  bool behind_clear = false;
  bool ahead_clear = false;
  // check the vehicle behind ego, in the target lane
  double ego_pred_s = predicted_s
                    + v * Config::lane_change_time;
  Agent behind = check_behind(lane);
  Agent ahead = check_ahead(lane);
  if (behind.lane >= 0) { // if there's a car coming from behind
    double behind_pred_s = behind.s + behind.v * Config::lane_change_time;
    if (ego_pred_s - behind_pred_s > Config::lane_change_safety_margin) {
      behind_clear = true;
    }
  } else {
    behind_clear = true;
  }
  if (ahead.lane >= 0) { // if there's a car ahead
    double ahead_pred_s = ahead.s + ahead.v * Config::lane_change_time;
    if (ahead_pred_s - ego_pred_s > Config::lane_change_safety_margin) {
      ahead_clear = true;
    }
  } else {
    ahead_clear = true;
  }
  return behind_clear && ahead_clear;
}

double Behaviour::lane_speed(int lane) {
  // speed defined to be negative in case of an
  // impossible (or prohibitive) lane change
  double speed;
  if (lane < 0 || lane >= Config::number_of_lanes) {
    speed = -1.0;
  } else if (!can_change_lane(lane)) {
    speed = -1.0;
  } else {
    // define the speed as the maximum speed we can take in that lane
    Agent agt_ahead = check_ahead(lane);
    if (agt_ahead.lane < 0) {
      speed = Config::max_speed;
    } else {
      if (agt_ahead.s > predicted_s + Config::lookahead_dist) {
        speed = Config::max_speed;
      } else {
        speed = agt_ahead.v;
      }
    }
  }
  return speed;
}

Path::Path(vector<double>& prev_x, vector<double>& prev_y,
           vector<vector<double>>& wp,
           double car_x, double car_y, double car_yaw) {
  path_x.clear();
  path_y.clear();
  prev_size = prev_x.size();
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

double Path::motion_plan(double ego_pred_s, Agent agent_ahead,
                         double ref_vel, double target_vel) {
  // calculate the number of points to generate
  int pts = Config::wp_to_keep - prev_size;
  double desired_acc;
  // look at the vehicle ahead of ego in the same lane
  // (only the closest one in the same lane is included by the caller, if any)
  if (agent_ahead.lane >= 0) {
    // dubious meaning:
    // (1) time it takes to reach the front vehicle's position (at current v)
    // (2) time it takes to provide the desired distance to the front vehicle
    double dt = Config::front_time_gap;
    // calculate distances traveled by ego & agent, respectively
    // (in a given period of time)
    double ego_v = ref_vel / Config::mps_to_mph;
    double agt_v = agent_ahead.v;
    double ego_t = ego_pred_s + ego_v * dt;
    double agt_t = agent_ahead.s + agt_v * dt;
    // calculate target distance to keep (distance to travel in dt)
    double target_dist = dt * ego_v;
    // calculate the desired acceleration
    desired_acc = 2 * (agt_t - ego_t - target_dist) / (dt * dt);
  } else { // set the desired accleration to the maximum value
    desired_acc = Config::max_acceleration;
  }
  // trim acceleration value if it's too much
  if (desired_acc > Config::max_acceleration) {
    desired_acc = Config::max_acceleration;
  } else if (desired_acc < -1.0 * Config::max_deceleration) {
    desired_acc = -1.0 * Config::max_deceleration;
  }
  // if we are going to get over the target speed, we
  // reduce the acceleration to meet the target
  double required_acc = (target_vel - ref_vel) / Config::mps_to_mph;
  if (desired_acc * pts * Config::dt > required_acc) {
    desired_acc = required_acc / (pts * Config::dt);
  }
  // generate waypoints to follow
  double vel = ref_vel / Config::mps_to_mph;
  // set a target point (in X-Y coordinate)
  double t_x = 0.5 * desired_acc * (pts * Config::dt) * (pts * Config::dt);
  double t_y = sp(t_x);
  double t_d = distance(0.0, 0.0, t_x, t_y);
  double px = 0.0;
  double py;
  for (int i = 0; i < pts; i++) {
    vel += desired_acc * Config::dt;
    px += vel * Config::dt;
    py = sp(px);
    double x_point = ref_x + px * cos(ref_yaw) - py * sin(ref_yaw);
    double y_point = ref_y + px * sin(ref_yaw) + py * cos(ref_yaw);
    path_x.push_back(x_point);
    path_y.push_back(y_point);
  }
  return vel * Config::mps_to_mph;
}
