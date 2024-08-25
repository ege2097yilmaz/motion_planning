/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file behavior_planner_FSM.cpp
 **/

#include "behavior_planner_FSM.h"
#include "velocity_profile_generator.h"

State BehaviorPlannerFSM::get_closest_waypoint_goal(
    const State& ego_state, const SharedPtr<cc::Map>& map,
    const float& lookahead_distance, bool& is_goal_junction) {
  // Nearest waypoint on the center of a Driving Lane.
  auto waypoint_0 = map->GetWaypoint(ego_state.location);

  if (_active_maneuver == DECEL_TO_STOP || _active_maneuver == STOPPED) {
    State waypoint;
    auto wp_transform = waypoint_0->GetTransform();
    waypoint.location = wp_transform.location;
    waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
    waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
    waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
    return waypoint;
  }

  // Waypoints at a lookahead distance
  // NOTE: "GetNext(d)" creates a list of waypoints at an approximate distance
  // "d" in the direction of the lane. The list contains one waypoint for each
  // deviation possible.

  // NOTE 2: GetNextUntilLaneEnd(d) returns a list of waypoints a distance "d"
  // apart. The list goes from the current waypoint to the end of its
  // lane.

  auto lookahead_waypoints = waypoint_0->GetNext(lookahead_distance);
  auto n_wp = lookahead_waypoints.size();
  if (n_wp == 0) {
    // LOG(INFO) << "Goal wp is a nullptr";
    State waypoint;
    return waypoint;
  }
  // LOG(INFO) << "BP - Num of Lookahead waypoints: " << n_wp;

  waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];

  is_goal_junction = waypoint_0->IsJunction();
  // LOG(INFO) << "BP - Is Last wp in junction? (0/1): " << is_goal_junction;
  auto cur_junction_id = waypoint_0->GetJunctionId();
  if (is_goal_junction) {
    if (cur_junction_id == _prev_junction_id) {
      // LOG(INFO) << "BP - Last wp is in same junction as ego. Junction ID: "
      //          << _prev_junction_id;
      is_goal_junction = false;
    } else {
      // LOG(INFO) << "BP - Last wp is in different junction than ego. Junction
      // ID: "
      //          << cur_junction_id;
      _prev_junction_id = cur_junction_id;
    }
  }
  State waypoint;
  auto wp_transform = waypoint_0->GetTransform();
  waypoint.location = wp_transform.location;
  waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
  waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
  waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
  return waypoint;
}

double BehaviorPlannerFSM::get_look_ahead_distance(const State& ego_state) {
  auto velocity_mag = utils::magnitude(ego_state.velocity);
  auto accel_mag = utils::magnitude(ego_state.acceleration);

  // TODO-Lookahead: One way to find a reasonable lookahead distance is to find
  // the distance you will need to come to a stop while traveling at speed V and
  // using a comfortable deceleration.
  // NOTE: this discards the current acceleration and assumes we can reach _max_accel in 0 time (== infinite jerk), 
  // so in case that was positive then this underestimates the needed time/distance.
  auto look_ahead_distance = VelocityProfileGenerator::calc_distance(velocity_mag, 0.0, -_max_accel);  // <- Fix This

  // LOG(INFO) << "Calculated look_ahead_distance: " << look_ahead_distance;

  look_ahead_distance =
      std::min(std::max(look_ahead_distance, _lookahead_distance_min),
               _lookahead_distance_max);

  // LOG(INFO) << "Final look_ahead_distance: " << look_ahead_distance;

  return look_ahead_distance;
}

State BehaviorPlannerFSM::get_goal(const State& ego_state,
                                   SharedPtr<cc::Map> map) {
  // Get look-ahead distance based on Ego speed

  auto look_ahead_distance = get_look_ahead_distance(ego_state);

  // Nearest waypoint on the center of a Driving Lane.
  bool is_goal_in_junction{false};
  auto goal_wp = get_closest_waypoint_goal(ego_state, map, look_ahead_distance,
                                           is_goal_in_junction);

  // LOG(INFO) << "Is the FINAL goal on a junction: " << is_goal_in_junction;
  string tl_state = "none";
  State goal =
      state_transition(ego_state, goal_wp, is_goal_in_junction, tl_state);

  return goal;
}

double BehaviorPlannerFSM::get_speed_for_goal_velocity(State goal) {
  double requested_goal_speed = std::sqrt(std::pow(goal.velocity.x, 2) + std::pow(goal.velocity.y, 2));
  double new_speed = std::min(requested_goal_speed, _speed_limit);
  return new_speed;
}

State BehaviorPlannerFSM::state_transition(const State& ego_state, State goal,
                                           bool& is_goal_in_junction,
                                           string tl_state) {

   // Reset acceleration components to zero
  goal.acceleration.x = 0.0;
  goal.acceleration.y = 0.0;
  goal.acceleration.z = 0.0;

  if (_active_maneuver == FOLLOW_LANE) {

    if (is_goal_in_junction) {

      _active_maneuver = DECEL_TO_STOP;

      // Adjust the goal position to a "buffer" distance behind the stop line.
      auto reverse_yaw = goal.rotation.yaw + M_PI;
      goal.location.x += _stop_line_buffer * std::cos(reverse_yaw);
      goal.location.y += _stop_line_buffer * std::sin(reverse_yaw);


      // Set goal velocity to zero at the stop line.
      goal.velocity.x = 0.0;
      goal.velocity.y = 0.0;

    } else {
      // In the nominal state, set the goal velocity to the speed limit.
      double new_speed = _speed_limit;
      goal.velocity.x = new_speed * std::cos(goal.rotation.yaw);
      goal.velocity.y = new_speed * std::sin(goal.rotation.yaw);
    }

  } else if (_active_maneuver == DECEL_TO_STOP) {
    LOG(INFO) << "BP- IN DECEL_TO_STOP STATE";

    // Maintain the previous goal to keep the vehicle at the stop line.
    goal = _goal;

    // Calculate the distance to the stop line.
    double distance_to_stop_line =
        utils::magnitude(goal.location - ego_state.location);
    LOG(INFO) << "Ego distance to stop line: " << distance_to_stop_line;

    // Check if the distance to the stop line is within the threshold.
    if (distance_to_stop_line <= P_STOP_THRESHOLD_DISTANCE) {
      // Switch to STOPPED state when the vehicle is near or at the stop line.
      _active_maneuver = STOPPED;
      _start_stop_time = std::chrono::high_resolution_clock::now();
    }

  } else if (_active_maneuver == STOPPED) {

    long long stopped_duration =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::high_resolution_clock::now() - _start_stop_time)
            .count();
    // LOG(INFO) << "BP- Stopped for " << stopped_duration << " secs";

    if (stopped_duration >= _req_stop_time && tl_state != "Red") {
      // Transition to FOLLOW_LANE state after the required stop time has passed and the light is not red.
      _active_maneuver = FOLLOW_LANE;

    } else {
      // Maintain the previous goal to keep the vehicle at the stop line.
      goal = _goal;
    }
  }

  _goal = goal;
  return goal;
}
