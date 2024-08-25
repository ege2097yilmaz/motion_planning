/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>


#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"

#include <iostream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>

#include <unistd.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    } 
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Declare and initialize the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;
  
  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  	ego_state.velocity.x = v_points[v_points.size()-1];	
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;
  	
  }
  
  /*std::cout << "Original goal (x, y): (" << goal.location.x << ", " << goal.location.y << "), "
    		<< "(vx, vy): " << goal.velocity.x << ", " << goal.velocity.y << ") "
    		<< "is junction: " << is_junction << std::endl;*/
  
  //std::cout << "Existing trajectory (x, y, v): ";
  for (size_t i = 0; i < x_points.size(); ++i) {
    //std::cout << "(" << x_points[i] << ", " << y_points[i] << ", " << v_points[i] << "), ";
  }
  //std::cout << std::endl;

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);
  // NOTE: behavior can be changed by the state transition, so getting it was moved after the state transition
  Maneuver behavior = behavior_planner.get_active_maneuver();

//   std::cout << "Behavior: " << behavior << std::endl
//     << " ego p: (" << ego_state.location.x << ", " << ego_state.location.y << "),"
//     << "), ego v: (" << ego_state.velocity.x << ", " << ego_state.velocity.y << ")" << std::endl
//     << " goal p: (" << goal.location.x << ", " << goal.location.y 
//     << "), goal v: (" << goal.velocity.x << ", " << goal.velocity.y << ")" << std::endl;
  
  if(behavior == STOPPED){

  	size_t max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);
  	}
    //std::cout << "Stopped at: " << point_x << ", " << point_y << std::endl;
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0) {
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for(size_t i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(size_t j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);  
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);
  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];
  
  size_t index = 0;
  size_t max_points = 20;
  size_t add_points = spirals_x[best_spiral_idx].size();
  // NOTE: even though a full trajectory is calculated (e.g. decelerate to stop), it is probably not fully
  // passed to the simulator. At each time instance we already receive some points from the simulator that were
  // previously calculated, but not yet actuated, so only aronud 5 points are added at the end of that trajectory
  // from the current calculation. This delayed actuation causes some problems during state changes,
  // for example even though a smooth deceleration to stop is calculated, the final part of that is not yet
  // added to this list when the state transition to stopped state already happens, causing a 'sudden stop'.
  //std::cout << "Best spiral (x, y, v): ";
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    
    //std::cout << "(" << point_x << ", " << point_y << ", " << velocity << "), ";
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }
  
  //std::cout << std::endl;


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( size_t i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);  
	}
	obst_flag = true;
}

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t /*length*/, uWS::OpCode /*opCode*/)
  {
    
    cout << endl << "Message received" << endl;
        auto s = hasData(data);

        if (s != "") {
          auto data = json::parse(s);
          
          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];
          
          // std::cout << "SIM TIME: " << sim_time << std::endl;

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);

          }

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;

          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

          json msgJson;
          msgJson["throttle"] = 0.25;
          msgJson["steer"] = 0.0;
          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          // min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4 
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();
          
          // std::cout << msg << std::endl;
  
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); 
      
    }

  });
  
  
  h.onConnection([](uWS::WebSocket<uWS::SERVER> /*ws*/, uWS::HttpRequest /*req*/) 
  {
      cout << "Connected!!!" << endl;
    });

  
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int /*code*/, char */*message*/, size_t /*length*/) 
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

    int port = 4567;
    if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    } 
    else 
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }
    

}
