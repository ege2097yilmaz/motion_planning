/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file cost_functions.cpp
 **/

#include "cost_functions.h"

using namespace std;

namespace cost_functions {
// COST FUNCTIONS

double diff_cost(vector<double> coeff, double duration,
                 std::array<double, 3> goals, std::array<float, 3> sigma,
                 double cost_weight) {
  /*
  Penalizes trajectories whose coordinate(and derivatives)
  differ from the goal.
  */
  double cost = 0.0;
  vector<double> evals = evaluate_f_and_N_derivatives(coeff, duration, 2);
  //////////////cout << "26 - Evaluating f and N derivatives Done. Size:" <<
  /// evals.size() << endl;

  for (size_t i = 0; i < evals.size(); i++) {
    double diff = fabs(evals[i] - goals[i]);
    cost += logistic(diff / sigma[i]);
  }
  ////////////cout << "diff_coeff Cost Calculated " << endl;
  return cost_weight * cost;
}


double collision_circles_cost_spiral(const std::vector<PathPoint>& spiral,
                                const std::vector<State>& obstacles) {
  bool collision_detected{false};
  auto num_circles = CIRCLE_OFFSETS.size();

  for (const auto& waypoint : spiral) {
    if (collision_detected) {
      // LOG(INFO) << " ***** COLLISION DETECTED *********" << std::endl;
      break;
    }
    double current_x = waypoint.x;
    double current_y = waypoint.y;
    double current_yaw = waypoint.theta; //

    for (size_t i = 0; i < num_circles && !collision_detected; ++i) {
      // Determine the placement of the collision circles using the current 
      // position and orientation. Use sine and cosine functions to calculate
      // the x and y coordinates.
      double circle_center_x = current_x + CIRCLE_OFFSETS[i] * std::cos(current_yaw);  
      double circle_center_y = current_y + CIRCLE_OFFSETS[i] * std::sin(current_yaw);  

      for (const auto& obstacle : obstacles) {
        if (collision_detected) {
          break;
        }
        double obstacle_yaw = obstacle.rotation.yaw;
        for (size_t j = 0; j < num_circles && !collision_detected; ++j) {
          double obstacle_center_x = 
              obstacle.location.x + CIRCLE_OFFSETS[j] * std::cos(obstacle_yaw);
          double obstacle_center_y = 
              obstacle.location.y + CIRCLE_OFFSETS[j] * std::sin(obstacle_yaw);

          // Calculate the distance between the center of the collision circle 
          // and the obstacle, considering only the horizontal plane.
          double distance = utils::magnitude(carla::geom::Vector3D(circle_center_x - obstacle_center_x, 
                                                                   circle_center_y - obstacle_center_y, 
                                                                   0));

          collision_detected = (distance < (CIRCLE_RADII[i] + CIRCLE_RADII[j]));
        }
      }
    }
  }
  return (collision_detected) ? COLLISION : 0.0;
}

double close_to_main_goal_cost_spiral(const std::vector<PathPoint>& spiral,
                                      State main_goal) {
  // The final point on the spiral is used to assess how close it is to the main goal.
  // This helps to favor spirals that end closer to the lane centerline and are collision-free.
  auto last_index = spiral.size() - 1;

  // Calculate the distance between the last point on the spiral and the main goal location.
  double delta_x = main_goal.location.x - spiral[last_index].x;  // Difference in x coordinates
  double delta_y = main_goal.location.y - spiral[last_index].y;  // Difference in y coordinates
  double delta_z = main_goal.location.z - spiral[last_index].z;  // Difference in z coordinates

  double distance_to_goal = std::sqrt((delta_x * delta_x) + 
                                      (delta_y * delta_y) + 
                                      (delta_z * delta_z));

  double cost = logistic(distance_to_goal);
  
  // LOG(INFO) << "Distance to main goal: " << distance_to_goal;
  // LOG(INFO) << "Calculated cost: " << cost;

  return cost;
}

}  // namespace cost_functions
