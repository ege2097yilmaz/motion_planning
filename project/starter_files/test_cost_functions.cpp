#include <gtest/gtest.h>

#include "planning_params.h"
#include "structs.h"
#include "cost_functions.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

TEST(CostFunctionsTest, NoCollision) {
  std::vector<PathPoint> spiral;
  PathPoint pp;
  pp.x = 10;
  pp.y = 10;
  spiral.emplace_back(pp);
  pp.x = 20;
  pp.y = 20;
  spiral.emplace_back(pp);
  
  std::vector<State> obstacles;
  State s;
  s.location.x = 30;
  s.location.y = 30;
  obstacles.emplace_back(s);
  s.location.x = 40;
  s.location.y = 40;
  obstacles.emplace_back(s);
  
  double cost = cost_functions::collision_circles_cost_spiral(spiral, obstacles);
  EXPECT_FLOAT_EQ(0, cost);
}

TEST(CostFunctionsTest, PerfectCollision) {
  std::vector<PathPoint> spiral;
  PathPoint pp;
  pp.x = 10;
  pp.y = 10;
  spiral.emplace_back(pp);
  pp.x = 20;
  pp.y = 20;
  spiral.emplace_back(pp);
  
  std::vector<State> obstacles;
  State s;
  s.location.x = 10;
  s.location.y = 10;
  obstacles.emplace_back(s);
  s.location.x = 40;
  s.location.y = 40;
  obstacles.emplace_back(s);
  
  double cost = cost_functions::collision_circles_cost_spiral(spiral, obstacles);
  EXPECT_FLOAT_EQ(COLLISION, cost);
}

TEST(CostFunctionsTest, CornerCollision) {
  std::vector<PathPoint> spiral;
  PathPoint pp;
  pp.x = 10;
  pp.y = 10;
  spiral.emplace_back(pp);
  
  std::vector<State> obstacles;
  State s;
  s.location.x = pp.x + VEHICLE_SIZE[0];
  s.location.y = pp.y + VEHICLE_SIZE[1];
  obstacles.emplace_back(s);
  
  double cost = cost_functions::collision_circles_cost_spiral(spiral, obstacles);
  EXPECT_FLOAT_EQ(COLLISION, cost);
}

TEST(CostFunctionsTest, ReachedMainGoal) {
  std::vector<PathPoint> spiral;
  PathPoint pp;
  pp.x = 10;
  pp.y = 10;
  spiral.emplace_back(pp);
  
  State main_goal;
  main_goal.location.x = pp.x;
  main_goal.location.y = pp.y;
  
  double cost = cost_functions::close_to_main_goal_cost_spiral(spiral, main_goal);
  EXPECT_FLOAT_EQ(0, cost);
}

TEST(CostFunctionsTest, FarFromMainGoal) {
  std::vector<PathPoint> spiral;
  PathPoint pp;
  pp.x = 10;
  pp.y = 10;
  spiral.emplace_back(pp);
  
  State main_goal;
  main_goal.location.x = pp.x + 1000;
  main_goal.location.y = pp.y + 1000;
  
  double cost = cost_functions::close_to_main_goal_cost_spiral(spiral, main_goal);
  EXPECT_FLOAT_EQ(1, cost);
}