#include <gtest/gtest.h>

#include "planning_params.h"
#include "structs.h"
#include "motion_planner.h"


class MotionPlannerTest : public ::testing::Test {
 protected:
  std::unique_ptr<MotionPlanner> motion_planner;
  
  void SetUp() override {
     motion_planner = std::make_unique<MotionPlanner>(
      P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);
  }

  // void TearDown() override {}
};

TEST_F(MotionPlannerTest, OffsetGoalsStraight) {
  State goal;
  goal.location.x = 10;
  
  std::vector<State> offset_goals = motion_planner->generate_offset_goals(goal);
  
  for (int i = 0; i < P_NUM_PATHS; ++i) {
    float offset = (i - (int)(P_NUM_PATHS / 2)) * P_GOAL_OFFSET;
    EXPECT_FLOAT_EQ(goal.location.x, offset_goals[i].location.x);
    EXPECT_FLOAT_EQ(goal.location.y + offset, offset_goals[i].location.y);
  }
}

TEST_F(MotionPlannerTest, ValidVelocitiesGeneratedForDecelToStop) {
  State ego_state;
  ego_state.velocity.x = 10;
  
  State goal_state;
  goal_state.location.x = 100;
  goal_state.velocity.x = 0;
  auto desired_speed = utils::magnitude(goal_state.velocity);
  std::vector<State> goal_set {goal_state};
  
  auto behavior = Maneuver::DECEL_TO_STOP;
  
  State lead_car_state; // currently unused
  
  auto spirals = motion_planner->generate_spirals(ego_state, goal_set);
  auto trajectory = motion_planner->_velocity_profile_generator.generate_trajectory( spirals[0], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);
  
  for (size_t j = 0; j < trajectory.size(); j++) {
    double velocity = trajectory[j].v;
    // std::cout << "t, x, y, v: " << trajectory[j].relative_time << ", " << trajectory[j].path_point.x << ", " << trajectory[j].path_point.y << ", " << trajectory[j].v << std::endl;
    // Make sure that no inf/nan values are returned in the velocity, as that crashes the whole simulation.
    ASSERT_FALSE(velocity == std::numeric_limits<double>::infinity() || std::isnan(velocity));
    if (j < 1) {
      continue;
    }
    
    if (j == trajectory.size() - 1) {
      EXPECT_FLOAT_EQ(0.0, trajectory[j].v);
    }
    
    auto direction = carla::geom::Vector3D(trajectory[j].path_point.x - trajectory[j-1].path_point.x,
                                           trajectory[j].path_point.y - trajectory[j-1].path_point.y,
                                           0);
    auto distance = utils::magnitude(direction);
    auto dt = trajectory[j].relative_time - trajectory[j-1].relative_time;
    auto avg_v_from_velocity = (trajectory[j].v + trajectory[j-1].v) / 2;
    auto avg_v_from_displacement = distance / dt;
    // Calculate the average speed both from the initial/final speed and also from the displacement between trajectory points.
    // Ideally these should perfectly match, though this is not the case, as velocity generation is still implemented in a 
    // very crude manner. However there was a bug that caused huge differences here, which made the car to teleport when decelerating, 
    // which looked rather weird.
    EXPECT_LT(std::abs(avg_v_from_velocity - avg_v_from_displacement), 1.0);

    auto a_from_velocity = (trajectory[j].v - trajectory[j-1].v) / dt;
    EXPECT_LE(std::abs(a_from_velocity), P_MAX_ACCEL + 0.1);
    auto a_from_displacement = VelocityProfileGenerator::calc_acceleration(trajectory[j].v, trajectory[j-1].v, distance);
    EXPECT_LE(std::abs(a_from_displacement), P_MAX_ACCEL + 0.1);
    // std::cout << "avg_v_from_velocity: " << avg_v_from_velocity << " avg_v_from_displacement: " << avg_v_from_displacement << ", dt: " << dt << std::endl;
  }
}

// Same test as above, but requires stopping on a much shorter distance, causing the 
// 'hard brake' condition to be true in the velocity generator.
TEST_F(MotionPlannerTest, ValidVelocitiesGeneratedForDecelToStopHardBrake) {
  State ego_state;
  ego_state.velocity.x = 10;
  
  State goal_state;
  goal_state.location.x = 10;
  goal_state.velocity.x = 0;
  auto desired_speed = utils::magnitude(goal_state.velocity);
  std::vector<State> goal_set {goal_state};
  
  auto behavior = Maneuver::DECEL_TO_STOP;
  
  State lead_car_state; // currently unused
  
  auto spirals = motion_planner->generate_spirals(ego_state, goal_set);
  auto trajectory = motion_planner->_velocity_profile_generator.generate_trajectory( spirals[0], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);
  
  for (size_t j = 0; j < trajectory.size(); j++) {
    double velocity = trajectory[j].v;
    // std::cout << "x, y, v: " << trajectory[j].path_point.x << ", " << trajectory[j].path_point.y << ", " << trajectory[j].v << std::endl;
    // Make sure that no inf/nan values are returned in the velocity, as that crashes the whole simulation.
    ASSERT_FALSE(velocity == std::numeric_limits<double>::infinity() || std::isnan(velocity));
    if (j < 1) {
      continue;
    }
    
    auto direction = carla::geom::Vector3D(trajectory[j].path_point.x - trajectory[j-1].path_point.x,
                                           trajectory[j].path_point.y - trajectory[j-1].path_point.y,
                                           0);
    auto distance = utils::magnitude(direction);
    auto dt = trajectory[j].relative_time - trajectory[j-1].relative_time;
    auto avg_v_from_velocity = (trajectory[j].v + trajectory[j-1].v) / 2;
    auto avg_v_from_displacement = distance / dt;
    EXPECT_LT(std::abs(avg_v_from_velocity - avg_v_from_displacement), 1.0);
    
    auto a_from_velocity = (trajectory[j].v - trajectory[j-1].v) / dt;
    EXPECT_LE(std::abs(a_from_velocity), P_MAX_ACCEL + 0.1);
	auto a_from_displacement = VelocityProfileGenerator::calc_acceleration(trajectory[j].v, trajectory[j-1].v, distance);
    EXPECT_LE(std::abs(a_from_displacement), P_MAX_ACCEL + 0.1);
    // std::cout << "avg_v_from_velocity: " << avg_v_from_velocity << " avg_v_from_displacement: " << avg_v_from_displacement << ", dt: " << dt << std::endl;
  }
}

// Originall the desired speed was always set as the speed of the final trajectory point, 
// even when the vehicle could not reach it given the acceleration limits. This caused
// a huge jump in the velocity for the last point of the trajectory. Check that this is fixed.
TEST_F(MotionPlannerTest, EvenAccelerationWhenNotReachingDesiredSpeed) {
  State ego_state;
  ego_state.velocity.x = 0;
  
  State goal_state;
  goal_state.location.x = 8;
  goal_state.velocity.x = 9;
  auto desired_speed = utils::magnitude(goal_state.velocity);
  std::vector<State> goal_set {goal_state};
  
  auto behavior = Maneuver::FOLLOW_LANE;
  
  State lead_car_state; // currently unused
  
  auto spirals = motion_planner->generate_spirals(ego_state, goal_set);
  auto trajectory = motion_planner->_velocity_profile_generator.generate_trajectory(spirals[0], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);
  
  for (size_t j = 1; j < trajectory.size(); j++) {
    auto dt = trajectory[j].relative_time - trajectory[j-1].relative_time;
    auto dv = trajectory[j].v - trajectory[j-1].v;
    auto a = dv / dt;
    EXPECT_LE(std::abs(a), P_MAX_ACCEL + 0.1);
  }
}

TEST(VelocityProfileGeneratorTest, CalcDistanceZeroAcceleration) {
  auto distance = VelocityProfileGenerator::calc_distance(10.0, 10.0, 10.0);
  EXPECT_FLOAT_EQ(0.0, distance);
}

TEST(VelocityProfileGeneratorTest, CalcDistance) {
  auto distance = VelocityProfileGenerator::calc_distance(10.0, 0.0, -10.0);
  EXPECT_FLOAT_EQ(5.0, distance);
}

TEST(VelocityProfileGeneratorTest, CalcFinalSpeedFullStop) {
  auto speed = VelocityProfileGenerator::calc_final_speed(10.0, -10.0, 5.0);
  EXPECT_FLOAT_EQ(0.0, speed);
}

// The case when the available distance is long, so given the constant acceleration the car would 'reverse',
// which means that the discriminant becomes negative. This must simply yield a 0 result.
TEST(VelocityProfileGeneratorTest, CalcFinalSpeedFullStopLongDistance) {
  auto speed = VelocityProfileGenerator::calc_final_speed(5.0, -1.5, 25.0);
  EXPECT_FLOAT_EQ(0.0, speed);
}