#include <gtest/gtest.h>

#include "planning_params.h"
#include "structs.h"
#include "behavior_planner_FSM.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

class BehaviorPlannerTest : public ::testing::Test {
 protected:
  std::unique_ptr<BehaviorPlannerFSM> behavior_planner;
  
  State ego_state;
  State goal_input;
  
  void SetUp() override {
     behavior_planner = std::make_unique<BehaviorPlannerFSM>(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);
    
      ego_state.location.x = 0;
      ego_state.location.y = 0;
      ego_state.velocity.x = 0;
      ego_state.velocity.y = 0;
      ego_state.rotation.yaw = 0;
  }

  // void TearDown() override {}
};

TEST_F(BehaviorPlannerTest, FollowLaneSameSpeedStraight) {
  
  // Start state
  ego_state.velocity.x = 2;
  
  // Goal
  goal_input = ego_state;
  goal_input.location.x = 20;
  goal_input.velocity.x = P_SPEED_LIMIT; // ego_state.velocity.x; 
    // NOTE: originally I wanted to allow the system to follow a lower-than-speed-limit set speed, but that does not currently
    // play well with other parts of the framework
  
  // Execute
  bool is_junction = false;
  State goal_output = behavior_planner->state_transition(ego_state, goal_input, is_junction, /*tl_state*/ "Green");
  Maneuver behavior = behavior_planner->get_active_maneuver();
  
  // Verify
  EXPECT_EQ(Maneuver::FOLLOW_LANE, behavior);
  EXPECT_FLOAT_EQ(goal_input.velocity.x, goal_output.velocity.x);
  EXPECT_FLOAT_EQ(goal_input.velocity.y, goal_output.velocity.y);
}

TEST_F(BehaviorPlannerTest, FollowLaneSpeedLimitStraight) {
  // Start state
  ego_state.velocity.x = 2;
  
  // Goal
  goal_input = ego_state;
  goal_input.location.x = 20;
  goal_input.velocity.x = P_SPEED_LIMIT + 10; // over speed limit, should not be reached
  
  // Execute
  bool is_junction = false;
  State goal_output = behavior_planner->state_transition(ego_state, goal_input, is_junction, /*tl_state*/ "Green");
  Maneuver behavior = behavior_planner->get_active_maneuver();
  
  // Verify
  EXPECT_EQ(Maneuver::FOLLOW_LANE, behavior);
  EXPECT_FLOAT_EQ(P_SPEED_LIMIT, goal_output.velocity.x);
  EXPECT_FLOAT_EQ(0, goal_output.velocity.y);
}

TEST_F(BehaviorPlannerTest, FollowLaneSameSpeedAtAngle) {
  // Start state
  ego_state.velocity.x = 2;
  
  // Goal
  goal_input = ego_state;
  goal_input.location.x = 20;
  goal_input.rotation.yaw = M_PI/4;
  double expected_speed = P_SPEED_LIMIT; // ego_state.velocity.x; 
    // NOTE: originally I wanted to allow the system to follow a lower-than-speed-limit set speed, but that does not currently
    // play well with other parts of the framework
  goal_input.velocity.x = expected_speed * std::cos(goal_input.rotation.yaw);
  goal_input.velocity.y = expected_speed * std::sin(goal_input.rotation.yaw);

  
  // Execute
  bool is_junction = false;
  State goal_output = behavior_planner->state_transition(ego_state, goal_input, is_junction, /*tl_state*/ "Green");
  Maneuver behavior = behavior_planner->get_active_maneuver();
  
  // Verify
  EXPECT_EQ(Maneuver::FOLLOW_LANE, behavior);
  EXPECT_FLOAT_EQ(goal_input.velocity.x, goal_output.velocity.x);
  EXPECT_FLOAT_EQ(goal_input.velocity.y, goal_output.velocity.y);
}

TEST_F(BehaviorPlannerTest, DecelToStopAtJunctionStraight) {
  // Start state
  ego_state.velocity.x = 2;
  
  // Goal
  goal_input = ego_state;
  goal_input.location.x = 20;
  
  // Execute
  bool is_junction = true;
  State goal_output = behavior_planner->state_transition(ego_state, goal_input, is_junction, /*tl_state*/ "Green");
  Maneuver behavior = behavior_planner->get_active_maneuver();
  
  // Verify
  EXPECT_EQ(Maneuver::DECEL_TO_STOP, behavior);
  EXPECT_FLOAT_EQ(goal_input.location.x - P_STOP_LINE_BUFFER, goal_output.location.x);
  EXPECT_NEAR(0.0, goal_output.location.y, 1e-10); // EXPECT_FLOAT_EQ failed due to receiving a small non-0 value
  EXPECT_FLOAT_EQ(0.0, goal_output.velocity.x);
  EXPECT_FLOAT_EQ(0.0, goal_output.velocity.y);
}

TEST_F(BehaviorPlannerTest, StopThenStartAtJunctionStraight) {
  // STEP 1: FOLLOW_LANE -> DECEL_TO_STOP
  
  // Start state
  ego_state.velocity.x = 2;
  
  // Goal
  goal_input = ego_state;
  goal_input.location.x = 2;
  
  // Execute
  bool is_junction = true;
  State goal_output = behavior_planner->state_transition(ego_state, goal_input, is_junction, /*tl_state*/ "Green");
  Maneuver behavior = behavior_planner->get_active_maneuver();
  
  // Verify
  EXPECT_EQ(Maneuver::DECEL_TO_STOP, behavior);
  EXPECT_FLOAT_EQ(goal_input.location.x - P_STOP_LINE_BUFFER, goal_output.location.x);
  EXPECT_NEAR(0.0, goal_output.location.y, 1e-10); // EXPECT_FLOAT_EQ failed due to receiving a small non-0 value
  EXPECT_FLOAT_EQ(0.0, goal_output.velocity.x);
  EXPECT_FLOAT_EQ(0.0, goal_output.velocity.y);
  
  // STEP 2: DECEL_TO_STOP -> STOPPED
  
  ego_state = goal_output;

  goal_output = behavior_planner->state_transition(ego_state, goal_input, is_junction, /*tl_state*/ "Green");
  behavior = behavior_planner->get_active_maneuver();
  
  EXPECT_EQ(Maneuver::STOPPED, behavior);
  EXPECT_FLOAT_EQ(goal_input.location.x - P_STOP_LINE_BUFFER, goal_output.location.x);
  EXPECT_NEAR(0.0, goal_output.location.y, 1e-10); // EXPECT_FLOAT_EQ failed due to receiving a small non-0 value
  EXPECT_FLOAT_EQ(0.0, goal_output.velocity.x);
  EXPECT_FLOAT_EQ(0.0, goal_output.velocity.y);
  
  // STEP 3: Stay in STOPPED
  ASSERT_GE(P_REQ_STOPPED_TIME, 1);
  sleep(static_cast<unsigned int>(std::floor(P_REQ_STOPPED_TIME / 2))); // Sleep less than the time required to stay at the intersection
  
  ego_state = goal_output;

  goal_output = behavior_planner->state_transition(ego_state, goal_input, is_junction, /*tl_state*/ "Green");
  behavior = behavior_planner->get_active_maneuver();
  
  EXPECT_EQ(Maneuver::STOPPED, behavior);
  EXPECT_FLOAT_EQ(goal_input.location.x - P_STOP_LINE_BUFFER, goal_output.location.x);
  EXPECT_NEAR(0.0, goal_output.location.y, 1e-10); // EXPECT_FLOAT_EQ failed due to receiving a small non-0 value
  EXPECT_FLOAT_EQ(0.0, goal_output.velocity.x);
  EXPECT_FLOAT_EQ(0.0, goal_output.velocity.y);
  
  // STEP 4: STOPPED -> FOLLOW_LANE
  sleep(static_cast<unsigned int>(std::ceil(P_REQ_STOPPED_TIME))); // Sleep again to definitely pass the time required for staying at the intersection
  
  ego_state = goal_output;
  goal_input.location.x = 10; // put goal further

  goal_output = behavior_planner->state_transition(ego_state, goal_input, is_junction, /*tl_state*/ "Green");
  behavior = behavior_planner->get_active_maneuver();
  
  EXPECT_EQ(Maneuver::FOLLOW_LANE, behavior);
  EXPECT_FLOAT_EQ(goal_input.location.x, goal_output.location.x);
  EXPECT_NEAR(0.0, goal_output.location.y, 1e-10); // EXPECT_FLOAT_EQ failed due to receiving a small non-0 value
  EXPECT_FLOAT_EQ(goal_input.velocity.x, goal_output.velocity.x);
  EXPECT_FLOAT_EQ(goal_input.velocity.y, goal_output.velocity.y);
}

TEST_F(BehaviorPlannerTest, Lookahead) {
  // NOTE: The default parameters look incorrect, as the speed limit is set to 3m/s, but in this
  // case the calculated lookahead distance (3) would not even reach the minimum lookahead distance (8).
  // Start state
  ego_state.velocity.x = 10;
  
  double lookahead = behavior_planner->get_look_ahead_distance(ego_state);
  EXPECT_NEAR(16.67, lookahead, 0.1);
}
