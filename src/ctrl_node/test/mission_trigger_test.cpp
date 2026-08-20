#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/time.h>

#include "input.hpp"
#include "fsm_nodelet.hpp"

namespace ctrl_node
{
namespace
{

TEST(MissionTrigger, ActivatesOnlyFreshPendingGeneration)
{
    ros::Time::init();
    Mission_Trigger_t trigger;
    geometry_msgs::PoseStampedPtr message(
        new geometry_msgs::PoseStamped());

    trigger.feed(message);
    const ros::Time received = trigger.received_at;

    EXPECT_TRUE(trigger.activatePending(
        received + ros::Duration(0.1), 1.0));
    EXPECT_TRUE(trigger.active);
    EXPECT_EQ(trigger.sequence, trigger.pending_sequence);

    trigger.deactivate();
    EXPECT_FALSE(trigger.active);
    EXPECT_FALSE(trigger.activatePending(
        received + ros::Duration(0.2), 1.0));
}

TEST(MissionTrigger, RejectsAndConsumesExpiredPendingGeneration)
{
    ros::Time::init();
    Mission_Trigger_t trigger;
    geometry_msgs::PoseStampedPtr message(
        new geometry_msgs::PoseStamped());

    trigger.feed(message);
    const ros::Time received = trigger.received_at;

    EXPECT_FALSE(trigger.activatePending(
        received + ros::Duration(2.0), 1.0));
    EXPECT_FALSE(trigger.active);
    EXPECT_EQ(trigger.sequence, trigger.pending_sequence);
    EXPECT_TRUE(trigger.received_at.isZero());
}

TEST(MissionTrigger, CancelPendingDoesNotReuseOldTrigger)
{
    ros::Time::init();
    Mission_Trigger_t trigger;
    geometry_msgs::PoseStampedPtr message(
        new geometry_msgs::PoseStamped());

    trigger.feed(message);
    trigger.cancelPending();

    EXPECT_FALSE(trigger.activatePending(ros::Time::now(), 1.0));
    EXPECT_FALSE(trigger.active);
    EXPECT_EQ(trigger.sequence, trigger.pending_sequence);
}

TEST(RetryGate, AcceptedRequestRemainsRateLimitedUntilIntervalEnds)
{
    RetryGate gate(0.5, 3);
    const ros::Time start(10.0);

    EXPECT_TRUE(gate.should_attempt(start));
    gate.update(start, SrvResult::Accepted);
    EXPECT_FALSE(gate.should_attempt(start + ros::Duration(0.49)));
    EXPECT_TRUE(gate.should_attempt(start + ros::Duration(0.5)));
}

TEST(RetryGate, RetryableFailuresExhaustConfiguredBudget)
{
    RetryGate gate(0.1, 2);
    const ros::Time start(20.0);

    gate.update(start, SrvResult::Retryable);
    EXPECT_FALSE(gate.exhausted());
    gate.update(start + ros::Duration(0.1), SrvResult::TransportError);
    EXPECT_TRUE(gate.exhausted());
    EXPECT_FALSE(gate.should_attempt(start + ros::Duration(1.0)));
}

TEST(CommandData, StoresAndMatchesTrajectoryGeneration)
{
    ros::Time::init();
    Command_Data_t command;
    quadrotor_msgs::PositionCommandPtr message(
        new quadrotor_msgs::PositionCommand());
    message->trajectory_id = 42U;
    message->trajectory_flag =
        quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

    command.feed(message);

    EXPECT_TRUE(command.isReadyForTrajectory(42U));
    EXPECT_FALSE(command.isReadyForTrajectory(41U));
    EXPECT_FALSE(command.isReadyForTrajectory(0U));
}

TEST(CommandData, InvalidateClearsTrajectoryGeneration)
{
    ros::Time::init();
    Command_Data_t command;
    quadrotor_msgs::PositionCommandPtr message(
        new quadrotor_msgs::PositionCommand());
    message->trajectory_id = 7U;
    message->trajectory_flag =
        quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

    command.feed(message);
    ASSERT_TRUE(command.isReadyForTrajectory(7U));

    command.invalidate();

    EXPECT_TRUE(command.rcv_stamp.isZero());
    EXPECT_EQ(0U, command.trajectory_id);
    EXPECT_EQ(quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY,
              command.trajectory_flag);
    EXPECT_FALSE(command.isReadyForTrajectory(7U));
}

TEST(NavigationFailsafe, UsesGraceThenHoverThenAutoLand)
{
    EXPECT_EQ(NavigationFailsafeStage::Grace,
              classify_navigation_failsafe(0.99, 1.0, 3.0));
    EXPECT_EQ(NavigationFailsafeStage::Hover,
              classify_navigation_failsafe(1.0, 1.0, 3.0));
    EXPECT_EQ(NavigationFailsafeStage::Hover,
              classify_navigation_failsafe(2.99, 1.0, 3.0));
    EXPECT_EQ(NavigationFailsafeStage::AutoLand,
              classify_navigation_failsafe(3.0, 1.0, 3.0));
}

TEST(CriticalInputLoss, RequestsAutoLandOnlyForControlledFlight)
{
    EXPECT_FALSE(critical_input_loss_requires_auto_land(
        false, false, false, false, false));

    // Fresh state feedback: require OFFBOARD plus armed/internal-flight/ARM
    // evidence; leaving OFFBOARD remains an explicit pilot/PX4 takeover.
    EXPECT_FALSE(critical_input_loss_requires_auto_land(
        true, false, false, false, false));
    EXPECT_FALSE(critical_input_loss_requires_auto_land(
        true, true, false, true, false));
    EXPECT_TRUE(critical_input_loss_requires_auto_land(
        true, true, true, false, false));
    EXPECT_TRUE(critical_input_loss_requires_auto_land(
        true, false, true, true, false));
    EXPECT_TRUE(critical_input_loss_requires_auto_land(
        true, false, true, false, true));

    // Stale state feedback: preserve the last known OFFBOARD state or the
    // internal flight state as conservative evidence that flight is active.
    EXPECT_TRUE(critical_input_loss_requires_auto_land(
        false, true, true, false, false));
    EXPECT_TRUE(critical_input_loss_requires_auto_land(
        false, false, false, true, false));
}

} // namespace
} // namespace ctrl_node
