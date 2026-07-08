#include <gtest/gtest.h>

#include <cmath>

#include "rc26_decision/navigation/straight_line_trajectory.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

double pointLineCross(const rc26_decision::navigation::StraightLinePose &start,
                      const rc26_decision::navigation::StraightLinePose &target,
                      const rc26_decision::navigation::StraightLinePose &point) {
  return (target.x - start.x) * (point.y - start.y) -
         (target.y - start.y) * (point.x - start.x);
}

} // namespace

TEST(StraightLineTrajectory, XTurnXTargetMatchesLegacyEndpoint) {
  const rc26_decision::navigation::StraightLinePose start{1.0, 2.0, kPi / 6.0};

  const auto target = rc26_decision::navigation::xTurnXTarget(
      start, 0.5, -kPi / 2.0, -0.8);

  const double target_yaw = start.yaw - kPi / 2.0;
  const double expected_x =
      start.x + 0.5 * std::cos(start.yaw) - 0.8 * std::cos(target_yaw);
  const double expected_y =
      start.y + 0.5 * std::sin(start.yaw) - 0.8 * std::sin(target_yaw);

  EXPECT_NEAR(target.x, expected_x, 1.0e-9);
  EXPECT_NEAR(target.y, expected_y, 1.0e-9);
  EXPECT_NEAR(target.yaw, target_yaw, 1.0e-9);
}

TEST(StraightLineTrajectory, ReferencePointsStayOnStartTargetSegment) {
  const rc26_decision::navigation::StraightLinePose start{0.0, 0.0, 0.25};
  const rc26_decision::navigation::StraightLinePose target{1.2, -0.6, -1.0};

  const auto begin =
      rc26_decision::navigation::straightLineReference(start, target, 0.0, 0.0);
  const auto middle =
      rc26_decision::navigation::straightLineReference(start, target, 0.5, 0.0);
  const auto end =
      rc26_decision::navigation::straightLineReference(start, target, 1.0, 0.0);

  EXPECT_NEAR(begin.pose.x, start.x, 1.0e-9);
  EXPECT_NEAR(begin.pose.y, start.y, 1.0e-9);
  EXPECT_NEAR(end.pose.x, target.x, 1.0e-9);
  EXPECT_NEAR(end.pose.y, target.y, 1.0e-9);
  EXPECT_NEAR(pointLineCross(start, target, middle.pose), 0.0, 1.0e-9);
  EXPECT_GE(middle.progress, 0.0);
  EXPECT_LE(middle.progress, 1.0);
}

TEST(StraightLineTrajectory, YawUsesShortestSmoothstepInterpolation) {
  const rc26_decision::navigation::StraightLinePose start{0.0, 0.0, 170.0 * kPi / 180.0};
  const rc26_decision::navigation::StraightLinePose target{1.0, 0.0, -170.0 * kPi / 180.0};

  const auto middle =
      rc26_decision::navigation::straightLineReference(start, target, 0.5, 0.0);
  const auto end =
      rc26_decision::navigation::straightLineReference(start, target, 1.0, 0.0);

  EXPECT_NEAR(middle.pose.yaw, kPi, 1.0e-9);
  EXPECT_NEAR(end.pose.yaw, target.yaw, 1.0e-9);
}

TEST(StraightLineTrajectory, ProjectionProgressDoesNotMoveBackward) {
  const rc26_decision::navigation::StraightLinePose start{0.0, 0.0, 0.0};
  const rc26_decision::navigation::StraightLinePose target{2.0, 0.0, 0.0};

  const double progressed = rc26_decision::navigation::projectProgressOnLine(
      start, target, 1.0, 0.4, 0.0);
  const double after_lateral_error = rc26_decision::navigation::projectProgressOnLine(
      start, target, 0.6, -0.8, progressed);

  EXPECT_NEAR(progressed, 0.5, 1.0e-9);
  EXPECT_NEAR(after_lateral_error, progressed, 1.0e-9);
}

TEST(StraightLineTrajectory, RotateRetreatTargetMatchesRetreatFrameFormula) {
  const rc26_decision::navigation::StraightLinePose start{2.0, -1.0, 0.3};
  const double target_delta = 175.0 * kPi / 180.0;
  const double retreat_x = -0.4;
  const double retreat_y = -0.4;

  const auto target = rc26_decision::navigation::rotateRetreatTarget(
      start, target_delta, retreat_x, retreat_y);

  const double target_yaw =
      rc26_decision::navigation::normalizeAngle(start.yaw + target_delta);
  const double c = std::cos(target_yaw);
  const double s = std::sin(target_yaw);

  EXPECT_NEAR(target.x, start.x + retreat_x * c - retreat_y * s, 1.0e-9);
  EXPECT_NEAR(target.y, start.y + retreat_x * s + retreat_y * c, 1.0e-9);
  EXPECT_NEAR(target.yaw, target_yaw, 1.0e-9);

  const auto middle =
      rc26_decision::navigation::straightLineReference(start, target, 0.5, 0.0);
  EXPECT_NEAR(pointLineCross(start, target, middle.pose), 0.0, 1.0e-9);
}
