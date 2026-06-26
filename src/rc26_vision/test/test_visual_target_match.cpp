#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "rc26_vision/shared/target/visual_target_match.hpp"

namespace {

rc26_vision::Detection det(float x1, float y1, float x2, float y2,
                           const std::string& label = "T_03",
                           float score = 0.8F)
{
    rc26_vision::Detection detection;
    detection.x1 = x1;
    detection.y1 = y1;
    detection.x2 = x2;
    detection.y2 = y2;
    detection.score = score;
    detection.class_id = 7;
    detection.class_name = label;
    return detection;
}

}  // namespace

TEST(VisualTargetMatch, LabelFallsBackToClassId)
{
    auto detection = det(0.0F, 0.0F, 10.0F, 10.0F, "");
    detection.class_id = 12;

    EXPECT_EQ(rc26_vision::visualTargetLabel(detection), "class_12");
}

TEST(VisualTargetMatch, SnapshotCopiesDetectionBboxAndSequence)
{
    const auto snapshot = rc26_vision::makeVisualTargetSnapshot(
        det(10.0F, 20.0F, 30.0F, 40.0F, "T_08", 0.91F), 42);

    EXPECT_EQ(snapshot.label, "T_08");
    EXPECT_DOUBLE_EQ(snapshot.distance_m, 0.0);
    EXPECT_DOUBLE_EQ(snapshot.score, 0.91F);
    EXPECT_EQ(snapshot.sequence, 42);
    EXPECT_DOUBLE_EQ(snapshot.x1, 10.0);
    EXPECT_DOUBLE_EQ(snapshot.y1, 20.0);
    EXPECT_DOUBLE_EQ(snapshot.x2, 30.0);
    EXPECT_DOUBLE_EQ(snapshot.y2, 40.0);
}

TEST(VisualTargetMatch, BboxIouHandlesOverlapFarAndReversedBoxes)
{
    const auto reference = rc26_vision::makeVisualTargetSnapshot(
        det(10.0F, 10.0F, 50.0F, 50.0F), 1);
    const auto close = rc26_vision::makeVisualTargetSnapshot(
        det(15.0F, 15.0F, 55.0F, 55.0F), 2);
    const auto far = rc26_vision::makeVisualTargetSnapshot(
        det(80.0F, 80.0F, 120.0F, 120.0F), 3);
    const auto reversed = rc26_vision::makeVisualTargetSnapshot(
        det(50.0F, 50.0F, 10.0F, 10.0F), 4);

    EXPECT_GT(rc26_vision::bboxIou(reference, close), 0.30);
    EXPECT_DOUBLE_EQ(rc26_vision::bboxIou(reference, far), 0.0);
    EXPECT_DOUBLE_EQ(rc26_vision::bboxIou(reference, reversed), 1.0);
}

TEST(VisualTargetMatch, BboxIouReturnsZeroForEmptyArea)
{
    const auto reference = rc26_vision::makeVisualTargetSnapshot(
        det(10.0F, 10.0F, 10.0F, 50.0F), 1);
    const auto candidate = rc26_vision::makeVisualTargetSnapshot(
        det(10.0F, 10.0F, 50.0F, 50.0F), 2);

    EXPECT_DOUBLE_EQ(rc26_vision::bboxIou(reference, candidate), 0.0);
}

TEST(VisualTargetMatch, SameTargetUsesLabelAndIouThreshold)
{
    const auto reference = rc26_vision::makeVisualTargetSnapshot(
        det(10.0F, 10.0F, 50.0F, 50.0F, "T_03"), 1);
    const auto close = rc26_vision::makeVisualTargetSnapshot(
        det(15.0F, 15.0F, 55.0F, 55.0F, "T_03"), 2);
    const auto far = rc26_vision::makeVisualTargetSnapshot(
        det(80.0F, 80.0F, 120.0F, 120.0F, "T_03"), 3);
    const auto wrong_label = rc26_vision::makeVisualTargetSnapshot(
        det(15.0F, 15.0F, 55.0F, 55.0F, "T_04"), 4);

    EXPECT_TRUE(rc26_vision::isSameVisualTarget(reference, close, 0.30));
    EXPECT_FALSE(rc26_vision::isSameVisualTarget(reference, far, 0.30));
    EXPECT_FALSE(rc26_vision::isSameVisualTarget(reference, wrong_label, 0.30));
}

TEST(VisualTargetMatch, IgnoredTargetUsesSameTargetRule)
{
    const auto ignored = rc26_vision::makeVisualTargetSnapshot(
        det(10.0F, 10.0F, 50.0F, 50.0F, "T_03"), 1);
    const std::vector<rc26_vision::VisualTargetSnapshot> ignored_targets{ignored};
    const auto same = rc26_vision::makeVisualTargetSnapshot(
        det(14.0F, 14.0F, 54.0F, 54.0F, "T_03"), 2);
    const auto another_same_label = rc26_vision::makeVisualTargetSnapshot(
        det(120.0F, 120.0F, 160.0F, 160.0F, "T_03"), 3);

    EXPECT_TRUE(rc26_vision::isIgnoredVisualTarget(same, ignored_targets, 0.30));
    EXPECT_FALSE(rc26_vision::isIgnoredVisualTarget(another_same_label, ignored_targets, 0.30));
}
