#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "src/inference/runtime/vision_frame_snapshot_utils.hpp"

namespace {

rc26_vision::VisionInferenceManager::FrameSnapshot displayOnlySnapshot() {
    rc26_vision::VisionInferenceManager::FrameSnapshot snapshot;
    snapshot.has_display = true;
    snapshot.display_sequence = 42;
    snapshot.display_stamp_ns = 123456789;
    return snapshot;
}

}  // namespace

TEST(VisionFrameSnapshot, FillsMissingColorFromDisplayFrame)
{
    auto snapshot = displayOnlySnapshot();
    const cv::Mat display_frame(4, 6, CV_8UC3, cv::Scalar(10, 20, 30));

    const bool filled =
        rc26_vision::runtime_detail::fillSnapshotColorFromDisplayFrame(
            snapshot, display_frame);

    EXPECT_TRUE(filled);
    EXPECT_TRUE(snapshot.has_color);
    ASSERT_FALSE(snapshot.color_bgr.empty());
    EXPECT_EQ(snapshot.color_bgr.rows, display_frame.rows);
    EXPECT_EQ(snapshot.color_bgr.cols, display_frame.cols);
    EXPECT_EQ(snapshot.color_stamp_ns, snapshot.display_stamp_ns);

    snapshot.color_bgr.at<cv::Vec3b>(0, 0) = cv::Vec3b(1, 2, 3);
    EXPECT_EQ(display_frame.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));
}

TEST(VisionFrameSnapshot, KeepsExistingColorFrame)
{
    auto snapshot = displayOnlySnapshot();
    snapshot.has_color = true;
    snapshot.color_stamp_ns = 111;
    snapshot.color_bgr = cv::Mat(2, 3, CV_8UC3, cv::Scalar(1, 2, 3));
    const cv::Mat display_frame(4, 6, CV_8UC3, cv::Scalar(10, 20, 30));

    const bool filled =
        rc26_vision::runtime_detail::fillSnapshotColorFromDisplayFrame(
            snapshot, display_frame);

    EXPECT_FALSE(filled);
    EXPECT_TRUE(snapshot.has_color);
    EXPECT_EQ(snapshot.color_bgr.rows, 2);
    EXPECT_EQ(snapshot.color_bgr.cols, 3);
    EXPECT_EQ(snapshot.color_stamp_ns, 111);
}

TEST(VisionFrameSnapshot, RequiresValidDisplayFrame)
{
    auto snapshot = displayOnlySnapshot();
    snapshot.has_display = false;
    const cv::Mat display_frame(4, 6, CV_8UC3, cv::Scalar(10, 20, 30));

    EXPECT_FALSE(rc26_vision::runtime_detail::fillSnapshotColorFromDisplayFrame(
        snapshot, display_frame));
    EXPECT_FALSE(snapshot.has_color);
    EXPECT_TRUE(snapshot.color_bgr.empty());

    snapshot = displayOnlySnapshot();
    EXPECT_FALSE(rc26_vision::runtime_detail::fillSnapshotColorFromDisplayFrame(
        snapshot, cv::Mat{}));
    EXPECT_FALSE(snapshot.has_color);
    EXPECT_TRUE(snapshot.color_bgr.empty());
}
