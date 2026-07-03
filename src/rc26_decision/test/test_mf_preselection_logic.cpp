#include <cstdint>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "rc26_decision/mf/merlin_map.hpp"
#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"
#include "rc26_decision/stair/stair_area.hpp"
#include "rc26_vision/shared/sensors/depth_roi_sampler.hpp"

namespace {

rc26_vision::DepthRoiSamplerConfig diagnosticDepthConfig() {
  rc26_vision::DepthRoiSamplerConfig config;
  config.roi_size = 3;
  config.min_valid_count = 4;
  config.min_depth_m = 0.35;
  config.max_depth_m = 1.00;
  return config;
}

rc26_vision::DepthRoiSamplerConfig kfsDepthConfig() {
  rc26_vision::DepthRoiSamplerConfig config;
  config.roi_size = 7;
  config.min_valid_count = 10;
  config.min_depth_m = 0.35;
  config.max_depth_m = 1.40;
  return config;
}

rc26_decision::MfPreselectionLogicResult::NearestKfsCandidate
nearestCandidate(
    rc26_decision::MfPreselectionLogicResult::NearestKfsKind kind,
    std::string label, double distance_m, int offset_px, double score = 0.8) {
  rc26_decision::MfPreselectionLogicResult::NearestKfsCandidate candidate;
  candidate.kind = kind;
  candidate.target.label = std::move(label);
  candidate.target.distance_m = distance_m;
  candidate.target.score = score;
  candidate.offset_px = offset_px;
  candidate.depth_source =
      rc26_decision::MfPreselectionLogicResult::KfsDepthSource::BboxMultiRoi;
  return candidate;
}

TEST(StairSpeedProfile, SamplesLinearFastToSlow) {
  const rc26_decision::StairSpeedProfile profile{0.10, 0.05, 1.0};

  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 0.0), 0.10);
  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 0.5), 0.075);
  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 1.0), 0.05);
  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 2.0), 0.05);
}

TEST(StairSpeedProfile, DurationZeroReturnsSlowSpeed) {
  const rc26_decision::StairSpeedProfile profile{0.40, 0.20, 0.0};

  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 0.0), 0.20);
  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 3.0), 0.20);
}

TEST(StairSpeedProfile, NormalizesSpeedsAndSlowUpperBound) {
  const auto profile = rc26_decision::normalizeStairSpeedProfile(
      rc26_decision::StairSpeedProfile{-0.05, -0.10, -1.0});

  EXPECT_DOUBLE_EQ(profile.fast_speed_mps, 0.05);
  EXPECT_DOUBLE_EQ(profile.slow_speed_mps, 0.05);
  EXPECT_DOUBLE_EQ(profile.slowdown_duration_s, 0.0);
}

TEST(MfPreselectionLogic, LabelMatchesExactAndPrefix) {
  const std::vector<std::string> exact{"R_R1", "B_R1"};
  const std::vector<std::string> prefixes{"T_", "F_"};

  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::labelMatches(
      "R_R1", exact, prefixes));
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::labelMatches(
      "T_03", exact, prefixes));
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::labelMatches(
      "F_18", exact, prefixes));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::labelMatches(
      "B_R2", exact, prefixes));
}

TEST(MfPreselectionLogic, R1KfsScoreThresholdFiltersOnlyR1Kfs) {
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::r1KfsScoreAccepted(
      "R1_KFS", 0.49, 0.50));
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::r1KfsScoreAccepted(
      "R1_KFS", 0.50, 0.50));
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::r1KfsScoreAccepted(
      "R1_KFS", 0.90, 0.50));
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::r1KfsScoreAccepted(
      "R_R1", 0.10, 0.50));
}

TEST(MfPreselectionLogic, GrabRetryActionClassifiesVisibleFailures) {
  using rc26_decision::MfPreselectionLogicResult;
  using rc26_decision::MfPreselectionPickupSource;

  EXPECT_EQ(MfPreselectionLogicResult::grabRetryAction(
                false, MfPreselectionPickupSource::Stair2, true, true),
            MfPreselectionLogicResult::GrabRetryAction::None);
  EXPECT_EQ(MfPreselectionLogicResult::grabRetryAction(
                true, MfPreselectionPickupSource::Stair2, false, false),
            MfPreselectionLogicResult::GrabRetryAction::EntryBackoff);
  EXPECT_EQ(MfPreselectionLogicResult::grabRetryAction(
                true, MfPreselectionPickupSource::Stair2, true, false),
            MfPreselectionLogicResult::GrabRetryAction::EntryBackoff);
  EXPECT_EQ(MfPreselectionLogicResult::grabRetryAction(
                true, MfPreselectionPickupSource::None, false, true),
            MfPreselectionLogicResult::GrabRetryAction::GridCenterRetry);
  EXPECT_EQ(MfPreselectionLogicResult::grabRetryAction(
                true, MfPreselectionPickupSource::None, false, false),
            MfPreselectionLogicResult::GrabRetryAction::None);
  EXPECT_TRUE(MfPreselectionLogicResult::mandatoryEntryStair2Retry(
      MfPreselectionPickupSource::Stair2, false));
  EXPECT_FALSE(MfPreselectionLogicResult::mandatoryEntryStair2Retry(
      MfPreselectionPickupSource::Stair2, true));
  EXPECT_FALSE(MfPreselectionLogicResult::mandatoryEntryStair2Retry(
      MfPreselectionPickupSource::Stair1, false));
}

TEST(MfPreselectionLogic, PickupLimitUsesStrictMaximum) {
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::canPickup(0, 2));
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::canPickup(1, 2));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::canPickup(2, 2));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::canPickup(0, 0));
}

TEST(MfPreselectionLogic, NearestKfsSelectsR1WhenR1DepthIsCloser) {
  using Kind = rc26_decision::MfPreselectionLogicResult::NearestKfsKind;

  const auto selected =
      rc26_decision::MfPreselectionLogicResult::selectNearestKfsCandidate(
          {nearestCandidate(Kind::R2, "T_01", 0.82, 5, 0.95),
           nearestCandidate(Kind::R1, "R1_KFS", 0.56, 90, 0.60)});

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->kind, Kind::R1);
  EXPECT_EQ(selected->target.label, "R1_KFS");
  EXPECT_DOUBLE_EQ(selected->target.distance_m, 0.56);
}

TEST(MfPreselectionLogic, NearestKfsSelectsR2WhenR2DepthIsCloser) {
  using Kind = rc26_decision::MfPreselectionLogicResult::NearestKfsKind;

  const auto selected =
      rc26_decision::MfPreselectionLogicResult::selectNearestKfsCandidate(
          {nearestCandidate(Kind::R2, "T_01", 0.48, 120, 0.55),
           nearestCandidate(Kind::R1, "R_R1", 0.73, 3, 0.99)});

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->kind, Kind::R2);
  EXPECT_EQ(selected->target.label, "T_01");
  EXPECT_DOUBLE_EQ(selected->target.distance_m, 0.48);
}

TEST(MfPreselectionLogic, NearestKfsIgnoresInvalidDepthCandidates) {
  using Kind = rc26_decision::MfPreselectionLogicResult::NearestKfsKind;

  const auto selected =
      rc26_decision::MfPreselectionLogicResult::selectNearestKfsCandidate(
          {nearestCandidate(Kind::R1, "R1_KFS", 0.0, 0, 0.99),
           nearestCandidate(Kind::R2, "T_01",
                            std::numeric_limits<double>::quiet_NaN(), 0, 0.99),
           nearestCandidate(Kind::R2, "T_02", 0.68, 80, 0.40)});

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->kind, Kind::R2);
  EXPECT_EQ(selected->target.label, "T_02");
  EXPECT_DOUBLE_EQ(selected->target.distance_m, 0.68);
}

TEST(MfPreselectionLogic, NearestKfsKeepsClosestR2AcrossMultipleR2Targets) {
  using Kind = rc26_decision::MfPreselectionLogicResult::NearestKfsKind;

  const auto selected =
      rc26_decision::MfPreselectionLogicResult::selectNearestKfsCandidate(
          {nearestCandidate(Kind::R2, "T_center_high_score", 0.95, 1, 0.99),
           nearestCandidate(Kind::R2, "T_nearest", 0.61, 160, 0.30),
           nearestCandidate(Kind::R2, "T_middle", 0.80, 20, 0.80)});

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->kind, Kind::R2);
  EXPECT_EQ(selected->target.label, "T_nearest");
  EXPECT_DOUBLE_EQ(selected->target.distance_m, 0.61);
}

TEST(MfPreselectionLogic, NearestKfsTiesByOffsetThenScore) {
  using Kind = rc26_decision::MfPreselectionLogicResult::NearestKfsKind;

  auto selected =
      rc26_decision::MfPreselectionLogicResult::selectNearestKfsCandidate(
          {nearestCandidate(Kind::R2, "T_farther_offset", 0.70, -40, 0.99),
           nearestCandidate(Kind::R1, "R_R1", 0.70, 12, 0.50)});

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->kind, Kind::R1);
  EXPECT_EQ(selected->target.label, "R_R1");

  selected =
      rc26_decision::MfPreselectionLogicResult::selectNearestKfsCandidate(
          {nearestCandidate(Kind::R2, "T_lower_score", 0.70, -12, 0.60),
           nearestCandidate(Kind::R1, "B_R1", 0.70, 12, 0.92)});

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->kind, Kind::R1);
  EXPECT_EQ(selected->target.label, "B_R1");
}

TEST(MfPreselectionLogic, DepthRoiDiagnosticReportsZeroDepthHole) {
  cv::Mat depth(5, 5, CV_16UC1, cv::Scalar(0));
  const auto diagnostic =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnostic(
          depth, 2, 2, diagnosticDepthConfig());

  EXPECT_FALSE(diagnostic.sampled);
  EXPECT_EQ(diagnostic.primary_failure, "ROI无有效原始深度");
  EXPECT_EQ(diagnostic.total_pixels, 9);
  EXPECT_EQ(diagnostic.zero_depth_count, 9);
  EXPECT_EQ(diagnostic.raw_valid_count, 0);
  const std::string detail =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnosticDetail(
          diagnostic);
  EXPECT_NE(detail.find("深度失败主因=ROI无有效原始深度"),
            std::string::npos);
  EXPECT_NE(detail.find("深度采样点=(2,2)"), std::string::npos);
  EXPECT_NE(detail.find("ROI=3"), std::string::npos);
  EXPECT_NE(detail.find("窗口内有效点=0"), std::string::npos);
}

TEST(MfPreselectionLogic, DepthRoiDiagnosticReportsDepthAboveWindow) {
  cv::Mat depth(5, 5, CV_16UC1, cv::Scalar(1500));
  const auto diagnostic =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnostic(
          depth, 2, 2, diagnosticDepthConfig());

  EXPECT_FALSE(diagnostic.sampled);
  EXPECT_EQ(diagnostic.primary_failure, "有效深度高于窗口");
  EXPECT_EQ(diagnostic.raw_valid_count, 9);
  EXPECT_EQ(diagnostic.above_max_count, 9);
  EXPECT_DOUBLE_EQ(diagnostic.raw_median_m, 1.5);
  const std::string detail =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnosticDetail(
          diagnostic);
  EXPECT_NE(detail.find("raw_median=1.5"), std::string::npos);
}

TEST(MfPreselectionLogic, DepthRoiDiagnosticReportsDepthBelowWindow) {
  cv::Mat depth(5, 5, CV_16UC1, cv::Scalar(200));
  const auto diagnostic =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnostic(
          depth, 2, 2, diagnosticDepthConfig());

  EXPECT_FALSE(diagnostic.sampled);
  EXPECT_EQ(diagnostic.primary_failure, "有效深度低于窗口");
  EXPECT_EQ(diagnostic.raw_valid_count, 9);
  EXPECT_EQ(diagnostic.below_min_count, 9);
}

TEST(MfPreselectionLogic, DepthRoiDiagnosticReportsInsufficientWindowSamples) {
  cv::Mat depth(5, 5, CV_16UC1, cv::Scalar(0));
  depth.at<uint16_t>(2, 2) = 600;
  depth.at<uint16_t>(2, 3) = 610;
  depth.at<uint16_t>(3, 2) = 620;
  const auto diagnostic =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnostic(
          depth, 2, 2, diagnosticDepthConfig());

  EXPECT_FALSE(diagnostic.sampled);
  EXPECT_EQ(diagnostic.primary_failure, "窗口内有效点不足");
  EXPECT_EQ(diagnostic.raw_valid_count, 3);
  EXPECT_EQ(diagnostic.window_valid_count, 3);
  EXPECT_EQ(diagnostic.zero_depth_count, 6);
}

TEST(MfPreselectionLogic, DepthRoiDiagnosticReportsUnsupportedType) {
  cv::Mat depth(5, 5, CV_8UC1, cv::Scalar(20));
  const auto diagnostic =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnostic(
          depth, 2, 2, diagnosticDepthConfig());

  EXPECT_FALSE(diagnostic.sampled);
  EXPECT_TRUE(diagnostic.unsupported_type);
  EXPECT_EQ(diagnostic.primary_failure, "深度类型不支持");
  const std::string detail =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnosticDetail(
          diagnostic);
  EXPECT_NE(detail.find("深度类型=CV_8UC1"), std::string::npos);
}

TEST(MfPreselectionLogic, DepthRoiDiagnosticReportsSuccessfulSample) {
  cv::Mat depth(5, 5, CV_16UC1, cv::Scalar(600));
  const auto diagnostic =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnostic(
          depth, 2, 2, diagnosticDepthConfig());

  EXPECT_TRUE(diagnostic.sampled);
  EXPECT_TRUE(diagnostic.primary_failure.empty());
  EXPECT_EQ(diagnostic.window_valid_count, 9);
  EXPECT_DOUBLE_EQ(diagnostic.sampled_depth_m, 0.6);
  const std::string detail =
      rc26_decision::MfPreselectionLogicResult::depthRoiDiagnosticDetail(
          diagnostic);
  EXPECT_NE(detail.find("深度失败主因=无"), std::string::npos);
  EXPECT_NE(detail.find("sampled_depth=0.6"), std::string::npos);
}

TEST(MfPreselectionLogic, KfsBboxDepthSampleUsesNonCenterRoi) {
  cv::Mat depth(100, 100, CV_16UC1, cv::Scalar(0));
  for (int y = 32; y <= 38; ++y) {
    for (int x = 32; x <= 38; ++x) {
      depth.at<uint16_t>(y, x) = 700;
    }
  }

  const auto sample =
      rc26_decision::MfPreselectionLogicResult::sampleKfsDepthFromBbox(
          depth, 20.0, 20.0, 80.0, 80.0, kfsDepthConfig(),
          {0.25, 0.50, 0.75}, 1);

  EXPECT_TRUE(sample.has_depth);
  EXPECT_DOUBLE_EQ(sample.depth_m, 0.7);
  EXPECT_EQ(sample.source,
            rc26_decision::MfPreselectionLogicResult::KfsDepthSource::
                BboxMultiRoi);
  EXPECT_EQ(sample.sample_point_count, 9);
  EXPECT_EQ(sample.success_count, 1);
  EXPECT_NE(sample.detail.find("bbox采样成功数=1"), std::string::npos);
}

TEST(MfPreselectionLogic, KfsBboxDepthSampleReportsRepresentativeFailure) {
  cv::Mat depth(100, 100, CV_16UC1, cv::Scalar(0));

  const auto sample =
      rc26_decision::MfPreselectionLogicResult::sampleKfsDepthFromBbox(
          depth, 20.0, 20.0, 80.0, 80.0, kfsDepthConfig(),
          {0.25, 0.50, 0.75}, 1);

  EXPECT_FALSE(sample.has_depth);
  EXPECT_EQ(sample.sample_point_count, 9);
  EXPECT_EQ(sample.success_count, 0);
  EXPECT_EQ(sample.representative_failure.primary_failure,
            "ROI无有效原始深度");
  EXPECT_NE(sample.detail.find("bbox采样点数=9"), std::string::npos);
  EXPECT_NE(sample.detail.find("深度失败主因=ROI无有效原始深度"),
            std::string::npos);
}

TEST(MfPreselectionLogic, KfsBboxDepthSampleRequiresConfiguredSuccessCount) {
  cv::Mat depth(100, 100, CV_16UC1, cv::Scalar(0));
  for (int y = 32; y <= 38; ++y) {
    for (int x = 32; x <= 38; ++x) {
      depth.at<uint16_t>(y, x) = 700;
    }
  }

  const auto sample =
      rc26_decision::MfPreselectionLogicResult::sampleKfsDepthFromBbox(
          depth, 20.0, 20.0, 80.0, 80.0, kfsDepthConfig(),
          {0.25, 0.50, 0.75}, 2);

  EXPECT_FALSE(sample.has_depth);
  EXPECT_EQ(sample.sample_point_count, 9);
  EXPECT_EQ(sample.success_count, 1);
  EXPECT_NE(sample.detail.find("bbox最少成功点=2"), std::string::npos);
  EXPECT_NE(sample.detail.find("成功点未达阈值"), std::string::npos);
}

TEST(MfPreselectionLogic, KfsBboxDepthSampleUsesConfiguredRatios) {
  cv::Mat depth(100, 100, CV_16UC1, cv::Scalar(0));
  for (int y = 48; y <= 52; ++y) {
    for (int x = 48; x <= 52; ++x) {
      depth.at<uint16_t>(y, x) = 650;
    }
  }

  const auto sample =
      rc26_decision::MfPreselectionLogicResult::sampleKfsDepthFromBbox(
          depth, 20.0, 20.0, 80.0, 80.0, kfsDepthConfig(), {0.50}, 1);

  EXPECT_TRUE(sample.has_depth);
  EXPECT_DOUBLE_EQ(sample.depth_m, 0.65);
  EXPECT_EQ(sample.sample_point_count, 1);
  EXPECT_EQ(sample.success_count, 1);
  EXPECT_NE(sample.detail.find("bbox采样比例数=1"), std::string::npos);
}

TEST(MfPreselectionLogic, MonocularDepthFallbackUsesConservativeCubeEstimate) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_mono_distance_fallback_enable = true;
  params.kfs_mono_target_width_m = 0.35;
  params.kfs_mono_target_height_m = 0.35;
  params.kfs_mono_fx_px = 385.83319091796875;
  params.kfs_mono_fy_px = 385.83319091796875;
  params.kfs_mono_min_bbox_px = 40;
  params.kfs_mono_max_delta_from_locked_m = 0.25;
  const double bbox_px = params.kfs_mono_fx_px * 0.35;

  const auto estimate =
      rc26_decision::MfPreselectionLogicResult::estimateKfsMonocularDepth(
          bbox_px, bbox_px, 0.98, params, 0.35, 1.40);

  EXPECT_TRUE(estimate.usable);
  EXPECT_NEAR(estimate.depth_m, 1.0, 1e-9);
  EXPECT_NEAR(estimate.z_width_m, 1.0, 1e-9);
  EXPECT_NEAR(estimate.z_height_m, 1.0, 1e-9);
  EXPECT_NE(estimate.detail.find("尺寸估距可用=是"), std::string::npos);
}

TEST(MfPreselectionLogic, MonocularDepthFallbackRejectsUnsafeInputs) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_mono_distance_fallback_enable = true;
  params.kfs_mono_target_width_m = 0.35;
  params.kfs_mono_target_height_m = 0.35;
  params.kfs_mono_fx_px = 385.83319091796875;
  params.kfs_mono_fy_px = 385.83319091796875;
  params.kfs_mono_min_bbox_px = 40;
  params.kfs_mono_max_delta_from_locked_m = 0.25;
  const double bbox_px = params.kfs_mono_fx_px * 0.35;

  auto estimate =
      rc26_decision::MfPreselectionLogicResult::estimateKfsMonocularDepth(
          bbox_px, bbox_px, 0.0, params, 0.35, 1.40);
  EXPECT_FALSE(estimate.usable);
  EXPECT_EQ(estimate.reject_reason, "无真实锁定深度");

  estimate =
      rc26_decision::MfPreselectionLogicResult::estimateKfsMonocularDepth(
          30.0, bbox_px, 1.0, params, 0.35, 1.40);
  EXPECT_FALSE(estimate.usable);
  EXPECT_EQ(estimate.reject_reason, "bbox尺寸过小");

  estimate =
      rc26_decision::MfPreselectionLogicResult::estimateKfsMonocularDepth(
          bbox_px, bbox_px, 0.50, params, 0.35, 1.40);
  EXPECT_FALSE(estimate.usable);
  EXPECT_EQ(estimate.reject_reason, "尺寸估距偏离真实锁定深度过大");

  estimate =
      rc26_decision::MfPreselectionLogicResult::estimateKfsMonocularDepth(
          bbox_px, bbox_px, 1.0, params, 0.35, 0.80);
  EXPECT_FALSE(estimate.usable);
  EXPECT_EQ(estimate.reject_reason, "尺寸估距超出深度窗口");
}

TEST(MfPreselectionLogic, EntryInterruptWaitsForCenteredTarget) {
  rc26_decision::MfPreselectionParams params;
  params.entry_interrupt_max_offset_px = 180;

  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          0, params));
  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          -180, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          -181, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          240, params));
}

TEST(MfPreselectionLogic, EntryInterruptCompensatesMcuSineStopModel) {
  rc26_decision::MfPreselectionParams params;
  params.entry_interrupt_max_offset_px = 170;
  params.entry_interrupt_dynamic_comp_enable = true;
  params.entry_interrupt_latency_s = 0.15;
  params.entry_interrupt_fx_px = 450.0;
  params.entry_interrupt_extra_px_min = 20;
  params.entry_interrupt_extra_px_max = 80;
  params.entry_mcu_vy_acc_mps2 = 1.0;

  EXPECT_NEAR(
      rc26_decision::MfPreselectionLogicResult::mcuSineStopTime(0.30, 1.0),
      M_PI * 0.30 / 2.0, 1e-9);
  EXPECT_NEAR(
      rc26_decision::MfPreselectionLogicResult::mcuSineStopDistance(0.30, 1.0),
      M_PI * 0.30 * 0.30 / 4.0, 1e-9);

  const int extra =
      rc26_decision::MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
          0.30, 1.0, params);
  EXPECT_EQ(extra, 52);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryInterruptEffectiveOffsetLimitPx(0.30, 1.0, params),
      222);
  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          222, 0.30, 1.0, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          223, 0.30, 1.0, params));
}

TEST(MfPreselectionLogic, EntryInterruptCompensationClampsAndCanDisable) {
  rc26_decision::MfPreselectionParams params;
  params.entry_interrupt_max_offset_px = 170;
  params.entry_interrupt_dynamic_comp_enable = true;
  params.entry_interrupt_latency_s = 0.15;
  params.entry_interrupt_fx_px = 450.0;
  params.entry_interrupt_extra_px_min = 20;
  params.entry_interrupt_extra_px_max = 80;
  params.entry_mcu_vy_acc_mps2 = 1.0;

  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
          0.30, 10.0, params),
      20);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
          0.80, 0.30, params),
      80);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
          0.30, 0.0, params),
      0);

  params.entry_interrupt_dynamic_comp_enable = false;
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryInterruptEffectiveOffsetLimitPx(0.30, 1.0, params),
      170);
}

TEST(MfPreselectionLogic, EntryMcuStopSettleUsesStopTimeMarginAndCap) {
  rc26_decision::MfPreselectionParams params;
  params.entry_mcu_stop_settle_enable = true;
  params.entry_mcu_vy_acc_mps2 = 1.0;
  params.entry_mcu_stop_margin_s = 0.08;
  params.entry_mcu_stop_max_wait_s = 0.70;

  EXPECT_NEAR(
      rc26_decision::MfPreselectionLogicResult::entryMcuStopSettleDuration(
          0.30, params),
      M_PI * 0.30 / 2.0 + 0.08, 1e-9);

  params.entry_mcu_stop_max_wait_s = 0.40;
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::entryMcuStopSettleDuration(
          0.30, params),
      0.40);

  params.entry_mcu_stop_settle_enable = false;
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::entryMcuStopSettleDuration(
          0.30, params),
      0.0);
}

TEST(MfPreselectionLogic,
     EntryReturnToCenterCompensatesMcuStopTailAndLatency) {
  rc26_decision::MfPreselectionParams params;
  params.entry_mcu_vy_acc_mps2 = 1.0;
  params.entry_interrupt_latency_s = 0.15;

  double compensation_m = 0.0;
  const double distance_m =
      rc26_decision::MfPreselectionLogicResult::
          entryReturnToCenterCompensatedDistance(1.369, 0.30, params,
                                                 compensation_m);
  const double expected_compensation = M_PI * 0.30 * 0.30 / 4.0 + 0.30 * 0.15;

  EXPECT_NEAR(compensation_m, expected_compensation, 1e-9);
  EXPECT_NEAR(distance_m, 1.369 - expected_compensation, 1e-9);

  compensation_m = 0.0;
  const double reverse_distance_m =
      rc26_decision::MfPreselectionLogicResult::
          entryReturnToCenterCompensatedDistance(1.369, -0.30, params,
                                                 compensation_m);
  EXPECT_NEAR(compensation_m, expected_compensation, 1e-9);
  EXPECT_NEAR(reverse_distance_m, distance_m, 1e-9);
}

TEST(MfPreselectionLogic, EntryReturnToCenterCompensationClampsDistance) {
  rc26_decision::MfPreselectionParams params;
  params.entry_mcu_vy_acc_mps2 = 1.0;
  params.entry_interrupt_latency_s = 0.15;

  double compensation_m = 0.0;
  const double distance_m =
      rc26_decision::MfPreselectionLogicResult::
          entryReturnToCenterCompensatedDistance(0.05, 0.30, params,
                                                 compensation_m);

  EXPECT_GT(compensation_m, 0.05);
  EXPECT_DOUBLE_EQ(distance_m, 0.0);
}

TEST(MfPreselectionLogic,
     EntryReturnToCenterCompensationFallsBackToLatencyWhenAccInvalid) {
  rc26_decision::MfPreselectionParams params;
  params.entry_mcu_vy_acc_mps2 = 0.0;
  params.entry_interrupt_latency_s = 0.15;

  double compensation_m = 0.0;
  const double distance_m =
      rc26_decision::MfPreselectionLogicResult::
          entryReturnToCenterCompensatedDistance(1.369, 0.30, params,
                                                 compensation_m);

  EXPECT_NEAR(compensation_m, 0.30 * 0.15, 1e-9);
  EXPECT_NEAR(distance_m, 1.369 - 0.30 * 0.15, 1e-9);

  compensation_m = 1.0;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double invalid_speed_distance_m =
      rc26_decision::MfPreselectionLogicResult::
          entryReturnToCenterCompensatedDistance(1.369, nan, params,
                                                 compensation_m);
  EXPECT_DOUBLE_EQ(compensation_m, 0.0);
  EXPECT_DOUBLE_EQ(invalid_speed_distance_m, 1.369);
}

TEST(MfPreselectionLogic, AlignTimeoutPickupRequiresDepthAndLooseTolerance) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_align_tolerance_px = 5;
  params.kfs_align_timeout_pickup_tolerance_px = 40;

  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
          29, true, params));
  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
          -40, true, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
          -41, true, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
          10, false, params));
}

TEST(MfPreselectionLogic, KfsAlignmentConfigUsesTipAlignmentVelocity) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_align_tolerance_px = 20;
  params.kfs_align_max_jump_px = 80;
  params.kfs_lost_stop_frames = 4;
  params.kfs_align_kp = 0.001;
  params.kfs_align_min_speed_mps = 0.015;
  params.kfs_align_max_speed_mps = 0.06;
  params.kfs_invert_lateral_direction = false;
  params.heading_kp = 1.3;
  params.heading_max_speed_radps = 0.25;
  params.kfs_odom_yaw_tolerance_deg = 5.0;
  params.kfs_align_heading_gate_deg = 9.0;

  auto config =
      rc26_decision::MfPreselectionLogicResult::kfsAlignmentConfig(params, 1.2);

  EXPECT_TRUE(config.target_lock_enable);
  EXPECT_EQ(config.target_lock_max_jump_px, 80);
  EXPECT_EQ(config.lost_stop_frames, 4);
  EXPECT_EQ(config.tolerance_px, 20);
  EXPECT_DOUBLE_EQ(config.kp, 0.001);
  EXPECT_DOUBLE_EQ(config.min_speed_mps, 0.015);
  EXPECT_DOUBLE_EQ(config.max_speed_mps, 0.06);
  EXPECT_FALSE(config.invert_direction);
  EXPECT_TRUE(config.heading_hold_enable);
  EXPECT_DOUBLE_EQ(config.target_yaw_rad, 1.2);
  EXPECT_DOUBLE_EQ(config.heading_kp, 1.3);
  EXPECT_DOUBLE_EQ(config.heading_max_speed_radps, 0.25);
  EXPECT_NEAR(config.heading_tolerance_rad, 5.0 * M_PI / 180.0, 1e-9);
  EXPECT_NEAR(config.heading_gate_rad, 9.0 * M_PI / 180.0, 1e-9);

  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(10, config), 0.0);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(30, config), -0.03);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(-30, config), 0.03);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(200, config), -0.06);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(21, config), -0.021);

  params.kfs_align_kp = 0.0001;
  config =
      rc26_decision::MfPreselectionLogicResult::kfsAlignmentConfig(params, 1.2);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(30, config), -0.015);

  params.kfs_invert_lateral_direction = true;
  config =
      rc26_decision::MfPreselectionLogicResult::kfsAlignmentConfig(params, 1.2);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(30, config), 0.015);
}

TEST(MfPreselectionLogic, KfsApproachPlanUsesArmReachAndXAxisSign) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_grab_distance_m = 0.40;
  params.kfs_approach_x_sign = 1;

  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDistance(
                       0.55, 0.40),
                   0.15);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(
                       0.15, 0.05),
                   3.0);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsApproachOdomDistance(0.55,
                                                                        params),
      0.15);

  params.kfs_approach_x_sign = -1;
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsApproachOdomDistance(0.55,
                                                                        params),
      -0.15);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDistance(
                       0.35, 0.40),
                   0.0);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsApproachOdomDistance(0.35,
                                                                        params),
      -0.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(
                       0.0, 0.0),
                   0.0);
  EXPECT_TRUE(std::isinf(
      rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(0.10, 0.0)));
  EXPECT_GT(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(
                0.90, 0.10),
            8.0);
}

TEST(MfPreselectionLogic, KfsOdomParamsNormalizeInvalidValues) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_align_tolerance_px = -10;
  params.kfs_align_target_line_offset_px = -120;
  params.kfs_align_max_jump_px = -1;
  params.kfs_odom_xy_kp = -1.0;
  params.kfs_approach_odom_tolerance_m =
      std::numeric_limits<double>::quiet_NaN();
  params.kfs_odom_yaw_tolerance_deg =
      std::numeric_limits<double>::quiet_NaN();
  params.kfs_odom_stable_ticks = 0;
  params.kfs_approach_speed_mps = 0.02;
  params.kfs_approach_min_speed_mps = 0.05;

  rc26_decision::MfPreselectionLogicResult::normalizeKfsOdomParams(params);

  EXPECT_EQ(params.kfs_align_tolerance_px, 0);
  EXPECT_EQ(params.kfs_align_target_line_offset_px, -120);
  EXPECT_EQ(params.kfs_align_max_jump_px, 0);
  EXPECT_DOUBLE_EQ(params.kfs_odom_xy_kp, 0.8);
  EXPECT_DOUBLE_EQ(params.kfs_approach_odom_tolerance_m, 0.02);
  EXPECT_DOUBLE_EQ(params.kfs_odom_yaw_tolerance_deg, 3.0);
  EXPECT_EQ(params.kfs_odom_stable_ticks, 1);
  EXPECT_DOUBLE_EQ(params.kfs_approach_min_speed_mps, 0.02);

  params.kfs_align_target_line_offset_px = 20000;
  rc26_decision::MfPreselectionLogicResult::normalizeKfsOdomParams(params);
  EXPECT_EQ(params.kfs_align_target_line_offset_px, 10000);
}

TEST(MfPreselectionLogic, FakeAvoidanceDirectionUsesPickupSource) {
  rc26_decision::MfPreselectionParams params;
  params.stair1_direction_yaw_rad = 1.0;
  params.stair3_direction_yaw_rad = -1.0;

  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceYaw(
                       rc26_decision::MfPreselectionPickupSource::Stair1,
                       params),
                   1.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceYaw(
                       rc26_decision::MfPreselectionPickupSource::Stair2,
                       params),
                   1.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceYaw(
                       rc26_decision::MfPreselectionPickupSource::None, params),
                   1.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceYaw(
      rc26_decision::MfPreselectionPickupSource::Stair3,
      params),
                   -1.0);
}

TEST(MfPreselectionLogic, FakeAvoidanceTargetGridUsesSideColumns) {
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                2, rc26_decision::MfPreselectionPickupSource::Stair1),
            1);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                5, rc26_decision::MfPreselectionPickupSource::Stair1),
            4);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                8, rc26_decision::MfPreselectionPickupSource::Stair1),
            7);

  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                2, rc26_decision::MfPreselectionPickupSource::Stair3),
            3);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                5, rc26_decision::MfPreselectionPickupSource::Stair3),
            6);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                8, rc26_decision::MfPreselectionPickupSource::Stair3),
            9);

  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          0, rc26_decision::MfPreselectionPickupSource::Stair1)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          1, rc26_decision::MfPreselectionPickupSource::Stair1)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          3, rc26_decision::MfPreselectionPickupSource::Stair3)
          .has_value());
}

TEST(MfPreselectionLogic, FakeAvoidanceTargetGridMirrorsForBlueSide) {
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                5, rc26_decision::MfPreselectionPickupSource::Stair1, 1),
            4);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                5, rc26_decision::MfPreselectionPickupSource::Stair3, 1),
            6);

  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                5, rc26_decision::MfPreselectionPickupSource::Stair1, -1),
            6);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                5, rc26_decision::MfPreselectionPickupSource::Stair3, -1),
            4);
}

TEST(MfPreselectionLogic, FakeAvoidanceTargetGridCanBeClimbOrDescendByMap) {
  rc26_decision::MerlinMapManager red_map;
  ASSERT_TRUE(red_map.initRedMap());
  rc26_decision::MerlinMapManager blue_map;
  ASSERT_TRUE(blue_map.initBlueMap());

  const auto from_grid5_to_stair1 =
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          5, rc26_decision::MfPreselectionPickupSource::Stair1);
  ASSERT_TRUE(from_grid5_to_stair1.has_value());
  EXPECT_EQ(*from_grid5_to_stair1, 4);
  EXPECT_EQ(red_map.getDepth(*from_grid5_to_stair1) - red_map.getDepth(5), 1);
  EXPECT_EQ(blue_map.getDepth(*from_grid5_to_stair1) - blue_map.getDepth(5), -1);

  const auto from_grid5_to_stair3 =
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          5, rc26_decision::MfPreselectionPickupSource::Stair3);
  ASSERT_TRUE(from_grid5_to_stair3.has_value());
  EXPECT_EQ(*from_grid5_to_stair3, 6);
  EXPECT_EQ(red_map.getDepth(*from_grid5_to_stair3) - red_map.getDepth(5), -1);
  EXPECT_EQ(blue_map.getDepth(*from_grid5_to_stair3) - blue_map.getDepth(5), 1);
}

TEST(MfPreselectionLogic, FakeAvoidanceForwardTargetGridUsesSideColumns) {
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(1),
      4);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(4),
      7);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(7),
      10);

  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(3),
      6);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(6),
      9);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(9),
      12);

  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(0)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(2)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(5)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(8)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(10)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(12)
          .has_value());
}

TEST(MfPreselectionLogic, EntryPickupSourceUsesLateralOffset) {
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(0.20, 0.03),
      rc26_decision::MfPreselectionPickupSource::Stair1);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(-0.20, 0.03),
      rc26_decision::MfPreselectionPickupSource::Stair3);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(0.02, 0.03),
      rc26_decision::MfPreselectionPickupSource::Stair2);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(
              std::numeric_limits<double>::quiet_NaN(), 0.03),
      rc26_decision::MfPreselectionPickupSource::Stair2);
}

TEST(MfPreselectionLogic, EntryPickupSourceMirrorsForBlueSide) {
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(0.20, 0.03, 1),
      rc26_decision::MfPreselectionPickupSource::Stair1);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(-0.20, 0.03, 1),
      rc26_decision::MfPreselectionPickupSource::Stair3);

  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(-0.20, 0.03, -1),
      rc26_decision::MfPreselectionPickupSource::Stair1);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(0.20, 0.03, -1),
      rc26_decision::MfPreselectionPickupSource::Stair3);
}

TEST(MfPreselectionLogic, EntryReturnToCenterCommandUsesOffsetSign) {
  double vy = 0.0;
  double distance_m = 0.0;

  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::
                  entryReturnToCenterCommand(0.30, 0.03, 0.20, vy,
                                             distance_m));
  EXPECT_DOUBLE_EQ(vy, -0.20);
  EXPECT_DOUBLE_EQ(distance_m, 0.30);

  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::
                  entryReturnToCenterCommand(-0.40, 0.03, 0.20, vy,
                                             distance_m));
  EXPECT_DOUBLE_EQ(vy, 0.20);
  EXPECT_DOUBLE_EQ(distance_m, 0.40);

  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::
                  entryReturnToCenterCommand(0.02, 0.03, 0.20, vy,
                                             distance_m));
  EXPECT_DOUBLE_EQ(vy, 0.0);
  EXPECT_DOUBLE_EQ(distance_m, 0.0);

  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::
                   entryReturnToCenterCommand(
                       std::numeric_limits<double>::quiet_NaN(), 0.03, 0.20,
                       vy, distance_m));
}

TEST(MfPreselectionLogic, FinalExitCenterTargetUsesForwardEntryHeading) {
  constexpr double kHalfPi = 1.57079632679489661923;
  double target_x = 0.0;
  double target_y = 0.0;
  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::finalExitCenterTarget(
      10.0, 20.0, 0.0, 1.2, target_x, target_y));
  EXPECT_NEAR(target_x, 11.2, 1e-9);
  EXPECT_NEAR(target_y, 20.0, 1e-9);

  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::finalExitCenterTarget(
      10.0, 20.0, -kHalfPi, -2.0, target_x, target_y));
  EXPECT_NEAR(target_x, 10.0, 1e-9);
  EXPECT_NEAR(target_y, 18.0, 1e-9);

  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::finalExitCenterTarget(
      std::numeric_limits<double>::quiet_NaN(), 20.0, 0.0, 1.2, target_x,
      target_y));
}

TEST(MfPreselectionLogic, GrabCommandFollowsHighSide) {
  rc26_decision::MfPreselectionParams params;
  params.grab_kfs_up_command_id =
      static_cast<int>(rc26_serial::CommandID::GRAB_KFS_UP);
  params.grab_kfs_down_command_id =
      static_cast<int>(rc26_serial::CommandID::GRAB_KFS_DOWN);
  params.entry_grab_kfs_up_command_id =
      static_cast<int>(rc26_serial::CommandID::ENTRY_GRAB_KFS_UP);
  params.entry_grab_kfs_up_done_feedback_id =
      static_cast<int>(rc26_serial::FeedbackID::ENTRY_GRAB_KFS_UP_DONE);

  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForHighSide(
                true, params),
            static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_UP));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForHighSide(
                false, params),
            static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_DOWN));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForPickup(
                true, rc26_decision::MfPreselectionPickupSource::None, false,
                params),
            static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_UP));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabDoneFeedbackForPickup(
                true, rc26_decision::MfPreselectionPickupSource::None, false,
                params),
            -1);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForPickup(
                true, rc26_decision::MfPreselectionPickupSource::Stair2, false,
                params),
            static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_UP));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabDoneFeedbackForPickup(
                true, rc26_decision::MfPreselectionPickupSource::Stair2, false,
                params),
            -1);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForPickup(
                false, rc26_decision::MfPreselectionPickupSource::Stair1, true,
                params),
            static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_DOWN));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabDoneFeedbackForPickup(
                false, rc26_decision::MfPreselectionPickupSource::Stair1, true,
                params),
            -1);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForPickup(
                true, rc26_decision::MfPreselectionPickupSource::Stair1, true,
                params),
            static_cast<uint8_t>(rc26_serial::CommandID::ENTRY_GRAB_KFS_UP));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabDoneFeedbackForPickup(
                true, rc26_decision::MfPreselectionPickupSource::Stair1, true,
                params),
            static_cast<int>(rc26_serial::FeedbackID::ENTRY_GRAB_KFS_UP_DONE));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForPickup(
                true, rc26_decision::MfPreselectionPickupSource::None, true,
                params),
            static_cast<uint8_t>(rc26_serial::CommandID::ENTRY_GRAB_KFS_UP));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabDoneFeedbackForPickup(
                true, rc26_decision::MfPreselectionPickupSource::None, true,
                params),
            static_cast<int>(rc26_serial::FeedbackID::ENTRY_GRAB_KFS_UP_DONE));
}

TEST(MfPreselectionLogic, PostGrabCenterAlignOnlyAppliesInsideMerlin) {
  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::None, false));

  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::None, true));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::Stair1, false));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::Stair2, false));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::Stair3, false));
}

TEST(MfPreselectionLogic, BboxIouAndSameTargetUseLabelAndOverlap) {
  rc26_decision::MfPreselectionTargetSnapshot reference;
  reference.label = "T_03";
  reference.x1 = 10.0;
  reference.y1 = 10.0;
  reference.x2 = 50.0;
  reference.y2 = 50.0;

  rc26_decision::MfPreselectionTargetSnapshot close = reference;
  close.x1 = 15.0;
  close.y1 = 15.0;
  close.x2 = 55.0;
  close.y2 = 55.0;

  rc26_decision::MfPreselectionTargetSnapshot far = reference;
  far.x1 = 80.0;
  far.y1 = 80.0;
  far.x2 = 120.0;
  far.y2 = 120.0;

  rc26_decision::MfPreselectionTargetSnapshot wrong_label = close;
  wrong_label.label = "T_04";

  EXPECT_GT(rc26_decision::MfPreselectionLogicResult::bboxIou(reference, close),
            0.30);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::bboxIou(reference, far), 0.0);
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::isSameVisualTarget(
      reference, close, 0.30));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::isSameVisualTarget(
      reference, far, 0.30));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::isSameVisualTarget(
      reference, wrong_label, 0.30));
}

TEST(MfPreselectionLogic, IgnoredTargetUsesSameVisualTargetRule) {
  rc26_decision::MfPreselectionTargetSnapshot ignored;
  ignored.label = "T_03";
  ignored.x1 = 10.0;
  ignored.y1 = 10.0;
  ignored.x2 = 50.0;
  ignored.y2 = 50.0;
  const std::vector<rc26_decision::MfPreselectionTargetSnapshot> ignored_targets{
      ignored};

  rc26_decision::MfPreselectionTargetSnapshot same = ignored;
  same.x1 = 14.0;
  same.y1 = 14.0;
  same.x2 = 54.0;
  same.y2 = 54.0;

  rc26_decision::MfPreselectionTargetSnapshot another_same_label = ignored;
  another_same_label.x1 = 120.0;
  another_same_label.y1 = 120.0;
  another_same_label.x2 = 160.0;
  another_same_label.y2 = 160.0;

  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::isIgnoredTarget(
      same, ignored_targets, 0.30));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::isIgnoredTarget(
      another_same_label, ignored_targets, 0.30));
}

} // namespace
