#include <gtest/gtest.h>

#include <filesystem>

#include <behaviortree_cpp/bt_factory.h>
#include <opencv2/imgproc.hpp>

#include "rc26_decision/navigation/bt_odom_relative_nav.hpp"
#include "rc26_decision/second_preselection/second_preselection.hpp"

namespace {

cv::Mat makeFrameWithHsvRect(const cv::Scalar &hsv, const cv::Rect &rect) {
  cv::Mat hsv_frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(hsv_frame, rect, hsv, cv::FILLED);
  cv::Mat bgr;
  cv::cvtColor(hsv_frame, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}

rc26_decision::SecondPreselectionParams defaultParams() {
  rc26_decision::SecondPreselectionParams params;
  params.roi_x = 200;
  params.roi_y = 140;
  params.roi_width = 240;
  params.roi_height = 200;
  params.occupied_min_area_px = 1000;
  params.red_hsv1 = {0, 10, 80, 60};
  params.red_hsv2 = {170, 180, 80, 60};
  params.blue_hsv1 = {95, 130, 80, 50};
  params.blue_hsv2 = {95, 130, 80, 50};
  return params;
}

} // namespace

TEST(SecondPreselectionLogic, RedTeamDetectsBlueOpponentOccupied) {
  const auto frame =
      makeFrameWithHsvRect(cv::Scalar(110, 220, 220), cv::Rect(260, 190, 90, 90));
  const auto observation = rc26_decision::evaluateSecondPreselectionOccupancy(
      frame, defaultParams(), "red");
  EXPECT_TRUE(observation.occupied);
  EXPECT_GE(observation.best_area_px, 1000.0);
}

TEST(SecondPreselectionLogic, BlueTeamDetectsRedOpponentOccupied) {
  const auto frame =
      makeFrameWithHsvRect(cv::Scalar(0, 220, 220), cv::Rect(260, 190, 90, 90));
  const auto observation = rc26_decision::evaluateSecondPreselectionOccupancy(
      frame, defaultParams(), "blue");
  EXPECT_TRUE(observation.occupied);
  EXPECT_GE(observation.best_area_px, 1000.0);
}

TEST(SecondPreselectionLogic, EmptyRoiIsNotOccupied) {
  const cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  const auto observation = rc26_decision::evaluateSecondPreselectionOccupancy(
      frame, defaultParams(), "red");
  EXPECT_FALSE(observation.occupied);
  EXPECT_EQ(observation.best_area_px, 0.0);
}

TEST(SecondPreselectionLogic, SmallOpponentBlobIsIgnored) {
  const auto frame =
      makeFrameWithHsvRect(cv::Scalar(110, 220, 220), cv::Rect(260, 190, 12, 12));
  const auto observation = rc26_decision::evaluateSecondPreselectionOccupancy(
      frame, defaultParams(), "red");
  EXPECT_FALSE(observation.occupied);
  EXPECT_LT(observation.best_area_px, 1000.0);
}

TEST(SecondPreselectionLogic, BehaviorTreeXmlLoadsWithRegisteredNodes) {
  BT::BehaviorTreeFactory factory;
  rc26_decision::registerSecondPreselectionNodes(factory);
  rc26_decision::registerOdomNavigationNodes(factory);

  const auto tree_path =
      std::filesystem::path(RC26_DECISION_SOURCE_DIR) / "behavior_trees" /
      "second_preselection_tree.xml";
  auto blackboard = BT::Blackboard::create();
  EXPECT_NO_THROW({
    auto tree = factory.createTreeFromFile(tree_path.string(), blackboard);
    EXPECT_TRUE(tree.rootNode() != nullptr);
  });
}
