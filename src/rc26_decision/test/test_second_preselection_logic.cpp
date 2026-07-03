#include <gtest/gtest.h>

#include <array>
#include <filesystem>

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/navigation/bt_odom_relative_nav.hpp"
#include "rc26_decision/second_preselection/second_preselection.hpp"

namespace {

rc26_decision::SecondPreselectionParams defaultParams() {
  return rc26_decision::SecondPreselectionParams{};
}

rc26_vision::Detection makeDetection(const std::string &label, float x1,
                                     float y1, float x2, float y2) {
  rc26_vision::Detection detection;
  detection.x1 = x1;
  detection.y1 = y1;
  detection.x2 = x2;
  detection.y2 = y2;
  detection.score = 0.9F;
  detection.class_id = 1;
  detection.class_name = label;
  return detection;
}

} // namespace

TEST(SecondPreselectionLogic, DynamicProjectionUsesColor640Intrinsics) {
  const auto params = defaultParams();
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);

  const auto &middle_left = observation.grid_cells[3];
  const auto &middle_center = observation.grid_cells[4];
  const auto &middle_right = observation.grid_cells[5];

  EXPECT_EQ(middle_center.col, 0);
  EXPECT_EQ(middle_center.row, 0);
  EXPECT_NEAR(middle_center.center.x, params.grid_camera_ppx_px, 1.0e-3);
  EXPECT_NEAR(middle_center.center.y,
              params.grid_camera_ppy_px +
                  params.grid_camera_fy_px *
                      (params.grid_camera_height_m -
                       params.grid_middle_center_height_m) /
                      params.grid_initial_distance_m,
              1.0e-3);
  EXPECT_NEAR(middle_right.center.x - middle_center.center.x,
              params.grid_camera_fx_px *
                  ((params.grid_center_col_width_m +
                    params.grid_right_col_width_m) *
                   0.5) /
                  params.grid_initial_distance_m,
              1.0e-3);
  EXPECT_NEAR(middle_center.center.x - middle_left.center.x,
              params.grid_camera_fx_px *
                  ((params.grid_center_col_width_m +
                    params.grid_left_col_width_m) *
                   0.5) /
                  params.grid_initial_distance_m,
              1.0e-3);
  EXPECT_NEAR(middle_center.roi.width,
              params.grid_camera_fx_px * params.grid_safe_width_m /
                  params.grid_initial_distance_m,
              1.0e-3);
  EXPECT_NEAR(middle_center.roi.height,
              params.grid_camera_fy_px * params.grid_safe_height_m /
                  params.grid_initial_distance_m,
              1.0e-3);
}

TEST(SecondPreselectionLogic, DynamicCenterPointInsideCellMarksOccupied) {
  const auto params = defaultParams();
  const auto projection = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);
  const auto center = projection.grid_cells[4].center;

  const std::vector<rc26_vision::Detection> detections{makeDetection(
      "KFS_A", center.x - 10.0F, center.y - 10.0F, center.x + 10.0F,
      center.y + 10.0F)};
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      detections, params, 0.0, 0.0);

  EXPECT_FALSE(observation.occupied);
  EXPECT_EQ(observation.grid_occupied_mask, 1U << 4);
  EXPECT_EQ(observation.grid_detection_counts[4], 1);
  ASSERT_TRUE(observation.selected_middle_col.has_value());
  EXPECT_EQ(*observation.selected_middle_col, -1);
}

TEST(SecondPreselectionLogic, DynamicBboxEdgeIntersectionWithoutCenterIsIgnored) {
  const auto params = defaultParams();
  const auto projection = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);
  const auto roi = projection.grid_cells[4].roi;

  const std::vector<rc26_vision::Detection> detections{makeDetection(
      "KFS_A", roi.x - 20.0F, roi.y + 5.0F, roi.x + 4.0F,
      roi.y + 30.0F)};
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      detections, params, 0.0, 0.0);

  EXPECT_FALSE(observation.occupied);
  EXPECT_EQ(observation.grid_occupied_mask, 0U);
  EXPECT_EQ(observation.matched_detections, 0);
  ASSERT_TRUE(observation.selected_middle_col.has_value());
  EXPECT_EQ(*observation.selected_middle_col, 0);
}

TEST(SecondPreselectionLogic, DynamicMultipleLabelsAssignToDifferentCells) {
  const auto params = defaultParams();
  const auto projection = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);
  const auto left = projection.grid_cells[3].center;
  const auto upper_right = projection.grid_cells[2].center;

  const std::vector<rc26_vision::Detection> detections{
      makeDetection("KFS_LEFT", left.x - 8.0F, left.y - 8.0F, left.x + 8.0F,
                    left.y + 8.0F),
      makeDetection("KFS_UPPER_RIGHT", upper_right.x - 8.0F,
                    upper_right.y - 8.0F, upper_right.x + 8.0F,
                    upper_right.y + 8.0F)};
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      detections, params, 0.0, 0.0);

  EXPECT_EQ(observation.grid_occupied_mask, (1U << 3) | (1U << 2));
  EXPECT_EQ(observation.grid_detection_counts[3], 1);
  EXPECT_EQ(observation.grid_detection_counts[2], 1);
  EXPECT_EQ(observation.matched_detections, 2);
  ASSERT_TRUE(observation.selected_middle_col.has_value());
  EXPECT_EQ(*observation.selected_middle_col, 0);
}

TEST(SecondPreselectionLogic, DynamicEmptyClassNameDoesNotOccupyCell) {
  const auto params = defaultParams();
  const auto projection = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);
  const auto center = projection.grid_cells[4].center;

  const std::vector<rc26_vision::Detection> detections{makeDetection(
      "", center.x - 8.0F, center.y - 8.0F, center.x + 8.0F,
      center.y + 8.0F)};
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      detections, params, 0.0, 0.0);

  EXPECT_FALSE(observation.occupied);
  EXPECT_EQ(observation.grid_occupied_mask, 0U);
  EXPECT_EQ(observation.matched_detections, 0);
}

TEST(SecondPreselectionLogic, DynamicMiddleSelectionPrefersCenterColumn) {
  const auto params = defaultParams();
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);

  ASSERT_TRUE(observation.selected_middle_col.has_value());
  EXPECT_EQ(*observation.selected_middle_col, 0);
  EXPECT_NEAR(observation.selected_lateral_m, 0.0, 1.0e-6);
  EXPECT_FALSE(observation.occupied);
}

TEST(SecondPreselectionLogic, DynamicMiddleSelectionUsesBasePositiveYSideNext) {
  const auto params = defaultParams();
  const auto projection = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);
  const auto center = projection.grid_cells[4].center;

  const std::vector<rc26_vision::Detection> detections{makeDetection(
      "KFS_CENTER", center.x - 8.0F, center.y - 8.0F, center.x + 8.0F,
      center.y + 8.0F)};
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      detections, params, 0.0, 0.0);

  ASSERT_TRUE(observation.selected_middle_col.has_value());
  EXPECT_EQ(*observation.selected_middle_col, -1);
  EXPECT_NEAR(observation.selected_lateral_m,
              (params.grid_center_col_width_m + params.grid_left_col_width_m) *
                  0.5,
              1.0e-6);
  EXPECT_FALSE(observation.occupied);
}

TEST(SecondPreselectionLogic, DynamicMiddleSelectionUsesBaseNegativeYSideLast) {
  const auto params = defaultParams();
  const auto projection = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);
  const auto left = projection.grid_cells[3].center;
  const auto center = projection.grid_cells[4].center;

  const std::vector<rc26_vision::Detection> detections{
      makeDetection("KFS_LEFT", left.x - 8.0F, left.y - 8.0F, left.x + 8.0F,
                    left.y + 8.0F),
      makeDetection("KFS_CENTER", center.x - 8.0F, center.y - 8.0F,
                    center.x + 8.0F, center.y + 8.0F)};
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      detections, params, 0.0, 0.0);

  ASSERT_TRUE(observation.selected_middle_col.has_value());
  EXPECT_EQ(*observation.selected_middle_col, 1);
  EXPECT_NEAR(observation.selected_lateral_m,
              -(params.grid_center_col_width_m +
                params.grid_right_col_width_m) *
                  0.5,
              1.0e-6);
  EXPECT_FALSE(observation.occupied);
}

TEST(SecondPreselectionLogic, DynamicMiddleSelectionMirrorsMovableSpaceToBlueNegativeY) {
  auto params = defaultParams();
  params.grid_base_y_to_grid_x_sign = 1.0;
  const auto projection = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);
  const auto center = projection.grid_cells[4].center;

  const std::vector<rc26_vision::Detection> detections{makeDetection(
      "KFS_CENTER", center.x - 8.0F, center.y - 8.0F, center.x + 8.0F,
      center.y + 8.0F)};
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      detections, params, 0.0, 0.0);

  ASSERT_TRUE(observation.selected_middle_col.has_value());
  EXPECT_EQ(*observation.selected_middle_col, -1);
  EXPECT_NEAR(observation.selected_lateral_m,
              -(params.grid_center_col_width_m + params.grid_left_col_width_m) *
                  0.5,
              1.0e-6);
  EXPECT_FALSE(observation.occupied);
}

TEST(SecondPreselectionLogic, DynamicMiddleSelectionReportsNoEmptyWhenFull) {
  const auto params = defaultParams();
  const auto projection = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, 0.0);

  std::vector<rc26_vision::Detection> detections;
  for (const std::size_t index : std::array<std::size_t, 3>{3, 4, 5}) {
    const auto center = projection.grid_cells[index].center;
    detections.push_back(makeDetection("KFS_FULL", center.x - 8.0F,
                                       center.y - 8.0F, center.x + 8.0F,
                                       center.y + 8.0F));
  }
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      detections, params, 0.0, 0.0);

  EXPECT_FALSE(observation.selected_middle_col.has_value());
  EXPECT_TRUE(observation.occupied);
  EXPECT_EQ(observation.grid_occupied_mask, (1U << 3) | (1U << 4) | (1U << 5));
}

TEST(SecondPreselectionLogic, DynamicSelectedLateralAccountsForOdomDeltaY) {
  const auto params = defaultParams();
  const double odom_delta_y_m = 0.20;
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, odom_delta_y_m);

  ASSERT_TRUE(observation.selected_middle_col.has_value());
  EXPECT_EQ(*observation.selected_middle_col, 0);
  EXPECT_NEAR(observation.selected_lateral_m,
              params.grid_base_y_to_grid_x_sign * odom_delta_y_m, 1.0e-6);
}

TEST(SecondPreselectionLogic, DynamicSelectedLateralMirrorsOdomDeltaY) {
  auto params = defaultParams();
  params.grid_base_y_to_grid_x_sign = 1.0;
  const double odom_delta_y_m = 0.20;
  const auto observation = rc26_decision::evaluateSecondPreselectionGridOccupancy(
      {}, params, 0.0, odom_delta_y_m);

  ASSERT_TRUE(observation.selected_middle_col.has_value());
  EXPECT_EQ(*observation.selected_middle_col, 0);
  EXPECT_NEAR(observation.selected_lateral_m, -odom_delta_y_m, 1.0e-6);
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
