#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "gtest/gtest.h"
#include "rc26_terrain/terrain_risk_model.hpp"

namespace {

std::string writeTempModelFile(const std::string& content) {
    char path_template[] = "/tmp/terrain_risk_model_XXXXXX.yaml";
    const int fd = mkstemps(path_template, 5);
    if (fd < 0) {
        throw std::runtime_error("mkstemps failed");
    }
    close(fd);
    std::ofstream ofs(path_template);
    ofs << content;
    ofs.close();
    return std::string(path_template);
}

}  // namespace

TEST(TerrainRiskModelTest, FallsBackWhenModelDisabled) {
    rc26_terrain::TerrainRiskModel model;
    std::string error;
    const std::string path = writeTempModelFile(
        "terrain_risk_model:\n"
        "  enabled: false\n"
        "  obstacle:\n"
        "    enabled: true\n"
        "    intercept: 0.0\n"
        "    coefficients: {obstacle_proxy: 4.0}\n"
        "  drop:\n"
        "    enabled: true\n"
        "    intercept: 0.0\n"
        "    coefficients: {drop_proxy: 4.0}\n");
    ASSERT_TRUE(model.loadFromFile(path, error));
    EXPECT_FALSE(model.enabled());

    rc26_terrain::TerrainRiskFeatures f;
    f.obstacle_proxy = 0.3f;
    f.drop_proxy = 0.8f;
    EXPECT_NEAR(model.predictObstacle(f, 0.3f), 0.3f, 1e-6f);
    EXPECT_NEAR(model.predictDrop(f, 0.8f), 0.8f, 1e-6f);
    std::remove(path.c_str());
}

TEST(TerrainRiskModelTest, PredictsFromLogisticCoefficients) {
    rc26_terrain::TerrainRiskModel model;
    std::string error;
    const std::string path = writeTempModelFile(
        "terrain_risk_model:\n"
        "  enabled: true\n"
        "  obstacle:\n"
        "    enabled: true\n"
        "    intercept: -2.0\n"
        "    coefficients: {obstacle_proxy: 6.0}\n"
        "  drop:\n"
        "    enabled: true\n"
        "    intercept: -2.0\n"
        "    coefficients: {drop_proxy: 6.0}\n");
    ASSERT_TRUE(model.loadFromFile(path, error));
    ASSERT_TRUE(model.enabled());

    rc26_terrain::TerrainRiskFeatures f_low;
    f_low.obstacle_proxy = 0.1f;
    f_low.drop_proxy = 0.1f;

    rc26_terrain::TerrainRiskFeatures f_high;
    f_high.obstacle_proxy = 0.9f;
    f_high.drop_proxy = 0.9f;

    EXPECT_LT(model.predictObstacle(f_low, 0.5f), 0.5f);
    EXPECT_GT(model.predictObstacle(f_high, 0.5f), 0.5f);
    EXPECT_LT(model.predictDrop(f_low, 0.5f), 0.5f);
    EXPECT_GT(model.predictDrop(f_high, 0.5f), 0.5f);
    std::remove(path.c_str());
}
