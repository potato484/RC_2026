#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "gtest/gtest.h"

#include "rc26_xhu_nav/mode_manager/profile_loader.hpp"

namespace {

class TempYamlFile {
public:
    explicit TempYamlFile(const std::string& contents) {
        const auto unique_suffix = std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
        path_ = std::filesystem::temp_directory_path() /
                ("rc26_xhu_nav_profile_loader_" + unique_suffix + ".yaml");

        std::ofstream out(path_);
        out << contents;
        out.close();
    }

    ~TempYamlFile() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
    }

    const std::filesystem::path& path() const {
        return path_;
    }

private:
    std::filesystem::path path_;
};

TEST(ProfileLoaderTest, AcceptsCanonicalAccelerationKeys) {
    const TempYamlFile yaml_file(R"(
profiles:
  safe:
    fallback_profile: safe
    controller:
      a_linear_max: 0.40
      a_angular_max: 0.60
)");

    const auto result =
        rc26_xhu_nav::mode_manager::ProfileLoader::loadFromFile(yaml_file.path().string());

    ASSERT_TRUE(result.success) << result.error_message;
    const auto safe_it = result.profiles.find("safe");
    ASSERT_NE(safe_it, result.profiles.end());
    ASSERT_TRUE(safe_it->second.controller.acc_linear.has_value());
    ASSERT_TRUE(safe_it->second.controller.acc_angular.has_value());
    EXPECT_DOUBLE_EQ(*safe_it->second.controller.acc_linear, 0.40);
    EXPECT_DOUBLE_EQ(*safe_it->second.controller.acc_angular, 0.60);
}

TEST(ProfileLoaderTest, RejectsConflictingAccelerationAliases) {
    const TempYamlFile yaml_file(R"(
profiles:
  safe:
    fallback_profile: safe
    controller:
      acc_linear: 0.20
      a_linear_max: 0.40
)");

    const auto result =
        rc26_xhu_nav::mode_manager::ProfileLoader::loadFromFile(yaml_file.path().string());

    ASSERT_FALSE(result.success);
    EXPECT_NE(result.error_message.find("conflicting controller fields 'acc_linear' and 'a_linear_max'"),
              std::string::npos);
}

}  // namespace
