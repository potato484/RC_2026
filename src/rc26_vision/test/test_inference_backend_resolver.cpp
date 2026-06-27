#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "rc26_vision/inference/runtime/backend_resolver.hpp"

namespace rc26_vision {
namespace {

ModelProfile makeProfile(EngineType engine) {
    ModelProfile profile;
    profile.id = "test_profile";
    profile.engine = engine;
    profile.model_path = "/tmp/test.onnx";
    profile.labels = {"target"};
    profile.conf_thresh = 0.25F;
    profile.iou_thresh = 0.45F;
    return profile;
}

InferenceBackendRuntimeInfo makeRuntimeInfo(bool aidlite_paths_detected,
                                            bool aidlite_compiled,
                                            bool onnxruntime_compiled) {
    InferenceBackendRuntimeInfo info;
    info.aidlite_paths_detected = aidlite_paths_detected;
    info.aidlite_compiled = aidlite_compiled;
    info.onnxruntime_compiled = onnxruntime_compiled;
    return info;
}

TEST(InferenceBackendResolverTest, AutoWithoutAidLitePathsFallsBackToOpenCvOnnxWhenCompiled) {
    const ModelProfile profile = makeProfile(EngineType::Auto);
    const InferenceBackendRuntimeInfo runtime_info = makeRuntimeInfo(false, false, true);
    const InferenceBackendSelection selection = resolveInferenceBackend(profile, runtime_info);

    EXPECT_EQ(selection.resolved_engine, EngineType::LocalOnnx);
    EXPECT_NE(selection.reason.find("未检测到 AidLux/AidLite 路径"), std::string::npos);
    EXPECT_TRUE(selection.onnxruntime_compiled);
}

TEST(InferenceBackendResolverTest, AutoWithAidLitePathsAndCompiledUsesAidLiteEvenWithoutOnnxRuntime) {
    const ModelProfile profile = makeProfile(EngineType::Auto);
    const InferenceBackendRuntimeInfo runtime_info = makeRuntimeInfo(true, true, false);

    const InferenceBackendSelection selection = resolveInferenceBackend(profile, runtime_info);
    EXPECT_EQ(selection.resolved_engine, EngineType::AidLite);
    EXPECT_NE(selection.reason.find("优先使用 AidLite 推理链"), std::string::npos);
}

TEST(InferenceBackendResolverTest, AutoWithAidLitePathsButWithoutCompiledFallsBackToOpenCvOnnx) {
    const ModelProfile profile = makeProfile(EngineType::Auto);
    const InferenceBackendRuntimeInfo runtime_info = makeRuntimeInfo(true, false, true);

    const InferenceBackendSelection selection = resolveInferenceBackend(profile, runtime_info);
    EXPECT_EQ(selection.resolved_engine, EngineType::LocalOnnx);
    EXPECT_NE(selection.reason.find("构建未启用 AidLite"), std::string::npos);
}

TEST(InferenceBackendResolverTest, AutoWithoutAnyCompiledBackendThrowsChineseError) {
    const ModelProfile profile = makeProfile(EngineType::Auto);
    const InferenceBackendRuntimeInfo runtime_info = makeRuntimeInfo(false, false, false);

    EXPECT_THROW(
        {
            try {
                (void)resolveInferenceBackend(profile, runtime_info);
            } catch (const std::runtime_error& ex) {
                EXPECT_NE(std::string(ex.what()).find("没有可用推理后端"), std::string::npos);
                throw;
            }
        },
        std::runtime_error);
}

TEST(InferenceBackendResolverTest, ExplicitOpenCvOnnxStaysOpenCvOnnx) {
    const ModelProfile profile = makeProfile(EngineType::LocalOnnx);
    const InferenceBackendRuntimeInfo runtime_info = makeRuntimeInfo(true, true, true);

    const InferenceBackendSelection selection = resolveInferenceBackend(profile, runtime_info);
    EXPECT_EQ(selection.resolved_engine, EngineType::LocalOnnx);
    EXPECT_NE(selection.reason.find("显式指定 ONNX Runtime"), std::string::npos);
}

TEST(InferenceBackendResolverTest, ExplicitOpenCvOnnxWithoutCompiledSupportThrowsChineseError) {
    const ModelProfile profile = makeProfile(EngineType::LocalOnnx);
    const InferenceBackendRuntimeInfo runtime_info = makeRuntimeInfo(true, true, false);

    EXPECT_THROW(
        {
            try {
                (void)resolveInferenceBackend(profile, runtime_info);
            } catch (const std::runtime_error& ex) {
                EXPECT_NE(std::string(ex.what()).find("ONNX Runtime C++ 支持"), std::string::npos);
                throw;
            }
        },
        std::runtime_error);
}

TEST(InferenceBackendResolverTest, ExplicitAidLiteWithoutCompiledSupportThrowsChineseError) {
    const ModelProfile profile = makeProfile(EngineType::AidLite);
    const InferenceBackendRuntimeInfo runtime_info = makeRuntimeInfo(true, false, true);

    EXPECT_THROW(
        {
            try {
                (void)resolveInferenceBackend(profile, runtime_info);
            } catch (const std::runtime_error& ex) {
                EXPECT_NE(std::string(ex.what()).find("构建未启用 AidLite"), std::string::npos);
                throw;
            }
        },
        std::runtime_error);
}

TEST(InferenceBackendResolverTest, FormattedLogIncludesCompiledBackendState) {
    const ModelProfile profile = makeProfile(EngineType::Auto);
    const InferenceBackendRuntimeInfo runtime_info = makeRuntimeInfo(true, true, false);
    const InferenceBackendSelection selection = resolveInferenceBackend(profile, runtime_info);

    const std::string log_line = formatInferenceBackendSelectionLog(profile, selection);
    EXPECT_NE(log_line.find("AidLite编译支持=已启用"), std::string::npos);
    EXPECT_NE(log_line.find("ONNX Runtime编译支持=未启用"), std::string::npos);
    EXPECT_NE(log_line.find("最终后端=aidlite"), std::string::npos);
}

}  // namespace
}  // namespace rc26_vision
