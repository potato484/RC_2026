#include <gtest/gtest.h>

#include "rc26_vision/runtime/inference_backend_resolver.hpp"

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

TEST(InferenceBackendResolverTest, AutoWithoutAidLitePathsFallsBackToOpenCvOnnx) {
    const ModelProfile profile = makeProfile(EngineType::Auto);
    const InferenceBackendRuntimeInfo runtime_info{
        false,
        false,
    };

    const InferenceBackendSelection selection = resolveInferenceBackend(profile, runtime_info);
    EXPECT_EQ(selection.resolved_engine, EngineType::LocalOnnx);
    EXPECT_NE(selection.reason.find("未检测到 AidLux/AidLite 路径"), std::string::npos);
}

TEST(InferenceBackendResolverTest, AutoWithAidLitePathsAndCompiledUsesAidLite) {
    const ModelProfile profile = makeProfile(EngineType::Auto);
    const InferenceBackendRuntimeInfo runtime_info{
        true,
        true,
    };

    const InferenceBackendSelection selection = resolveInferenceBackend(profile, runtime_info);
    EXPECT_EQ(selection.resolved_engine, EngineType::AidLite);
    EXPECT_NE(selection.reason.find("优先使用 AidLite 推理链"), std::string::npos);
}

TEST(InferenceBackendResolverTest, AutoWithAidLitePathsButWithoutCompiledFallsBackToOpenCvOnnx) {
    const ModelProfile profile = makeProfile(EngineType::Auto);
    const InferenceBackendRuntimeInfo runtime_info{
        true,
        false,
    };

    const InferenceBackendSelection selection = resolveInferenceBackend(profile, runtime_info);
    EXPECT_EQ(selection.resolved_engine, EngineType::LocalOnnx);
    EXPECT_NE(selection.reason.find("构建未启用 AidLite"), std::string::npos);
}

TEST(InferenceBackendResolverTest, ExplicitOpenCvOnnxStaysOpenCvOnnx) {
    const ModelProfile profile = makeProfile(EngineType::LocalOnnx);
    const InferenceBackendRuntimeInfo runtime_info{
        true,
        true,
    };

    const InferenceBackendSelection selection = resolveInferenceBackend(profile, runtime_info);
    EXPECT_EQ(selection.resolved_engine, EngineType::LocalOnnx);
    EXPECT_NE(selection.reason.find("显式指定 ONNX Runtime"), std::string::npos);
}

TEST(InferenceBackendResolverTest, ExplicitAidLiteWithoutCompiledSupportThrowsChineseError) {
    const ModelProfile profile = makeProfile(EngineType::AidLite);
    const InferenceBackendRuntimeInfo runtime_info{
        true,
        false,
    };

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

}  // namespace
}  // namespace rc26_vision
