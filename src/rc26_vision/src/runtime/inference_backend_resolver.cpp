#include "rc26_vision/runtime/inference_backend_resolver.hpp"

#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace rc26_vision {

namespace {

bool hasAidLiteLibraryArtifact() {
    namespace fs = std::filesystem;

    const fs::path lib_dir("/usr/local/lib");
    std::error_code ec;
    if (!fs::exists(lib_dir, ec) || !fs::is_directory(lib_dir, ec)) {
        return false;
    }

    for (const auto& entry : fs::directory_iterator(lib_dir, ec)) {
        if (ec) {
            return false;
        }
        const std::string name = entry.path().filename().string();
        if (name.rfind("libaidlite", 0) == 0 || name.rfind("aidlite", 0) == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

bool isAidLiteBackendCompiled() {
#if defined(RC26_VISION_HAS_AIDLITE) && RC26_VISION_HAS_AIDLITE
    return true;
#else
    return false;
#endif
}

InferenceBackendRuntimeInfo detectInferenceBackendRuntimeInfo() {
    namespace fs = std::filesystem;

    const bool has_header = fs::exists("/usr/local/include/aidlux/aidlite/aidlite.hpp");
    const bool has_library = hasAidLiteLibraryArtifact();

    InferenceBackendRuntimeInfo info;
    info.aidlite_paths_detected = has_header && has_library;
    info.aidlite_compiled = isAidLiteBackendCompiled();
    return info;
}

InferenceBackendSelection resolveInferenceBackend(const ModelProfile& profile) {
    return resolveInferenceBackend(profile, detectInferenceBackendRuntimeInfo());
}

InferenceBackendSelection resolveInferenceBackend(const ModelProfile& profile,
                                                  const InferenceBackendRuntimeInfo& runtime_info) {
    InferenceBackendSelection selection;
    selection.requested_engine = profile.engine;
    selection.aidlite_paths_detected = runtime_info.aidlite_paths_detected;
    selection.aidlite_compiled = runtime_info.aidlite_compiled;

    switch (profile.engine) {
        case EngineType::Auto:
            if (runtime_info.aidlite_paths_detected && runtime_info.aidlite_compiled) {
                selection.resolved_engine = EngineType::AidLite;
                selection.reason = "检测到 AidLux/AidLite 路径，优先使用 AidLite 推理链。";
                return selection;
            }
            selection.resolved_engine = EngineType::LocalOnnx;
            if (!runtime_info.aidlite_paths_detected) {
                selection.reason = "未检测到 AidLux/AidLite 路径，切换到默认 ONNX Runtime 推理链。";
            } else {
                selection.reason =
                    "检测到 AidLux/AidLite 路径，但当前 rc26_vision 构建未启用 AidLite，回退默认 "
                    "ONNX Runtime 推理链。";
            }
            return selection;

        case EngineType::LocalOnnx:
            selection.resolved_engine = EngineType::LocalOnnx;
            selection.reason = "配置显式指定 ONNX Runtime 推理链。";
            return selection;

        case EngineType::AidLite:
            if (!runtime_info.aidlite_compiled) {
                throw std::runtime_error(
                    "配置显式指定 AidLite 推理链，但当前 rc26_vision 构建未启用 AidLite，无法启动。");
            }
            selection.resolved_engine = EngineType::AidLite;
            selection.reason = "配置显式指定 AidLite 推理链。";
            return selection;

        case EngineType::AidLiteQnnYolo:
            if (!runtime_info.aidlite_compiled) {
                throw std::runtime_error(
                    "配置显式指定 AidLite QNN 测试链，但当前 rc26_vision 构建未启用 AidLite，无法启动。");
            }
            selection.resolved_engine = EngineType::AidLiteQnnYolo;
            selection.reason = "配置显式指定 AidLite QNN 测试链。";
            return selection;
    }

    throw std::runtime_error("未知的视觉推理后端类型。");
}

std::string formatInferenceBackendSelectionLog(const ModelProfile& profile,
                                               const InferenceBackendSelection& selection) {
    std::ostringstream oss;
    oss << "[rc26_vision] profile='" << profile.id << "' 请求后端="
        << engineTypeToString(selection.requested_engine)
        << "，AidLux路径=" << (selection.aidlite_paths_detected ? "已检测到" : "未检测到")
        << "，AidLite编译支持=" << (selection.aidlite_compiled ? "已启用" : "未启用")
        << "，最终后端=" << engineTypeToString(selection.resolved_engine)
        << "。 " << selection.reason;
    return oss.str();
}

}  // namespace rc26_vision
