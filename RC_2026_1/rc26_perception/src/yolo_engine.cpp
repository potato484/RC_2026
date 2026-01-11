#include "rc26_perception/yolo_engine.hpp"

// AidLite SDK 仅在 AidLux 平台可用
// 非 AidLux 平台编译时使用 stub 实现
#ifdef AIDLUX_PLATFORM
#include "aidlux/aidlite/aidlite.hpp"
using namespace Aidlux::Aidlite;
#endif

#include <iostream>

namespace rc26_perception {

YoloEngine::YoloEngine() = default;

YoloEngine::~YoloEngine() {
#ifdef AIDLUX_PLATFORM
    if (interpreter_) {
        interpreter_->destory();
        interpreter_ = nullptr;
    }
#endif
}

bool YoloEngine::init(const YoloEngineConfig& cfg) {
    input_size_ = cfg.input_size;
    nchw_ = cfg.nchw;

#ifdef AIDLUX_PLATFORM
    std::cout << "[YoloEngine] Aidlite version: " << get_library_version() << std::endl;

    model_ = Model::create_instance(cfg.model_path);
    if (!model_) {
        std::cerr << "[YoloEngine] Failed to create model from: " << cfg.model_path << std::endl;
        return false;
    }

    config_ = Config::create_instance();
    if (!config_) {
        std::cerr << "[YoloEngine] Failed to create config" << std::endl;
        return false;
    }
    config_->framework_type = FrameworkType::TYPE_QNN231;
    config_->accelerate_type = AccelerateType::TYPE_DSP;

    interpreter_ = InterpreterBuilder::build_interpretper_from_model_and_config(model_, config_);
    if (!interpreter_) {
        std::cerr << "[YoloEngine] Failed to build interpreter" << std::endl;
        return false;
    }

    if (interpreter_->init() != 0) {
        std::cerr << "[YoloEngine] Failed to init interpreter" << std::endl;
        return false;
    }

    if (interpreter_->load_model() != 0) {
        std::cerr << "[YoloEngine] Failed to load model" << std::endl;
        return false;
    }

    initialized_ = true;
    std::cout << "[YoloEngine] Initialized successfully" << std::endl;
    return true;
#else
    std::cerr << "[YoloEngine] AidLite SDK not available (non-AidLux platform)" << std::endl;
    std::cerr << "[YoloEngine] Running in stub mode - no actual inference" << std::endl;
    initialized_ = false;
    return false;
#endif
}

bool YoloEngine::infer(const float* input_data) {
#ifdef AIDLUX_PLATFORM
    if (!initialized_ || !interpreter_) {
        return false;
    }

    if (interpreter_->set_input_tensor(0, (void*)input_data) != 0) {
        std::cerr << "[YoloEngine] set_input_tensor failed" << std::endl;
        return false;
    }

    if (interpreter_->invoke() != 0) {
        std::cerr << "[YoloEngine] invoke failed" << std::endl;
        return false;
    }

    return true;
#else
    (void)input_data;
    return false;
#endif
}

bool YoloEngine::getOutput(float*& box_data, uint32_t& box_bytes, float*& score_data, uint32_t& score_bytes) {
#ifdef AIDLUX_PLATFORM
    if (!initialized_ || !interpreter_) {
        return false;
    }

    float* out0 = nullptr;
    uint32_t len0 = 0;
    float* out1 = nullptr;
    uint32_t len1 = 0;

    if (interpreter_->get_output_tensor(0, (void**)&out0, &len0) != 0 ||
        interpreter_->get_output_tensor(1, (void**)&out1, &len1) != 0) {
        std::cerr << "[YoloEngine] get_output_tensor failed" << std::endl;
        return false;
    }

    // YOLO输出tensor顺序可能因模型而异
    // 假设: boxes tensor (N, 4) 的字节数 < scores tensor (N, num_classes) 的字节数
    // 警告: 此假设仅在 num_classes > 4 时成立
    // TODO: 更健壮的做法是检查tensor shape而非buffer大小
    //       但AidLite SDK可能不提供shape查询接口
    if (len0 == 0 || len1 == 0) {
        std::cerr << "[YoloEngine] Warning: empty output tensor (len0=" << len0 << ", len1=" << len1 << ")"
                  << std::endl;
        return false;
    }

    if (len0 <= len1) {
        box_data = out0;
        box_bytes = len0;
        score_data = out1;
        score_bytes = len1;
    } else {
        box_data = out1;
        box_bytes = len1;
        score_data = out0;
        score_bytes = len0;
    }

    // 健壮性检查: boxes应该是4的倍数 (每个box有4个float: x, y, w, h)
    if ((box_bytes / sizeof(float)) % 4 != 0) {
        std::cerr << "[YoloEngine] Warning: box_bytes=" << box_bytes << " is not divisible by 16 (4 floats per box)"
                  << std::endl;
    }

    return true;
#else
    box_data = nullptr;
    box_bytes = 0;
    score_data = nullptr;
    score_bytes = 0;
    return false;
#endif
}

}  // namespace rc26_perception
