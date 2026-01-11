#pragma once

#include <string>
#include <vector>
#include <cstdint>

// 前向声明 AidLite 类型
namespace Aidlux {
namespace Aidlite {
class Interpreter;
class Model;
class Config;
}
}

namespace rc26_perception {

// YOLO 推理引擎配置
struct YoloEngineConfig {
    std::string model_path;     // 模型路径
    int input_size = 640;       // 输入尺寸
    bool nchw = false;          // 输入格式 (true: NCHW, false: NHWC)
};

// YOLO 推理引擎封装类
class YoloEngine {
public:
    YoloEngine();
    ~YoloEngine();

    // 初始化引擎
    bool init(const YoloEngineConfig& config);

    // 执行推理
    bool infer(const float* input_data);

    // 获取输出张量
    bool getOutput(float*& box_data, uint32_t& box_bytes,
                   float*& score_data, uint32_t& score_bytes);

    int getInputSize() const { return input_size_; }
    bool isNchw() const { return nchw_; }
    bool isInitialized() const { return initialized_; }

private:
    // 使用原始指针，因为 AidLite 类型是不完整类型
    Aidlux::Aidlite::Interpreter* interpreter_ = nullptr;
    Aidlux::Aidlite::Model* model_ = nullptr;
    Aidlux::Aidlite::Config* config_ = nullptr;
    
    int input_size_ = 640;
    bool nchw_ = false;
    bool initialized_ = false;
};

}  // namespace rc26_perception
