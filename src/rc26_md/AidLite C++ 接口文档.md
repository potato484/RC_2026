# AidLite C++ 接口文档

> **💡注意 使用 Aidlite-SDK C++ 开发需要了解如下事项：**
> 
>   - 编译时需要包含头文件，存放路径 /usr/local/include/aidlux/aidlite/aidlite.hpp
>   - 链接时需要指定库文件，存放路径 /usr/local/lib/libaidlite.so

### 模型数据类型.enum DataType

对于 AidliteSDK 而言，会用来处理不同框架的不同模型，每个模型也会有不同的输入数据类型和不同的输出数据类型。调用某些接口时，需要设置模型的输入输出数据类型，就需要用到此数据类型的枚举。

| **成员变量名**     | **类型**   | **值** | **描述**            |
| :------------ | :------- | :---- | :---------------- |
| TYPE\_DEFAULT | uint8\_t | 0     | 无效数据类型            |
| TYPE\_UINT8   | uint8\_t | 1     | Unsigned char 数据  |
| TYPE\_INT8    | uint8\_t | 2     | Char 数据           |
| TYPE\_UINT32  | uint8\_t | 3     | Unsigned int32 数据 |
| TYPE\_FLOAT32 | uint8\_t | 4     | Float32 数据        |
| TYPE\_INT32   | uint8\_t | 5     | Int32 数据          |
| TYPE\_INT64   | uint8\_t | 6     | Int64 数据          |
| TYPE\_UINT64  | uint8\_t | 7     | Unsigned int64 数据 |
| TYPE\_INT16   | uint8\_t | 8     | Int16 数据          |
| TYPE\_UINT16  | uint8\_t | 9     | Unsigned int16 数据 |
| TYPE\_FLOAT16 | uint8\_t | 10    | Float16 数据        |
| TYPE\_BOOL    | uint8\_t | 11    | Bool 数据           |

### 推理实现类型.enum ImplementType

| **成员变量名**     | **类型**   | **值** | **描述**                       |
| :------------ | :------- | :---- | :--------------------------- |
| TYPE\_DEFAULT | uint8\_t | 0     | 无效 ImplementType 类型          |
| TYPE\_MMKV    | uint8\_t | 1     | 通过 MMKV 实现（现在没有相关的实现，后期可能移除） |
| TYPE\_REMOTE  | uint8\_t | 2     | 远程推理实现                       |
| TYPE\_LOCAL   | uint8\_t | 3     | 本地推理实现                       |

> **💡注意**
> 
> 特别说明:
> 
> TYPE\_LOCAL 与 TYPE\_REMOTE 的区别在于：对于 Aplux 融合系统而言，前者的推理实现过程是在 Linux 环境本地完成的，无需数据的传输。而后者则是通过 IPC（进程间通信）的方式与 Android 环境实现数据交互，推理实现过程在 Android 环境完成的，故而会涉及到数据的传输。综上所述，TYPE\_LOCAL 方式是用来完成推理过程的优选项。

### 模型框架类型.enum FrameworkType

由于 AidliteSDK 整合了多种深度学习推理框架，所以在使用过程中，需要设置当前使用哪个框架的模型，就需要设置所需推理框架的类型枚举。

| **成员变量名**     | **类型**   | **值** | **描述**              |
| :------------ | :------- | :---- | :------------------ |
| TYPE\_DEFAULT | uint8\_t | 0     | 无效 FrameworkType 类型 |
| TYPE\_SNPE    | uint8\_t | 1     | SNPE1.x(dlc) 模型     |
| TYPE\_TFLITE  | uint8\_t | 2     | TFLite 模型           |
| TYPE\_RKNN    | uint8\_t | 3     | RKNN 模型             |
| TYPE\_QNN     | uint8\_t | 4     | QNN 模型              |
| TYPE\_SNPE2   | uint8\_t | 5     | SNPE2.x(dlc) 模型     |
| TYPE\_NCNN    | uint8\_t | 6     | NCNN 模型             |
| TYPE\_MNN     | uint8\_t | 7     | MNN 模型              |
| TYPE\_TNN     | uint8\_t | 8     | TNN 模型              |
| TYPE\_PADDLE  | uint8\_t | 9     | Paddle 模型           |
| TYPE\_ONNX    | uint8\_t | 11    | onnx 模型             |
| TYPE\_QNN216  | uint8\_t | 101   | QNN2.16 模型          |
| TYPE\_SNPE216 | uint8\_t | 102   | SNPE2.16 模型         |
| TYPE\_QNN223  | uint8\_t | 103   | QNN2.23 模型          |
| TYPE\_SNPE223 | uint8\_t | 104   | SNPE2.23 模型         |
| TYPE\_QNN229  | uint8\_t | 105   | QNN2.29 模型          |
| TYPE\_SNPE229 | uint8\_t | 106   | SNPE2.29 模型         |
| TYPE\_QNN231  | uint8\_t | 107   | QNN2.31 模型          |
| TYPE\_QNN236  | uint8\_t | 108   | QNN2.36 模型          |

> **💡注意**
> 
> 特别说明:
> 
>   - 如 果用户环境同时安装有 AidLite-QNN216/AidLite-QNN223/AidLite-QNN231 等后端实现软件包中的多个，而代码中 FrameworkType 设置为 TYPE\_QNN 的话（即没有明确指定使用那个特定版本的 QNN 后端），那么实际运行的 QNN 后端应该是 QNN 版本最高的后端（即优先选择 AidLite-QNN231 来完成推理）。SNPE2的处理同理如是。
>   - 同一个程序中不能同时使用不同版本的 QNN 后端软件包，就是说如果程序中先指定了 TYPE\_QNN216 来完成推理，那么就不能再使用 TYPE\_QNN223/TYPE\_QNN231 来做模型推理。SNPE2的处理同理如是。

### 加速硬件类型.enum AccelerateType

对于每个深度学习推理框架而言，可能会支持运行在不同的加速设备上（如 SNPE/QNN 模型运行在 DSP 设备，RKNN 模型运行在 NPU 设备），所以在使用过程中，需要设置当前模型期望运行在哪个设备，就需要使用此加速硬件枚举。

| **成员变量名**     | **类型**   | **值** | **描述**               |
| :------------ | :------- | :---- | :------------------- |
| TYPE\_DEFAULT | uint8\_t | 0     | 无效 AccelerateType 类型 |
| TYPE\_CPU     | uint8\_t | 1     | CPU 加速运行             |
| TYPE\_GPU     | uint8\_t | 2     | GPU 加速运行             |
| TYPE\_DSP     | uint8\_t | 3     | DSP 加速运行             |
| TYPE\_NPU     | uint8\_t | 4     | NPU 加速运行             |

### 日志级别.enum LogLevel

AidliteSDK 提供有设置日志相关信息的接口（后续会介绍），如果需要指定 AidliteSDK 当前使用哪个日志级别，就需要使用此日志级别枚举。

| **成员变量名** | **类型**   | **值** | **描述** |
| :-------- | :------- | :---- | :----- |
| INFO      | uint8\_t | 0     | 消息     |
| WARNING   | uint8\_t | 1     | 警告     |
| ERROR     | uint8\_t | 2     | 错误     |
| FATAL     | uint8\_t | 3     | 致命错误   |

### 输入输出信息类.class TensorInfo

每 个模型都会有各自的输入输出，对于开发者不那么熟悉的模型而言，开发过程中可能需要查询该模型的输入输出 Tensor 信息，以方便开发者做模型推理的前后处理操作。Aidlite 为开发者提供了查询模型输入输出详细信息的接口，以及用于存储输入输出详细信息的 TensorInfo 数据类型。

#### 成员变量列表

TensorInfo 对象用于存储模型输入输出 Tensor 的详细信息，其中就包含如下的这些参数：

|    |                                            |
| -- | ------------------------------------------ |
| 成员 | version                                    |
| 类型 | uint32\_t                                  |
| 默认 | 无默认值                                       |
| 描述 | 此成员用于标识 TensorInfo 版本，如果以后需要扩展更多信息时，会更新版本号 |

|    |             |
| -- | ----------- |
| 成员 | index       |
| 类型 | uint32\_t   |
| 默认 | 无默认值        |
| 描述 | Tensor 的索引值 |

|    |               |
| -- | ------------- |
| 成员 | name          |
| 类型 | std::string   |
| 默认 | 无默认值          |
| 描述 | Tensor 的名称字符串 |

|    |                        |
| -- | ---------------------- |
| 成员 | element\_count         |
| 类型 | uint64\_t              |
| 默认 | 0                      |
| 描述 | Tensor 中数据元素的数量（非字节数量） |

|    |                         |
| -- | ----------------------- |
| 成员 | element\_type           |
| 类型 | enum DataType           |
| 默认 | DataType::TYPE\_DEFAULT |
| 描述 | Tensor 中数据元素的类型         |

|    |                  |
| -- | ---------------- |
| 成员 | dimensions       |
| 类型 | uint32\_t        |
| 默认 | 0                |
| 描述 | Tensor 中数据元素的维度值 |

|    |                            |
| -- | -------------------------- |
| 成员 | shape                      |
| 类型 | std::vector\< uint64\_t \> |
| 默认 | 无默认值                       |
| 描述 | Tensor 中数据元素的 shape 描述     |

### 模型类.class Model

在创建推理解释器之前，需要设置具体模型的相关详细参数。Model 类主要用于记录模型的文件信息、结构信息、运行过程中模型相关内容。

#### 创建实例对象.create\_instance()

想要设置模型相关的详细信息，当然就需要先有模型实例对象，此函数用于创建 Model 实例对象。如果模型框架（如 snpe、tflite 等）只需一个模型文件，则调用此接口。

|     |                                          |
| --- | ---------------------------------------- |
| API | create\_instance                         |
| 描述  | 通过传递模型文件的路径名称，构造 Model 类型的实例对象           |
| 参数  | model\_path：模型文件的路径名称                    |
| 返回  | 如果为 nullptr 值，说明函数构建对象失败；否则就是 Mode 对象的指针 |

    // 使用当前路径下 inceptionv3_float32.dlc 文件创建模型对象，返回值为空则报错
    Model* model = Model::create_instance("./inceptionv3_float32.dlc");
    if(model == nullptr){
        printf("Create model failed.\n");
        return EXIT_FAILURE;
    }

#### 创建实例对象.create\_instance()

想要设置模型相关的详细信息，当然就需要先有模型实例对象，此函数用于创建 Model 实例对象。如果模型框架（如 ncnn、tnn 等）涉及两个模型文件，则调用此接口。

API

create\_instance

描述

通过传递模型文件的路径名称，构造 Model 类型的实例对象

参数

model\_struct\_path：模型结构文件的路径名称

model\_weight\_path：模型权重参数文件的路径名称

返回

如果为 nullptr 值，说明函数构建对象失败；否则就是 Model 对象的指针

    // 使用当前路径下 ncnn 的两个模型文件创建模型对象，返回值为空则报错
    Model* model = Model::create_instance( "./mobilenet_ssd_voc_ncnn.param", 
         "./mobilenet_ssd_voc_ncnn.bin");
    if(model == nullptr){
        printf("Create model failed.\n");
        return EXIT_FAILURE;
    }

#### 设置模型属性.set\_model\_properties()

模型实例对象创建成功之后，需要设置模型的输入输出数据类型和输入输出 tensor 数据的 shape 信息。

API

set\_model\_properties

描述

设置模型的属性，输出输出数据 shape 以数据类型

参数

input\_shapes：输入 tensor 的 shape 数组，二维数组结构

input\_data\_type：输入 tensor 的数据类型，DataType 枚举类型

output\_shapes：输出 tensor 的 shape 数组，二维数组结构

output\_data\_type：输出 tensor 的数据类型，DataType 枚举类型

返回

void

注意

1\. 当且仅当 ImplementType 为 REMOTE 时，需要调用此接口来设置输入输出的详细信息。  
2\. 对于 SNPE/QNN 两个推理框架来说，输入输出的数据只能是 float32 数据类型，  
故而使用此接口只能是 DataType::TYPE\_FLOAT32 作为传入参数。如果是其他推理框架，  
输入输出的数据类型则与模型文件的输入输出 Tensor 的数据类型保持一致。

    // 数据类型使用前述 DataType 枚举，输入输出 shape 是二维数组
    std::vector<std::vector<uint32_t>> input_shapes = {{1,299,299,3}};
    std::vector<std::vector<uint32_t>> output_shapes = {{1,1001}};
    model->set_model_properties(input_shapes, DataType::TYPE_FLOAT32, 
        output_shapes, DataType::TYPE_FLOAT32);

#### 获取模型路径.get\_model\_absolute\_path()

如果模型框架（如 SNPE、TFLite 等）只涉及一个模型文件，在构建 Model 对象没有异常的情况下，会将传入的模型文件参数转换为绝对路径形式的文件路径。

|     |                            |
| --- | -------------------------- |
| API | get\_model\_absolute\_path |
| 描述  | 用于获取模型文件的存在路径              |
| 参数  | void                       |
| 返回  | Model 对象所对应的模型文件路径字符串      |

#### 获取模型路径.get\_model\_struct\_absolute\_path()

如果模型框架（如 NCNN、TNN 等）涉及两个模型文件，在构建 Model 对象没有异常的情况下，会将传入的模型结构文件参数转换为绝对路径形式的文件路径。

|     |                                    |
| --- | ---------------------------------- |
| API | get\_model\_struct\_absolute\_path |
| 描述  | 用于获取模型结构文件的存在路径                    |
| 参数  | void                               |
| 返回  | Model 对象所对应的模型结构文件路径字符串            |

#### 获取模型路径.get\_model\_weight\_absolute\_path()

如果模型框架（如 NCNN、TNN 等）涉及两个模型文件，在构建 Model 对象没有异常的情况下，会将传入的模型权重参数文件参数转换为绝对路径形式的文件路径。

|     |                                    |
| --- | ---------------------------------- |
| API | get\_model\_weight\_absolute\_path |
| 描述  | 用于获取模型权重参数文件的存在路径                  |
| 参数  | void                               |
| 返回  | Model 对象所对应的模型权重参数文件路径字符串          |

### 配置类.class Config

在创建推理解释器之前，除了需要设置 Model 具体信息之外，还需要设置一些推理时的配置信息。Config 类用于记录需要预先设置的配置选项，这些配置项在运行时会被用到。

#### 创建实例对象.create\_instance()

想要设置运行时的配置信息，当然就需要先有配置实例对象，此函数用于创建 Config 实例

|     |                                            |
| --- | ------------------------------------------ |
| API | create\_instance                           |
| 描述  | 用于构造 Config 类的实例对象                         |
| 参数  | void                                       |
| 返回  | 如果为 nullptr 值，说明函数构建对象失败；否则就是 Config 对象的指针 |

    // 创建配置实例对象，返回值为空则报错
    Config* config = Config::create_instance();
    if(config == nullptr){
        printf("Create config failed.\n");
        return EXIT_FAILURE;
    }

#### 成员变量列表

Config 对象用于设置运行时的配置信息，其中就包含如下的这些参数：

|    |                          |
| -- | ------------------------ |
| 成员 | accelerate\_type         |
| 类型 | enum AccelerateType      |
| 默认 | AccelerateType.TYPE\_CPU |
| 描述 | 加速硬件的类型                  |

<table>
<colgroup>
<col style="width: 50%" />
<col style="width: 50%" />
</colgroup>
<tbody>
<tr class="odd">
<td>成员</td>
<td>implement_type</td>
</tr>
<tr class="even">
<td>类型</td>
<td>enum ImplementType</td>
</tr>
<tr class="odd">
<td>默认</td>
<td>ImplementType.TYPE_DEFAULT</td>
</tr>
<tr class="even">
<td>描述</td>
<td>用于区分后端实现推理功能的不同环境</td>
</tr>
<tr class="odd">
<td>注意</td>
<td>开发者可以不用显式设置此配置项，<br />
AidliteSDK 会根据当前的系统环境、模型框架等因素，适配得出更优的 ImplementType 类型</td>
</tr>
</tbody>
</table>

|    |                             |
| -- | --------------------------- |
| 成员 | framework\_type             |
| 类型 | enum FrameworkType          |
| 默认 | FrameworkType.TYPE\_DEFAULT |
| 描述 | 底层推理框架的类型                   |

|    |                                             |
| -- | ------------------------------------------- |
| 成员 | number\_of\_threads                         |
| 类型 | int8\_t                                     |
| 默认 | \-1                                         |
| 描述 | 线程数，大于 0 有效                                 |
| 注意 | 当且仅当 framework\_type 为 TFLITE 时，可以选择性设置此配置项 |

|    |                                        |
| -- | -------------------------------------- |
| 成员 | snpe\_out\_names                       |
| 类型 | 字符串数组                                  |
| 默认 | 空                                      |
| 描述 | 模型输出节点的名称列表                            |
| 注意 | 当且仅当 framework\_type 为 SNPE 时，需要设置此配置项 |

|    |                                          |
| -- | ---------------------------------------- |
| 成员 | is\_quantify\_model                      |
| 类型 | Int32\_t                                 |
| 默认 | 0                                        |
| 描述 | 是否为量化模型，1 表示量化模型                         |
| 注意 | 当且仅当 implement\_type 为 REMOTE 时，需要设置此配置项 |

|    |                                             |
| -- | ------------------------------------------- |
| 成员 | remote\_timeout                             |
| 类型 | Int32\_t                                    |
| 默认 | \-1                                         |
| 描述 | 接口超时时间（毫秒，大于 0 有效）                          |
| 注意 | 当且仅当 implement\_type 为 REMOTE 时，可以选择性设置此配置项 |

    config->framework_type = FrameworkType::TYPE_QNN;
    config->accelerate_type = AccelerateType::TYPE_DSP;
    // 按需设置其他配置项
    // ... ...

### 上下文类.class Context

用于存储执行过程中所涉及的运行时上下文，就包括 Model 对象和 Config 对象，后续可能会扩展运行时的上下文数据。

#### 构造函数.Context()

API

Context

描述

构造 Context 实例对象

参数

model：Model 实例对象指针

config：Config 实例对象指针

返回

正常：Context 实例对象

> **💡注意**
> 
> 特别说明，参数 model 和 config 两个指针指向的内存空间，必须是堆空间上开辟的。***构造 Context 成功之后，它们生命周期将会被 Context 接管( Context 释放时，会自动释放 model 和 config 对象)，故而开发者无需再手动释放两者的空间，否则会报错重复释放对象。***

#### 获取Model成员变量.get\_model()

|     |                           |
| --- | ------------------------- |
| API | get\_model                |
| 描述  | 获取 context 管理的 Model 对象指针 |
| 参数  | void                      |
| 返回  | Model 对象的指针               |

#### 获取Config成员变量.get\_config()

|     |                            |
| --- | -------------------------- |
| API | get\_config                |
| 描述  | 获取 context 管理的 Config 对象指针 |
| 参数  | void                       |
| 返回  | Config 对象的指针               |

### 解释器类.class Interpreter

Interpreter 类型的对象实例是执行推理操作的主体，用于执行具体的推理过程。创建解释器对象之后，所有的操作都是基于解释器对象来完成的，所以它是 AidliteSDK 绝对的核心内容。

#### 创建实例对象.create\_instance()

想要执行推理相关的操作，推理解释器肯定必不可少，此函数就用于构建推理解释器的实例对象。

|     |                                               |
| --- | --------------------------------------------- |
| API | create\_instance                              |
| 描述  | 利用 Context 对象所管理的各种数据，构造 Interpreter 类型的对象    |
| 参数  | context：Context 实例对象的指针                       |
| 返回  | 为 nullptr 值则说明函数构建对象失败；否则就是 Interpreter 对象的指针 |

``` 
 // 使用 Context 对象指针来创建解释器对象，返回值为空则报错
Interpreter* current_interpreter = Interpreter::create_instance(context);
if(current_interpreter == nullptr){
    printf("Create Interpreter failed.\n");
    return EXIT_FAILURE;
}
```

#### 初始化.init()

解释器对象创建之后，需要执行一些初始化操作（比如环境检查、资源构建等）。

|     |                                |
| --- | ------------------------------ |
| API | init                           |
| 描述  | 完成推理所需的相关的初始化工作                |
| 参数  | void                           |
| 返回  | 0 值则说明初始化操作执行成功，否则非 0 值就说明执行失败 |

    // 解释器初始化，返回值非0则报错
    int result = current_interpreter->init();
    if(result != EXIT_SUCCESS){
        printf("sample : interpreter->init() failed.\n");
        return EXIT_FAILURE;
    }

#### 加载模型.load\_model()

解释器对象完成初始化操作之后，就可以为解释器加载所需的模型文件，完成模型加载工作。后续的推理过程就使用加载的模型资源。

<table>
<colgroup>
<col style="width: 50%" />
<col style="width: 50%" />
</colgroup>
<tbody>
<tr class="odd">
<td>API</td>
<td>load_model</td>
</tr>
<tr class="even">
<td>描述</td>
<td>完成模型加载相关的工作。<br />
因为前述在 Model 对象中已经设置模型文件的路径，所以直接执行模型加载操作即可</td>
</tr>
<tr class="odd">
<td>参数</td>
<td>void</td>
</tr>
<tr class="even">
<td>返回</td>
<td>0 值则说明模型加载操作执行成功，否则非 0 值就说明执行失败</td>
</tr>
</tbody>
</table>

    // 解释器加载模型，返回值非0则报错
    int result = current_interpreter->load_model();
    if(result != EXIT_SUCCESS){
        printf("sample : interpreter->load_model() failed.\n");
        return EXIT_FAILURE;
    }

#### 获取输入Tensor详情.get\_input\_tensor\_info()

如果想要获取模型的输入 Tensor 的详细信息，可以使用此接口获取。

|     |                                     |
| --- | ----------------------------------- |
| API | get\_input\_tensor\_info            |
| 描述  | 获取模型中输入 Tensor 的详细信息                |
| 参数  | in\_tensor\_info：存储输入 tensor 详情的数组  |
| 返回  | 0 值则说明获取 Tensor 详情成功，否则非 0 值就说明执行失败 |
| 说明  | 此接口从 AidliteSDK 版本号 2.2.5 开始支持      |

    // 获取模型的输入详情数据，返回值非0则报错
    std::vector<std::vector<TensorInfo>> in_tensor_info;
    result = current_interpreter->get_input_tensor_info(in_tensor_info);
    if(result != EXIT_SUCCESS){
        printf("interpreter->get_input_tensor_info() failed.\n");
        return EXIT_FAILURE;
    }
    for(int gi = 0; gi < in_tensor_info.size(); ++gi){
        for(int ti = 0; ti < in_tensor_info[gi].size(); ++ti){
    
            std::string tensor_shape_str = std::to_string(in_tensor_info[gi][ti].shape[0]);
            for(int si = 1; si < in_tensor_info[gi][ti].dimensions; ++si){
                tensor_shape_str += " ";
                tensor_shape_str += std::to_string(in_tensor_info[gi][ti].shape[si]);
            }
    
            printf("Input  tensor : Graph[%d]-Tensor[%d]-name[%s]-element_count[%ld]-element_type[%d]-dimensions[%d]-shape[%s]\n",
                    gi, ti, in_tensor_info[gi][ti].name.c_str(), in_tensor_info[gi][ti].element_count, 
                    (int)in_tensor_info[gi][ti].element_type, in_tensor_info[gi][ti].dimensions, tensor_shape_str.c_str());
        }
    }

#### 获取输出Tensor详情.get\_output\_tensor\_info()

如果想要获取模型的输出 Tensor 的详细信息，可以使用此接口获取。

|     |                                     |
| --- | ----------------------------------- |
| API | get\_output\_tensor\_info           |
| 描述  | 获取模型中输出 Tensor 的详细信息                |
| 参数  | out\_tensor\_info：存储输出 tensor 详情的数组 |
| 返回  | 0 值则说明获取 Tensor 详情成功，否则非 0 值就说明执行失败 |
| 说明  | 此接口从 AidliteSDK 版本号 2.2.5 开始支持      |

    // 获取模型的输出详情数据，返回值非0则报错
    std::vector<std::vector<TensorInfo>> out_tensor_info;
    result = current_interpreter->get_output_tensor_info(out_tensor_info);
    if(result != EXIT_SUCCESS){
        printf("interpreter->get_output_tensor_info() failed.\n");
        return EXIT_FAILURE;
    }
    for(int gi = 0; gi < out_tensor_info.size(); ++gi){
        for(int ti = 0; ti < out_tensor_info[gi].size(); ++ti){
    
            std::string tensor_shape_str = std::to_string(out_tensor_info[gi][ti].shape[0]);
            for(int si = 1; si < out_tensor_info[gi][ti].dimensions; ++si){
                tensor_shape_str += " ";
                tensor_shape_str += std::to_string(out_tensor_info[gi][ti].shape[si]);
            }
    
            printf("Output tensor : Graph[%d]-Tensor[%d]-name[%s]-element_count[%ld]-element_type[%d]-dimensions[%d]-shape[%s]\n",
                    gi, ti, out_tensor_info[gi][ti].name.c_str(), out_tensor_info[gi][ti].element_count, 
                    (int)out_tensor_info[gi][ti].element_type, out_tensor_info[gi][ti].dimensions, tensor_shape_str.c_str());
        }
    }

#### 设置输入数据.set\_input\_tensor()

执行推理操作之前，需要传入模型所需的输入 Tensor 数据 。设置输入数据之前，对于不同的模型，需要完成不同的前处理操作以适配具体模型。

API

set\_input\_tensor

描述

设置推理所需的输入 tensor 的数据

参数

in\_tensor\_idx/in\_tensor\_name：输入 tensor 的索引值/名称字符串

input\_data：输入 tensor 的数据存储地址指针

返回

0 值则说明设置输入 Tensor 成功，否则非 0 值就说明执行失败

说明

1\. 参数 in\_tensor\_name 从版本号 2.2.5 开始支持  
2\. 对于 SNPE/QNN 两个推理框架来说，输入的数据只能是 float32 数据类型，所以前处理  
之后得到的输入数据必须是float32格式，其他推理框架则与模型输入 Tensor 保持一致 。

    // 设置推理的输入数据，返回值非0则报错
    // int result = current_interpreter->set_input_tensor(0, input_tensor_data);
    int result = current_interpreter->set_input_tensor("in_tensor_name", input_tensor_data);
    if(result != EXIT_SUCCESS){
        printf("interpreter->set_input_tensor() failed.\n");
        return EXIT_FAILURE;
    }

#### 执行推理.invoke()

设置输入数据之后，接下来的步骤自然就是对输入数据执行推理运行过程。

|     |                               |
| --- | ----------------------------- |
| API | invoke                        |
| 描述  | 执行推理运算过程                      |
| 参数  | void                          |
| 返回  | 0 值则说明推理过程执行成功，否则非 0 值就说明执行失败 |

    // 执行推理运行操作，返回值非0则报错
    int result = current_interpreter->invoke();
    if(result != EXIT_SUCCESS){
        printf("sample : interpreter->invoke() failed.\n");
        return EXIT_FAILURE;
    }

#### 获取输出数据.get\_output\_tensor()

推理完成之后，就需要将推理得到的结果数据取出来。取出结果数据之后，就可以对这些结果数据进行处理，判断结果是否正确。

API

get\_output\_tensor

描述

推理成功之后，获取推理结果数据

参数

out\_tensor\_idx/out\_tensor\_name：输出 tensor 的索引值/名称字符串

output\_data：该值是指针变量的地址，函数内部重置该指针变量的值

output\_length：该结果输出 tensor 的数据量（字节数）

返回

0 值则说明获取输出 Tensor 执行成功，否则非 0 值就说明执行失败

说明

1\. 参数 out\_tensor\_name 从版本号 2.2.5 开始支持  
2\. output\_length 参数表示的是输出 Tensor 数据的字节数量  
3\. 对于 SNPE/QNN 两个推理框架来说，输出的数据只会是 float32 数据类型，所以必须对其按  
float32 的数据格式来处理，其他推理框架则与模型输出 Tensor 保持一致 。

    // 获取推理结果数据，返回值非0则报错
    float* out_data = nullptr;
    uint32_t output_tensor_length = 0;
    // int result = current_interpreter->get_output_tensor(0, (void**)&out_data, &output_tensor_length);
    int result = current_interpreter->get_output_tensor("out_tensor_name", (void**)&out_data, &output_tensor_length);
    if(result != EXIT_SUCCESS){
        printf("interpreter->get_output_tensor() failed.\n");
        return EXIT_FAILURE;
    }

#### 资源释放.destory()

前面提到解释器对象需要执行 init() 初始化操作和 load\_model() 加载模型的操作，相应的，解释器也需要执行一些释放操作，将之前创建的资源予以销毁。

|     |                               |
| --- | ----------------------------- |
| API | destory                       |
| 描述  | 完成必要的释放操作                     |
| 参数  | void                          |
| 返回  | 0 值则说明执行释放操作成功，否则非 0 值就说明执行失败 |

    // 执行解释器释放过程，返回值非0则报错
    int result = current_interpreter->destory();
    if(result != EXIT_SUCCESS){
        printf("sample : interpreter-> destory() failed.\n");
        return EXIT_FAILURE;
    }

### 解释器创建类.class InterpreterBuilder

统一的解释器 Interpreter 对象的创建函数，通过此类来创建所需的解释器对象。

#### 构建解释器.build\_interpretper\_from\_path()

构建推理解释器对象，可以提供不同的参数，最简单的就是只提供模型文件的路径和名称。如果模型框架（如 SNPE、TFLite 等）只需一个模型文件，则调用此接口。

<table>
<colgroup>
<col style="width: 50%" />
<col style="width: 50%" />
</colgroup>
<tbody>
<tr class="odd">
<td>API</td>
<td>build_interpretper_from_path</td>
</tr>
<tr class="even">
<td>描述</td>
<td>通过模型文件的路径名称，直接创建对应的解释器对象</td>
</tr>
<tr class="odd">
<td>参数</td>
<td>path：模型文件的路径名称</td>
</tr>
<tr class="even">
<td>返回</td>
<td>返回值为 Interpreter 类型的 unique_ptr 指针。<br />
如果为 nullptr 值，则说明函数构建对象失败；否则就是 Interpreter 对象的指针</td>
</tr>
</tbody>
</table>

#### 构建解释器.build\_interpretper\_from\_path()

构建推理解释器对象，可以提供不同的参数，最简单的就是只提供模型文件的路径和名称。如果模型框架（如 NCNN、TNN 等）涉及两个模型文件，则调用此接口。

API

build\_interpretper\_from\_path

描述

通过模型文件的路径名称，直接创建对应的解释器对象

参数

model\_struct\_path：模型结构文件的路径名称

model\_weight\_path：模型权重参数文件的路径名称

返回

返回值为 Interpreter 类型的 unique\_ptr 指针。  
如果为 nullptr 值，则说明函数构建对象失败；否则就是 Interpreter 对象的指针

#### 构建解释器.build\_interpretper\_from\_model()

构建推理解释器对象，除了提供模型文件路径之外，也可以提供 Model 对象，这样不仅可以设置模型文件的路径名称，还可以设置模型的输入输出数据类型和输入输出数据的 shape 信息。

<table>
<colgroup>
<col style="width: 50%" />
<col style="width: 50%" />
</colgroup>
<tbody>
<tr class="odd">
<td>API</td>
<td>build_interpretper_from_model</td>
</tr>
<tr class="even">
<td>描述</td>
<td>通过传入 Model 对象来创建对应的解释器对象。而 Config 相关涉及的参数都使用默认值</td>
</tr>
<tr class="odd">
<td>参数</td>
<td>model：Model 类型的对象，包含模型相关的数据</td>
</tr>
<tr class="even">
<td>返回</td>
<td>返回值为 Interpreter 类型的 unique_ptr 指针。<br />
如果为 nullptr 值，则说明函数构建对象失败；否则就是 Interpreter 对象的指针</td>
</tr>
</tbody>
</table>

> **💡注意**
> 
> 特别说明，参数 model 指针指向的内存空间，必须是堆空间上开辟的。***构造 Interpreter 成功之后，它的生命周期将会被 Interpreter 接管( Interpreter 释放时，会自动释放 model 对象)，故而开发者无需再手动释放两者的空间，否则会报错重复释放对象。***

#### 构建解释器.build\_interpretper\_from\_model\_and\_config()

构建推理解释器对象，除了前述的方式，也可以同时提供 Model 对象和 Config 对象，这样不仅可以提供模型相关信息，还可以提供更多的运行时配置参数。

API

build\_interpretper\_from\_model\_and\_config

描述

通过传入 Model 对象和 Config 来创建对应的解释器对象

参数

model：Model 类型的对象，包含模型相关的数据

config：Config 类型的对象，包含一些配置参数

返回

返回值为 Interpreter 类型的 unique\_ptr 指针。  
如果为 nullptr 值，则说明函数构建对象失败；否则就是 Interpreter 对象的指针

> **💡注意**
> 
> 特别说明，参数 model 和 config 两个指针指向的内存空间，必须是堆空间上开辟的。***构造 Interpreter 成功之后，它们生命周期将会被 Interpreter 接管( Interpreter 释放时，会自动释放 model 和 config 对象)，故而开发者无需再手动释放两者的空间，否则会报错重复释放对象。***

    // 传入Model和Config对象来构建解释器，返回值为空则报错
    std::unique_ptr<Interpreter>&& current_interpreter = 
     InterpreterBuilder::build_interpretper_from_model_and_config(model, config);
    if(current_interpreter == nullptr){
        printf("build_interpretper failed.\n");
        return EXIT_FAILURE;
    }

### 其他方法

AidliteSDK 除了提供前述的那些推理相关的接口，也提供如下一些辅助接口。

#### 获取SDK版本信息.get\_library\_version()

|     |                            |
| --- | -------------------------- |
| API | get\_library\_version      |
| 描述  | 用于获取当前 AidliteSDK 的版本相关信息。 |
| 参数  | void                       |
| 返回  | 返回值为当前 AidliteSDK 的版本信息字符串 |

#### 设置日志级别.set\_log\_level()

<table>
<colgroup>
<col style="width: 50%" />
<col style="width: 50%" />
</colgroup>
<tbody>
<tr class="odd">
<td>API</td>
<td>set_log_level</td>
</tr>
<tr class="even">
<td>描述</td>
<td>设置当前的最低日志级别，输出大于等于该日志级别的日志数据。<br />
默认打印输出 WARNING 及以上级别的日志</td>
</tr>
<tr class="odd">
<td>参数</td>
<td>log_level：枚举类型 LogLevel 的取值</td>
</tr>
<tr class="even">
<td>返回</td>
<td>默认返回 0 值</td>
</tr>
</tbody>
</table>

#### 日志输出到标准终端.log\_to\_stderr()

|     |                 |
| --- | --------------- |
| API | log\_to\_stderr |
| 描述  | 设置日志信息输出到标准错误终端 |
| 参数  | void            |
| 返回  | 默认返回 0 值        |

#### 日志输出到文本文件.log\_to\_file()

API

log\_to\_file

描述

设置日志信息输出到指定的文本文件

参数

path\_and\_prefix：日志文件的存放路径与名称前缀

also\_to\_stderr：标识是否同时输出日志到 stderr 终端，默认值为 false

返回

0 值则说明执行成功，否则非 0 值就说明执行失败

#### 获取最近日志信息.last\_log\_msg()

|     |                               |
| --- | ----------------------------- |
| API | last\_log\_msg                |
| 描述  | 获取当前某个日志级别的最新日志信息，通常获取最新的错误信息 |
| 参数  | log\_level：枚举类型 LogLevel 的取值  |
| 返回  | 最新日志信息的存储地址指针                 |

# AidLite C++ 示例程序

> **💡注意**
> 
> 使用 Aidlite-SDK 开发需要了解如下事项：
> 
> Aidlite 的功能实现分为两个部分，软件包 Aidlite-SDK 是为开发者提供接口的，而不同框架的推理功能则是由不同的后端软件包（如Aidlite-QNN236等）提供。
> 
> 示例程序由后端推理软件包提供，示例程序随软件包安装，如 Aidlite-QNN236 的示例程序路径如下：
> 
>   - cpp 示例程序路径：/usr/local/share/aidlite/examples/aidlite\_qnn236/cpp/
>   - Python 示例程序路径：/usr/local/share/aidlite/examples/aidlite\_qnn236/python/

以后端程序 Aidlite-QNN236 为例，其 CPP 示例程序大致包含如下几个部分：

### 初始化配置与模型加载

如果开发者对模型推理有所了解，或者是看过 Aidlite API 接口文档，那么必然会知道，在真正实现推理操作之后，通常都需要设置一些关键信息，然后再为推理框架加载所需的模型文件。

    // 此处可以使用不同数据精度的模型文件
    if(acc_type == 1){
        model_path = "../../data/qnn_yolov5_multi/libcutoff_yolov5s_640_sigmoid_fp32.qnn236.aarch64.gcc9_4.so";
    }else if(acc_type == 2){
        model_path = "../../data/qnn_yolov5_multi/libcutoff_yolov5s_640_sigmoid_fp32.qnn236.aarch64.gcc9_4.so";
    }else if(acc_type == 3){
        model_path = "../../data/qnn_yolov5_multi/cutoff_yolov5s_640_sigmoid_w8a8.qnn236.ctx.bin";
    }else if(acc_type == 4){
        model_path = "../../data/qnn_yolov5_multi/libcutoff_yolov5s_640_sigmoid_w8a8.qnn236.aarch64.gcc9_4.so";
    }else{
        show_help_prompt();
        return EXIT_FAILURE;
    }
    
    // 创建 Model 对象实例，可以设置模型相关的信息
    Model* model = Model::create_instance(model_path);
    if(model == nullptr){
        printf("Create Model object failed.\n");
        return EXIT_FAILURE;
    }
    
    // 创建 Model 对象实例
    Config* config = Config::create_instance();
    if(config == nullptr){
        printf("Create Config object failed.\n");
        return EXIT_FAILURE;
    }
    
    // 配置一些运行时所需要的参数
    config->framework_type = FrameworkType::TYPE_QNN236;
    if(acc_type == 1){
        config->accelerate_type = AccelerateType::TYPE_CPU;
    }else if(acc_type == 2){
        config->accelerate_type = AccelerateType::TYPE_GPU;
    }else if(acc_type == 3){
        config->accelerate_type = AccelerateType::TYPE_DSP;
    }else if(acc_type == 4){
        config->accelerate_type = AccelerateType::TYPE_DSP;
    }else{
        show_help_prompt();
        return EXIT_FAILURE;
    }
    
    // 创建推理所需的解释器对象
    std::unique_ptr<Interpreter>&& qnn_interpreter = InterpreterBuilder::build_interpretper_from_model_and_config(model, config);
    if(qnn_interpreter == nullptr){
        printf("build_interpretper_from_model_and_config failed.\n");
        return EXIT_FAILURE;
    }
    
    // 完成资源初始化的操作
    int result = qnn_interpreter->init();
    if(result != EXIT_SUCCESS){
        printf("interpreter->init() failed.\n");
        return EXIT_FAILURE;
    }
    
    // 完成模型的加载操作
    result = qnn_interpreter->load_model();
    if(result != EXIT_SUCCESS){
        printf("interpreter->load_model() failed.\n");
        return EXIT_FAILURE;
    }

在上述示例代码中，开发者可以提供 ***不同精度的模型文件（包括非量化的 float32 模型、量化的 int8 模型等）*** 来创建 Model 对象。相应的，开发者也需要为此指定不同的加速硬件，也就是需要设置 Config 对象的 accelerate\_type 成员变量的值。利用 Model 对象和 Config 对象创建推理所需的解释器，然后即可完成所需的初始化、模型加载的操作。

### 输入数据的前处理

一般来讲，模型推理所需的输入数据需要满足一定的要求，所以就需要对已有的输入数据做一些处理，使其可以被模型利用。

    // 使用 Opencv 读取图片数据
    cv::Mat frame = cv::imread(image_path);
    
    // 输入图像的前处理操作
    // ... ...
    
    // 取得 Opencv Mat 对象所含数据的指针，并将其转为解释器 API 所需要的指针类型。
    void* input_tensor_data = (void*)input_data.data;

***前处理部分的内容究竟怎么实现，完全取决于开发者使用的模型需要什么样的输入数据。*** 示例程序中的前处理操作，就是针对我们示例程序提供的模型文件所适配的。

### 完成推理操作

前处理操作完成之后，就可以将得到的数据传入给解释器，解释器就可以利用这些数据来完成推理任务，而在推理完成之后，开发者需要取出推理操作的结果数据。

    // 将前处理完成后的数据传递给解释器对象
    // result = qnn_interpreter->set_input_tensor(0,input_tensor_data);
    result = qnn_interpreter->set_input_tensor("images",input_tensor_data);
    if(result != EXIT_SUCCESS){
        printf("interpreter->set_input_tensor() failed.\n");
        return EXIT_FAILURE;
    }
    
    // 执行推理过程
    result = qnn_interpreter->invoke();
    if(result != EXIT_SUCCESS){
        printf("interpreter->invoke() failed.\n");
        return EXIT_FAILURE;
    }
    
    // 模型有三个输出 Tensor ，所以需要获取三个 Tensor 的输出数据。
    uint32_t output_tensor_length_0 = 0;
    // result = qnn_interpreter->get_output_tensor(0, (void**)&stride8, &output_tensor_length_0);
    result = qnn_interpreter->get_output_tensor("_326", (void**)&stride8, &output_tensor_length_0);
    if(result != EXIT_SUCCESS){
        printf("interpreter->get_output_tensor() 0 failed.\n");
        return EXIT_FAILURE;
    }
    printf("sample : interpreter->get_output_tensor() 0 length is [%d].\n", output_tensor_length_0);
    
    uint32_t output_tensor_length_1 = 0;
    // result = qnn_interpreter->get_output_tensor(1, (void**)&stride16, &output_tensor_length_1);
    result = qnn_interpreter->get_output_tensor("_364", (void**)&stride16, &output_tensor_length_1);
    if(result != EXIT_SUCCESS){
        printf("interpreter->get_output_tensor() 1 failed.\n");
        return EXIT_FAILURE;
    }
    printf("sample : interpreter->get_output_tensor() 1 length is [%d].\n", output_tensor_length_1);
    
    uint32_t output_tensor_length_2 = 0;
    // result = qnn_interpreter->get_output_tensor(2, (void**)&stride32, &output_tensor_length_2);
    result = qnn_interpreter->get_output_tensor("_402", (void**)&stride32, &output_tensor_length_2);
    if(result != EXIT_SUCCESS){
        printf("interpreter->get_output_tensor() 2 failed !\n");
        return EXIT_FAILURE;
    }
    printf("sample : interpreter->get_output_tensor() 2 length is [%d].\n", output_tensor_length_2);

前处理完成之后，就可以将输入数据给到解释器，由解释器完成推理任务，进而返回结果数据。

> **💡注意**
> 
> 需要说明的是，对于 SNPE/QNN 两个推理框架来说，输入输出的数据只能是 float32 数据类型。
> 
> 不 管模型本身的输入 Tensor 是什么精度的数据类型，使用 set\_input\_tensor() 接口时传入的数据必须在前处理阶段处理成 float32 类型的数据；同理，也不管模型本身的输出 Tensor 是什么精度的数据类型，使用 get\_output\_tensor() 接口时获取到的数据，也都是按 float32 数据类型存储的结果。
> 
> 如果是其他的推理框架，那么前处理与后处理的数据类型，则与模型文件的输入输出 Tensor 的数据类型保持一致。

### 输出数据的后处理

一般来讲，模型推理结束之后，输出的结果数据不会是我们最终想要的数据，还需要做进一步的加工才是开发者所需要的，这就是后处理操作。

    // 输出数据的后处理操作
    // ... ...

***与前处理操作同理，后处理部分的内容究竟怎么实现，完全取决于开发者需要对推理结果数据做什么样的分析处理。*** 示例程序中的后处理操作，就是针对我们示例程序提供的模型文件所适配的。

### 资源释放

当开发者不再需要 Aidlite 解释器来完成推理任务时，可以显示地完成资源释放操作。

    result = qnn_interpreter->destory();
    if(result != EXIT_SUCCESS){
        printf("interpreter->destory() failed.\n");
        return EXIT_FAILURE;
    }

至此，已经展示了使用 Aidlite 做模型推理的全部流程，开发者只需要根据自己的模型做对应的修改，即可完成推理任务。
