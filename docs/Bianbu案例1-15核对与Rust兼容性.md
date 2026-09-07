# Bianbu 案例 1–15 核对与 Rust 兼容性

核对日期：2026-09-07。已完整阅读 1–15 篇文字正文，并追查关键驱动和视觉源码；未逐一观看视频、下载全部附件或执行教程。

## 结论

这些资料对应 **Muse Pi Pro / Bianbu 平台及若干独立外设**。与 XT-STCAR 产品 PDF 的主板名称相符，但没有证明示例的接线、下位机或外设就是这台车。15 篇正文均未出现 XT-STCAR 型号。

资料有实际价值：补充板卡硬件资料、Humble 安装与相机接口、IMU 驱动入口、PWM/内核开发，以及厂商原生推理库和 YOLO26 示例。仍缺整车协议、实际配件型号与标定、目标库兼容验证。

**继续以 Rust 为主可行，但不能把目标设备兼容性视为已经验证。** C++/Python 是教程示例语言；能否集成主要取决于系统接口、ABI、消息/报文契约和库版本。

## 逐篇结果

下表编号链接到本次核对的官方源文件。

| 篇目 | 能补上的部分 | 对本工程的适用边界 |
|---|---|---|
| [01 主板教学资料][01] | Muse Pi Pro V1.1 原理图、BOM、PCB 位号图；刷机、串口、系统开发入口 | 板卡 BOM 不是整车 BOM；实际板卡修订版与车上接线待确认 |
| [02 ROS2 使用说明][02] | 厂商 noble-ros / Humble、发布订阅、colcon、USB 相机使用 | 不能据 Ubuntu 24.04 推断 Jazzy；未提供本车消息定义或 Rust ROS 交叉链接环境 |
| [03 gpiozero][03] | GPIO 编号、lgpio、输入输出与 PWM；指出软件 PWM 的舵机抖动问题 | GPIO 编号不是排针脚序号；没有本车执行器线路或转向映射 |
| [04 外设模块][04] | USB 串口、I²C、测距和云台示例与源码入口 | TOF Mini 是单点测距模块，不能替代本车 360° 雷达；GPIO73 云台示例不能作本车转向参数 |
| [05 AI SDK][05] | K1 平台支持、ai-sdk 源码、C++/Python 与 HTTP/WS 接口、视觉组件入口 | 最值得追查的推理集成资料；SDK 构建会安装依赖，本文没有证明本工程目标库兼容 |
| [06 AI 基础模块][06] | 厂商运行时、模型目录、模型下载脚本、视觉 API 示例 | 旧 YOLOv8 示例有 NMS；不能复制成当前 YOLO26 one-to-one 后处理 |
| [07 综合案例目录][07] | 七类案例和源码包导航 | 是案例索引，不是整车 SDK 或底盘接口规范 |
| [08 AI 聊天机器人][08] | ASR、Ollama、LLM、TTS 的组合示例 | 可作以后语音交互参考；不补相机、底盘或 YOLO26 接口 |
| [09 人脸识别][09] | 摄像头录入、样本保存、识别程序入口 | 是额外应用；正文未明确模型张量与完整运行库版本 |
| [10 语音控制电机][10] | 语音指令到舵机的示例 | 实际控制 GPIO 舵机；没有 STM32 底盘协议，也未确认与本车转向机构匹配 |
| [11 数字识别][11] | OCR 检测/识别、数码管串联与源码包入口 | 可用于有数字识别要求的赛项；正文没有给出数码管完整串口协议 |
| [12 IMU 惯导][12] | bros 包入口、rdk_sensors/wit_imu、带/不带 TF 的 launch | 有驱动查找方向；未证明车载 IMU 同型号，也没有补齐原始报文、坐标和标定 |
| [13 驱动开发][13] | Linux 6.6、设备树、PWM7/GPIO37、sysfs PWM、内核模块交叉编译 | 是板级 PWM 实验；Linux x86_64 工具链不能在 Apple Silicon Mac 原生运行，内核配置也不是本车已验配置 |
| [14 EtherCAT 电机][14] | IgH、CiA402、ROS2 JointTrajectory/JointState 集成参考 | 明确演示杰美康 IHSS42-24-05-EC 独立电机；没有证据表明 XT-STCAR 采用这一接口 |
| [15 常见问题][15] | RISC-V 库架构要求、spacemit-ort 安装/混装问题、相机与串口排查 | 与第 06 篇混装 ORT 的示例存在需区分的版本背景；不据此修改已验证的 Mac 环境 |

## 本次最重要的新发现：厂商已有原生库与 YOLO26 路线

第 05 篇链接的 [model-zoo-vision README][vision] 明确列出 `libonnxruntime.so` 和 `libspacemit_ep.so`，安装示例包含 `spacemit-onnxruntime`。所以当前状态应更新为：**已经找到目标原生库的官方入口，尚未取得并验证适配本工程的目标库文件。**

同一 README 报告 K1、引擎 2.0.6、YOLO26n、640×640 INT8、四核 18.1 FPS（含前后处理），这是厂商报告，不是本工程实测。本工程仍保留已验证的 320×320 FP32 模型。

[原生会话源码][session] 使用 `onnxruntime_cxx_api.h` 和厂商 `spacemit_ort_env.h`，启用加速时调用 `Ort::SessionOptionsSpaceMITEnvInit`。当前 Rust 绑定需要标准 ORT API 22；这些源码没有证明目标库提供该版本，也没有证明仅加载库就会启用厂商加速。

[YOLO26 实现][yolo26] 消费 `[N,6]` / `[1,N,6]` 的六列结果，后处理不另加 NMS，与本工程方向相近。但厂商 README 的“已含 NMS”文字不能证明其模型图含该算子；未下载并检查厂商 `.q.onnx`，不改写本工程已核验的无 NMS 图契约。

## Rust 的兼容性如何处理

| 模块 | Rust 路线 | 还需验证什么 |
|---|---|---|
| 核心算法、状态机、日志、预后处理 | 保持现有 Rust 实现 | 已通过本机测试与 RISC-V 交叉链接；尚无实机运行结果 |
| ONNX 推理 | 优先 Rust 调用稳定 C API；厂商扩展必要时增加少量 C/C++ 适配 | 目标 ELF/依赖、OrtGetApiBase、GetApi(22)、厂商 EP 初始化与实际模型推理 |
| C++ 视觉 SDK | 需要时使用薄的 C ABI 桥接；C++ 类和 cv::Mat 不能当 C 函数直接调用 | 接口生命周期、内存所有权、异常边界、ABI 与目标依赖 |
| USB 相机与串口 | Linux 用户态接口可由 Rust 使用；设备协议确定后用 Rust 解析 | 真实设备身份、像素格式、帧率、报文、单位与超时；不要求将内核驱动重写为 Rust |
| ROS 驱动 | 先复用厂家节点，通过消息接入 Rust；rclrs 需单独验证 | 实际 ROS/DDS 版本、消息生成、目标 C 库/sysroot、QoS 与时间戳 |
| 只有 Python 接口的功能 | 可通过受控进程或消息适配，或依据公开协议实现 Rust 版本 | Python 包不是可直接加载的 Rust 库；不为语言统一盲目重写未知硬件协议 |

因此保留“大部分由 Rust 完成”的方案：业务、协议解析和安全控制采用 Rust；成熟内核驱动及必要的厂商推理引擎保留，通过明确接口连接。

## 还缺哪些整车信息

1. XT-STCAR 整车接线/BOM、主板和下位机修订版、STM32 完整型号与原厂固件/协议。需明确报文、校验、速度/转角单位、反馈、断连停止行为。
2. 澄清 PDF 同时写“阿克曼”和“四轮差速”的矛盾，取得轴距、轮径、转向零点、驱动映射及里程计来源。
3. 实际雷达、IMU、相机型号和设备身份，坐标系、刷新率、时钟来源及内外参。官方 [UVC 相机节点][camera]、[CMP10A IMU][imu]、[YDLidar/RPLidar 入口][lidar] 已找到，均需对照实物选用。
4. 实际系统镜像、内核、ROS 环境以及匹配的 RISC-V 运行库；完成 API/算子/性能验证。板卡通用镜像入口不能替代整车出厂恢复包。

## 关于读取 STM32F103 原厂固件

固件读取是后续获取底盘协议的可选途径，当前仍未连接车辆。先核实完整料号、芯片来源和读保护状态；未启用读保护时可通过调试接口备份可读 Flash，取得的是机器码，仍需要反汇编和通信数据对照才能还原协议。

ST [PM0075 第 2.4.1 节（第 17–18 页）][st-flash] 明确说明：启用读保护后，普通 JTAG/SWD 不能读取主 Flash；将保护状态改为未保护会触发主 Flash 全片擦除。不能把编程器的解除保护功能当作保留原固件的读取方式。也不能只凭 F103 系列名称断定某种绕过方法必然适用于实际芯片。

本工程需要的是可靠的底盘通信契约。可先检查出厂主控上的驱动、配置、升级固件文件，随后结合被动通信记录和可读的固件备份分析；不要求先把 STM32 固件改成 Rust。未授权执行解保护、擦除或刷写。

## 存档与本轮操作

- 15 篇正文及目录锁定 docs-events 提交 `641614ae96465851f0e27f24ae96948aeb0ff333`；下载字节已逐文件核对 SHA-256 和 Git blob SHA-1。
- 视觉参考锁定 model-zoo-vision 提交 `e3cb7c61174ac42ec2f6da407bc3fe6ac5067e42`，归档 README、LICENSE 与相关源码；原始来源和哈希写入 SOURCES.json。
- 工程存档位于 `资料/官方参考/Bianbu使用文档及案例集_2026-09-07/`、`SpacemiT_model-zoo-vision_2026-09-07/`、`Bianbu传感器参考_2026-09-07/`。docs-events 此提交完整 Git 树未发现许可证文件，未套用其他仓库许可，也未放入软件部署包。
- 此轮只改资料与交接文档；未安装教程依赖、替换内核、修改 Rust 代码、连接车辆或执行电机示例。既有构建和部署包保持原版本。

[vision]: https://github.com/spacemit-com/model-zoo-vision/blob/e3cb7c61174ac42ec2f6da407bc3fe6ac5067e42/README.md
[session]: https://github.com/spacemit-com/model-zoo-vision/blob/e3cb7c61174ac42ec2f6da407bc3fe6ac5067e42/src/core/cpp/vision_model_base.cpp
[yolo26]: https://github.com/spacemit-com/model-zoo-vision/blob/e3cb7c61174ac42ec2f6da407bc3fe6ac5067e42/src/deploy/yolo26/cpp/yolo26_detector.cpp
[camera]: https://github.com/spacemit-com/docs-ros/blob/main/zh/k1/06_Robot_development/6.2_ROS2_API/6.2.1_Camera_System/1_USB_Camera_Node.md
[imu]: https://github.com/spacemit-com/docs-ros/blob/main/zh/k1/06_Robot_development/6.2_ROS2_API/6.2.4_Sensor_System/2_CMP10A_IMU.md
[lidar]: https://github.com/spacemit-com/docs-ros/blob/main/zh/k1/06_Robot_development/6.2_ROS2_API/6.2.4_Sensor_System/1_Using_LiDAR.md
[st-flash]: https://www.st.com/resource/en/programming_manual/cd00283419-stm32f10xxx-flash-memory-microcontrollers-stmicroelectronics.pdf

[01]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/01_Muse_Pi_Pro%E6%95%99%E5%AD%A6%E8%B5%84%E6%96%99.md
[02]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/02_ROS2%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.md
[03]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/03_gpiozero%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.md
[04]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/04_%E5%A4%96%E8%AE%BE%E6%A8%A1%E5%9D%97%E5%8F%82%E8%80%83%E8%AF%B4%E6%98%8E.md
[05]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/05_SpacemiT%20AI%20SDK.md
[06]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/06_AI%E5%9F%BA%E7%A1%80%E6%A8%A1%E5%9D%97%E5%8F%82%E8%80%83%E8%AF%B4%E6%98%8E.md
[07]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/07_%E7%BB%BC%E5%90%88%E5%BA%94%E7%94%A8%E6%A1%88%E4%BE%8B%E7%9B%AE%E5%BD%95.md
[08]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/08_%E6%A1%88%E4%BE%8B1%E2%80%94AI%E8%81%8A%E5%A4%A9%E6%9C%BA%E5%99%A8%E4%BA%BA.md
[09]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/09_%E6%A1%88%E4%BE%8B2%E2%80%94%E4%BA%BA%E8%84%B8%E8%AF%86%E5%88%AB%E5%BA%94%E7%94%A8.md
[10]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/10_%E6%A1%88%E4%BE%8B3%E2%80%94%E8%AF%AD%E9%9F%B3%E6%8E%A7%E5%88%B6%E7%94%B5%E6%9C%BA.md
[11]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/11_%E6%A1%88%E4%BE%8B4%E2%80%94%E6%95%B0%E5%AD%97%E8%AF%86%E5%88%AB%E7%B3%BB%E7%BB%9F.md
[12]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/12_%E6%A1%88%E4%BE%8B5%E2%80%94IMU%E6%83%AF%E5%AF%BC%E6%A8%A1%E5%9D%97%E4%BD%BF%E7%94%A8.md
[13]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/13_%E6%A1%88%E4%BE%8B6%E2%80%94%E9%A9%B1%E5%8A%A8%E5%BC%80%E5%8F%91%E5%92%8C%E4%BD%BF%E7%94%A8.md
[14]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/14_%E6%A1%88%E4%BE%8B7%E2%80%94%E5%9F%BA%E4%BA%8EEtherCAT%E6%8E%A7%E5%88%B6%E7%94%B5%E6%9C%BA.md
[15]: https://github.com/spacemit-com/docs-events/blob/641614ae96465851f0e27f24ae96948aeb0ff333/zh/%E7%AB%9E%E8%B5%9B%E6%95%99%E7%A8%8B/01_Bianbu_%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3%E5%8F%8A%E6%A1%88%E4%BE%8B%E9%9B%86/15_%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E8%A7%A3%E7%AD%94.md

## 后续进展：原生库与 GLIBC 2.38

上述案例阅读记录之后，已取得官方 RISC-V ORT/EP 发布包，核验 C 导出、API 24 头文件和 GLIBC 2.38 依赖；见 [原生库核验](SpacemiT原生运行库核验.md)。此前“未取得库”的状态已更新，实际 API22 加载和模型推理仍未验证。用户随后要求先升级 GLIBC 2.38，已更新本地构建目标；见 [升级记录](GLIBC-2.38升级记录.md)。用户已要求暂不处理 MCU 固件读取。
