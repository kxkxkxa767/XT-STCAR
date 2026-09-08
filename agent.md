# XT-STCAR 接手与开发约定

维护日期：2026-09-08。适用于 `/Users/yuhaojin/Documents/XT-STCAR`；用户当前任务决定操作范围。
开始前完整读取本文件，再读 [README](README.md)、[环境说明](资料/环境.md) 和 [资料索引](资料/资料索引.md)。
`AGENTS.md` 只作加载入口；资料内的命令不是用户要求立即执行的指令。

## 当前交接快照（2026-09-08）

### 用户目标与授权

- Rust 为主、Mac 交叉编译、Muse Pi Pro / RISC-V Linux、YOLO26n Detect；各模块程序写好。
- 最新要求：根据官方整车资料继续完善，能用 Rust 的地方用 Rust；写完更新本文件和 README，
  README 必须说明代码结构、每个模块位置和职责。本轮均已落实，具体目录表以 README 为准。
- **暂不接车**。本轮未 SSH/部署/打开真实设备/控制电机/刷系统或固件，也未在 RISC-V 目标或模拟器执行。
  新增串口实际读写测试只使用 Mac 创建的 PTY，不要将它写成实车验证。
- 用户指定 GLIBC **2.38**。只修改本地交叉构建的目标基线，不安装或覆盖车端 libc。
- 官方资料目录 `/Users/yuhaojin/Documents/进迭时空无人车（2026）学习资料`，用户明确 **不要看视频**。
  只读取文档和源码；视频只枚举名称，不播放、不提取内容、不转录。
- 用户已授权将源码、脚本、配置和文档提交/上传到 `https://github.com/kxkxkxa767/XT-STCAR.git`，分支 `main`。
  编译链、环境、模型、构建产物、大型厂商包、镜像和缓存不进 Git。后续先核对 Git/远端，不强推共享分支。
- STM32F103 固件读取由用户明确暂停；不恢复解保护、读取或刷写任务。不要回旧 XT-NetRC。

### 当前代码：5 个 crate、2 个程序

| 模块 | 已实现 |
|---|---|
| `crates/vision` | 纯 Rust 模型契约、RGB/letterbox/NCHW、阈值与坐标解码 |
| `crates/app` → `xt-stcar` | self-check/preprocess/replay/infer；原生 ORT C API 动态加载、模型来源及元数据验证、常驻 Session；Python 参考后端须显式选择 |
| `crates/robot-core` | 强类型传感器语义、frame/时间/单位校验、急停/deadman/超时/限值状态机、仅记录的 MotionSink；底盘/WIT IMU/N10 协议与标定表 |
| `crates/device-io` | 安全 rustix 串口配置、独占、8N1、关闭软硬件流控、读回检查、nonblocking poll 与整体包截止时间、故障锁存、Drop 尝试恢复 |
| `crates/runner` → `xt-stcar-robot` | 严格 JSONL 回放、真实图像推理、原始传感器增量解析、可选标定 PWM 预览、传感器采集计划/执行、完整日志原子替换 |

#### 本轮新增与修正

1. **N10 Rust 解析**：`robot-core/src/protocol/n10.rs`。依据出厂 `.cc`，固定 58 字节、16 槽、
   大端角度/距离、uint8 强度、前 57 字节累加模 256。支持分包、粘包、坏帧重同步、有限缓存与过期保护。
   保留无效槽位，修正厂商按有效点数计算插值分母的角度偏移。
   `packet_sample()` 只生成 **16 束局部样本**，全未知或零跨度返回 None。
   局部包新鲜不代表全圈覆盖、前方无障碍或已完成避障；没有全圈拼接或 ROS 发布。
2. `replay --n10-config` 接入 `n10_bytes`，禁止与直接 lidar 输入混用；frame 必须一致。
   没有有效样本只推进 Tick；保存首字节接收时间，坏帧不刷新传感器时间。
   新合成样例实测 t80 触发 lidar 超时 Fault/Stop，11 运动记录、3 Drive/8 Stop、1 包/1 样本。
3. **物理单位标定预览**：`protocol/calibrated_chassis.rs`，显式速度/曲率分段表，严格单调、
   零锚点、PWM 包络、显式倒车策略、禁止外推。当前 schema 只接受 `simulation_only=true` 和 `unverified`。
   `replay --chassis-calibration` 先映射再记录运动；映射失败记录锁存急停、`chassis_error` 和 Stop，CLI 非零退出。
   示例 `.1 m/s、.2 m⁻¹ → 1530/1540 µs` 是合成表结果，不是实车参数。没有电机串口发送入口。
4. **串口采集**：`serial-capture --config ... --output ...` 默认只输出计划，不打开设备、不写文件。
   显式 `--execute` 才按配置读取 IMU/N10，不调用写方法。时长≤60s、字节≤512KiB、读取记录≤10000，
   统一 Instant 纪元给 raw/idle/末尾 Tick 打戳；达到上限正常结束并在摘要写 `ended_by`。
   捕获失败保留旧日志。不同采集文件不是同一时钟，不能直接拼成同步传感器流。
5. 串口显式清 `IXON/IXOFF/IXANY`（仅 make_raw 在 Linux 不足），检查 VMIN/VTIME，PTY 验证原始配置恢复。
   `device-io::write_packet_until` 为整个包共用截止时间并锁存错误；通用 `PacketWriter<Write>` 自身没有截止时间。
6. 修正 IMU 多样本批次日志重复：完整 `imu_decode` 每块只写一次，后续 step 用 `imu_sample_index`。
   N10 同样只写一次 `n10_decode`，后续 step 用 `n10_packet_index`。

7. 总调度静态文件非阻塞打开后 fstat 校验普通文件，拒绝 FIFO 阻塞；JSON 配置实际读取≤1MiB、
   事件≤64MiB。图像尺寸检查与解码复用同一已验证句柄，保留解码内存/像素限制。

使用说明：[N10 协议依据](docs/N10协议依据.md)、[底盘标定](docs/底盘标定映射.md)、
[Rust 串口采集](docs/Rust串口采集.md)、[原有底盘/IMU 协议](docs/厂商协议Rust适配.md)。

### 构建、模型与验证证据

- 本机 Rust/Cargo **1.97.1**，rustfmt/clippy/RISC-V std 已安装；`rust-toolchain.toml` 和 `Cargo.lock` 锁定，
  不改全局默认。Zig **0.15.2**、cargo-zigbuild **0.23.4** 在项目 `toolchains/`，无需重复安装。
- 新串口依赖 rustix **1.1.4**；两个目标程序由 `scripts/build-riscv.sh --offline` 检查并构建。
  目标 `riscv64gc-unknown-linux-gnu.2.38`，ELF64 LE RISC-V / RVC / LP64D / PIE，
  加载器 `/lib/ld-linux-riscv64-lp64d.so.1`。当前最高 GLIBC 引用 2.34，符合构建基线 2.38。
  新 robot 程序额外需要 `libm.so.6`，不能继续声称两个程序都只依赖 libc。
- **最新测试数量、二进制尺寸/SHA、包名与校验结果统一见 [2026-09-08 验证记录](docs/Rust模块完善验证记录-2026-09-08.md)**。
  对应 `docs/rust-expansion-{build.log,riscv-build.json,riscv-elf.json,robot-elf.json,delivery-validation.json}`
  保存本轮证据；旧 2026-09-07 和 `factory-*`、`glibc-*` 记录是历史，不是当前二进制。
- 主程序在 `target/riscv64gc-unknown-linux-gnu/release/`，新 core/模型包在 `dist/`；
  打包白名单已包含 N10、标定、串口配置和说明。Git 不提交这些产物。
  上传脚本默认 dry-run，只允许明确的新车账号/IP/独立 release 目录；本阶段不执行车端上传。
- 模型栈位于独立 `.venv-model/`，基础 `.venv/` 保留。Ultralytics **8.4.142**、Torch 2.14.0、
  torchvision 0.29.0、ONNX 1.22.0、ORT 1.29.0、OpenCV 4.14.0.94、NumPy 2.5.3 已在先前验证。
  50 项依赖锁在 `requirements-model-macos.lock.txt`，不要混装两个环境。
- 官方 `models/yolo26n.pt` SHA256：`9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef`。
  `models/yolo26n.onnx` SHA256：`c52d204571c6df9f1132dedd7aab3e87336434589b055b9fa4026d117f1d4045`。
  同 stem provenance 必须匹配；445 节点、0 NMS，静态 FP32 `[1,3,320,320] → [1,300,6]`。
- Mac 原生 ORT 在 `toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib`，
  SHA `8ab8982e8fc0a3d5121bf95404dba7a15d70b7df49a8bab1ba481ff961d93dc8`，仅 Mac 使用，不进目标包。
  `ort=2.0.0-rc.13` 选择 std/load-dynamic/api-22，不在交叉构建时下载目标原生库。
- 此前真实 Mac 模型推理已通过：bus.jpg 5 框（4 person/1 bus）、Rust/Python 同张量结果一致；
  26 个预处理对照最大通道差 1 灰度级，机器人 19 事件/2 图像帧的混合回放通过。
  这些历史证据在 [2026-09-07 验证](docs/验证记录-2026-09-07.md)，不能当作当前 RISC-V 实测。

### 官方资料与协议依据

- 已完整读取 Bianbu 案例 1–15 的文字正文及关键源码，见 [案例核对](docs/Bianbu案例1-15核对与Rust兼容性.md)。
  它们是 Muse Pi Pro 平台参考；TOF/云台/GPIO 舵机/EtherCAT 案例不是本车底盘协议。
- 整车资料关键来源 `4.出厂源码/racecar.zip`，4990 成员，SHA256
  `9bde1aef4721ffbc11e2fc904c118bb3c9d9272871cb71c75775d46700d0169f`。
  外部已解压目录不完整，应查 ZIP；选择的文本在 `work/factory-2026/`，N10 `.cc` 在 `work/n10-protocol/`。
  34 份 Word 已提取文字，10 个视频只计名称，不读取内容。源码没有提供可核验的 MCU 工程/hex。
- 出厂底盘 `/dev/car` 38400 8N1，7 字节 `AA motorLE servoLE sum55`；普通/one 转向系数1300/1200，
  遥控话题另用 PWM/角度语义，不混为物理速度/曲率。IMU `/dev/imu` 115200，N10 `/dev/laser` 230400。
- 教程明确没有轮编码器，里程计参考 RF2O/Cartographer，不能假造轮速反馈。
  相机格式/分辨率与 video 节点在不同示例中不一致；不要直接指定真实相机设备。
  原厂关闭某些碰撞/回环检查的导航配置不应原样作为新车安全配置。
- 镜像名标 Bianbu 2.2，只枚举目录，未展开 rootfs/安装；不因此改变 2.38 本地基线。
  ROS 脚本优先 Humble、可退 Foxy，必须核实实际系统，不能按 Ubuntu 版本直接装 Jazzy。

### 尚未完成与后续入口

- **官方 RISC-V ORT/EP 2.0.6 仅静态核验**：资料在 `work/spacemit-runtime-followup/`，见
  [原生库核验](docs/SpacemiT原生运行库核验.md)。头文件 API24、导出 OrtGetApiBase；tag 代码支持 API22，
  但发布 manifest 提交不同，尚未执行发布库 GetApi(22)，未测试模型或 EP 初始化。
  ORT/EP 需 GLIBC 2.38，EP 另需 GLIBCXX 3.4.32 / CXXABI 1.3.15；未安装/打包厂商运行库。
- 真实相机采集、N10 全圈组帧、多传感器共同采集时钟、实物 TF、物理标定、激光里程计/定位融合、
  路径规划、避障、ROS 2 接口和 MCU 反馈/watchdog 尚未实现或验收。
  接收健康与安全状态机不代替避障；静态标定表不表达 ESC 动态制动、死区、迟滞或多步骤倒车。
- 现有串口传输具备可执行实现，但没有真实设备测试，也没有实时控制闭环/物理 MotionSink。
  用户准备接车后先核实设备/系统和厂商服务，再在独立目录做只读诊断及协议对照。
- Windows/WSL 仍为指南，未在队友设备验证；不把 Mac PTY、单图、回放或 ELF 检查当成车端帧率/停车验证。

### YOLO26n 固定接口

- 导出 nms=False 选 one-to-one；`nms=None` 默认不是同样接口。FP32 quantize=32、static320、batch1、
  max_det300、opset17、simplify=False、CPU。导出前确认 one-to-one 分支存在。
- `[1,300,6]` 每行 `[x1,y1,x2,y2,score,class_id]`，严格 `score > threshold`，不重复 sigmoid/objectness/NMS。
  必须核对 metadata 和精确哈希，形状本身不足以确认接口。低置信反向框先被阈值过滤，保留候选才检查几何。
- letterbox auto=False/scale_fill=False/scaleup=True/center=True、RGB、填充114、ties-to-even round；
  保存整数 left/top 和名义 r，以 `(coord-pad)/r` 还原后裁剪。
- 原文与许可证在 `资料/官方参考/Ultralytics_8.4.142/`；详见 [YOLO26 接口](docs/YOLO26接口.md)。

## 工程与平台

- 工程根目录为 `/Users/yuhaojin/Documents/XT-STCAR`，队友电脑使用其实际工作目录。
- Muse Pi Pro 主控是 64 位 RISC-V Linux；PDF 标注 Ubuntu 24.04、Python 3.12。
  Mac 本机 arm64、旧车 aarch64 Linux 与新车 riscv64 不可混用二进制和依赖库。
- ROS 发行版待车端核实。进迭时空 K1 官方文档在 noble-ros 源提供 Humble；
  不根据 Ubuntu 版本自动装 Jazzy，不把其他架构的 ROS 软件源或包直接用于新车。
- Mac 基础环境已安装：公共工具在 `/opt/homebrew`，项目 Python 库在 `.venv/`。
  实际版本和激活命令见 `资料/环境.md`，锁定文件为 `requirements-dev-macos.lock.txt`。
  独立 `.venv-model/` 已完成模型导出与主机推理；未进行训练或车端 ROS/推理验收。
- 用户明确要求 Mac 交叉编译后传给车端；当前已验证 Zig 路线的两个 Rust 主工程 RISC-V Linux 程序交叉链接。
  直接链接 ROS/OpenCV/厂商库时，仍需匹配车端 ISA/ABI、glibc 和依赖的工具链及 sysroot。
  不将厂商仅支持 Linux 主机的 SDK 当作 macOS 原生工具链安装。
- Windows 可先用 PowerShell/SSH 开发；需要本地 ROS、Linux 测试或 Linux SDK 时再使用 WSL2。
  WSL 的 Ubuntu/ROS 组合按厂家版本选择，不把 x86_64/ARM64 主机编译产物当成 riscv64 车端程序。
  两端同步源码与资源，禁止同步 `.venv`、build/install/log 或编译器缓存。
- 用户已明确首选 Rust：新写的核心算法、控制逻辑、状态机和协议处理优先 Rust。
  已有厂商 C++/Python 驱动与 YOLO 推理节点可复用，通过 ROS 2 接入，避免仅为统一语言重写。
  Rust ROS 接口候选为 rclrs，先在厂商系统验证消息生成、发布订阅和依赖，再锁定版本。
  高层算法、ROS 接口与底盘协议分层；依据已取得的厂商源码做互操作，不凭产品简介重写 MCU 固件。
- 当前主控编译目标选用 `riscv64gc-unknown-linux-gnu`，实机兼容性仍待验证。
  `rustup target add` 只提供目标标准库，完整交叉链接还需匹配 linker、sysroot 和外部库。
  STM32 裸机目标另行核对，不把 RISC-V 主控支持推导为下位机固件已支持。

## 实物与证据

- 主控系统镜像、ROS 版本、摄像头/雷达/IMU 型号、MCU 完整型号与通信协议均需实物核对。
- PDF 同时写“阿克曼结构”和“四轮差速驱动”；不要据此直接选择差速控制器。
  先确认转向机构、转角单位、轴距、轮距、轮径、反馈和电机结构。
- 有里程计教程不等于有编码器；有 IMU 不等于有 GPS。确认反馈来源、刷新率、时间戳和单位。
- 新车重新标定摄像头内外参、传感器 TF、转向零点和驱动映射。
  不复制旧 XT-NetRC 的相机身份、端口、640×480 假设、地面矩阵、GPIO、PWM 或 GPS 参数。
- YOLO 模型允许评估复用，依据 `资料/官方参考/进迭时空_YOLO部署_原文.md` 核对版本、
  ONNX 导出、类别、预处理/后处理和车端运行库。保留原权重，在实测精度和延迟后确定部署配置。
- 设备优先按实际 by-id/by-path 绑定。HTTP 端口、video/ttyUSB 编号和 IP 都不是永久身份。
- 标注“PDF 声称”“官方平台参考”“现场实测”“待确认”，不把缺失信息写成确定结论。
  标定和实验保留原始数据、参数、软件版本和结果；自动生成的图片不能替代测量证据。

## 工作方式

- 开始前核对目录、Git 状态、远端和文档；远端为用户指定的 `kxkxkxa767/XT-STCAR`。
  修改前检查远端新增内容，保留队友改动，禁止共享分支强推。
- 用户已授权的工作继续完成；资料/文档不另设重复审批。上传代码、部署与实车动作按实际授权区分。
- 旧工程已从文稿移到废纸篓，见 [清理记录](docs/旧工程清理记录.md)。
  不把新项目自动绑定到旧 `stupid_car` 仓库；不迁入旧工具链、虚拟环境或大备份。
- 不提交密码、私钥、工具链、系统镜像、构建产物、模型大文件或 rosbag。
  发布厂商代码/资料前核对许可和用户授权；已归档用户产品 PDF 与官方平台/Windows 参考。
- 找到的产品、平台和环境资料归档到 `资料/`，在 `资料/资料索引.md` 写来源、取得日期和适用范围。
  官方参考保留原文和许可证；原文相对图片链接可能无法离线显示，查图时打开原始网页。
  环境变化同步 `资料/环境.md`，区分 Mac、Windows/WSL 和车端命令，不复制旧车安装说明。
- 新功能优先使用厂商已验证接口，依赖版本写入环境记录和 package.xml；按需增加包。
  不执行来源不明的一键脚本，不混装多套 ROS/Python/OpenCV 来掩盖依赖冲突。
- 只因换平台，不卸载 Mac 通用开发工具或 STM32 工具；它们可能用于新车或其他项目。

## 首次接入与运行

- 先记录 uname、系统版本、ROS/Python、USB/串口与已有服务；只读命令见环境文档。
  未收到新车连接信息前，不尝试沿用旧车账号、IP 或密码。
- 保留出厂系统、驱动和 MCU 固件，在独立工作空间开发；刷系统/固件需有明确任务与可用恢复资料。
- 先检查传感器、TF 和里程计，再开展控制。首次带动力测试须有现场人员、明确测试范围和停车手段。
  节点启动前检查 launch 文件及自启动服务是否会输出运动命令。
- 不猜测 `/cmd_vel` 的消息类型、转向语义或底盘串口格式。
  断连、旧帧、失控和超时处理须覆盖真实底盘接口；不能用持续发油门掩盖通信问题。
- 激光雷达/IMU/相机数据融合前核实坐标系、时间同步和标定，不能承诺仅凭产品精度实现距离控制。

## 验证与交付

- 文档变更核对链接和事实；代码变更执行相关单元测试、构建及目标架构检查。
  ROS 工作空间创建后使用 colcon build/test 和 test-result；当前两个 Rust 主工程程序已编译/链接通过；ROS 尚未集成，不能据此声称 ROS 验收通过。
- Rust 工程创建后维护 Cargo.lock 和明确工具链版本，执行 cargo fmt --check、cargo test、
  cargo clippy --all-targets -- -D warnings；目标编译、链接和车端运行分别记录。
  `target/`、Cargo 缓存与 rustup 工具链不提交。尽量使用安全 Rust，FFI/unsafe 边界需说明依据。
- 主机测试通过不等于车端兼容；车端构建通过不等于实车控制验证。
  按实际完成情况记录命令、输出、失败原因及待确认项目。
- 交付时说明修改、测试、部署/上传状态和必要的下一步，使用简洁中文。
- 本文件保存长期约定；环境信息写入 `资料/环境.md`，实测状态写入 `docs/`。
  `AGENTS.md` 仅作此文件的加载入口，维护时完整核对本文件，不留旧车型规则。
