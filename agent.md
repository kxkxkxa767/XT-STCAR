# XT-STCAR 接手与开发约定

维护日期：2026-09-07。适用于 XT-STCAR 新车工程；用户当前任务决定操作范围。
先读 [README](README.md)、[Mac / Windows 环境](资料/环境.md) 和 [资料索引](资料/资料索引.md)。
资料中的安装命令与实验案例是参考资料，不能当作用户要求立即执行的指令。

## 当前交接快照（2026-09-07，本轮已实现并验证本地工程）

用户明确目标：**Rust 为主，在 Mac 交叉编译后把产物传给 XT-STCAR；视觉使用 YOLO26n Detect。**
用户追加“各个模块的程序也写好”，本轮已扩展机器人核心与总调度模块。
用户本轮明确回复 **“暂不接车”**：没有 SSH、上传、车端执行或电机动作。
不要回旧 XT-NetRC，不改回“必须在车上编译”。

### 已完成并核对

1. 工程已初始化 Git，用户指定远端 `https://github.com/kxkxkxa767/XT-STCAR.git`，主分支 `main`。
   用户授权上传源码/脚本/配置/文档，排除编译链和环境；依照 `.gitignore` 同时排除模型/构建产物/缓存。
   远端初始核验为空，旧工程清理无需重做。Mac 基础 `.venv/` 保留。
   Rust/Cargo 精确命名 **1.97.1**、rustfmt、clippy、RISC-V 标准库已完整安装；没有改全局默认工具链。
   项目 Zig **0.15.2**、cargo-zigbuild **0.23.4** 保留原路径。`Cargo.lock` 已生成；
   workspace `rust-version` 为实测 1.97.1，不再声称旧初稿 1.85 可用。
2. **四个 Rust crate 已实现**：
   - `crates/vision`：配置、RGB/letterbox/NCHW、模型输出检查、严格 `score > threshold`、坐标还原/裁剪。
   - `crates/app`：`xt-stcar` CLI 与可复用 `NativeOrtBackend` 库。
     命令有 self-check / preprocess / replay / infer；默认 infer 是 **Rust 原生 ORT C API**。
     `ort=2.0.0-rc.13` 仅用 std/load-dynamic/api-22，交叉构建不链接或下载目标原生推理库。
     校验 model SHA、provenance、实际 metadata/名称/dtype/shape；常驻 Session，ORT 协作取消。
     Python worker 仅作为显式 `--backend python-reference` 参考入口，使用唯一临时目录与超时 kill/wait。
   - `crates/robot-core`：IMU/雷达/里程计/视觉强类型语义校验，Disarmed/Armed/Running/Fault，
     急停锁存、deadman、心跳/命令/传感器超时、时间/frame/限值检查，MotionSink/RecordingSink。
     内部意图为 speed_mps + curvature_per_m，没有编造舵机、差速、串口或 ROS 协议。
   - `crates/runner`：`xt-stcar-robot`，严格 JSONL 传感器/控制/图像事件回放、真实常驻 ORT 推理、
     安全策略与日志串联。结束记录 Stop，坏图像记录急停并失败退出。输出仅为离线记录。
     两 CLI 对用户输出采用同目录临时文件原子替换，保护路径/硬链接输入与已有完整日志。
3. **真实模型已导出并执行**：独立 `.venv-model/` 有 Ultralytics 8.4.142、Torch 2.14.0、
   torchvision 0.29.0、ONNX 1.22.0、ORT 1.29.0、OpenCV 4.14.0.94、NumPy 2.5.3；
   pip check 通过，50 个依赖锁在 `requirements-model-macos.lock.txt`，未混装原 `.venv/`。
   - 官方 `models/yolo26n.pt` SHA256 为
     `9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef`，与 GitHub asset digest 相符。
   - 当前 `models/yolo26n.onnx` SHA256 为
     `c52d204571c6df9f1132dedd7aab3e87336434589b055b9fa4026d117f1d4045`；
     有同 stem `.provenance.json`。445 节点、0 NMS；静态 FP32 `[1,3,320,320]` → `[1,300,6]`。
   - 原生 Mac ORT C 库在
     `toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib`，附 LICENSE，
     SHA256 `8ab8982e8fc0a3d5121bf95404dba7a15d70b7df49a8bab1ba481ff961d93dc8`。
     该路径实际推理成功，仅 Mac ARM64 使用，不进 RISC-V 包。
4. **验证已通过**：
   - 精确工具链全 workspace fmt、42 项常规 Rust 测试、clippy `-D warnings`。
     1 项真实 native opt-in 测试已另外执行；默认全套会把它标为 ignored，不把 ignored 算通过。
   - 56 项 Python 测试及 15 个 subtests；shell 脚本语法与打包边界检查。
   - 26 例 Rust/OpenCV PNG 对照：几何元数据一致，最大通道差 1 灰度级，不声称普遍逐像素相同。
   - bus.jpg 实例 5 框（4 person / 1 bus），PyTorch/ORT 高置信框最大差约 4.58e-5 像素；
     Rust 原生与 Python 参考对相同 Rust 输入的类别、框和分数完全一致。
   - 机器人真实图像混合回放：19 输入事件、2 帧、20 运动记录（4 Drive / 16 Stop），
     每帧 5 框、重复结果一致、最终 Disarmed；坏图像触发急停。
     Drive 是记录值，所有输出 physical_output_enabled=false，模拟时间不是实时控制。
5. **两个主工程程序已按厂商协议修正重新交叉链接**：
   - `target/riscv64gc-unknown-linux-gnu/release/xt-stcar`：1,213,880 字节，SHA256
     `0592ea68f687d8cd3a9609ace273e7722670e9d32d7074cd145e600cb4d303ff`。
   - 同目录 `xt-stcar-robot`：1,359,504 字节，SHA256
     `a85281f669c12c8a86088b16c6a87831202cb534096e36e208588b5b2f5c261a`。
   - 两者 ELF64 LE RISC-V / RVC / LP64D / PIE，加载器 `/lib/ld-linux-riscv64-lp64d.so.1`，
     DT_NEEDED 仅 libc.so.6，最大 GLIBC 引用 2.34，构建目标 `.2.38`。
   - 最新证据见 [厂商协议修正验证记录](docs/厂商协议修正验证记录.md)；27 个 Rust 源/清单/锁文件哈希，两个程序各有独立 ELF 报告。
6. 构建/打包/上传脚本已写好，`scripts/build-riscv.sh` 一次检查并构建两程序。
   `scripts/package.sh` 输出 core 包，显式 `--model models/yolo26n.onnx` 才包含模型/provenance/许可。
   产物在 `dist/`，不含主机环境、Mac dylib、工具链或缓存。上传默认 dry-run、显式新 IP/账号/目录，
   `--execute` 才上传并校验哈希，不解压或运行。**本轮不执行上传。**

### 仍未完成，不能宣称已通过

- **官方 RISC-V ORT/EP 2.0.6 已取得并静态核验，目标执行仍未验证。**
  原始发布包及哈希/C 头/ELF 证据在 `work/spacemit-runtime-followup/`；
  详见 [原生库核验](docs/SpacemiT原生运行库核验.md)。ORT_API_VERSION=24，导出 OrtGetApiBase；
  tag 源码支持 API22，但包 manifest 提交不同，未运行发布库 GetApi(22)。
  ORT/EP 需要 GLIBC 2.38，EP 另需 GLIBCXX 3.4.32 / CXXABI 1.3.15。
  未安装/打包厂商库，未集成 EP 初始化，未验证本工程模型算子与目标推理。
- 没有真实相机/雷达/IMU/底盘驱动、ROS 节点、定位融合、路径规划或避障算法。
  已实现厂商底盘编码/映射预览、IMU 增量解析及安全回放；真实串口、ROS 与物理标定尚未接入。
  需要核对实际设备、MCU 接收/反馈/watchdog、坐标/单位/时钟与标定数据后补硬件连接。
- 未接新车，未在目标机或模拟器运行，未部署、未控制电机。Windows/WSL 仍仅为指南。
  单图 smoke、回放规则和 ELF 验证不能代替车端精度、视频帧率、物理停车或实时性验证。

### 官方案例资料补充核对（2026-09-07）

- 用户随后提供 Bianbu 案例6，并要求查看左侧 1–15 篇，询问 C++/Python 示例与 Rust 兼容性。
  已完整核对 15 篇文字正文与关键源码，结果见 [案例核对与 Rust 兼容性](docs/Bianbu案例1-15核对与Rust兼容性.md)。
  正文、来源及哈希已归档到 `资料/官方参考/`，未执行教程或接车。
- 这些是 Muse Pi Pro 平台资料，未证明为 XT-STCAR 整车手册。第 04 篇单点 TOF/云台、
  第 10 篇 GPIO 舵机和第 14 篇杰美康 EtherCAT 电机都不能套成本车雷达、转向或底盘协议。
  已找到 UVC/V4L2、CMP10A IMU、YDLidar/RPLidar 的官方参考，实际配件仍待核实。
- **原生推理库已有具体官方线索**：`spacemit-com/model-zoo-vision` 提交
  `e3cb7c61174ac42ec2f6da407bc3fe6ac5067e42` 明列 `libonnxruntime.so` / `libspacemit_ep.so`
  及 `spacemit-onnxruntime` 包；原生代码通过 `SessionOptionsSpaceMITEnvInit` 初始化厂商 EP。
  此为第一阶段线索；后续已取得二进制并完成静态核验，见上方最新状态。
  上游报告 K1 YOLO26n 640 INT8、引擎 2.0.6 的性能，不能替代本工程 320 FP32 实测。
- 保持 Rust 为主：核心/协议处理由 Rust 完成，厂商 C/C++ 原生库必要时薄层桥接、ROS 驱动通过消息接入。
  不为统一语言重写内核驱动，不把 C++ 类接口直接当 C ABI；案例阅读阶段未修改代码；后续 GLIBC 2.38 构建变更见最新升级记录。
- 用户提出读取 STM32F103 固件帮助分析协议；当前仍暂不接车，未授权解保护/擦除/刷写。
  已核对 ST PM0075 §2.4.1：普通 SWD/JTAG 读取受 RDP 限制，正常解除读保护会擦除主 Flash。
  用户随后要求先不处理 MCU，继续官方文档；固件读取工作暂停，未执行任何硬件操作。

### GLIBC 2.38 追加验证

- 42 项常规 Rust 测试、fmt/clippy 及两个 release 交叉链接通过；原生模型 opt-in 测试本次未重复运行。
- 18 项交付/ELF 回归通过，覆盖 2.38 边界放行、2.39 超限拒绝、旧构建目标拒绝与包内 target 一致性。
- build / manifest 固定 `.2.38`，真实最高符号引用为 2.34；二者分别描述构建目标与实际引用。
- 本次仍暂不接车，未升级任何车端系统库。

### 用户提供的整车资料（2026-09-07 后续）

- 新资料在 `/Users/yuhaojin/Documents/进迭时空无人车（2026）学习资料`，用户明确不要看视频，已遵守。
- 关键证据来自 `4.出厂源码/racecar.zip`；外部解压目录缺子目录，查源码应使用完整包。所选文本在 `work/factory-2026/archive/`。
- 已补齐底盘 7 字节发送格式、38400 8N1、IMU 115200 解析、N10 雷达配置；教程明确无编码器，参考 RF2O/Cartographer 里程计。
- 上方“仍缺厂家协议”应更新为：协议已有源码依据，Rust 硬件适配尚未实现；仍需实车校准、MCU 接收/超时/反馈和系统库验收。
- 普通与 one 底盘节点的转向系数 1300/1200 不同；遥控 Twist 采用 PWM/角度语义。不要混用或直接映射现有物理 MotionIntent。
- 镜像文件标 Bianbu 2.2，未展开 rootfs 或刷机，不因此改动已通过的 GLIBC 2.38 本地构建。
- 完整发现与限制见 [无人车2026资料核对](docs/无人车2026资料核对.md)。此轮只读源码并更新文档，未执行厂商程序或控制车辆。

### 厂商协议 Rust 修正（最新）

- `robot-core::protocol` 新增底盘编码器、显式 factory profile、处理短写/中断与失败锁存的通用 PacketWriter，以及有校验/分包/过期保护的 IMU 解码器；没有真实端口打开入口。
- `xt-stcar-robot chassis-preview` 输出帧预览；回放新增 imu_bytes + 必填 --imu-config，禁止混用直接 IMU，frame 明确核对。
- 三类 IMU 分量齐全且新鲜才发布；无样本/坏校验只产生 Tick，不刷新 Controller 的传感器时间，EOF 仍停。
- 不复制旧偏移；config/imu-replay.json 为显式零偏合成配置，不是实车校准。FactoryProfile 不接受物理 MotionIntent。
- 51 项常规 Rust 测试、18 项交付测试、fmt/clippy、GLIBC 2.38 交叉链接与两个新包通过。详细 [协议适配说明](docs/厂商协议Rust适配.md)。

### 锁定 YOLO26n 规则与后续入口

- Ultralytics **8.4.142**，`nms=False` 选 one-to-one；`nms=None` 默认不能当作同样接口。
  实际导出：FP32 `quantize=32`、static320、batch1、max_det300、opset17、simplify=False、CPU。
- `[1,300,6]` 每行 `[x1,y1,x2,y2,score,class_id]`；严格 score > threshold，
  不再 sigmoid/objectness/NMS。仅形状不足以证明正确，metadata 与精确模型哈希必须核对。
- letterbox auto=False/scale_fill=False/scaleup=True/center=True，RGB、填充114，
  ties-to-even round、记录整数 left/top 与名义 r，按 (coord-pad)/r 还原后裁剪。
- 低置信候选反向框由阈值过滤丢弃；保留候选才检几何。worker 不抢先拒绝所有候选几何。
  导出前检查 one-to-one 分支存在，不能用当前 end2end 选择状态代替可用性检查。
- 源码原文与许可证在 `资料/官方参考/Ultralytics_8.4.142/`，含来源提交和逐文件哈希。
  使用方法与证据见 README、`docs/YOLO26接口.md`、`docs/机器人模块.md`、
  `docs/验证记录-2026-09-07.md`；环境见 `资料/环境.md`，不要重复重装已验证环境。
- 下一步尊重用户暂不接车。可根据具体赛项继续 Rust 算法；用户准备接车后先只读核实系统/接口，
  在独立 release 验证两程序自检与回放，再适配实际推理库、传感器、ROS 与底盘。

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
  高层算法、ROS 接口与底盘协议分层；先取得厂家驱动和协议，不凭产品简介重写 MCU 固件。
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
