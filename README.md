# XT-STCAR

Muse Pi Pro / RISC-V 无人车工程：**Rust 为主，在 Mac 交叉编译，视觉使用 YOLO26n Detect。**
工程目录为 `/Users/yuhaojin/Documents/XT-STCAR`，源码仓库为
[github.com/kxkxkxa767/XT-STCAR](https://github.com/kxkxkxa767/XT-STCAR)。

目前有 **5 个 Rust crate、2 个可执行程序**。视觉预处理/后处理、原生推理调用、传感器协议、
串口传输、状态机、底盘标定映射和回放调度均由 Rust 实现。Python 用于离线模型导出、校验与参考对照；
默认推理由 Rust 直接调用 ONNX Runtime C API，不启动 Python 进程，推理引擎仍是上游原生库。

用户当前明确“暂不接车”：本轮没有连接车辆、操作电机、升级车端 GLIBC 或读取视频。
串口在 Mac 的伪终端上验证；运动输出只记录和预览。
最新验证见 [Rust 模块完善记录](docs/Rust模块完善验证记录-2026-09-08.md)，交接完整状态见 [agent.md](agent.md)。

## 代码结构：模块在哪、负责什么

```text
XT-STCAR/
├── crates/
│   ├── vision/          视觉数学与模型契约库
│   ├── app/             xt-stcar：视觉 CLI、原生/参考推理后端
│   ├── robot-core/      传感器类型、安全状态机、厂商协议和标定表
│   ├── device-io/       有截止时间的串口配置与传输库
│   └── runner/          xt-stcar-robot：回放、采集、预览与日志
├── config/              显式配置；车辆相关示例均未经实车标定
├── examples/            合成 JSONL 回放输入
├── scripts/             模型工具、构建、ELF 检查、打包和上传
├── tests/               Python 模型/交付测试与跨实现对照工具
├── docs/                协议依据、使用说明、验证报告
├── 资料/                 来源索引、环境记录、已归档官方参考
├── Cargo.toml/lock      Rust workspace、依赖与精确版本锁
├── rust-toolchain.toml  Rust 1.97.1 工具链选择
├── AGENTS.md            开发约定的加载入口
└── agent.md             完整交接快照与长期约定
```

| 源码位置 | 职责与边界 |
|---|---|
| [`crates/vision/src/lib.rs`](crates/vision/src/lib.rs) | `ModelSpec`、RGB letterbox、NCHW FP32 张量、输出契约检查、阈值过滤、坐标还原；不访问设备或加载原生库 |
| [`crates/app/src/main.rs`](crates/app/src/main.rs) | `xt-stcar` 命令入口：`self-check`、`preprocess`、`replay`、`infer`，参数与输出文件保护 |
| [`crates/app/src/ort_backend.rs`](crates/app/src/ort_backend.rs) | `NativeOrtBackend`：动态加载 ORT C 库、模型 SHA/provenance/实际张量元数据验证、常驻 Session 和超时取消 |
| [`crates/app/src/backend.rs`](crates/app/src/backend.rs) | 显式 Python 参考后端、张量文件交换、子进程超时清理与原子输出工具 |
| [`crates/app/src/lib.rs`](crates/app/src/lib.rs) | 导出可复用的推理后端，供总调度调用 |
| [`crates/robot-core/src/sensors.rs`](crates/robot-core/src/sensors.rs) | IMU、雷达、里程计、视觉摘要及 frame/单位/时间/有限数值校验；里程计类型本身不计算里程计 |
| [`crates/robot-core/src/safety.rs`](crates/robot-core/src/safety.rs) | Disarmed/Armed/Running/Fault、急停锁存、deadman、心跳/指令/传感器超时、物理速度和曲率限值；`RecordingSink` 仅记录输出 |
| [`crates/robot-core/src/protocol/chassis.rs`](crates/robot-core/src/protocol/chassis.rs) | 厂商 7 字节底盘编码、三种不同话题语义的显式预览、通用短写/失败锁存适配器；经验系数不作为物理标定 |
| [`crates/robot-core/src/protocol/calibrated_chassis.rs`](crates/robot-core/src/protocol/calibrated_chassis.rs) | 速度→电机 PWM、曲率→舵机 PWM 的显式分段表；零锚点、限值、倒车策略校验，拒绝外推；当前只接受未验证模拟配置 |
| [`crates/robot-core/src/protocol/imu.rs`](crates/robot-core/src/protocol/imu.rs) | WIT 11 字节增量解析、校验和、三分量组合、单位/四元数转换、坏帧与过期控制 |
| [`crates/robot-core/src/protocol/n10.rs`](crates/robot-core/src/protocol/n10.rs) | 按厂商源码解析 N10 58 字节/16 点包、分包/粘包重同步、角度/距离/强度、保留无效点槽位；只生成局部扫描样本 |
| [`crates/robot-core/src/lib.rs`](crates/robot-core/src/lib.rs)、[`protocol/mod.rs`](crates/robot-core/src/protocol/mod.rs) | 核心类型与协议模块的公共导出入口 |
| [`crates/device-io/src/lib.rs`](crates/device-io/src/lib.rs) | 安全 Rust 串口库：显式设备、8N1 配置读回、关闭软硬件流控、独占、非阻塞 poll、整包截止时间、错误锁存和退出恢复 |
| [`crates/runner/src/main.rs`](crates/runner/src/main.rs) | `xt-stcar-robot` 命令解析、默认采集计划、输入文件保护、完整日志原子提交 |
| [`crates/runner/src/input.rs`](crates/runner/src/input.rs) | 非阻塞普通文件校验、配置/事件实际字节限制、严格 JSONL 与单调时序、图像路径/序号检查，解析 `imu_bytes` / `n10_bytes` |
| [`crates/runner/src/vision.rs`](crates/runner/src/vision.rs) | 图像读取、预处理与复用同一 ORT Session；将检测结果交给调度 |
| [`crates/runner/src/lib.rs`](crates/runner/src/lib.rs) | 串联输入→解码/推理→安全控制器→可选 PWM 预览→JSONL；EOF Stop、推理/映射失败急停；完整解码批次每块只记一次 |
| [`crates/runner/src/capture.rs`](crates/runner/src/capture.rs) | 有时间/字节/记录上限的 IMU 或 N10 采集，使用统一 `Instant` 时钟，空闲和结束写 Tick；不发送设备命令 |

数据流为：文件回放 → 传感器/视觉模块 → 安全状态机 → 运动记录 → 可选底盘 PWM 预览。
独立串口采集先生成可回放的原始字节文件；采集没有 Arm/Start/Motion 事件。
目前没有把采集与电机发送接成实时闭环。

## 配置与样例索引

| 位置 | 用途 |
|---|---|
| `config/yolo26n.json` | 模型尺寸、输出契约、检测阈值 |
| `config/robot-sim.json` + `examples/robot-sim.jsonl` | 多传感器、安全状态机、超时和急停的合成回放 |
| `config/imu-replay.json` + `config/robot-imu-replay.json` + `examples/robot-imu-replay.jsonl` | 显式零偏、frame、分量年龄，以及坏 IMU 帧不刷新状态的样例 |
| `config/n10-replay.json` + `config/robot-n10-replay.json` + `examples/robot-n10-replay.jsonl` | N10 合成分片、坏校验与雷达超时样例；有效包仅 16 束局部数据 |
| `config/chassis-calibration-sim.json` | 人工合成的分段标定表，明确 `simulation_only=true`、`measurement_status=unverified` |
| `config/serial-imu-capture.json` / `config/serial-n10-capture.json` | `/dev/imu` 115200 / `/dev/laser` 230400 的厂商参考采集计划，实际设备身份待核实 |

不要把示例超时、零偏、frame 或 PWM 中性值作为实车校准结果。N10 局部包的接收健康不能证明全圈覆盖或避障有效。

## 本机运行

在仓库根目录执行。以下命令不需要模型或设备：

```bash
cargo run --locked --bin xt-stcar -- self-check
cargo run --locked --bin xt-stcar-robot -- replay --events examples/robot-sim.jsonl
cargo run --locked --bin xt-stcar-robot -- replay \
  --events examples/robot-imu-replay.jsonl --config config/robot-imu-replay.json \
  --imu-config config/imu-replay.json
cargo run --locked --bin xt-stcar-robot -- replay \
  --events examples/robot-n10-replay.jsonl --config config/robot-n10-replay.json \
  --n10-config config/n10-replay.json --chassis-calibration config/chassis-calibration-sim.json
```

回放输出 step + summary JSONL，可加 `--output work/replay.jsonl`。每条运动记录标明
`physical_output_enabled=false`。映射失败记录完整 `chassis_error`/Stop 后非零退出；正常流结束也记录 Stop。
IMU/N10 样例故意触发超时，最终 Fault 是预期行为。回放时间为模拟时钟，不代表实时控制时延。

厂商旧话题语义的帧预览与新的物理标定预览是两个独立入口：

```bash
cargo run --locked --bin xt-stcar-robot -- chassis-preview \
  --profile teleop_pwm_degrees --linear 1500 --angular 90
mkdir -p work
cargo run --locked --bin xt-stcar-robot -- serial-capture \
  --config config/serial-n10-capture.json --output work/n10-capture.jsonl
```

最后一条只输出计划，不打开设备或写文件；显式加 `--execute` 才配置并读取指定传感器 tty。
本轮没有执行真实设备采集。限制、输出格式与空闲 Tick 见 [Rust 串口采集](docs/Rust串口采集.md)。

## YOLO26n 原生推理

使用实际本机图片（替换 `path/to/image.png`）：

```bash
cargo run --locked --bin xt-stcar -- infer --image path/to/image.png \
  --model models/yolo26n.onnx \
  --runtime-lib toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib \
  --output work/detections.json
```

模型旁必须有 `.provenance.json`。当前锁定官方 YOLO26n Detect、Ultralytics 8.4.142、FP32、
静态 320×320、batch=1、opset 17、one-to-one `[1,300,6]`；严格 `score > threshold`，不重复 NMS。
图像预处理与保存张量回放分别用 `preprocess` / `replay` 子命令，参数见 `xt-stcar --help`。
`--backend python-reference --python .venv-model/bin/python` 才会使用 Python 参考进程。
原生 Mac 模型推理已在此前验证，RISC-V 目标真实推理尚未验证。

## 构建、测试与交付

Rust 1.97.1、Zig 0.15.2、cargo-zigbuild 0.23.4 和 `Cargo.lock` 锁定依赖。
Linux 交叉链接目标固定为 **`riscv64gc-unknown-linux-gnu.2.38`**，没有修改车端系统库。

```bash
scripts/build-riscv.sh --offline
scripts/package.sh
scripts/package.sh --model models/yolo26n.onnx --python .venv-model/bin/python
```

| 路径 | 内容 |
|---|---|
| `crates/*/tests/`、源码内 `#[cfg(test)]` | Rust 协议/数学/状态机、CLI 文件保护、PTY 传输与采集集成测试 |
| `tests/test_delivery.py` | ELF、打包白名单、哈希与上传参数边界测试 |
| `tests/test_yolo26_tools.py` | Python 导出/输出契约回归 |
| `tests/measure_yolo26_reference.py`、`tests/verify_robot_reference.py` | 真实模型与机器人图像回放参考对照 |
| `scripts/export_yolo26.py`、`validate_yolo26.py`、`verify_preprocess.py` | 模型导出、静态校验及预处理对照 |
| `scripts/onnx_worker.py` | 仅供显式 Python 参考后端的常驻推理工作进程 |
| `scripts/build-riscv.sh`、`inspect_elf.py` | fmt/test/clippy、两程序交叉构建、独立 ELF/GLIBC 报告和源码哈希 |
| `scripts/delivery.py`、`package.sh`、`upload.sh` | 白名单打包与哈希核验；上传默认 dry-run，显式新账号/IP/release 目录 |
| `scripts/check-vehicle.sh` | 车端系统/设备/服务的只读检查脚本，不启动驱动或运动 |
| `target/riscv64gc-unknown-linux-gnu/release/` | 两个目标程序与 build/ELF 证据（本地产物，不进 Git） |
| `dist/` | core 或含模型的独立部署包（不进 Git） |

构建脚本完整执行 fmt、主机测试、clippy `-D warnings`，再检查两个 RISC-V ELF 的 ISA/ABI/加载器/GLIBC。
交付包不含工具链、虚拟环境、Mac dylib 或缓存；带模型包另带 provenance 与模型许可。
源码仓库也排除模型、厂商大包、镜像、`target/`、`dist/`、`work/` 和缓存。

## 已完成与待接车工作

已完成上述 Rust 模块及离线集成，不等于无人驾驶系统已实车验收。
还需要真实相机采集、N10 全圈组帧、多传感器统一时钟、TF/物理标定、激光里程计/定位融合、
路径规划、避障、ROS 2 接口与 MCU 反馈/watchdog；厂商教程明确没有轮编码器，不能假造轮速里程计。

车端真实推理需核实 RISC-V 标准 ORT C 库。官方 SpacemiT ORT/EP 2.0.6 已静态核验，仍未测试目标
`GetApi(22)`、模型算子或 EP 初始化。候选库要求 GLIBC 2.38，EP 另要求 GLIBCXX 3.4.32 / CXXABI 1.3.15。
相关说明见 [原生库核验](docs/SpacemiT原生运行库核验.md)。

进一步阅读：[机器人事件与状态机](docs/机器人模块.md)、[厂商协议适配](docs/厂商协议Rust适配.md)、
[N10 协议依据](docs/N10协议依据.md)、[底盘标定映射](docs/底盘标定映射.md)、[YOLO26 接口](docs/YOLO26接口.md)、
[部署包使用说明](docs/部署包使用说明.md)、[官方整车资料核对](docs/无人车2026资料核对.md)、[资料索引](资料/资料索引.md)。
