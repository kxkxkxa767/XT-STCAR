# XT-STCAR

Muse Pi Pro / RISC-V 智能车工程，采用 **Rust 为主、Mac 交叉编译、独立部署包** 的路线。
本机工程目录为 `/Users/yuhaojin/Documents/XT-STCAR`；仓库为
[github.com/kxkxkxa767/XT-STCAR](https://github.com/kxkxkxa767/XT-STCAR)。
仓库保留源码、脚本、配置、依赖锁文件、文档和验证记录；工具链、虚拟环境、模型、构建/部署产物和缓存由 `.gitignore` 排除。

本轮已实现两个 Rust 程序，主机测试、真实 YOLO26n 推理和两个程序的 RISC-V release 交叉链接均通过。
Python 保留用于离线模型导出、校验和参考对照；默认运行时由 Rust 直接调用 ONNX Runtime C API，
不启动 Python 进程。推理引擎本身为上游 ONNX Runtime 原生库。

用户明确“暂不接车”：未连接、上传或在车端执行，所有运动输出仅为离线记录。
实测证据见 [验证记录](docs/验证记录-2026-09-07.md)，接手前完整读取 [agent.md](agent.md)。

## 已实现的 Rust 模块

| 模块 | 程序与功能 |
|---|---|
| 视觉核心 | `crates/vision`：RGB、letterbox、NCHW、模型契约、严格阈值与坐标解码 |
| 原生推理 | `crates/app` → `xt-stcar`：ORT 动态加载、模型哈希/元数据校验、常驻 Session、取消与错误处理 |
| 图像输入 | `crates/runner/src/input.rs`：PNG/JPEG 帧文件、JSONL 时序和序号校验、大小限制 |
| 传感器语义 | `crates/robot-core/src/sensors.rs`：IMU、雷达、里程计、视觉摘要，时间/单位/frame/数值校验 |
| 状态机与底盘接口 | `crates/robot-core/src/safety.rs`：启用流程、急停锁存、deadman、超时、限值，MotionSink/RecordingSink |
| 总调度与日志 | `crates/runner` → `xt-stcar-robot`：传感器/控制/真实图像推理混合回放，完整 JSONL 与结束 Stop |

已新增厂商底盘帧编码/映射预览及 IMU 流解析，见 [厂商协议 Rust 适配](docs/厂商协议Rust适配.md)。
真实设备连接、ROS 接口、标定、定位融合、路径规划与避障策略尚未实现。
现有模块不会编造串口帧、舵机角或轮速映射；硬件适配需要厂家资料。
详细数据与状态约定见 [机器人模块](docs/机器人模块.md)。

## 直接运行本机程序

以下命令在工程根目录执行，无需模型或推理库：

```bash
cargo run --locked --bin xt-stcar -- self-check
cargo run --locked --bin xt-stcar-robot -- replay --events examples/robot-sim.jsonl
```

`self-check` 是明确标记的合成自检。机器人样例包括合成 IMU/雷达/里程计、
启用/指令/超时/复位/急停事件；程序记录模拟输出，流结束额外记录 Stop，不操作硬件。

图像预处理、保存的检测张量回放（将 `path/to/` 换成真实文件）：

```bash
mkdir -p work
cargo run --locked --bin xt-stcar -- preprocess --image path/to/image.png \
  --tensor work/input.f32le --transform work/transform.json
cargo run --locked --bin xt-stcar -- replay --image path/to/image.png \
  --tensor path/to/output-tensor.json --output work/replay.json
```

## Rust 真实推理

Mac 标准 ORT C 库已核验并保存在项目工具目录。使用本机实际图片：

```bash
cargo run --locked --bin xt-stcar -- infer --image path/to/image.png \
  --model models/yolo26n.onnx \
  --runtime-lib toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib \
  --output work/detections.json
```

默认读取模型旁的 `models/yolo26n.provenance.json`。初版锁定官方 YOLO26n Detect、
Ultralytics 8.4.142、FP32、静态 320×320、batch=1、one-to-one、opset 17；
消费端严格 `score > threshold`，不重复 NMS。参考实现须显式添加 `--backend python-reference`，
再指定 `--python .venv-model/bin/python`；它不是默认部署路径。

模型栈在独立 `.venv-model/`，基础开发环境仍为 `.venv/`。
导出/校验、来源、版本锁定与限制见 [YOLO26 接口](docs/YOLO26接口.md)。
机器人回放中的 `vision_frame` 会复用同一个原生 ORT Session，完整示例格式见 [机器人模块](docs/机器人模块.md)。

## 交叉编译与打包

`rust-toolchain.toml` 精确固定 Rust 1.97.1，`Cargo.lock` 锁定依赖；项目 Zig / cargo-zigbuild 为 0.15.2 / 0.23.4。

```bash
scripts/build-riscv.sh --offline
scripts/package.sh
scripts/package.sh --model models/yolo26n.onnx --python .venv-model/bin/python
```

构建脚本执行全 workspace 的 fmt/test/clippy，再交叉链接 `xt-stcar` 和 `xt-stcar-robot`，
分别检查 ELF。产物在 `target/riscv64gc-unknown-linux-gnu/release/`，包在 `dist/`。
最终两个程序均为 RISC-V ELF64 / RVC / LP64D / PIE，加载器 `/lib/ld-linux-riscv64-lp64d.so.1`，
当前交叉目标为 `riscv64gc-unknown-linux-gnu.2.38`，动态依赖仅 `libc.so.6`，
实际最高符号引用 GLIBC_2.34，符合 2.38 上限。最新证据见 [GLIBC 2.38 升级记录](docs/GLIBC-2.38升级记录.md)。

包不包含主机虚拟环境、工具链、Mac dylib 或缓存。**车端真实推理仍需核实并提供 RISC-V 标准 ORT C 动态库**；
官方候选库的 C 导出已静态核验，目标平台运行兼容性仍待验证。无该库时仍可运行合成自检、预处理、张量回放与无模型机器人回放。
上传默认 dry-run，本轮不执行。详见 [部署包说明](docs/部署包使用说明.md)。

[架构与验收边界](docs/架构与验收.md)、[环境说明](资料/环境.md) 与 [资料索引](资料/资料索引.md)
区分本机实测、官方参考与待车端确认事项。新车首次接入先检查系统/设备/厂家接口，再做实测适配。
