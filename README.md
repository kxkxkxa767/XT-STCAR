# XT-STCAR

Muse Pi Pro 上的 RISC-V 轻量无人车比赛工程。车载 Rust 程序 `route-engine race` 通过 ROS 2 Humble 获取雷达扫描和底盘反馈，结合 IMU、相机控制车辆。路线依次经过斑马线、逆时针绕行的首桶、顺时针绕行的次桶、红绿灯停车区和终点区。

## 两个在维护的程序

| 编号 | 路线定位 | 维护分支 | 11 月 22 日实跑构建 | 11 月 23 日候选构建 |
| --- | --- | --- | --- | --- |
| 1 | 赛场版。用现场雷达观测跟踪锥桶、挡板和终点直道开口 | `race/program1-y114-crosswalk` | `c5e8c05` | `7b9e076` |
| 2 | 建图版。用实测赛道坐标、雷达和 IMU 辅助定位，与原有绕桶规划并行 | `main` | `494570a` | `40047fd` |

两版实车均用 **N10P，460800 波特率**。本车安装标定为距离补偿 +0.011 m、顺时针索引偏移 −3 格。相机由 Rust 直接采集。斑马线按条纹几何识别。灯色按黑框内左红、右绿两个端位各自的底色、灯珠和色晕判断，中间灯罩只用于辅助识别斜拍时的排列。2 号用场地边界和隔板定位车身。两桶的标称位置只用于起始预瞄，每轮由相机与雷达更新。入弯短时看不到桶时，程序用本轮已观测桶位和车姿继续计算转向。[1 号源码](https://github.com/kxkxkxa767/XT-STCAR/tree/race/program1-y114-crosswalk) · [2 号赛道地图](config/race-field-map.json) · [车体尺量记录](docs/车辆尺寸与几何来源.md)

## 车端使用

在车上的 Bash 终端使用 `start_程序号_最大等灯秒数`。秒数仅对本轮生效，可用 0 或非负小数。

| 命令 | 用途 |
| --- | --- |
| `start_1_7 --show` | 回读程序、源码提交、Rust 视觉和等灯秒数 |
| `start_2_7 --check-only` | 保持车辆锁定，启动相机、雷达和控制进程，检查设备及起步条件 |
| `start_2_7.5 --show` | 回读 2 号和本轮 7.5 秒等灯参数 |
| `start_1_7 --status`、`start_1_7 --stop` | 查看或停止共享的比赛服务 |

每轮摆车、复位现场设备并取得本轮发车确认后，运行对应入口，例如 `start_2_7`。控制回路使用底盘 Guard、实测车身近障 8 cm 保护和终点端板测距回退。车载安装使用带 `ros2` 特性的 64 位 RISC-V ELF。[编号入口与安装说明](docs/numbered-race-start.md)

11 月 24 日的视觉候选已用 7 轮赛场原图和 12 轮历史图像回放。识别结果与适用范围见[赛场灯珠验证](docs/rust-led-field-20261124.md)。

### 新候选的灯区行为

灯架位于终点直道内侧隔板的开口，光电线在灯架上游 **1.00 m**。控制器用雷达几何估算前轮首次过线的时刻，从这个时刻开始倒计时。停车目标为车头距灯架约 **0.55 m**，给车尾越过入口线预留约 **0.06 m**。

即使灯具未触发或等灯秒数设为 0，车辆也要先进区停稳。过线后取得两帧新鲜有效绿灯，可在停稳后放行。一直未识别到有效绿灯时，倒计时到点后放行。[灯区控制实现](route-engine/src/maneuver_light.rs)

11 月 23 日这批候选已交叉编译，实车验证待完成。两版共跑 14 组闭环仿真，14 组均在灯区内停稳后放行，12 组正常完成全程。另两组均为 1 号，在放行后的终点段触发 8 cm 保护，其中一组模型随后接触端板。

## 本机构建与验证

仓库使用 Rust 1.97.1。以下命令在工程根目录执行。控制台测试会调用本机的底盘桥、路线引擎和雷达几何程序。[Rust 工具链](rust-toolchain.toml) · [Mac 开发依赖](requirements-dev-macos.lock.txt)

```bash
cargo build --locked --bin vehicle-bridge
cargo build --release --locked --manifest-path route-engine/Cargo.toml
cargo build --release --locked --manifest-path tools/scan-geometry/Cargo.toml --bin xt-stcar-scan-geometry --example surface_sim
cargo test --release --locked --manifest-path route-engine/Cargo.toml
cargo clippy --release --locked --manifest-path route-engine/Cargo.toml
(cd web/vehicle-console/tests && python3 -m unittest discover -s . -p 'test_*.py')
```

### 仿真与续跑

通用闭环模拟器用真实 Rust 控制逻辑计算命令，车辆模型生成雷达、IMU 和相机观测。单轮接受一份工况 JSON 和一个新的输出目录。下例从工程根目录运行。[模拟器源码](tools/scan-geometry/examples/surface_sim/main.rs) · [赛道工况](config/track-spec-sim-20261112.json)

```bash
mkdir -p work
tools/scan-geometry/target/release/examples/surface_sim \
  config/track-spec-sim-20261112.json work/sim-first-run
```

项目工作站在 Git 之外保存工况和续跑进度。1、2 号续跑入口位于 `work/numbered-light-opening-20261123/继续仿真/`。在该目录运行 `./继续上次仿真.command --status` 查看已完成组合。追加一组种子可用 `./继续上次仿真.command --case dark7 --seed 9124`。进度按整轮保存，程序、工况、种子和模拟器构建一致时会跳过已完成项。普通闭环续跑使用本机资料，重放历史实车轨迹时连接归档 U 盘。

## 目录与实车记录

| 位置 | 内容 |
| --- | --- |
| `route-engine/src/` | 比赛状态机、绕桶与灯区控制、ROS 2 雷达和底盘进程 |
| `crates/robot-core/`、`crates/device-io/` | 雷达和底盘协议、扫描几何、Guard 与串口 I/O |
| `tools/scan-geometry/`、`scripts/` | 本机几何工具、闭环仿真、回放分析、打包与安装 |
| `web/vehicle-console/` | 车载驾驶台、遥控与采集、参考实现及测试 |
| `config/`、`docs/` | 车辆与赛道配置、标定依据、工程交接和验证报告 |

每轮实车目录保留程序身份和运行结果。先看 `launch-profile.json` 核对编号、构建和本轮参数，再看 `run/cli-result.json` 判断是否真正发车。`run/feedback.jsonl`、`run/imu.jsonl` 和 `vision/` 分别保存雷达与控制反馈、惯性数据和相机记录。原始实车数据按轮次归档到 U 盘，工作站保留仿真工具、输入和续跑进度。

项目另有两份本地交付副本，`test/` 使用 N10P 和固定 2.8 m 斑马线测试配置，`提交/` 为组委会交付副本，使用 N10（230400 波特率）及对应标定。

历史 3–5 号源码已按 Git 标签归档。在仓库根目录运行 `git tag --list 'archive-program[345]-*-20261123'` 可查看对应版本。

接手开发时从工程约定和交接记录进入。早期硬件参数表、Rust 移植过程、历轮数据和旧命令保存在原版首页，可按日期查阅。[工程约定](AGENTS.md) · [共同记忆](memoryshare.md) · [历轮实车验证](docs/vehicle-dynamic-turn-validation-20261007.json) · [2026-11-21 版完整 README](https://github.com/kxkxkxa767/XT-STCAR/blob/00664bd4890aa0c0910992a10f54558c21a43fd3/README.md)
