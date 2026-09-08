# Rust 比赛自主闭环

2026-09-08。按用户提供的赛项三规则初稿增加纯 Rust 任务、视觉、扫描、定位和导航模块。
保持 5 个 crate、2 个程序；没有新增 Python 在线算法、ROS1 或 bag 依赖。
本轮暂不接车，所有执行证据来自 Mac；配置带 `sim` 的均为合成场景参数。
交叉编译通过和模拟完成不能替代相机、定位、底盘制动或 RISC-V 推理的实车验收。

## 模块与数据流

| 位置 | 做什么 |
|---|---|
| `crates/robot-core/src/autonomy.rs` | 世界/车体坐标、位姿质量、车体足迹、道路观察、灯色、障碍圆盘的共享类型 |
| `crates/robot-core/src/mission.rs` | 发车、人行道停车、绕锥桶、灯前停车、绿灯确认、终点及锁存故障的比赛状态机 |
| `crates/vision/src/road.rs` | RGB 中提取白条斑马线、红黄绿灯、红蓝锥桶；地面单应矩阵把位置转为车体米制坐标 |
| `crates/robot-core/src/scan.rs` | N10 局部包拼整圈、角度覆盖/有效点/连续盲区/时序检查；未知点不当作空地 |
| `crates/robot-core/src/localization.rs` | 有点数/迭代上限的平面 ICP 激光里程计，质量、跳变、退化、时间间隔门控 |
| `crates/robot-core/src/navigation.rs` | 障碍膨胀、网格连通检查、带转弯约束的路径搜索、路径跟踪、轨迹及制动包络检查 |
| `crates/runner/src/laser_pose.rs` | 整圈质量检查 → 明确雷达到车体外参 → ICP，保留扫描采集时刻 |
| `crates/runner/src/perception.rs` | 常驻 ORT Session 与同一 RGB 图像复用；感知线程一次处理一帧，待处理只留最新一帧 |
| `crates/runner/src/autonomy.rs` | 检查同步的位姿/扫描/道路观察，串联 Mission → Navigator → Safety，产生运动意图或 Stop |
| `crates/runner/src/control_runtime.rs` | 有界后台规划线程和独立轮询看门狗；命令按输入时刻过期，迟到计算结果不能恢复故障 |
| `crates/runner/src/simulation.rs` | 生成 RGB、360 束射线雷达、位姿反馈，运行控制器并推进阿克曼运动学模型和下一帧传感器 |
| `crates/runner/src/autonomy_replay.rs` | 严格的同步传感器快照输入，不接受人工 Motion 指令；EOF 总是输出 Stop |
| `crates/runner/src/telemetry.rs` | 有界内存日志，默认阶段变化/故障/终态，显式 trace 才记录调试细节 |

合成闭环实际执行以下链路，而不是回放预先写好的油门/转向：

```text
场景 + 车辆状态 → RGB → RoadDetector ──────┐
               → 360 束测距 ─────────────┤
               → 模拟位姿反馈 ───────────┤
                                        ↓
任务状态机 → 目标/停车 → 规划与跟踪 → Safety → Drive/Stop
    ↑                                         ↓
    └────────── 下一帧反馈 ← 有加减速的车辆模型 ─┘
```

实物输入接口则是 N10Decoder → N10RevolutionAssembler → LaserPosePipeline，以及
RGB → RoadPipeline/PerceptionWorker → RoadFrame，再以统一时钟形成 SensorSnapshot。
这些库已有实现和测试；真实相机采集、共同采集时钟、位姿插值/融合及底盘发送尚未接通。
模拟器使用明确标注的理想位姿反馈，不把它冒充由 N10 推算的实测定位结果。

## 比赛行为

状态依次为 `Idle → ApproachCrosswalk → CrosswalkStop → Cones → ApproachLight → WaitGreen → Finish → Completed`，
任一安全失败进入锁存 `Fault`。必须显式调用 `start`，故障后不自动重新发车。

- 斑马线先从图像检测前沿，投影到世界并固定本次停车目标；车体足迹不能压线。
  连续的新鲜位姿确认平移和角运动均停止后才累计至少 3000 ms，移动会重新计时。
  通过后不会把同一斑马线重复当成下一次任务。
- 两锥桶通过场地配置的绕行侧目标表达任务顺序，扫描和视觉更新实际障碍，导航每次检查路径。
  示例坐标来自合成场景，实赛须按获准公布的场地尺寸和任务路线重新配置，不能照抄。
- 灯前必须全车足迹进入停车区并满足朝向与停止条件；绿灯连续时间和不同新帧数都达标才放行。
  红、黄、未知、矛盾、丢帧不会靠固定倒计时放行。放行后未完全通过区域又丢失绿灯会停车。
  场地光电触发仅由模拟器生成灯变化，控制器不会读取这条“真值”。
- 终点按配置的停车朝向检查全车足迹、速度和位置，发出 Stop 后还要确认模拟车辆速度归零。
  足迹使用覆盖车体及车轮的保守矩形，上车要测量其边界。
- 普通路段停止超过 10 秒按初稿标为任务失败；失败不解除急停，也不会为了计分强行开车。

## 视觉与模型

原 YOLO26n Detect 的 80 类权重和 `[1,300,6]` 契约保持不变。`traffic light` 类可提供灯框；
`zebra` 是动物，不能当斑马线。斑马线、灯色和红蓝锥桶由新增 Rust 图像算法处理。

`RoadDetector` 使用 HSV/亮度、连通域、白条方向/间隔/重叠和颜色区域形状判定。
灯色仅在 YOLO 灯框或显式配置的 ROI 内判断，拒绝整幅图中任意红色物体触发红灯。
输入 YOLO 灯框无效时返回未知，不借另一个区域假装有效。
图像工作分辨率、连通域数、候选数和输入像素均有界。

相机单应矩阵按 `u=x/width、v=y/height` 归一化坐标映射到车体 `x` 向前、`y` 向左的米制地面。
纯色合成图覆盖多分辨率、条纹几何、灯色冲突和负例，但不代表真实逆光、反光、阴影、运动模糊或远处精度。
只接受 `simulation_only=true、calibration_status=unverified` 的示例配置；正式启用实车配置前需建立测量及验收流程。

`road-detect` 不加模型参数时测试显式 ROI 的 Rust 算法；加模型和运行库时，由 Rust 原生 ORT 获取检测框，
同一 RGB 图像继续做道路识别。`RoadPipeline` 复用会话，在线处理不生成图片或中间张量文件。

## 定位、导航与时序边界

整圈组帧保存首包接收时刻；持续缺包、长盲区、覆盖不足、无效距离或时间倒退都会拒绝。
接收时刻不等于硬件曝光/测距时刻，当前没有运动去畸变。
ICP 的第一圈只建立参考，第二个有效扫描才可输出位姿；超过间隔后必须以外部已知位姿显式 reset。
质量值是几何匹配启发式评分，不是协方差；没有编码器反馈、完整 SLAM、回环或全局重定位声明。

导航依据车体足迹膨胀障碍，网格和车模型搜索都有预算，局部预览检查轨迹、曲率变化和制动余量。停车可达区域使用车体外接圆加反应/制动行程的保守圆盘，
覆盖逐步回中时的中间曲率，不能只检查原曲率弧和一条直线就认定两者之间安全。
无法找出可执行路径时 Stop，预算耗尽不会返回未经检查的直线。
模拟 Stop 的目标为速度0、曲率0，曲率按同一速率限制回中；正常停车和最终故障制动使用同样语义。
模拟配置要求实际模型制动能力不少于规划假设，实车需要测量才能建立该前提。
这不是路径完备性或最优性保证。地图误差、动态物体、坏外参和未标定的制动性能仍会影响实车结果。

`AutonomyController::tick` 是确定性计算接口。扫描和用于其世界投影的位姿必须同一采集时刻；
灯色保留原图时刻。斑马线世界锁定和视觉锥桶投影也必须使用同一图像采集时刻的位姿，
不能仅因相差小于100ms就直接套用当前位姿；不同步时停车。重复时间戳不能换内容或续新鲜度。
未来真实适配器须对齐时钟并把几何与位姿配对，当前快照只含一个位姿，不支持跨时刻几何混投影。

`PerceptionWorker` 把慢推理放在后台，只保留一个待处理图像，繁忙时替换或拒绝，不累积旧视频。
`AutonomyWorker` 同样隔离可能较慢的路径搜索。独立周期调用 `poll(now)` 获取命令，
看门狗按源快照及最旧传感器时刻检查命令期限；先检查旧期限，后接纳新结果。
过期、处理异常或时间异常锁存 Stop，之后即使后台返回 Drive 也不能续跑。
正常比赛停车没有被当成 Fault，收到及时且有效的后续结果仍可继续。
调用者不能在这个周期里同步推理、规划、写文件或等待阻塞锁。

这些线程接口没有接物理输出。线程释放采用协作停止，不在 Drop 中无限 join 被卡住的原生调用；
不能通过反复创建替代线程掩盖卡死。进程整体停调度/崩溃仍需 MCU 侧独立 watchdog 和已验证停车语义。
搜索有次数上限不等于已证明目标 CPU 的最坏实时耗时。

## 配置与运行

| 配置 | 消费者 |
|---|---|
| `config/competition-sim.json` | `autonomy-sim` 完整场景、控制器、视觉和日志配置 |
| `config/competition-controller-sim.json` | `autonomy-replay` 的 AutonomyConfig；内容是上项的 `autonomy` 字段 |
| `config/road-perception-sim.json` | `road-detect` / RoadPipeline，内容是场景的 `road` 字段 |
| `config/scan-assembly-sim.json` | ScanConfig 库接口，不是 `--n10-config` 的包协议配置 |
| `config/laser-localization-sim.json` | LocalizationConfig 库接口，还需明确初始位姿和雷达到车体外参 |

Mac 在工程根目录：

```bash
mkdir -p work
cargo run --locked --offline --bin xt-stcar-robot -- autonomy-sim \
  --config config/competition-sim.json --output work/competition-run.jsonl
cargo run --locked --offline --bin xt-stcar-robot -- road-detect \
  --config config/road-perception-sim.json --image /实际图片路径.png
```

不指定 `--output` 时模拟和传感器快照回放默认 stdout 仅摘要；显式 `--trace` 才输出详细记录。
模拟异常时先保存终态 Stop 与摘要，再以非零状态退出；配置错误不覆盖已有日志。
故障注入可在复制到 `work/` 的配置中把 `fault` 改为 `camera_dropout`、`lidar_dropout`、`pose_dropout`、
`blocked` 或 `red_only`，并设置 `fault_at_ms`。红灯持续不会被当成到了某秒就可以放行。

`autonomy-replay --config config/competition-controller-sim.json --events FILE` 每行接受：
`{"at":毫秒,"pose":PoseEstimate,"scan":LidarSample,"road":RoadFrame}`。
它与原 `replay` 的 `{"at":...,"event":...}` 格式不同，拒绝附带 Motion 指令。
仅供开发诊断，不能以预录路线代替比赛实时自主决策，也不推导出规则允许比赛采集/回放。

更完整的 Mac 开发、车端系统检查、上传、使用步骤见根目录 [Mac与车端命令手册.txt](../Mac与车端命令手册.txt)。
部署包含手册与这些配置；在包目录运行时把 `cargo run ... --` 换成 `bin/xt-stcar-robot`。

## eMMC 与日志

**正常运行和比赛优先于节省擦写。** 不减少传感器刷新率，不限速，不改系统挂载、swap、journald，
也不为省写入取消必要的错误诊断。所有实时计算和线程队列在内存中进行。

默认 `TelemetryConfig` 为 `transitions`：阶段变化、故障和最终摘要先在有界内存保存，结束后一次原子提交。
`summary` 进一步减少可选阶段记录，`trace` 用于明确需要的调试，默认细节预算 1 MiB，可配置不超过 8 MiB。
重要事件使用独立有界保留空间，trace 用尽只丢调试记录并增加 `dropped_trace_records`；
序列化/日志预算错误只计 `logging_errors`，不会变成控制 Fault。
最终文件提交失败是诊断输出错误，不表示计算过程产生过运动授权或清除过故障。

默认不逐帧保存相机图片、雷达点云、原生模型中间张量，不每个 tick 执行 fsync。
明确采集或 trace 时可正常写；长时间真实调试应在独立日志线程轮转，未来实车适配器不得同步等待磁盘。
需要保留的数据仍应导出到 Mac。eMMC 健康仅按实际设备只读查询，不执行写入压测或系统“优化”。

## 验收记录

单元与整合用例位于三个 crate 的 `tests/`，包括规则计时、灯色去抖、足迹、视觉负例、整圈质量、
ICP 跳变、车模型路径、掉线停车、源时间 lease、日志预算、CLI 文件保护。
完整构建、真实 Mac ORT 对照、模拟摘要及部署包证据统一使用 `docs/competition-*` 前缀，
此前 `architecture-*` 是本功能扩展前的审查历史。
实物待验：系统/运行库、相机/灯尺寸、扫描时钟及去畸变、TF/初始地图、转向制动标定、MCU watchdog、
多场景连续运行与最终规则；当前不宣称能直接通电参赛。


## 本次最终验证（2026-09-08）

完整结果见 [competition-validation.json](competition-validation.json) 与 [模拟摘要](competition-simulation-summary.json)。

| 检查 | 结果 |
|---|---|
| Rust workspace 常规测试 | 191通过，0失败；另1项需模型的测试默认ignored |
| 真实 Mac ORT opt-in | 1通过，同一Session复用/模型加载取消；机器人2帧回放每帧5个检测 |
| Python模型/交付 | 48 + 34通过 |
| 格式/Clippy | 全workspace格式、all-targets且-D warnings通过 |
| RISC-V交叉链接 | 两程序通过；基线2.38，实际最高GLIBC引用均2.34 |
| 完整合成闭环 | 55.3s模拟时间、554tick，全部任务完成；路径10.871m，最小锥桶车体间隙0.301m |
| 停车/灯色 | 斑马线连续停止3000ms，绿灯确认300ms，终点速度0、足迹/朝向符合示例区域 |
| 默认日志 | 24,375字节，0丢弃、0日志错误；trace额度耗尽不改变故障停车轨迹的回归通过 |

以上距离、时间和余量都是合成模型结果，不是实车测量或目标CPU帧率。
真实 bus.jpg 的新 RoadPipeline 测试只证明 Rust ORT 与道路算法可串联：灯为unknown、无斑马线，
出现1个未经真值确认的锥桶候选，不能据此声称真实锥桶识别准确；还需要实拍负例和比赛数据校准。

本轮总体审查修复了网格切角造成规划/跟踪矛盾、停车朝向不足、图像几何错用当前位姿、
运行中感知Err绕过终态制动、任务Completed与安全Fault同时发生却误报完成，以及Stop回中制动模型不一致。
修复均有对应回归。未接车、未打开真实设备、未执行RISC-V程序或读取视频。

部署包包括上述两份简要验证JSON、配置及命令手册；完整构建/测试日志、原生模型报告与包哈希
保存在源码仓库的 `docs/competition-*` 中。包哈希与成员统计在 `docs/competition-delivery.json`，
构建源码集合及每个源码SHA由包内 `build.json` 记录；工具链、环境和模型不进源码仓库。
