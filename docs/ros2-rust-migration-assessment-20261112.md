# ROS2 + Rust 迁移可行性（2026-11-12）

后续已获用户授权实施；本文件保留迁移前评估。实施、实际通信开销和Claude复核入口见 [ROS2迁移记录](ros2-rust-migration-20261112.md)。

用户在暂停省赛赛道核对/仿真后，询问能否迁移现有代码并保持性能，以及已有bridge是否承担了ROS2的作用。本次完成源码、现有车端取证及上游文档核对；未开始迁移、未改系统安装、未部署、未发车，新赛道仿真仍暂停。

## 判断

迁移可行，可以保留现有Rust算法、bridge和控制节拍。ROS2支持Rust，但语言相同并不保证新增中间件、线程与通信的开销为零。能够以“控制行为等价、实测时延无明显退化”为验收目标，不能预先保证每个周期、CPU和圈时完全不变。

已核验车上为Bianbu2.2、riscv64、MUSE-Pi-Pro，内核6.6.63标为PREEMPT_RT；已装ROS2 Humble、rcl5.3.9、rmw_fastrtps_cpp6.2.7。基于这次检查，没有必须更换Linux发行版的理由。PREEMPT_RT存在不等于比赛线程已经采用实时调度。原始取证在 `work/provincial-rules-20261112/platform/live-audit.json`。

上游 `rclrs` 当前主线Cargo版本0.8.0、最低Rust1.85，构建配置含Humble。本工程Rust1.97.1满足该最低语言版本；但是这还不是本车RISC-V链接/运行验证。后续需固定依赖版本，用本车ROS2原生库构建，核对真实Humble ABI和消息类型支持。禁止把仅供文档构建的 `use_ros_shim` 当成真实ROS2实现。

## bridge与ROS2的关系

| 层 | 现有职责 | 迁移安排 |
|---|---|---|
| `vehicle-bridge` | 底盘串口/PWM、雷达解析、回执、输出约束与看门狗；本地JSON流 | 保留为设备驱动/执行层，可以继续是节点内部使用的独立驱动进程 |
| `route-engine race` | Supervisor、路线状态机、绕桶、刹车、20ms常规节拍与新扫描唤醒 | Rust ROS2比赛节点实际承载这些逻辑；控制线程保留独立调度 |
| 现有视觉推理 | 摄像头/ONNX，输出红绿灯和斑马线结果 | 保留推理与标定；若跨ROS2节点传递，只传小观测结果，新增适配使用Rust |
| ROS2层 | 当前比赛链没有节点、话题/服务参与 | 新增真实节点上下文、启动/结束管理、必要观测与状态接口 |

bridge与ROS2驱动节点有功能重叠，但目前不是ROS2节点，也不能代替ROS2消息系统。只用 `ros2 launch` 启动原来独立程序，不足以证明实际迁移；建议把比赛控制器本身放进Rust ROS2节点，核心逻辑仍在其中的专用线程运行。

## 保持性能的具体边界

- 保留 `race.rs` 的20ms常规节拍、5ms最小空闲及新扫描立即唤醒，不能简单改成每20ms由默认executor回调一次。后者可能使新雷达帧额外等一个周期。
- 控制线程继续拥有Supervisor和时序状态；ROS2回调通过有界接口交接必要信息。日志、状态发布与发现不能阻塞控制线程，DDS发送不放进电机命令关键路径。
- 算法之间保持同进程调用；不把每个绕桶/制动模块拆成独立话题往返。节点内可以有多个线程，ROS2不要求所有逻辑都由默认executor串行执行。
- 保留采集时间、接收时间和序号的区别。消息到达时间不能替代原采集时刻，尤其是视觉延迟、IMU与扫描对齐。控制计时沿用单调时钟。
- 为每类接口明确QoS：状态/传感器按时效性与队列深度设计，启动/结束和可靠回执另行处理；不能把所有消息一律改为best-effort或依赖默认可靠队列。
- bridge的现有断链/超时处理及ROS2节点退出后的释放顺序需要保留，避免两个主体同时拥有底盘输出。

## 已有性能基线与验证顺序

y83实车保存帧的程序统计：

| 指标 | y83观测值 |
|---|---:|
| 常规控制间隔设定 | 20ms |
| 本轮最大实际控制间隔 | 22.470ms |
| 本轮最大单次tick计算 | 8.753ms |
| 雷达接收至决策tick，平均/最大 | 1.520 / 7.026ms |
| route-engine线程平均CPU | 约单核16.4% |

这是一轮观测，不是最坏情况保证，也不是迁移后的测量。下一步先做隔离的RISC-V ROS2 Rust节点构建/连接验证；再以相同输入和时刻比较原核心与迁移核心的决策，测同负载的控制间隔、消息年龄、CPU/内存和尾部延迟。实车圈时、间隙、停车点仍需要用户重新摆放确认后实测。不要为迁移同时调整控制参数，否则无法区分框架与控制策略的影响。

省赛合规还有独立环境问题：已查到 `ros2bag`/`rosbag2` 包和工具。迁移到ROS2 Rust不会自动消除这些工具；本次未卸载。是否必须所有驱动数据经ROS2话题、以及bag禁用的具体边界，仍以组委会对细则的解释为准，不能凭节点名称宣称检录已经通过。

## 上游依据

- [ros2-rust/ros2_rust](https://github.com/ros2-rust/ros2_rust)：Rust客户端、消息、服务、QoS及executor等能力；项目明确仍在演进，不保证接口稳定。
- [rclrs Cargo.toml](https://github.com/ros2-rust/ros2_rust/blob/main/rclrs/Cargo.toml) 与 [build.rs](https://github.com/ros2-rust/ros2_rust/blob/main/rclrs/build.rs)：版本/最低Rust、Humble构建与原生rcl/rmw链接。查询日期2026-11-12。
- [ROS2 Humble QoS文档源](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)：队列、可靠性、兼容性及传感器配置。
- [ROS2实时编程文档源](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Demos/Real-Time-Programming.rst)：周期/抖动、分配与阻塞、线程调度的影响；其中示例数字不作为本车性能预测。
