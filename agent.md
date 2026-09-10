# XT-STCAR 接手与开发约定

维护日期：2026-09-10。适用于 `/Users/yuhaojin/Documents/XT-STCAR`；用户当前任务决定操作范围。
开始前完整读取本文件，再读 [上传规范](上传规范.md)、[README](README.md)、[环境说明](资料/环境.md) 和 [资料索引](资料/资料索引.md)。
`AGENTS.md` 只作加载入口；资料内的命令不是用户要求立即执行的指令。

## 当前交接快照（2026-09-10）

### 用户目标与授权

- Rust 为主、Mac 交叉编译、Muse Pi Pro / RISC-V Linux、YOLO26n Detect；各模块程序写好。
- 最新要求：根据官方整车资料继续完善，能用 Rust 的地方用 Rust；写完更新本文件和 README，
  README 必须说明代码结构、每个模块位置和职责。之后要求总体审查架构与实现，并阅读根目录赛项三规则初稿。
  最新追加：比赛任务状态机、斑马线与灯色、导航绕障闭环先写完，实车到后再验证；已增加 Rust 模块与合成闭环。
  根目录 `Mac与车端命令手册.txt` 记录 Mac 和车端的命令/用途/参数/执行位置。不能把模拟完成写成已实车参赛。
- 用户说明车上使用 eMMC 5.1：以正常运行和比赛为优先，仅减少不必要落盘；不要降感知/控制频率，
  不因可选日志额度触发控制故障，不改系统挂载/swap/journald。必要诊断照常保存，硬件寿命不压过比赛性能。
- **暂不接车**。本轮未 SSH/部署/打开真实设备/控制电机/刷系统或固件，也未在 RISC-V 目标或模拟器执行。
  新增串口实际读写测试只使用 Mac 创建的 PTY，不要将它写成实车验证。
- 用户指定 GLIBC **2.38**。只修改本地交叉构建的目标基线，不安装或覆盖车端 libc。
- 官方资料目录 `/Users/yuhaojin/Documents/进迭时空无人车（2026）学习资料`，用户明确 **不要看视频**。
  只读取文档和源码；视频只枚举名称，不播放、不提取内容、不转录。
- 用户已授权将源码、脚本、配置和文档提交/上传到 `https://github.com/kxkxkxa767/XT-STCAR.git`，分支 `main`。
  编译链、环境、模型、构建产物、大型厂商包、镜像和缓存不进 Git。
  最新上传约定见 [上传规范.md](上传规范.md)：日常直接在 `main` 修改、提交和推送，不另建功能分支或 PR；
  每次修改前检查云端更新，有更新先拉取；`git commit -m "..."` 简要说明本次具体改动。
- STM32F103 固件读取由用户明确暂停；不恢复解保护、读取或刷写任务。不要回旧 XT-NetRC。

### 最新运动执行状态、过渡峰值、通过点与模拟积分修正（2026-09-10）

- 修改前 fetch 已确认 `9b59125` 与 origin/main 一致；继续按用户授权直接 main 开发/验证/提交/推送。
  本轮已完成下述主机验证与交叉链接，提交/推送状态以 Git HEAD/origin/main 核对；不沿用 v2 的数字或哈希。
  新说明为[运动执行与通过点修正](docs/运动执行与通过点修正.md)，当前证据统一使用 `docs/motion-v3-*`。
- `robot-core/src/navigation.rs` 新增 `SteeringEstimate { at, commanded_curvature_per_m, applied_curvature_per_m }`。
  估计只按已采用命令推进，`advance_to` 沿旧目标推进，`adopt` 先推进再换目标；Stop 归零目标但估计逐步回中。
  即使字段各自有限，其差值溢出也会原子拒绝，失败不修改原估计。
  `plan_with_arrival` / `plan_stop` 不采用规划结果；旧同步 step/stop 包装采用其返回命令。
  `runner/src/autonomy.rs` 同步 `tick` 在最终 Safety 后采用命令，异步 `tick_with_execution_state` 只规划、不自行采用。
- `runner/src/control_runtime.rs` 的真实 Worker 使用上述异步接口。只有输出线程 `poll` 最终的 `ControlPoll.command`
  推进执行估计，固定大小原子槽回送给提交方；未采用、被替换或晚到的计划不会更新执行历史。
  `try_submit` 把提交时已知状态绑定到快照，只有估计时刻不晚于源时刻才能向前推进；否则返回 `ExecutionAhead` 等待新源。
  不倒推未来状态、不阻塞 poll、不续租旧命令；同源 Duplicate 和故障锁存保持。
  同时刻先提交后采用的新命令不会追溯更新排队副本，持续滞后实车源仍需共同时间轴/有界历史关联；这不是测得的转向反馈。
  专项 `control_runtime` 14 项及对应 clippy 已通过，包含在下述本轮完整验证范围中。
- 新 Rust `robot-core/src/motion_transition.rs` 的 `MotionTransition` / `lateral_acceleration_peak` 解析检查
  速度和曲率独立渐变时 `v²|曲率|` 的端点、分界及内部峰值，固定计算量、不分配堆内存。
  导航检查至少覆盖完整控制周期和配置预瞄时域，不因接近目标缩短几何预测而漏检；无效数值/超限拒绝候选。
- `ArrivalBehavior::Stop/PassThrough` 区分必停目标与锥桶通过点。续段从第一段实际末端位置/朝向/曲率连接并检查，
  只为速度制动上限提供经过验证的余量；第一段仍单独跟踪、独立按原容差验收，前视和进度不能跳到续段跳过必经点。
  `next_max_speed_mps` 提供下一阶段上限，进入其验收半径前提前限速，避免阶段切换后的速度上限突变。
  它约束候选目标，不承诺入区瞬间测量速度必定达标；外部超速/测量偏差仍可触发空速度区间 Stop。
  续段不可行则按停车处理。锥桶阶段就约束灯前续段的禁越线，停稳/绿灯/普通停车期限等规则门限未放宽。
- `runner/src/simulation.rs` 使用不超过 1 ms 的子步、速度/曲率渐变分界、包含交叉项的航向积分与位置数值积分，
  每个子步检查车体/障碍/边界，故障后制动沿用同一逻辑。控制/感知周期不降低，也不新增逐子步磁盘日志。
- 新积分暴露 PP 灯前回归，额外修正两个已复现问题：带目标朝向时，缓存参考横向偏离超过 `goal_tolerance_m × 0.5`
  便提前清路重规划，不能只因缓存无碰撞就假定末端仍可达；oriented rollout 整段评分改为对应四个车身角世界位置差的
  最大值，使用纯米制车体位姿误差，避免提前回正后仍残留横向误差。这改变了位置与朝向的相对影响；
  未调整的是 LQR q/R 参数及比赛门限，不能声称评分权重完全未变。恒候选曲率预览不代表所有未来策略。
- 本轮最终复验 PP 55.6 秒、LQR 83.9 秒均完成，终态速度均为 0；最小锥桶间隙分别为
  0.3032708854919852 m / 0.2735281037908029 m，两者斑马线停稳 3000 ms、绿灯确认 300 ms。
  首次可恢复阻塞分别为 18600 ms / 22500 ms，均保存 17 帧。完整结果见 `docs/motion-v3-competition-comparison.json`，
  中途 PP 失败保存在 `docs/motion-v3-before.json`，不宣称实现期间始终通过；控制周期、容差及时限未放宽。
  测试/构建见 `docs/motion-v3-validation.json`、`motion-v3-build.json`；本地包状态和哈希以 `motion-v3-delivery.json` 为准。
  **PP 默认不变、LQR 仍为实验项。** 暂不接车、不读视频、不执行 RISC-V 目标或真实设备，GLIBC 2.38 仅本地交叉基线。
  本地包、模型、编译链及环境不进 Git；用户规则 PDF 不修改/不暂存，STM32 任务继续暂停。

### 前轮导航预测、灯前约束与诊断修正（2026-09-10，v2 历史）

- 已读取用户[第二份修改意见](https://chatgpt.com/share/6aa27143-7204-83ee-ad9a-7067ef49a1bc)，按源码和独立复现核验。
  修改前 fetch 确认 `8aceeaf` 与 origin/main 一致（0/0）；用户已授权本轮直接 main 提交/上传。
  本轮按既有规范完成检查后直接提交 main；后续接手以 Git HEAD/origin/main 核对同步状态。
  规则 PDF 保持原件，不修改、不暂存；公开分享及完整原始 trace 仅留 `work/`。
- `robot-core/src/navigation.rs` 将正常候选预测与紧急停车范围分开：正常速度按加减速率、曲率按变化率渐变，
  从当前测量速度和上次指令曲率推进；停车可达圆盘仍用 `max(当前速度, 目标速度)`，覆盖命令周期、反应及制动。
  较低目标速度或较短剩余路程不能缩小当前速度所需的保守停车范围，真实制动能力仍待标定。
- 终端连接从父节点曲率实际积分，先尝试单段渐变曲率，再用有界双段 S 形连接补足带朝向的短距离到达。
  两段交界继承实际末端曲率，按实际端点/朝向/碰撞验收；不再把末点直接替换成目标，不允许原地旋转补朝向。
- `robot-core/src/reference.rs` 在每次导航调用中一次验证并准备局部参考窗口，候选复用；弧长 cursor 顺序推进，
  不为每个预测步重扫整个路径。带显式目标朝向时，在原 rollout 的每一步累计参考位置/航向误差，
  曲率偏好按 `0.5 × (v × dt)^2` 缩放，避免固定偏好压过末端误差；无目标朝向时保留追踪点距离评分，
  曲率偏好按剩余距离与前视距离之比的平方淡出。**PP 仍为默认，LQR 权重未改且仍属实验项。**
- `robot-core/src/autonomy.rs` 新增 `HalfPlane`，与 `mission.rs` 共用灯前停止线几何。
  `runner/src/autonomy.rs` 在进入 `ApproachLight` 的同一拍向导航设置边界，全局路径、局部预测、停车可达圆盘及
  `Reached` 都受约束；保持原绿灯新鲜帧/停稳/去抖门限，任务进入 `Finish` 才解除。比赛容差、时限和安全门限未放宽。
- `TrackingDiagnostics` / `NavigationDiagnostics` 提供实际跟踪模式、LQR 已算误差、路径版本/进度、请求/限速/选中曲率、
  预测误差及候选拒绝计数。`runner/src/navigation_diagnostics.rs` 在 simulation 记录首次非预期阻塞/故障前 8 帧、
  触发帧和后 8 帧，最多 17 帧，只保留紧凑内存上下文，结束进入摘要 `first_navigation_failure`。
  正常任务停车/目标制动不触发，首次异常可能恢复，不能自动当成最终故障根因。无每 tick 落盘，不降比赛控制或感知频率。
- 该轮实际结果：导航集成 11 项通过；同一完整合成比赛 PP 在 55,900 ms、LQR 在 53,300 ms 完成，二者终态速度均为 0。
  这不代表实车验收或 LQR 普遍优于 PP。修改前 LQR 在 17,400 ms 首次阻塞、27,900 ms 停车超时的复现保存在
  `docs/motion-v2-before.json`；前轮 `motion-control-*` 的通过项和失败项继续保留为历史。
- 历史说明见[导航预测与失败首因修正](docs/导航预测与失败首因修正.md)。该轮全量测试、fmt/clippy 与两程序 RISC-V 交叉链接已通过，
  最终测试数量、ELF/源码哈希、包及验收结果分别以 `docs/motion-v2-validation.json`、`docs/motion-v2-build.json`、
  `docs/motion-v2-delivery.json` 为准；这些 v2 产物不是本轮 v3 二进制。
- 当前仍暂不接车，不读视频，不执行真实设备或 RISC-V 目标；GLIBC 2.38 仅为本地交叉链接基线。
  低速切换滞回、规划器原生参考曲率采样、共同时间轴/扫描去畸变、真实速度/转向反馈及物理 MotionSink 仍待后续。
  本轮新增折线投影与弧长 cursor 不等于已经保留规划器的原始连续曲率。

### 前轮运动控制评估与实现（2026-09-09，历史）

- 用户要求读取分享链接、评估运动控制并同步README。已成功获取并提取[《算法比较建议》正文](https://chatgpt.com/share/6aa0bf59-c8f8-83ee-b001-13dd5945ce14)，
  先前网络超时不再是阻塞。公开页面原始数据仅留`work/`，没有发布整个分享HTML。建议按当前源码独立核验。
- 修改前fetch确认`7fb06c8`与origin/main一致。继续直接main提交/推送，规则PDF不修改、不暂存。
- `robot-core/src/tracking.rs`新增`PathTracker`、`PurePursuitTracker`及实验`LqrTracker`。
  `NavigationConfig.tracking`省略时仍PP；serde拒绝未知算法和多余字段。PP前视链/公式保持，导航既有1e-6m/s测量容差传入跟踪前统一归零/夹限。
  LQR使用原缓存路径局部投影、三点曲率前馈及横向/航向反馈，闭式连续直线附近模型，无新依赖/轴距猜测/设备I/O。
  低速PP回退，线性化误差域或输入不合法则导航Stop；后续候选、碰撞、停车与Safety通路共用。
- `robot-core/examples/tracking_comparison.rs`：4条路径×PP/LQR×0/200ms假定延迟，共16组进入终点容差；
  当前LQR权重下横向RMS反而更高，不宣称优于PP，模型未含定位噪声/制动，模拟时间不代表CPU时延。
- `runner/examples/motion_comparison.rs`：同一整场比赛PP仍55.3s完成；LQR在绕锥桶阶段停住，27.9s普通停车超时，最终模拟速度0。
  **该历史版本 LQR 未通过全场验收。** 2026-09-10 的修正与新结果见上节；保留该失败证据，默认算法仍不替换。
- README新增运动控制模块/单位/通路/实测边界，TXT增加两个Rust A/B命令；完整说明见[运动控制设计与对比](docs/运动控制设计与对比.md)。
  当前没有ESKF/EKF、速度PI、真实转向反馈或电机MotionSink；先解决时间同步、去畸变、标定及反馈质量，再扩展这些模块。
- 额外审查未找到证据充分的新制动/看门狗/PWM缺陷。侧向加速度检查约束候选目标，Stop记录的是归零目标；
  实际过渡/回中待测，保守停车圆盘考虑中间转向但依赖实际制动能力满足配置。

### 前轮文档补充（2026-09-08）

- README新增“整车与硬件详细参数”：产品介绍第1页全部12项、规则初稿第10页及出厂源码通信/相机配置已对照。
  包含尺寸、CPU/GPU/内存/eMMC/供电、MCU、电调/BEC、电池、舵机、雷达/相机/IMU和零碎接口参数。
- 型号证据分层：出厂驱动选择LSlidar N10，实物待核对；相机/IMU/舵机完整型号未提供，不把平台CMP10A等参考外设认作本车。
  电池5400mAh（产品）/5300mAh（规则）、底盘阿克曼/四轮差速表述、MCU32KB/4KB/40MHz及“最大转速40km/h”均单列来源/疑点。
- `car_controller_new.cpp`的L=0.305、lfw=0.1675、controller_freq=30只记录为源码默认参考，不代替测量。
  该轮仅改README和交接文档，未改变代码/配置。旧0f3ab64、motion-control及motion-v2构建证据保留历史，当前证据使用下述motion-v3前缀。

### 当前代码：5 个 crate、2 个程序

| 模块 | 已实现 |
|---|---|
| `crates/vision` | 纯 Rust 模型契约、RGB/letterbox/NCHW、阈值与坐标解码；road.rs 斑马线/灯色/锥桶及地面投影 |
| `crates/app` → `xt-stcar` | self-check/preprocess/replay/infer；原生 ORT C API 动态加载、模型来源及元数据验证、常驻 Session；Python 参考后端须显式选择；file_io 提供两套 CLI 共用文件边界 |
| `crates/robot-core` | 强类型传感器语义、frame/时间/单位校验、急停/deadman/超时/限值状态机、仅记录的 MotionSink；底盘/WIT IMU/N10 协议与标定表；比赛任务、Stop/PassThrough、灯前半平面、整圈扫描、ICP、执行曲率估计、解析过渡峰值、车模型导航/局部参考/PP与实验LQR |
| `crates/device-io` | 安全 rustix 串口配置、独占、8N1、关闭软硬件流控、读回检查、nonblocking poll 与整体包截止时间、故障锁存、Drop 尝试恢复 |
| `crates/runner` → `xt-stcar-robot` | 严格 JSONL 回放、真实图像推理、原始传感器解析、可选标定 PWM 预览及传感器采集；自主模拟/快照回放、背景感知/规划/独立看门狗及采用状态回传、连续渐变子步车辆模型、有界内存日志/首次异常窗口与结束原子提交 |

#### 前轮比赛扩展（2026-09-08）

本次修改前 fetch 确认 `d245174` 与 origin/main 一致；用户规则 PDF 未修改/未暂存。
完整说明：[Rust比赛自主闭环](docs/Rust比赛自主闭环.md)，日常命令：[Mac与车端命令手册.txt](Mac与车端命令手册.txt)。

- `robot-core/autonomy.rs`（均在 `src/` 下）提供米制坐标、足迹、位姿质量、道路观察共享结构；
  `mission.rs` 实现 Idle→ApproachCrosswalk→CrosswalkStop→Cones→ApproachLight→WaitGreen→Finish→Completed/Fault。
  连续新鲜反馈确认停稳才累计3秒；灯前全部足迹/朝向/停止与绿灯新帧去抖；终点全车进区；普通停>10秒失败。
  正常比赛 Stop 与锁存安全 Fault 分开，移动/未知灯色/旧帧不能偷偷完成任务。
- `vision/src/road.rs`：Rust HSV/连通域、白条几何、红蓝锥桶脚点、ROI 灯色和地面投影。
  当前80类 YOLO不含斑马线/锥桶/灯色，`zebra`仍为动物；不更换原权重，YOLO9类只辅助圈定交通灯。
- `robot-core/src/scan.rs` 整圈组帧与覆盖/连续盲区检查；`localization.rs` 有界 ICP，无编码器/全局SLAM声明。
  `runner/src/laser_pose.rs` 检查整圈后按显式TF生成车体点云，输出保留扫描源时戳；掉圈超时需显式已知位姿reset。
- `robot-core/src/navigation.rs` 用膨胀网格连通检查、带转弯和转向速率约束的搜索、跟踪及保守停车可达圆盘检查，覆盖转向回中期间的中间曲率。
  修复全局车模型允许穿网格角而局部正确拒绝导致反复停住的问题；全局/局部均拒绝切障碍角。
  灯前和终点带目标朝向，不允许靠原地旋转伪装阿克曼调头。搜索预算有界但不保证目标CPU最坏时延。
- `runner/src/autonomy.rs` 串联同步Pose/Scan/Road→Mission→Navigation→Safety。
  扫描与用于其世界投影的位姿必须同采集时刻，斑马线锁定/视觉锥桶投影同样要求图像与位姿采集时戳一致；
  不能用freshness/skew代替关联。重复帧不能改内容；故障锁存，输出仍只有检查后的值/记录。
- `runner/src/perception.rs` 常驻原生ORT和同图RGB复用，PerceptionWorker只保留最新待处理图像；
  `control_runtime.rs` AutonomyWorker将规划放后台，独立周期调用poll获取命令。
  lease由源快照/最旧传感器时刻决定，先检查旧期限后接纳结果；晚到Drive不能恢复Fault。
  必须使用ControlPoll.command，latest仅诊断，不能旁路；Drop不无限join卡住的原生线程，也不能无限重建线程。
- `runner/src/simulation.rs` 真实执行合成RGB识别、360束测距、任务/导航与有加减速运动学反馈；
  不是手写Motion回放。模拟Pose明确为理想反馈，不冒充ICP实测。运行中视觉错误进入Fault并保存终态；
  Fault/超时后继续模拟制动直至速度0并检查碰撞，不从中途Err直接绕过停车分支；Stop目标速度/曲率均0，按速率回中。
- `runner/src/autonomy_replay.rs` 单调SensorSnapshot{at,pose,scan,road}诊断输入，拒绝Motion与旧event格式；EOF强制Stop。
  `telemetry.rs` 默认transitions，结束才落盘；trace默认1MiB、上限8MiB、重要事件独立保留，满额只丢调试记录，
  logging_errors/dropped_trace_records可查，不让可选日志影响控制。默认不存逐帧图像/点云/张量。
- 新CLI：autonomy-example、autonomy-sim、autonomy-replay、road-detect。
  新配置：competition-sim、competition-controller-sim、road-perception-sim、scan-assembly-sim、laser-localization-sim（均config/*.json）。
  最后两份供库接口，不能当成已有设备CLI参数。全部为合成值，禁止把simulation_only/unverified改标签就当实车已标定。

#### 已完成的 Rust 模块（前轮扩展）

1. **N10 Rust 解析**：`robot-core/src/protocol/n10.rs`。依据出厂 `.cc`，固定 58 字节、16 槽、
   大端角度/距离、uint8 强度、前 57 字节累加模 256。支持分包、粘包、坏帧重同步、有限缓存与过期保护。
   保留无效槽位，修正厂商按有效点数计算插值分母的角度偏移。
   `packet_sample()` 只生成 **16 束局部样本**，全未知或零跨度返回 None。
   局部包新鲜不代表全圈覆盖或前方无障碍。当前新 `scan.rs` 另行整圈拼接，旧replay仍为局部包；没有 ROS 发布。
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

#### 前轮总体审查修复

审查开始前 fetch 确认 `6c7de79` 与 origin/main 一致、工作区干净；用户随后放入规则 PDF，保留原件但不暂存。
完整结论、回归证据与当前产物见 [总体架构审查](docs/总体架构审查-2026-09-08.md)。

- `app/src/file_io.rs` 统一静态输入非阻塞打开/fstat/实际读取量；runner 重新导出旧 API，删除重复实现。
  app 图像尺寸与解码复用同一文件句柄，配置/输出张量/provenance ≤1MiB、ONNX ≤64MiB；Python 独立工具同样有界。
- 视觉输出和 Python worker 拒绝覆盖 FIFO 等非普通目标；预处理两个输出先共同预检。
  显式/PATH 中的 Python 解释器纳入输入保护，保留虚拟环境符号链接；PATH 未设置时裸命令须改用显式路径。
- 雷达派生角跨度/末束角度也须有限；溢出进入锁存 Fault/Stop，不被后续正常样本清除。
- ELF 版本核验绑定动态加载映射，检查所有 verneed 要求，新增 `version_requirements`；不能用 section 表伪装低 GLIBC。
  ELF 输入非阻塞且实际 ≤64MiB；报告拒绝输入别名和非普通目标，通过后原子保存，失败保留原报告。
- 交付测试取消对未跟踪 `tmp/` 目录的依赖，新克隆未构建时正确 skip。

crate 分层无环；最新扩展增加 vision→robot-core 的纯类型依赖。历史审查不覆盖全部新模块，新模块证据单列。

### 构建、模型与验证证据

- 当前 v3 运动执行修正：Rust 常规 266 通过、0 失败、原生 ORT opt-in 1 项忽略；跟踪 example 2 项、Python 交付 34 项通过。
  fmt、all-targets clippy -D warnings、两个 RISC-V 交叉链接及 PP/LQR 完整场景复验通过。
  当前汇总入口为 `docs/motion-v3-validation.json`，构建与交付为 `docs/motion-v3-build.json`、`docs/motion-v3-delivery.json`。
  交叉链接通过不代表本地包已核验；包的状态、路径和哈希以 delivery 报告为准。不把以下历史证据算入本轮。
- 前轮 v2 运动修正（2026-09-10，历史）：Rust常规237通过、跟踪example测试2通过、交付测试34通过；
  fmt、all-targets clippy -D warnings、两个RISC-V交叉链接通过；导航原11项及PP/LQR全场通过。
  历史汇总入口为 `docs/motion-v2-validation.json`，构建与交付分别见 `docs/motion-v2-build.json`、
  `docs/motion-v2-delivery.json`；不把该轮数字计入本轮。
  历史robot大小1,918,960字节，SHA256 `f5652cb05757ea135eaff714be5f5af4e2770151118c50197534356942bd6863`；
  历史xt-stcar SHA为`ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9`，当时二者最高GLIBC引用2.34。
  该轮PP首个可恢复阻塞在14300ms、窗口17帧，最终仍完成；LQR没有触发首因窗口。模型/原生推理未重跑。
- 前轮运动扩展（2026-09-09，历史）：Rust常规209通过、跟踪example测试2通过、Python交付34通过，fmt/all-targets clippy及两个RISC-V交叉链接通过。
  `docs/motion-control-validation.json`汇总，配套build/ELF/log/两份A/B JSON均在`docs/motion-control-*`。
  该轮模型和原生推理代码未改，未重跑模型/ORT执行测试；这些通过项不能计入本轮实测。
  历史robot大小1,884,168字节，SHA256 `902b350111e26b1767d343c5ec43a450fb16fe8f5c86e2603a53c5016feb6704`；
  历史xt-stcar SHA为`ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9`，当时二者最高GLIBC引用2.34。
  历史core/模型包大小/哈希/路径与核验见`docs/motion-control-delivery.json`；不把它们当成本轮产物，编译链/环境不进包或Git。
- 前轮比赛扩展（历史）：Rust常规191通过、原生ORT opt-in 1通过、Python模型48/交付34通过；fmt/all-targets clippy通过。
  两程序RISC-V交叉链接通过。最终合成场景55.3s/554tick、最小锥桶间隙0.301m、斑马线3000ms/绿灯300ms、终点速度0。
  默认日志24,375字节、无丢弃/错误；这些是合成结果，不是实车性能。
  详见 `docs/competition-validation.json`、`competition-simulation-summary.json` 及对应日志。

- 本机 Rust/Cargo **1.97.1**，rustfmt/clippy/RISC-V std 已安装；`rust-toolchain.toml` 和 `Cargo.lock` 锁定，
  不改全局默认。Zig **0.15.2**、cargo-zigbuild **0.23.4** 在项目 `toolchains/`，无需重复安装。
- 新串口依赖 rustix **1.1.4**；两个目标程序由 `scripts/build-riscv.sh --offline` 检查并构建。
  目标 `riscv64gc-unknown-linux-gnu.2.38`，ELF64 LE RISC-V / RVC / LP64D / PIE，
  加载器 `/lib/ld-linux-riscv64-lp64d.so.1`。v2最高 GLIBC 引用 2.34；当前实际引用和依赖以 motion-v3 ELF 报告为准。
  前轮 robot 程序额外需要 `libm.so.6`，不能声称两个程序都只依赖 libc。
- **当前构建证据使用 `docs/motion-v3-*`；`docs/motion-v2-*`、`docs/motion-control-*`、[Rust比赛自主闭环](docs/Rust比赛自主闭环.md) 与 `docs/competition-*`保留历史证据。**
  `docs/architecture-*` 及 [总体架构审查](docs/总体架构审查-2026-09-08.md) 保存前轮扩展前的构建、测试和包，不能当当前版本。
  `rust-expansion-*`、旧 2026-09-07、`factory-*`、`glibc-*` 记录保留为历史，不是当前二进制。
- 主程序在 `target/riscv64gc-unknown-linux-gnu/release/`，新 core/模型包在 `dist/`；
  打包白名单已包含 N10、标定、串口、5份比赛配置、比赛说明及TXT命令手册。Git 不提交这些产物。
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

- 根目录用户提供的《赛项三-RISC-V_轻量无人车赛比赛规则（初稿）》已完整读取 10 页并逐页查看渲染。
  封面日期 2026 年 8 月，SHA256 `24dfa87d60142be349eb4e7e862e09b1ec0c4e3771010e9471d2c850144eb7d6`。
  [规则与工程差距](docs/赛项三规则与工程差距.md) 按页列要求与歧义；PDF 原件留本机，未读视频/网盘/提交材料。
  Rust 未禁、ROS2 未明文强制；ROS1 与 bag 工具明确禁用，范围需最终规则澄清，不引入它们作为比赛依赖。
  斑马线前停足3秒、按图绕两锥桶、绿灯才放行且四轮在灯前指定区域停车、终点四轮入区；技术案例占20分。
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

- 新任务/视觉/扫描/定位/导航模块和独立线程接口已有实现，真实相机采集、共同单调时钟、扫描去畸变、
  位姿插值/融合、实际地面矩阵/灯ROI/场地图和真实制动反馈仍待验证。模拟配置不允许直接实车启用。
  新闭环中的Motion来自传感器计算；旧replay仍按输入事件诊断，不能混为比赛实时程序。
- **官方 RISC-V ORT/EP 2.0.6 仅静态核验**：资料在 `work/spacemit-runtime-followup/`，见
  [原生库核验](docs/SpacemiT原生运行库核验.md)。头文件 API24、导出 OrtGetApiBase；tag 代码支持 API22，
  但发布 manifest 提交不同，尚未执行发布库 GetApi(22)，未测试模型或 EP 初始化。
  ORT/EP 需 GLIBC 2.38，EP 另需 GLIBCXX 3.4.32 / CXXABI 1.3.15；未安装/打包厂商运行库。
- N10整圈、ICP和局部导航已完成离线模块；真实多传感器连接、ROS 2 接口及 MCU 反馈/watchdog 尚未实现或验收。
  接收健康与安全状态机不代替避障；静态标定表不表达 ESC 动态制动、死区、迟滞或多步骤倒车。
- 现有串口传输具备可执行实现，但没有真实设备测试，物理实时闭环和 MotionSink 尚未接入。
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
  按 [上传规范](上传规范.md) 直接使用 `main`；修改前 fetch 检查，有远端更新先拉取后修改。
  完成检查后以简短、具体的 `git commit -m` 提交说明提交并推送；推送前再次检查远端，保留队友改动，禁止强推。
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
