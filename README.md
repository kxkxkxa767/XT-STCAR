# XT-STCAR

Muse Pi Pro / RISC-V 无人车工程：**Rust 为主，在 Mac 交叉编译，视觉使用 YOLO26n Detect。**
工程目录为 `/Users/yuhaojin/Documents/XT-STCAR`，源码仓库为
[github.com/kxkxkxa767/XT-STCAR](https://github.com/kxkxkxa767/XT-STCAR)。

目前有 **5 个 Rust crate、2 个可执行程序**。视觉预处理/后处理、原生推理调用、传感器协议、
串口传输、比赛状态机、场地规格与布局生成、在线元素跟踪、局部可通行空间、道路识别、激光里程计、路径规划、底盘标定映射和调度均由 Rust 实现。Python 用于离线模型导出、校验、参考对照及独立硬件诊断/相机预览；
默认推理由 Rust 直接调用 ONNX Runtime C API，不启动 Python 进程，推理引擎仍是上游原生库。

**2026-09-16 已进入车辆架空模块测试。** 用户确认车辆到场、车轮架空，SSH已接通；旧“暂不接车”是历史边界。
车上实测为 Bianbu 2.2 / RISC-V、GLIBC 2.39、约8GB内存；保持系统libc不变。已将原哈希核验通过的含模型包放入独立测试目录，未覆盖厂商工作区。
Rust运动主程序目前仍使用记录输出，架空硬件诊断与自主闭环验收分开。厂商资料视频未读取。

| 实车模块 | 本轮状态 |
| --- | --- |
| 载荷与目标程序 | 车端SHA256SUMS全部通过，动态依赖齐全，视觉无模型自检及机器人离线回放通过 |
| USB相机 | 连续640×480/1920×1080取帧均无读取失败；V4L2 MJPG约29.8fps，OpenCV转BGR读循环分别约14.9/9.93fps，不能混为推理帧率 |
| 蓝牙 | 修复并安装系统完成：临时扫描8设备、系统扫描7设备；厂商服务active/enabled，控制器已上电。配对和重启后验证未做 |
| 雷达 / IMU | 雷达追加10秒采集2815包，协议错误0；主机用生产组帧器得到99圈、覆盖率99.17%–100%、拒绝0圈。IMU原3秒299样本，平均加速度模长9.819m/s²；安装外参和零偏未标定 |
| ONNX Runtime | 已安装SpacemiT 2.0.6 / ORT1.24.2+spacemit.a1到车端用户目录，并设为工程启动命令默认库；API22与真实图片推理通过。系统1.18.1和Python1.18.0保留，未替换libc |
| 规划与任务 | 完整固定场目标端执行通过：模拟75.6秒/757拍，主机耗时105.64秒；首次90秒超时保留。仅合成闭环，在线正常整场此前0/15完成 |
| 底盘 | 用户确认1450向右、1550向左、1500回中；0.5秒测试1510/1519未起转、1520向前转并停住；延长至10秒后1519间歇转了两次，1520起转且最终停住，不能据短脉冲确定固定死区。1515过程未确认；速度/转角/响应与制动时延未标定 |

真实相机图像推理输出人（0.870）和杯子（0.357），单次命令报告约1.71秒；这是单帧功能证据，未启用厂商EP，也不是实时帧率验收。道路检测入口同图运行成功，但沿用模拟标定时在室内画面报出一个未验证锥桶候选，不能作为实车识别通过，须用本车外参及赛道样本验证。

实际命令、结果和限制见 [车辆到场模块检查](docs/车辆到场模块检查.md)、[机器验证记录](docs/vehicle-bringup-validation.json)和[雷达相机追加验证](docs/vehicle-sensor-followup-validation.json)。
8080相机预览脚本为 [camera-preview.py](scripts/camera-preview.py)，已在车端验证单帧和连续MJPEG；[后台启动与停止命令](docs/车辆到场模块检查.md#相机8080网页预览)。测试服务已停止，按需启动。
车端现在可用 `~/.local/bin/xt-stcar infer --image /绝对路径/图片.png`，自动选择已部署模型和新运行库；[安装、使用与回退](docs/车端ONNXRuntime升级.md)。入口源码为 [vehicle-vision.sh](scripts/vehicle-vision.sh)，Rust程序和原部署包未改写。
蓝牙工具及配套固件已通过临时和系统扫描验证，安装到系统目录；已停用冲突的用户级Blueman KillSwitch插件。[修复步骤与状态](docs/车端蓝牙修复.md)。
新增[网页驾驶台](web/vehicle-console/README.md)：方向键/WASD按住持续输出、松开回中，PWM可调；相机/雷达实时预览，直接保存相机JPG、雷达PNG或单张双画面PNG；原始数据/录制ZIP为可选。车端127.0.0.1:8081经SSH隧道访问，默认锁定、前进上限1620，倒车代码已实现但实车默认禁用待标定。已验证真实图像、点云及保存，本次未启动电机。[验证记录](docs/vehicle-console-validation.json)。
晚间追加1550/2秒和1600/0.3秒均由用户确认未动；1600/2秒用户估计前进4–5米并确认停住；1620/0.3秒前进不到1米并停住复位；1620/2秒用户观察向左偏（原因未定），已回中补发停止帧，用户确认完全停住并要求结束测试。当前场地扫描未通过质量门，最高速度仍未测。[追加测试记录](docs/vehicle-speed-probe-validation.json)。
已新增Rust自适应起步辅助：连续有效定位确认未动时限时、小步增加PWM；微动后禁止继续加档，确认起步交回速度控制，失效/超时/上限停车锁存。提供`startup-replay`离线入口，尚未接入实车自动油门。[设计与使用](docs/自适应起步辅助.md)。
最新1秒起步测试：1510–1545每5一档未检出明显净位移；1550首轮仅前倾回位、复测前进约18厘米，1551/1552分别约34/39厘米。最低已观察起步为1550，但固定阈值和重复性未建立；最高车速未测，当前保持停车。[起步PWM记录](docs/vehicle-startup-pwm-validation.json)。
9月17日落地测试已结束：1550起步重复性不稳定；后续用户指定1600、0.3秒，用户观察约1.25米、无明显偏移，超出1米目标。离线雷达宽范围匹配约1.22米但质量不足，不能算定距成功；当前停止追加运动。[本轮记录](docs/vehicle-one-metre-attempt-validation.json)。
9月17日固件复查：发现U-Boot/OpenSBI 2.2.7候选，但安装会直接写启动分区，未完成车辆镜像兼容核验，未升级；视频/GPU配套源无新版。[固件检查记录](docs/车端固件更新检查.md)。
车端系统更新已重新检查当前配套源：普通升级0包；完整升级模拟会删除当前内核并降级系统包，未执行。[更新检查与蓝牙异常](docs/车端系统更新检查.md)。
新增比赛模块与验证边界见 [Rust 比赛自主闭环](docs/Rust比赛自主闭环.md)，完整命令见 [Mac与车端命令手册.txt](Mac与车端命令手册.txt)。
前轮 [总体架构审查](docs/总体架构审查-2026-09-08.md) 保留作历史，交接状态见 [agent.md](agent.md)。
已完整核对用户提供的 10 页比赛规则初稿，见 [赛项三规则与工程差距](docs/赛项三规则与工程差距.md)。
日常直接在 `main` 开发和推送；修改前检查并拉取远端更新，提交信息简述本次改动，详见 [上传规范](上传规范.md)。

## 在线元素驱动任务

新增在线任务模式：**任务顺序固定，元素几何由连续观测更新，短距离目标随当前任务生成。**
RGB 提供带颜色的锥桶候选、斑马线近沿与方向；锥桶必须经过同刻雷达关联。局部记忆保留身份、时间、误差与已处理状态，
LocalWorld 的完整面积空闲证明最多用 64 次圆域核验：保留原外接圆成功路径，否则用固定栈自适应二分外包矩形，所有叶矩形都须完整被已证空闲圆覆盖，额度耗尽则拒绝。Obstacle 独立检查真实回波到最多 32 点凸包的欧氏距离及原量程/位姿/年龄误差，覆盖区域多出的面积不制造碰撞结论。半径候选区分 KnownFree/Obstacle/Unknown，未来 Unknown 只可保留为规划假设，当前目标车体和完整执行停车包络仍必须 KnownFree。目标仍经过原导航、默认 Pure Pursuit、碰撞/制动与异步采用检查。

已确认锥桶可由当前雷达几何维护原身份：`last_visual_at` 与 `last_geometry_at` 分开，缺少新几何满 1.8 秒失效，未获新视觉满 20 秒失效；雷达不创建颜色/身份或增加视觉确认数。
位置/方向仅在纯 roundoff 范围内保留旧浮点表示（最多 1 nm / 1e-9 rad），毫米变化仍更新，误差和时间不冻结。
绕桶半径按实际车体半宽、桶几何误差与原曲率能力选取，前后悬保留在全扫掠检查中；已选半径仍可行时保持，避免视点变化使目标向外跳。
完成绕桶还须真实沿出口方向越过桶中心位置误差，出口目标延伸位置误差加两份原目标容差；原半圈、外侧、位置和航向条件继续生效，靠近圆顶不能提前计数。
剩余转角为零仍按实际切向证明完整车体和净空 KnownFree，不自动放行，也不直接判成 Unknown；真实出桶完成门保持。
前方短弧未知时提前使用原接近限速 0.18 m/s，完整已知空闲才允许原巡航上限 0.3 m/s；当前执行门不变。斑马线停车目标也计入区域航向误差造成的边角位移，不能只减位置误差。
入口必要时用原 Stop 条件切向对齐，无新增 hold。稳定带航向目标可用两个局部 horizon（默认 1.6 m），更远仍是单 horizon 的无航向临时引导；搜索保持默认 0.8 m。
真实 26,580 ms worker 短连接回归补充了同目标漂移后的原到达区域重认证，仍共用原求解预算；这些修改不等于整场验证通过。
导航初始化时默认 `TargetPolicy::Fixed`，在线统一为 `RollingLocal`。49,580 ms 的滚动无航向局部点问题由同账短单弧修正：适用距离 `[0.35 m, min(2 × lookahead, 2 m))`，严格中心连接失败后可认证原半位置容差内的真实积分端点。21,380 ms 的定向斑马线接近则在原单/双弧后，额外认证当前执行曲率按原模型向零渐变的短前进候选，仍要求累计误差下的原半位置与航向容差。Fixed 保留原搜索顺序；最新 `legacy-integration` 八场已完成并通过原性能门，九项关键指标、输出计数和单因素比较与 v9 一致，初版通用开关的退化仍保留。这里未声称所有命令与完整轨迹逐点相同。
小于 0.35 m 的定向 RollingLocal 目标按原严格双弧→原严格单弧→原到达区域→lattice 顺序、同账重证当前边界，修复真实 18,100 / 18,800 ms 短路线变成长回环。仅 RollingLocal 在原停车圆盘证明失败时追加完整 StoppingEnvelope 证明，保留两控制周期、全制动、历史速度上界和最大曲率下任意变向；Fixed 不改。
RollingLocal 原到达区域和零目标曲率候选均失败后，可同账尝试“保持实际曲率→按原变化率回中→直行”三段短接，连续携带误差，真实端点仍须满足原半位置/航向容差。候选动作的原严格 continuation 全部失败后，另可从真实周期预测状态用旧到达区域求解器证明连续性，不直接认定任务到达或采用命令。
仅 RollingLocal 连续 recovery 的有限侧带边界可追加完整车体扫掠证明：端点车体凸包加 `L/2 × (1 + Kmax × 车体外接半径)` 及原余量和累计误差，整体须处于同一允许侧；世界边界与障碍仍使用原胶囊，Fixed 和点状参考检查不变。
若原胶囊及一阶界仍拒绝，第三份证明用实际速度/转向模型的车体点二阶界 `M = A + V²K + r(AK + VS + V²K²)`，将完整运动包在端点车体凸包加 `M×dt²/8` 内。dt 与实际积分、加减速缩尾完全一致，保留原 A/S、端点误差和全部余量，不用低速除法反推时间，也不增加采样或预算。真实 32,000 ms 与邻帧专项区分“路线可行”和“执行可认证”；严格短接提前成功可能改变后续轨迹，整场结果单独列于下文。

`OrbitSpeedHint` 只为绕桶选速：实际姿态加五个观测短弧参考姿态检查完整停车包络，原上限已证则保持，否则最多八次二分取已证正下界，同时限制当前与后续目标。这些离散参考姿态不是连续执行证书，原来源、历史速度、负向约束、lease 与实际停车证明保持。
绕桶时还按当前原始来源姿态检查 pending/confirmed 停车线负约束，用相同误差时钟和完整停车包络给出有界速度提示。只接受已证且不低于“同源速度按原减速度、本拍实际间隔及 adoption 区间允许”的速度上限，同时限制 continuation；冲突时不向上凑限值，也不假装真实车辆已降速。原 source/history、负线和最终采用门保持，不能用提示直接放行。
停车区/终点槽内真实唯一重观察可续接原身份，跨几何 TTL 首新帧重新计一次、默认第二不同采集帧才恢复确认；invalid/歧义不复活，processed 不清除。runner 显式 pin 至多两个当前引用的区域槽，防止身份回收；不延长 TTL、不伪造新观测或放行过期几何。

`FieldSpec` 在该模式中只用于模拟器生成图像、雷达和裁判真值；不同场景使用逐值相同的在线控制器配置。
灯前/终点的真实可见图案尚未确认，目前只支持**显式实验单/双白条协议**。标记检测器默认禁用，在线示例显式启用；
视觉条纹跨度与协议子区宽度分开：灯前 0.6 m 条纹对应 0.8 m 宽停车子区，终点两条 0.9 m 条纹对应 0.9 m 宽子区；这些是实验协议，不保证真实赛场存在相同标记。
存在两个独立缺口：官方资料没有可用的左桶出口→灯距离正下界，固定灯色 ROI 又不能定位；已确认的地面线也可能在清尾前退出前向 ROI。特定默认直行估算清尾至少约 2.44 s、计既有误差约 4.89 s，超过 1.8 s 几何期限；该估算不是全场不可行证明或实跑成绩。
保留灯色无几何时 Stop。过期 StopLine 的负向约束持续增加误差，完整包络仍可在保守线前或扩大侧带之外通行；过期证据不能授权新任务获取、绿灯或清尾。后续需要可验证的新定位信息或在原约束下保持实际地标可见的路径，当前不能保证在线整场完成；车辆现已进入架空模块测试，尚未验证落地闭环。
终点完成显式要求全车清空灯区：即使合法终点与灯区重叠，已到达停稳而未清尾仍 Fault，不提前标记终点；实际清尾后的部分重叠可以正常处理。

入口为 `online-example` → `online-compile` → `autonomy-sim`，详见[在线元素驱动任务](docs/在线元素驱动任务.md)和命令手册第十二节。
冻结源码后的最终 19 场矩阵已完成：**正常场 0/15 完成，在线比赛闭环尚未验收**。4/19 符合预定检查，仅为缺标记、缺锥桶各同步/异步的安全停止。14 场独立裁判证明两桶（12 个正常场、2 个缺标记场），随后均在 ApproachLight 因看见灯而缺少确认的停车区几何停止。三个小场仍未通过第一桶：同步 46.200 s、异步 43.863 s、扰动异步 45.045 s 因普通停车超过 10 秒结束；不能把所有失败归为灯区缺信息。
首轮至第五轮及 82.8 s、84.383 s 等开发失败均保留，不能替代最终记录。矩阵的运行前后源码账、真实命令、可执行文件及原始输出哈希已核对；最终工作区 565 项通过、3 项按设计忽略，双 RISC-V 链接通过。构建与包完整性不表示在线比赛完成。
模拟结果同时核对独立 `online_referee` 的实际轨迹/顺序证据；任务自报完成不能替代裁判。摘要保留末 16 项诊断，显式 `online_matrix async-trace` 最多保存前 2048 个计划，仅用于模拟排查。
本轮实际成功、失败和限制以 `docs/online-mission-*` 正式报告为准，旧场地矩阵不作为在线验收。
开发记录中的 `OpenEmpty` 若发生在 rollout 之前，只能说明原约束下有界规划未找到路径；不能据此声称几何无解、节点耗尽或已经被执行门拒绝。最终在线矩阵如实保留失败，局部改善和旧八场恢复均不作为在线通过结论。

2026-09-16 接手诊断已补齐小场异步 33.180 / 33.260 / 33.380 s 的真实输入和完整导航缓存，见[首拒快照](docs/小场异步首拒快照.md)。原版与隔离观测版的 439 条计划及整场功能字段一致；三帧夹具可直接回放。33.260 s 的17次拒绝定位到有限停车线侧带：当前候选的端点车体余量已不足原净空或净空加起步预留，仅缩紧扫掠管不能解决。生产算法未修改，小场仍失败；后续检查原约束下的目标、路线形状和候选覆盖，不能据此判断物理无解。

## 赛前测量后生成固定布局（保留的历史模式）

已增加 Rust 场地规格模块，输入实际长宽、上下直道宽度/跨度、锥桶中心及灯前区域，就能生成发车区、搜索区、绕桶通过点和终点。
规则初稿的外场范围为 **长 5–8 m、宽 4–6 m**，上下直道分别 **1–2 m**；各项在范围内还要检查组合是否相容。
80 cm 发车/终点区、白条、锥桶底座、车体尺寸、控制限值和任务等待条件都不按比例缩放。

这是**赛前测量后适配**：运行前编译并冻结布局。斑马线依靠实际图像前沿和同采集时刻位姿确定停车点，实时障碍继续参与避障；
该模式不自动巡视测场；本轮在线分支另由短期元素身份和观测几何产生目标，仍未验证现场灯前/终点识别。
模板与逐字段测量方法、命令、限制见[赛场规格自适应](docs/赛场规格自适应.md)。旧八场时序回归仍是同一个简化布局，不是八种赛场。

历史固定九种布局的同步/异步矩阵：**PP 18/18 通过，实验性 LQR 17/18**；LQR 的 5×6 m 同步场仍在左桶阶段停滞。
旧八场时序回归完成并通过原性能门，时间/路程保持 v9。结果及失败均见[场地矩阵](docs/field-adaptation-matrix.json)和[验证报告](docs/field-adaptation-validation.json)。
这不等于无人测场或保证尺寸范围内任意组合都能完成，实车仍待验证。

## 整车与硬件详细参数

资料核对日期：2026-09-08。以下是**资料标称与出厂代码参考**；本车2026-09-16实测结果见顶部模块表，未核验的规格不代表本批次已验收。
来源标记：

- **P**：[XT-STCAR 产品介绍](资料/XT-STCAR_产品介绍.pdf)，第1页“产品参数”，共12项；本次逐项核对原始表述和页面。
- **R**：用户提供的《赛项三-RISC-V_轻量无人车赛比赛规则（初稿）》，2026年8月初稿，第10页设备表；原件留本机，见 [规则来源与核对记录](docs/赛项三规则与工程差距.md)。
- **F**：2026学习资料的 `4.出厂源码/racecar.zip` 与配套文字教程；见 [整车资料核对](docs/无人车2026资料核对.md) 和 [文件来源/哈希](资料/官方参考/无人车2026资料核对/SOURCES.json)。源码路径以下均相对压缩包内 `racecar/src/`。
- **U**：用户在本任务中补充的信息；平台通用教程的外设型号不自动归入本车清单。

### 整车、主控板与存储

| 项目 | 参数或型号 | 来源与适用范围 |
|---|---|---|
| 整车名称 | 进迭时空智能车 **XT-STCAR** | P；R指定北京小豚科技提供的标准智能车，具体批次仍需核对 |
| 外形尺寸 | **388 × 221 × 290 mm** | P原文顺序；未明确标出三轴名称，不直接用作控制足迹 |
| 上位机板卡 | **MUSE Pi Pro / MUSE Pi PRO** | P、R；具体PCB修订号、SoC完整料号待铭牌/系统核验 |
| CPU | **8核、64位 RISC-V X60，1.8 GHz** | 核数/架构见P、R；频率见P，X60标注不代替完整芯片料号 |
| GPU | **Imagination IMG BXE-2-32** | P；不表示已启用Rust推理加速 |
| 标称算力 | **20 GFLOPS** | P、R未注明计算精度/测试口径；不是YOLO实测FPS |
| 内存 | **8 GB LPDDR4X** | P；R也标8GB |
| 板载存储 | **64 GB eMMC 5.1** | R；P标64GB，U明确eMMC 5.1；闪存品牌/料号/剩余寿命未测 |
| 无线网络 | 板载 **Wi-Fi 6 + Bluetooth 5.2** 模组 | P；无线芯片料号、天线和实际驱动版本未给出 |
| USB接口 | **4 × USB 3.0、1 × USB 2.0** | P标称；实际接口形态、共享带宽和车上占用待核对 |
| 主控板供电 | **5V / 9V / 12V DC，经USB-C** | P标称；不是整车电池接口或直接接线说明，供电协商/转换板待核对 |
| 产品软件标注 | **Ubuntu 24.04、Python 3.12、ROS2生态** | P；这是厂商资料的软件环境，不限制本工程使用Rust |
| 出厂镜像线索 | `RaceCar-MUSEPiPro-Bianbu2.2-v1.0.zip` | F文件名和烧录教程；未读取rootfs或核实本车系统版本 |
| ROS版本线索 | 启动脚本优先Humble，部分链路可退Foxy | F；本车发行版、驱动依赖和服务需登录后只读核验 |

本工程交叉链接基线仍为 **GLIBC 2.38**；它是构建设置，不代表车上已经安装2.38。
存储策略以正常运行和比赛为先，只减少不必要的图像/点云/中间张量和逐tick落盘，详见后文及比赛模块说明。

### 底盘、下位机、电机与供电

| 项目 | 参数或型号 | 来源与适用范围 |
|---|---|---|
| 底盘结构 | 原文同时写 **“阿克曼结构”**、**“四轮差速驱动”** | P存在表述冲突；需核对实际转向/传动机构，不能据此直接选择差速运动学 |
| 下位机MCU | **STM32F103** | P、U；仅有系列名，没有完整后缀、封装、板号或可核验固件工程 |
| MCU存储/时钟原文 | 程序存储 **32 KB**、RAM **4 KB**、时钟 **40 MHz** | P原样记录，尚未核验；不能据此指定Flash布局、Rust裸机目标或下载参数 |
| 电机类型 | **碳刷电机，15T** | P；R也写碳刷电机，具体品牌/料号未给出 |
| 速度相关原文 | R写“最大转速 **40 km/h**” | 单位是速度而非转速；不当作电机rpm、实测最高车速或比赛限速 |
| 电机驱动器输入 | **7.4～11.1 V** | P；电调完整型号、工作/峰值电流、制动及倒车逻辑未给出 |
| BEC输出 | **5 V / 3 A** | P；实际接线、负载分配和舵机供电来源待核对 |
| 电池类型 | **3S锂电池** | P、R；电芯品牌、倍率、保护板、插头与充电器型号未给出 |
| 电池容量 | **5400 mAh（P） / 5300 mAh（R）** | 两份资料不一致，分别保留；以交付电池铭牌为准，不合并成一个确定值 |
| 轮编码器 | 教程明确写 **没有编码器** | F建图实验；程序中存在encoder回调名称不证明本车装有编码器 |
| 里程计来源 | RF2O激光里程计，或Cartographer激光+IMU链路 | F；属于软件估计，不是轮速反馈，精度/漂移需要实测 |

供电表只归档厂商标称，不由3S、电调输入和USB-C三组数字推导出可直接互连的方案。
整车重量、载重、轮胎尺寸、减速比、轮距、离地间隙、实际轴距、最小转弯半径、制动距离和续航未取得可靠实测值。
2026-09-17复核：产品外形388 × 221 × 290 mm；源码轴距参考305 mm；导航足迹另有640 × 360及400 × 250 mm，不能混作物理尺寸。详见[车辆尺寸与几何来源](docs/车辆尺寸与几何来源.md)。
另有静止箱面单点参照：后轴到箱面1米，雷达中位读数0.912米，推算前置偏移约8.8厘米（含测量误差、待复核，未写入控制外参）。

### 转向舵机

| 项目 | 标称值 | 来源与备注 |
|---|---|---|
| 品牌/完整型号 | **未提供** | P、R仅列规格；不能凭扭矩/外观猜型号 |
| 工作电压 | **4.8～6.0 V** | P、R |
| 空载转动时间 | **0.16～0.18 s / 60°** | P、R；不代表安装负载后的转向响应 |
| 堵转扭矩 | **17.25～20.32 kg·cm** | P、R原单位；不是连续工作扭矩 |
| 死区 | **2 μs** | P、R |
| 输入脉宽 | **500～2500 μs** | P、R；不是本车已验证安全转向范围 |
| 标称角度 | **180°** | P、R；是舵机参数，不是车轮转角范围 |
| 转向零点/频率 | 机械零点、PWM重复频率、方向及连杆映射 **待测** | 中性命令1500 μs见下文协议，但不能据此认定车轮正直 |

### 传感器型号与性能

| 传感器/项目 | 参数或型号 | 来源与适用范围 |
|---|---|---|
| 激光雷达型号线索 | **LSlidar N10** | F：`lsn10_launch.py`、`lsn10.yaml`明确选择`lidar_name: N10`；本车铭牌待核对，不能混用N10_P/M10协议 |
| 雷达测距原理 | **TOF** | P、R |
| 激光波长 | **905 nm** | P、R |
| 测距范围 | **0.02～12 m** | P；R注明条件为 **70%反射率**，不保证所有目标/光照下达到此距离 |
| 测距精度 | **±3 cm（0～6 m）**；**±4.5 cm（6～12 m）** | 前一项P、R，后一项R；标称精度不是定位或停车精度 |
| 扫描频率 | **6～12 Hz，可调** | P、R；实际输出频率、丢包和时钟需测量 |
| 水平视场角 | **360°** | P；车体遮挡和无效距离会影响有效覆盖 |
| 水平角分辨率 | **0.48°～0.96°** | P；不是本工程360-bin模拟配置的实测分辨率 |
| 雷达接口 | 出厂选择 **串口**，230400 baud、8N1 | F；详见下表，不把YAML中备用网络字段当作实际网络连接 |
| 摄像头品牌/型号 | **未提供完整型号**；USB免驱广角相机 | P、R；`usb_cam`是驱动包名，不是相机型号；未确认传感器芯片料号 |
| 最大图像尺寸 | **1920 × 1080** | P、R；该尺寸下支持的帧率/像素格式未给出 |
| 有效像素 | **210万** | P、R原文 |
| 标称视角 | **150°广角** | P、R未注明水平/垂直/对角口径；畸变、内外参需标定 |
| 相机接入 | **USB免驱**，出厂示例使用`usb_cam` | P、R、F；实际USB身份、video节点和曝光设置未核验 |
| IMU/陀螺仪模块型号 | **未提供完整型号**；驱动为WIT兼容11字节协议 | P没有模块料号，F提供解析实现；不能由协议认定为CMP10A/JY901或某一具体型号 |
| 姿态角静态精度 | **0.2°** | P；动态精度未给出 |
| 航向角静态精度 | **0.5°** | P；不能推导长期航向无漂移 |
| 姿态角分辨率 | **0.0055°** | P |
| 陀螺仪量程 | **±2000°/s** | P；F也使用该量程换算 |
| 陀螺仪分辨率 | **0.061 (°/s)/LSB** | P |
| 加速度换算 | 驱动按 **±16g**、`raw/32768 × 16 × 9.8`换算 | F `imu_get/src/transform.hpp`；这是源码解码尺度，不是已实测量程/零偏 |
| IMU输出频率参考 | 发布节点配置 **100 Hz（10 ms周期）** | F `publisher_imu.cpp`；不是硬件刷新率、端到端延迟或新鲜样本率的实测值 |
| GPS/GNSS、磁力计及其他外设 | **未确认安装与具体型号** | 雷达`use_gps_ts: false`只表示不用该授时；不能由姿态模块/平台案例推导车上有GPS、TOF模块或云台 |

### 通信、设备名、坐标系与相机模式参考

下表均是 **F出厂源码/教程配置**，便于接车核对；不是固定设备身份，也不直接覆盖本工程配置。

| 模块 | 出厂参考 | 使用时要区分的细节 |
|---|---|---|
| 底盘串口 | `/dev/car`，**38400 baud、8N1** | 7字节`AA motor_low motor_high servo_low servo_high checksum 55`；两路uint16小端、脉宽单位μs |
| 底盘校验 | 中间4个PWM数据字节求和模256 | 不包括首`AA`和末`55`；只证明上位机发送格式，MCU反馈/watchdog待确认 |
| 普通/one控制入口 | `/car_cmd_vel` / `/cmd_vel`，`geometry_msgs/msg/Twist` | `motor=1500+linear.x×100`，转向系数分别1300/1200；这些经验系数不是物理速度/曲率标定 |
| 遥控入口 | `/teleop_cmd_vel`，Twist | `linear.x`直接作电机PWM，`servo=2500-angular.z×2000/180`；与上一行单位语义不同 |
| IMU串口与帧 | `/dev/imu`，**115200 baud**；`0x55`开头、11字节 | `0x51`加速度、`0x52`角速度、`0x53`欧拉角；有符号16位小端数据，详见[IMU协议](docs/厂商协议Rust适配.md) |
| IMU话题/frame | `/imu_data`→`imu_link`；`/IMU_data`→`IMU_link` | 名称大小写不同；两套发布并不表示有两个实体IMU |
| 雷达串口与帧 | `/dev/laser`，**230400 baud、8N1**；普通N10每包**58字节/16点槽** | 包头`A5 5A`、长度`3A`，按厂商源码逐字节加和；无效点仍保留槽位，详见[N10协议](docs/N10协议依据.md) |
| 雷达话题/frame | `/scan`→`laser_link`；`pubScan=true`、`pubPointCloud2=false` | `interface_selection=serial`、`use_gps_ts=false`；UDP/IP字段属于同一模板的备用网络配置 |
| 雷达距离过滤 | YAML设 **0.2～200 m**；Cartographer另设 **0.15～10 m** | 两者都是软件过滤范围，**不能把200m写成N10硬件量程** |
| 基础相机launch | `/dev/video0`、**320×240、yuyv**，frame=`camera` | `racecar/launch/camer.launch.py`；没有据此确认本车只能输出该分辨率 |
| 相机配置1 | `/dev/video0`、**640×480、10 FPS、mjpeg2rgb、mmap** | `racecar/config/camera/params_1.yaml`，frame=`camera` |
| 相机配置2 | `/dev/video2`、**640×480、15 FPS、mjpeg2rgb、mmap** | `params_2.yaml`，frame=`camera2`；示例文件存在不证明本车装有第二个相机 |
| 两份相机配置的控制选项 | 自动白平衡=true、自动曝光=true、自动对焦=false | 另留有白平衡4000/exposure100等示例值；自动模式下不能当成固定实测曝光或色温 |
| 视觉课程相机示例 | `/dev/video20`、**640×480、30 FPS、MJPG** | 配套文字教程参考；没有读取视频，这些设置不保证当前设备支持 |
| 里程计话题 | RF2O链路使用 `/odom_rf2o` | 坐标转换/时间同步与实际发布链必须核对，不套用旧车TF |

其他源码几何/控制默认值位于 `racecar/src/car_controller_new.cpp`：`L=0.305`（注释为前后轴距，米制参考）、
`lfw=0.1675`（注释为前轮到车体中心的距离）、`controller_freq=30`。
**这些是控制器默认值，不是本车轴距或传感器采样率的测量报告**，不能直接填入Rust标定表；
也不能用产品外形尺寸算出轴距、轮距或雷达/相机安装位姿。

### 到车后补录的参数

| 类别 | 需要补录 |
|---|---|
| 实物身份 | 整车批次、主板修订/SoC完整料号、MCU完整料号、雷达铭牌、相机USB VID/PID与芯片/镜头型号、IMU型号、电调/舵机/电池型号 |
| 机械与运动 | 轴距、前后轮距、轮径、车轮/车体完整足迹、安装原点、左右最大转角、零点、转向速率、PWM-速度/曲率表、制动/倒车语义 |
| 感知 | 相机支持格式/帧率/曝光及内外参，雷达实际频率/盲区/接收时序，IMU输出配置/单位/零偏/朝向，统一时钟与所有TF |
| 供电与运行 | 电池实际容量/倍率、各供电支路和转换板、额定负载/温升、eMMC料号/健康信息、系统/ROS/GLIBC/ORT版本、MCU超时与反馈 |

上述硬件补录仅归档资料，不将标称或示例数字写入比赛控制参数；当前仍不读取视频、不连接设备。

## 代码结构：模块在哪、负责什么

```text
XT-STCAR/
├── crates/
│   ├── vision/          视觉数学与模型契约库
│   ├── app/             xt-stcar：视觉 CLI、原生/参考推理后端
│   ├── robot-core/      任务/安全、扫描/定位/导航、厂商协议和标定表
│   ├── device-io/       有截止时间的串口配置与传输库
│   └── runner/          xt-stcar-robot：自主模拟、感知/规划线程、采集与回放
├── config/              显式配置；车辆相关示例均未经实车标定
├── examples/            合成 JSONL 回放输入
├── scripts/             模型工具、构建、ELF 检查、打包和上传
├── tests/               Python 模型/交付测试与跨实现对照工具
├── docs/                协议依据、使用说明、验证报告
├── 资料/                 来源索引、环境记录、已归档官方参考
├── Cargo.toml/lock      Rust workspace、依赖与精确版本锁
├── rust-toolchain.toml  Rust 1.97.1 工具链选择
├── AGENTS.md            开发约定的加载入口
├── 上传规范.md           main 直接开发、修改前同步与提交推送流程
├── Mac与车端命令手册.txt   按执行机器分类的命令、参数与使用步骤
└── agent.md             完整交接快照与长期约定
```

| 源码位置 | 职责与边界 |
|---|---|
| [`crates/vision/src/lib.rs`](crates/vision/src/lib.rs) | `ModelSpec`、RGB letterbox、NCHW FP32 张量、输出契约检查、阈值过滤、坐标还原；不访问设备或加载原生库 |
| [`crates/app/src/main.rs`](crates/app/src/main.rs) | `xt-stcar` 命令入口：`self-check`、`preprocess`、`replay`、`infer`，参数与输出文件保护 |
| [`crates/app/src/ort_backend.rs`](crates/app/src/ort_backend.rs) | `NativeOrtBackend`：动态加载 ORT C 库、模型 SHA/provenance/实际张量元数据验证、常驻 Session 和超时取消 |
| [`crates/app/src/backend.rs`](crates/app/src/backend.rs) | 显式 Python 参考后端、张量文件交换、子进程超时清理与原子输出工具 |
| [`crates/app/src/file_io.rs`](crates/app/src/file_io.rs) | 两套 CLI 共用的静态文件边界：非阻塞打开后检查普通文件、限制实际读取量、拒绝非普通输出目标 |
| [`crates/app/src/lib.rs`](crates/app/src/lib.rs) | 导出可复用的推理后端，供总调度调用 |
| [`crates/robot-core/src/sensors.rs`](crates/robot-core/src/sensors.rs) | IMU、雷达、里程计、视觉摘要及 frame/单位/时间/有限数值校验；里程计类型本身不计算里程计 |
| [`crates/robot-core/src/safety.rs`](crates/robot-core/src/safety.rs) | Disarmed/Armed/Running/Fault、急停锁存、deadman、心跳/指令/传感器超时、物理速度和曲率限值；`RecordingSink` 仅记录输出 |
| [`crates/robot-core/src/protocol/chassis.rs`](crates/robot-core/src/protocol/chassis.rs) | 厂商 7 字节底盘编码、三种不同话题语义的显式预览、通用短写/失败锁存适配器；经验系数不作为物理标定 |
| [`crates/robot-core/src/protocol/calibrated_chassis.rs`](crates/robot-core/src/protocol/calibrated_chassis.rs) | 速度→电机 PWM、曲率→舵机 PWM 的显式分段表；零锚点、限值、倒车策略校验，拒绝外推；当前只接受未验证模拟配置 |
| [`crates/robot-core/src/startup_assist.rs`](crates/robot-core/src/startup_assist.rs) | 根据新鲜扫描里程计确认静止、受限步进PWM、起步交接及故障锁存；不打开执行器 |
| [`crates/runner/src/startup_replay.rs`](crates/runner/src/startup_replay.rs) | 自适应起步离线回放与PWM建议记录，显式标记无物理输出 |
| [`crates/robot-core/src/protocol/imu.rs`](crates/robot-core/src/protocol/imu.rs) | WIT 11 字节增量解析、校验和、三分量组合、单位/四元数转换、坏帧与过期控制 |
| [`crates/robot-core/src/protocol/n10.rs`](crates/robot-core/src/protocol/n10.rs) | 按厂商源码解析 N10 58 字节/16 点包、分包/粘包重同步、角度/距离/强度、保留无效点槽位；只生成局部扫描样本 |
| [`crates/robot-core/src/lib.rs`](crates/robot-core/src/lib.rs)、[`protocol/mod.rs`](crates/robot-core/src/protocol/mod.rs) | 核心类型与协议模块的公共导出入口 |
| [`crates/device-io/src/lib.rs`](crates/device-io/src/lib.rs) | 安全 Rust 串口库：显式设备、8N1 配置读回、关闭软硬件流控、独占、非阻塞 poll、整包截止时间、错误锁存和退出恢复 |
| [`crates/runner/src/main.rs`](crates/runner/src/main.rs) | `xt-stcar-robot` 命令解析、默认采集计划、输入文件保护、完整日志原子提交 |
| [`crates/runner/src/input.rs`](crates/runner/src/input.rs) | 非阻塞普通文件校验、配置/事件实际字节限制、严格 JSONL 与单调时序、图像路径/序号检查，解析 `imu_bytes` / `n10_bytes` |
| [`crates/runner/src/vision.rs`](crates/runner/src/vision.rs) | 图像读取、预处理与复用同一 ORT Session；将检测结果交给调度 |
| [`crates/runner/src/lib.rs`](crates/runner/src/lib.rs) | 串联输入→解码/推理→安全控制器→可选 PWM 预览→JSONL；EOF Stop、推理/映射失败急停；完整解码批次每块只记一次 |
| [`crates/runner/src/capture.rs`](crates/runner/src/capture.rs) | 有时间/字节/记录上限的 IMU 或 N10 采集，使用统一 `Instant` 时钟，空闲和结束写 Tick；不发送设备命令 |
| [`crates/vision/src/road.rs`](crates/vision/src/road.rs) | 白条几何、HSV 灯色、红蓝锥桶及相机地面投影；普通 YOLO `zebra` 不用于斑马线 |
| [`crates/robot-core/src/autonomy.rs`](crates/robot-core/src/autonomy.rs) | 米制坐标、位姿质量、车体足迹、道路观察及有向禁越线 HalfPlane 共享类型 |
| [`crates/robot-core/src/field.rs`](crates/robot-core/src/field.rs) | 米制 `FieldSpec`、规则尺寸与组合校验、固定纸条/锥桶/发车终点几何、生成绕桶顺序及直道入口朝向；不缩放车体或控制限值 |
| [`crates/runner/src/field.rs`](crates/runner/src/field.rs) | 赛前编译 `FieldScenario` 为冻结的任务配置，校验派生坐标一致性，限制合成灯的可见区/触发区；不把模拟灯时钟交给任务 |
| [`crates/runner/examples/field_matrix.rs`](crates/runner/examples/field_matrix.rs) | 固定九组场地/元素布局，分别运行同步和真实 worker 的 PP/LQR 闭环；保留失败，不代表连续尺寸范围保证 |
| [`config/field-example.json`](config/field-example.json) | 合成场地规格模板；填入现场测量值后先生成布局、再编译验证 |
| [`crates/robot-core/src/local_world.rs`](crates/robot-core/src/local_world.rs) | 有界元素身份、时效/误差、视觉雷达锥桶关联，局部空闲/障碍/未知及完整通道覆盖检查 |
| [`crates/robot-core/src/online_mission.rs`](crates/robot-core/src/online_mission.rs) | 固定任务顺序、搜索、当前桶身份与绕行进度、观测区域停车，生成动态局部目标 |
| [`crates/vision/src/ground_markers.rs`](crates/vision/src/ground_markers.rs) | 默认禁用的实验白条协议、地面米制方向/区域几何；不认定真实赛事存在同样标记 |
| [`crates/runner/src/online.rs`](crates/runner/src/online.rs) | 同刻元素/扫描/位姿接入，局部任务与原导航连接，完整停车包络的已知空闲检查 |
| [`crates/runner/src/online_simulation.rs`](crates/runner/src/online_simulation.rs) | 隔离场景真值，生成统一控制器配置；仅模拟器渲染与独立裁判使用真实元素位置 |
| [`crates/runner/examples/online_matrix.rs`](crates/runner/examples/online_matrix.rs) | 预先声明的场地/元素移动、遮挡和缺失场景，同步/异步 PP；保留未完成与故障 |
| [`crates/robot-core/src/mission.rs`](crates/robot-core/src/mission.rs) | 斑马线实际停止计时、锥桶顺序、可选通过点朝向及 Stop/PassThrough 到达策略、灯前全车停车/绿灯确认、终点与故障锁存 |
| [`crates/robot-core/src/scan.rs`](crates/robot-core/src/scan.rs) | N10 整圈组帧、覆盖/盲区/时间检查；不同于旧局部包回放 |
| [`crates/robot-core/src/localization.rs`](crates/robot-core/src/localization.rs) | 有界 ICP 激光里程计；退化/跳变/过期门控，不假造编码器 |
| [`crates/robot-core/src/navigation.rs`](crates/robot-core/src/navigation.rs) | 路径搜索、跟踪、障碍/制动和停车朝向；将异步采用约束传入候选筛选，分别记录普通搜索和恢复；`SteeringEstimate` 区分命令目标与渐变执行估计，规划与 adopt 分离 |
| [`crates/robot-core/src/admission.rs`](crates/robot-core/src/admission.rs) | 共用源车体系停车矩形、当前采用区间和有限多拍前瞻；保留真实历史上界及实际命令变化时基，按未来采集相位预测正常减速后备动作 |
| [`crates/robot-core/src/admission/slew.rs`](crates/robot-core/src/admission/slew.rs) | `command_slew_allowance` 统一规划/前置检查/证书/输出的曲率时基；区分无变化历史的启动与已有修订，后备预测另跟踪模型命令变化时刻 |
| [`crates/robot-core/src/navigation/recovery.rs`](crates/robot-core/src/navigation/recovery.rs) | 前向搜索失败分层、首个被拒绝运动基元和普通/恢复工作量；连续几何恢复保留完整车体、间隙和障碍半径，避免把占用格近似当成物理不可达 |
| [`crates/robot-core/src/navigation/primitive_envelope.rs`](crates/robot-core/src/navigation/primitive_envelope.rs) | 按速度/转向饱和点分段积分，给出位置与航向数值误差界；沿运动基元、搜索节点和终端连接传递误差，供连续扫掠核验使用 |
| [`crates/robot-core/src/motion_transition.rs`](crates/robot-core/src/motion_transition.rs) | `MotionTransition` 解析检查过渡侧向峰值，`project_motion` 按渐变模型投影位置/速度/转向；计算有界、无堆分配，不是测量反馈 |
| [`crates/robot-core/src/reference.rs`](crates/robot-core/src/reference.rs) | 每拍验证一次的局部参考窗口；投影、弧长游标、整段评分参考及候选偏差诊断，无额外路径分配 |
| [`crates/robot-core/src/tracking.rs`](crates/robot-core/src/tracking.rs) | PathTracker 接口、默认 Pure Pursuit、实验性曲率前馈 + LQR；只给导航期望曲率 |
| [`crates/robot-core/examples/tracking_comparison.rs`](crates/robot-core/examples/tracking_comparison.rs) | 直线/弯道的 Rust 跟踪 A/B 实验，输出误差与合成转向响应指标 |
| [`crates/runner/src/laser_pose.rs`](crates/runner/src/laser_pose.rs) | 整圈检查、雷达到车体外参与 ICP 桥接，保留源时刻 |
| [`crates/runner/src/perception.rs`](crates/runner/src/perception.rs) | 同图 RGB + 常驻原生 YOLO + RoadDetector；new_online 输出最多16项带几何元素，旧入口不变；后台只留最新待处理帧 |
| [`crates/runner/src/autonomy.rs`](crates/runner/src/autonomy.rs) | 传感器 → 任务 → 导航 → Safety；同步 tick 采用最终命令；异步 tick_with_projection 分开使用源测量和模型规划状态，不采用后台计划 |
| [`crates/runner/src/control_runtime.rs`](crates/runner/src/control_runtime.rs) | 后台规划、最新快照队列、独立 poll 和源时间看门狗；仅最终 ControlPoll.command 推进执行历史；检查采用窗口、历史前提和原源期限 |
| [`crates/runner/src/control_diagnostics.rs`](crates/runner/src/control_diagnostics.rs) | 显式开启的排队/导航/终端/证书耗时、共享最新发布报告和有界计数；默认不读取主机时钟，离线调度钩子不进入输出线程 |
| [`crates/runner/src/async_simulation.rs`](crates/runner/src/async_simulation.rs) | 完整 RGB/雷达异步比赛，独立渐变车辆只执行 poll 最终命令；`simulate_async_observed` 提供显式离线只读回调，覆盖正常输出和最终制动 |
| [`crates/runner/examples/async_timing_matrix.rs`](crates/runner/examples/async_timing_matrix.rs) | 固定 2×2 时序、PP/LQR 共 8 场；在有界内存比较最早实际输出、同源计划和路线差异，另计实际 Stop 与拒绝期间保持 Drive |
| [`crates/runner/examples/support/performance_gate.rs`](crates/runner/examples/support/performance_gate.rs) | 八场结束后检查预先固定的整场时间/距离、阶段时长与实际 Stop 预算；只影响离线测试退出状态，不参与车辆控制 |
| [`crates/runner/src/host_clock_simulation.rs`](crates/runner/src/host_clock_simulation.rs) | 持续 Instant 时钟的短转弯/断流观察；计算期间输出时间继续推进，分开记录主机负载和功能结果 |
| [`crates/runner/examples/async_motion_comparison.rs`](crates/runner/examples/async_motion_comparison.rs)、[`async_host_clock.rs`](crates/runner/examples/async_host_clock.rs) | 前者执行完整异步 PP/LQR 与可选时序配置，后者显式采集一次实际主机时钟观察；失败报告仍输出并返回非零 |
| [`crates/runner/src/control_execution.rs`](crates/runner/src/control_execution.rs) | 固定容量已采用命令历史、源时刻查询和运动投影；最终独立检查整段采用窗口及原期限停车空间，返回证书或结构化首个失败 |
| [`crates/runner/src/control_execution/diagnostics.rs`](crates/runner/src/control_execution/diagnostics.rs) | 证书失败原因、障碍/角点索引、有单位的符号余量、源时刻/期限/采用窗和历史 V/K；无逐拍文件输出 |
| [`crates/runner/src/control_admission.rs`](crates/runner/src/control_admission.rs) | 每份源快照准备一次共享采用上下文，验证历史、时间及有限数值；准备失败明确拒绝，不能解释为关闭前置约束 |
| [`crates/runner/src/simulation.rs`](crates/runner/src/simulation.rs) | RGB/雷达/位姿合成闭环；车辆速度/曲率连续渐变，以不超过 1 ms 的子步积分并检查车体碰撞/边界，含故障后制动 |
| [`crates/runner/examples/motion_comparison.rs`](crates/runner/examples/motion_comparison.rs) | PP/LQR 进入同一完整模拟比赛，报告成功与失败结果 |
| [`crates/runner/examples/motion_profile.rs`](crates/runner/examples/motion_profile.rs) | 显式预热与重复测量完整模拟墙钟时间；记录平均/最大及完成状态，不能当作板卡时限 |
| [`crates/robot-core/src/navigation/terminal.rs`](crates/robot-core/src/navigation/terminal.rs) | 单/双段曲率渐变末端连接、接续数值初值、共享计算额度与求解失败诊断 |
| [`crates/robot-core/src/navigation/continuity.rs`](crates/robot-core/src/navigation/continuity.rs) | 记录末端初值的预测状态，延迟后调整初猜并重新认证；保留已恢复短连接族，计算同姿态旧剩余路线诊断；fixtures 内含原始窗口及专项实验快照 |
| [`crates/runner/src/autonomy_replay.rs`](crates/runner/src/autonomy_replay.rs) | 同步传感器快照回放，不接受人工 Motion；结束明确 Stop |
| [`crates/runner/src/navigation_diagnostics.rs`](crates/runner/src/navigation_diagnostics.rs) | 模拟首个非预期阻塞/故障的最多 17 帧内存上下文，结束随摘要输出 |
| [`crates/runner/src/phase_statistics.rs`](crates/runner/src/phase_statistics.rs) | 各阶段实际时长/距离、Stop 次数/命令时长、路径生成、拒绝原因及终端工作总数/每拍最大值的有界内存统计；最终制动单列 |
| [`crates/runner/src/telemetry.rs`](crates/runner/src/telemetry.rs) | 有界内存事件日志；调试预算耗尽不影响比赛控制 |

原有诊断流为：文件回放 → 传感器/视觉模块 → 安全状态机 → 可选 PWM 映射校验 → 运动记录/预览日志。
在线自主流为：RGB/雷达/位姿 → 元素关联与局部空间 → 任务顺序和动态局部目标 → 原规划/跟踪及安全检查 → 车辆模型 → 下一帧反馈。
独立串口采集先生成可回放的原始字节文件；采集没有 Arm/Start/Motion 事件。
目前没有把采集与电机发送接成实时闭环。

## 运动控制

已对照[用户分享的算法建议](https://chatgpt.com/share/6aa0bf59-c8f8-83ee-b001-13dd5945ce14)审查代码。
继续核对[修改后的意见](https://chatgpt.com/share/6aa27143-7204-83ee-ad9a-7067ef49a1bc)，
前轮说明见[导航预测与失败首因修正](docs/导航预测与失败首因修正.md)，
前轮执行状态、过渡峰值、通过点和模拟积分修正见[运动执行与通过点修正](docs/运动执行与通过点修正.md)。
前轮真实任务验收半径、完整车体参考偏离、阶段统计与异步时间对齐见[任务交接与异步运动修正](docs/任务交接与异步运动修正.md)。
前轮短连接接续、共享计算额度和异步停车空间见[终端连接延续与异步停车包络](docs/终端连接延续与异步停车包络.md)。
前轮 v9 已完整读取[第九份修改意见](https://chatgpt.com/share/6aa89fa3-3894-83e8-afe8-45451cbbc42f)，
结合 `XT-STCAR_a623638_review.zip`，复现并修正了共享时基修复之后的灯前长绕路。
说明见[路径连续性与性能回归](docs/路径连续性与性能回归.md)，来源与原始失效见[基线证据](docs/motion-v9-before.json)，
三版本消融见[消融记录](docs/motion-v9-ablation.json)。外部ZIP仅做Python数据复核，Rust运行由本工程另行完成。
前轮[采用时基与时序交叉验证](docs/采用时基与时序交叉验证.md)、[采用约束与前向恢复](docs/采用约束与前向恢复.md)
及 `motion-v8-*`、`motion-v7-*` 和更早报告保留历史。`field-adaptation-*` 记录赛前固定布局验收；本轮在线模式以 `online-mission-*` 为准。
当前采用前进车辆运动学：比赛任务给目标/限速 → A* 连通与带航向/曲率的车模型路线 → PathTracker →
候选轨迹预测、采用约束、碰撞和制动检查 → 安全状态机 → 异步最终采用证书 → 输出线程。
同步调用保留其对应检查；默认 **Pure Pursuit** 保留，**曲率前馈 + LQR** 继续作为实验选项。
不是单个 PID 直接把图像误差变成舵机 PWM，也没有实现 MPC 求解器。

`tracking.rs` 只输出期望曲率；加减速、横向加速度、曲率及其变化率、完整车体和停车可达域仍由 `navigation.rs` 检查。
正常候选从测量速度和已采用命令对应的执行曲率估计渐变到候选值；0.30→0.24 m/s 与保持 0.30 m/s 的预测可以不同。
`MotionTransition` 解析检查整个速度/曲率过渡的侧向加速度峰值，覆盖端点、分段切换及内部极值；
不能只检查目标 `v²|曲率|`。近目标缩短几何预测也不能缩短这项检查的完整命令周期。
紧急停车圆盘仍按两者较大的速度计算，不会被较低目标速度或较短终点距离缩小。
路径末端连接检查实际积分的曲率渐变轨迹；短距离同朝向横移可尝试双段 S 形连接，失败继续有界搜索。
两种跟踪器均走同一安全通路。命令单位为线速度 m/s、曲率 m⁻¹，模型 `yaw_rate = speed × curvature`；
转向正方向为左转。静态 PWM 映射在 `protocol/calibrated_chassis.rs`，不把厂商 Twist 的经验系数当成物理单位。

LQR 同时使用相对真实参考路径的横向和航向误差，以相邻路径几何作曲率前馈。
这是连续时间、固定正速度的直线附近误差模型，低速回退 PP，异常输入/超出误差域请求 Stop；
它没有证明在弯道、执行器延迟和所有离散周期下优于 PP。旧 JSON 省略 `navigation.tracking` 时仍选 PP。
选择方法、模型假设和逐项评估见 [运动控制设计与对比](docs/运动控制设计与对比.md)。

v7 将实际采用条件提前传入导航。`control_admission.rs` 从同一源测量、已采用历史和原 lease 准备上下文，
`admission.rs` 在候选筛选时检查速度区间、曲率变化率、过渡侧向峰值和停车矩形。

v8 将 `planned_at`、`last_command_change_at`、`adopted_revision` 与持有目标从同一份执行历史传入共享约束。
曲率候选按最早采用时刻（规划窗起点）到上次真实命令变化的间隔限幅；前置检查与最终证书使用同一定义，
输出端仍按真正采用时刻和执行修订复核。同一命令重复输出不刷新时基，仅速度改变仍算命令变化。
Stop 的目标曲率为零，模型继续渐变回中；恢复使用这次 Stop 的真实采用时刻。只有修订为零且确实没有命令变化的
启动状态才使用原一周期起步限幅；缺失历史、未来时刻或矛盾修订明确拒绝。不会在 poll 内无检查裁剪已认证动作。

候选检查调整为：轻量输入/当前采用检查 → 原运动/停车/Grid/足迹 rollout → 有限多源前瞻 → 终端认证与评分。
已被 rollout 拒绝的候选不再运行后备投影，存活候选仍须通过全部门限；rollout 只积分一次，参考误差随积分计算。
短路顺序可能改变诊断首因和计数，不能将计数差异当成控制退化，预扣积分区间也不能直接换算成 CPU 节省。
当前源的历史速度/曲率上界保持不变：当历史 V=0.30 m/s、K=2 m⁻¹ 已达配置上限时，
把新候选改为 0.24 m/s 不会缩小这一帧的停车空间。需要在到达该状态之前选出合适动作，不能靠删历史或放宽相切判定通过。

前瞻分别取采用窗最早、中间、最晚三个时刻，保留下一次源采集相对于规划时刻的相位，
随后按正常减速能力预测一个有界后备停车过程。每个未来源分别累计其采集到规划之间经过的预测命令，
旧命令仍在该窗口内就保留其上界；已有源的真实历史不会因此被改写。
每个样本最多64个未来规划拍、2秒模型时域，固定65个源记录槽；预算不足返回 `ForecastBudget`，不截断后冒充完整预测。
这是有限模型样本的候选约束，未来采样周期按导航周期、未来处理延迟按当前源年龄建模；
不证明任意采用时刻、未来障碍/延迟或后续路线都可行。最终 `control_execution::certify` 仍独立覆盖实际采用窗，
`poll` 继续检查原期限和执行修订号，车辆只能使用最终 `ControlPoll.command`。

停车后的恢复另行处理。普通占用网格可能因为车体膨胀与格角近似，把有连续几何余量的前向基元全部拒绝。
连续域另预留最小正常起步候选的制动空间，避免进入停稳后无法正常起步的贴障位置。
恢复检查仍保留完整车体外接圆、配置间隙、障碍圆盘及地图/灯前边界，使用连续扫掠包络区分栅格近似和真正的几何拒绝。
`primitive_envelope.rs` 为速度/曲率渐变积分携带位置和航向误差界，路径接续累积误差，不能把数值末点当成绝对精确。
普通搜索与恢复共享原节点和终端求解额度；没有引入倒车、原地旋转或扩大比赛容差。
原停稳快照及新障碍场景单独回归，整场是否避免再次进入停滞仍须看完整异步结果。

证书失败现在返回结构化首因，区分源/期限错误、正常运动限幅、地图、激光点、视觉锥桶和灯前边界。
`CertificateFailure` 含从零开始的约束索引、符号余量及单位、源/规划时刻、lease/采用窗、V/K 和停车矩形；
无定义或非有限余量留空，不把包络重叠当作真实车体碰撞证据。
导航的 `admission_current`、`admission_next`、`forward_search` 和 `admission_forecast_work` 分别说明候选拒绝、
搜索分支和投影工作量；候选拒绝次数、证书拒绝次数与实际 Stop 次数分别统计。
这些记录保存在内存和有界摘要中，默认不逐 tick 写 eMMC，也不因诊断额度降低比赛频率或改变输出命令。
新增诊断在 JSON 中省略全默认对象和 None；缺失表示零工作/未搜索/无约束或首因，有值记录完整保留。
`route_rebuild_reason` 记录导致该次路线搜索的首个缓存失效原因，包括初次建路、任务停车、到达策略/边界/目标变化、
车体参考偏离、跟踪拒绝和无候选；搜索失败期间保留该原因，缓存复用时为空。它是重建触发原因，不是整场失败的唯一因果证明。

| 输入延迟 ms | 采用偏移 ms | PP 时间 / 路程 | 实验 LQR 时间 / 路程 |
| --- | --- | --- | --- |
| [60, 80] | [3, 7, 9] | 75.467 秒 / 14.049016 米 | 54.663 秒 / 10.330780 米 |
| [60, 80] | [5, 9, 11] | 54.985 秒 / 10.749804 米 | 55.265 秒 / 10.310891 米 |
| [40, 60] | [3, 7, 9] | 56.443 秒 / 11.026392 米 | 54.249 秒 / 10.291006 米 |
| [40, 60] | [5, 9, 11] | 54.449 秒 / 10.679627 米 | 53.445 秒 / 10.221895 米 |

最终 **8/8 完成且全部通过试改前固定的性能门**，曲率变化率采用拒绝、最终证书拒绝和终端总预算耗尽均为0，
全部保留3000 ms斑马线停稳、300 ms绿灯确认、整车终点及终态速度/曲率归零。
早输入 `[40,60]` /早采用 `[3,7,9]` 的LQR从v8的88.647秒/16.515558米恢复到54.249秒/10.291006米；
灯前阶段59.400→24.994秒，实际Stop命令仍4513ms。其他三组LQR通过同一门，PP四组时间/路程逐值保持v8。
同步PP58.800秒/11.508841米、LQR54.300秒/10.249675米保持。
结果说明这些固定工况的退化已修复，不能证明任意时序、传感误差或实车都收敛；PP默认工况仍有原恢复活动。
完整首次差异、实际Stop和重建原因见[最终矩阵](docs/motion-v9-timing-matrix.json)，范围与测试见[验证汇总](docs/motion-v9-validation.json)。

历史 v8 修复之前的隔离基线 `7d28761` 的 8 场中 7 场完成，新增交叉工况暴露 1 场 LQR 停滞失败：

| 输入延迟 ms | 采用偏移 ms | 基线 PP | 基线实验 LQR |
| --- | --- | --- | --- |
| [60,80] | [5,9,11] | 完成，114.591 秒 / 21.435316 米 | 91.669 秒 / 14.627098 米，普通停车超过 10 秒失败 |
| [40,60] | [3,7,9] | 完成，87.043 秒 / 16.472032 米 | 完成，54.447 秒 / 10.317616 米 |

失败场首个采用拒绝发生时仍保持 Drive，不能把该拒绝直接当成最后停滞的唯一原因；最终速度/曲率均为零。
完整[基线矩阵](docs/motion-v8-baseline-matrix.json)保留真实失败、Stop、路线和恢复记录，全部观察轨迹均未截断。
原默认/扰动四场逐值复现 v7 报告（比较时只剔除主机墙钟字段）；移植离线观察器后八场功能摘要也逐值保持。

历史 v7 的原四次完整异步运行均完成，原碰撞、边界和 10 秒普通停滞门限保持：

| 场景 | PP | 实验 LQR |
| --- | --- | --- |
| 同步完整比赛 | 58.800 秒 / 11.508841 米 | 54.300 秒 / 10.249675 米 |
| 异步默认时序 | 73.863 秒 / 14.173789 米 | 89.069 秒 / 15.807183 米 |
| 异步扰动时序 | 56.849 秒 / 11.124416 米 | 53.849 秒 / 10.217582 米 |

同步结果与 v6 保持一致；异步默认时序的 PP 比 v6 慢 16.2 秒、路程更长，不能称为全面性能提升。
两组异步的最终证书拒绝数均为 0，但输出端曲率变化率拒绝仍分别为每种跟踪器 5 次、2 次，
这些拒绝计数不是实际 Stop 次数。二者均完成 3000 ms 斑马线停稳、300 ms 绿灯确认、整车进入终点且终态速度/曲率为零。
两份独立停稳 fixture 也恢复为 Drive，总分配节点 265 / 12；局部恢复与整场结果分别记录。
详见[历史默认异步](docs/motion-v7-async-comparison.json)、[历史扰动异步](docs/motion-v7-async-perturbed.json)和[历史同步对比](docs/motion-v7-competition-comparison.json)。

新增 `async_timing_matrix` 固定输入延迟 `[60,80]` / `[40,60]` ms 与采用偏移 `[3,7,9]` / `[5,9,11]` ms 的全部组合，
各跑 PP/LQR，其他场景与 Q/R 不变。实际输出按模型时间对齐；同源规划动作、路径坐标、点数、路线修订和恢复入口分别比较。
首次精确路径点差异不代表路线拓扑已经改变；`traces_complete=true` 时，空差异才表示共同观察区间未见差异。
每场只在内存保留最多 16384 次命令变化、2048 份计划、每计划 2048 个路径点；超额显式标记，结束才统一写报告。
实际 Stop 命令时长包含启动与最终制动，不能当成物理静止时长；拒绝时仍保持旧 Drive 的次数单列。
这些额外轨迹仅在专用离线例程启用，不影响默认比赛日志、感知/控制频率或 eMMC 写入策略。

```bash
# Mac：固定 8 场，stdout 保存完整 JSON，stderr 保存进度与失败
mkdir -p work
cargo run --release --locked --offline -p xt-stcar-robot-runner --example async_timing_matrix > work/async-timing-matrix.json 2> work/async-timing-matrix.stderr.log
```

任何场景失败、轨迹截断或性能超限都会在保留完整 JSON 后非零退出，须同时检查 `all_completed`、`traces_complete`、`performance_gate.all_passed` 和各场 `fault`。
性能预算在历史 v9 修复试验前固定于 `crates/runner/examples/fixtures/motion-v9-performance-budget.json`：
各组以v7/v8相同时序中较快的已完成场为参照，整场时间/距离上限110%，每阶段加 `max(1000ms,10%)`，实际Stop加1000ms。
这些门仅检查模拟结果，不让控制器为达到时间目标放宽安全；具体口径见[路径连续性与性能回归](docs/路径连续性与性能回归.md)。
等待后台计算时功能钟冻结，观察回调耗时进入主机墙钟；矩阵用于行为对比，不代替持续主机时钟或目标板截止时间测试。

```bash
# Mac：依次为跟踪层实验、完整比赛对比；均无设备读写
cargo run --release --locked --offline -p xt-stcar-robot-core --example tracking_comparison
cargo run --release --locked --offline -p xt-stcar-robot-runner --example motion_comparison
# Mac：每种跟踪器预热一次，再测3次完整simulate墙钟；不等于目标板实时性能
cargo run --release --locked --offline -p xt-stcar-robot-runner --example motion_profile -- 3
```

历史 v6 同步合成比赛 **PP 58.8 秒、LQR 54.3 秒均完成，终态速度均为零**；路程分别为11.508841米、10.249675米，
最小锥桶间隙分别约0.2425米、0.2735米；斑马线停稳均3000ms、绿灯确认均300ms，见[同步完整比赛对比](docs/motion-v6-competition-comparison.json)。
v6 与 `b9f75c7` 的同步结果逐值一致。v5 相对 `f83898a`，PP指标不变；LQR快16.3秒、少走3.190640米。灯前阶段从42.4秒/7.574950米变为26.2秒/4.390629米，
该阶段路径生成从8次降至5次；LQR无非预期Stop，PP原有两次短Stop仍保留。
初次扩大恢复候选后仍按旧评分选择，LQR曾退化到79.7秒/15.057722米，此方案已撤销；
中间实验与最终选择见[实验记录](docs/motion-v5-terminal-experiments.json)。v4/v3及更早报告保留历史。
有朝向目标沿预测整段弧长累计车体位姿误差：比较预测姿态与参考姿态对应的四个车身角在世界坐标中的距离，取最大值，
统一使用米，避免提前回正却留下横向偏差。这改变了位置与朝向在共用评分中的相对影响；
保持不变的是 LQR 的 q/R 参数和比赛门限，不能称为所有控制权重都没变。
缓存路径即使无碰撞也可能已无法从当前姿态到达末端；带朝向目标现在按对应车体四角的最大偏移检查，
同时纳入位置和航向，超过原目标容差一半时提前重新规划。
对于必须停车的定向目标，若已有有界单弧求解器未找到连接而双弧找到连接，候选额外验证完整下一周期后
仍能接上现有短连接族或严格进入目标范围，防止恒曲率预览逐拍耗尽 S 形末端可达性。
已被现有单弧求解器认证的接近保留原控制；该约束不代替任务停稳，不对每个候选重跑全局搜索，也不证明所有初态可达。
`navigation/terminal.rs` 保留已认证双段连接的数值初值和剩余段比；`navigation/continuity.rs` 还记录选中候选预测的下一状态。
v9定位到候选为100ms后的状态求解，而下一拍实际在120ms到来：原冷启动和未推进的旧初值都用尽单次迭代，导致短连接丢失。
当原方法均失败且仍有共享额度时，以相对预测状态的正向位移推进初猜，再从当前姿态、曲率和Grid完整重求解。
这个位移只调整初猜，不是里程计读数或安全凭证；提前到来的拍不反向虚构行程。
只有这样恢复成功的连接族，后续才优先重验接续；候选通过全部原硬检查后，优先选认证接续、最接近已认证首段曲率者，再按原评分比较。
因此恢复族内候选优先级有意改变，普通成功路径仍保留原选择；原冷候选全失败后的恢复轮也继续保留。
目标/朝向、到达策略、边界或任务停驻变化会清理提示，每次检查新障碍，不直接执行旧缓存轨迹。
`route_length_change` 仅在同目标、原缓存可用且车体参考偏离重建成功时记录同一当前姿态的旧剩余/新路线米数；
缺失表示不具备这一比较条件，不能据此断言没有绕路。诊断不参与控制。
每次导航共享最多256次域内求解、1024次迭代和65536份预扣采样额度；单个求解仍最多8次迭代，lattice节点另有原上限。
`terminal_work` 区分全局额度耗尽、单次迭代用尽、域拒绝和网格拒绝；`primitive_samples` 为预扣额度而非实际CPU操作数。
额度用尽不等于路径物理不可达，已有完整认证的候选可继续使用。
v6 已在原总额度内为接续保留一次完整求解机会；普通候选触及暂时上限时记为延后，不能提前锁死全局预算。
恢复前释放预留，未激活v9接续保护的普通已认证最佳仍优先。原42.2秒场景在21/41/61/81档及左右镜像均Drive；
21档仍预扣51541份样本，41档以上为63799，未提高65536上限。新障碍仍能否决缓存，前置搜索费用仍计入总账。
细节见[修复前](docs/motion-v6-budget-before.json)与[修复后](docs/motion-v6-budget-after.json)。
普通中间点保留追踪点评分，临近目标时减弱固定曲率偏好。实现、尺度与边界见新修正说明。
保持单个候选曲率目标的预览不能代表所有未来转向策略；单场完成不证明一般收敛性或所有场景都可达。
原有跟踪层16组和前轮“PP 55.3秒完成/LQR 27.9秒超时”保留为历史；两类基准周期不同，
单场合成结果不能证明算法普遍优劣，模拟秒数不代表板卡计算耗时。
前轮同机release每种跟踪器预热1次、顺序测3次：整场simulate平均墙钟PP约1.621→1.614秒，LQR约1.921→1.496秒。
PP差异很小；LQR较短任务过程减少了总计算时间，不能推出每个导航调用更快。原始样本与范围见[主机耗时](docs/motion-v5-host-profile.json)。
前轮异步证书单次360/2048点平均约8.9/19.2微秒；采样峰值与真实Worker结果见[异步验证](docs/motion-v5-async-validation.json)，均非目标板WCET。
历史 v6 同一基线与最终代码顺序预热/各测3次，整场同步平均墙钟PP约1.606→1.612秒（+0.37%），
LQR约1.481→1.481秒（−0.02%），未见明显总耗时变化；[原始样本](docs/motion-v6-host-profile.json)保留全部测量。
v6 的[完整异步分段计时](docs/motion-v6-async-timing.json)明确排队、导航、终端、证书、发布和采用的范围。
v6 的[持续主机时钟观察](docs/motion-v6-host-clock.json)中23份源输入、119次转弯Drive输出，最大poll间隔21ms；
末源2200ms，原期限2450ms，在下一次2461ms轮询Stop，2687ms完成模型刹停/回正。该采样不证明目标WCET。
v8顺序主机profile各预热1次/测3次：同步PP平均1619.924→1632.190 ms（+0.76%），LQR1491.721→1513.593 ms（+1.47%），
模拟时长和路程保持；小样本及外部负载未控制，不能据此下普遍性能结论，见[主机profile](docs/motion-v8-host-profile.json)。
历史v8默认异步导航阶段平均/实测最大：PP6.395/59.369 ms，LQR4.573/13.166 ms；PP该轮进入恢复，与v7路径和工作量不同，
单个Grid阻断帧省去前瞻不等于整场耗时下降。计时开启前后功能字段逐值一致；包含非Drive计划且功能时钟冻结，
不能据峰值或平均推导板卡时限，见[分段计时](docs/motion-v8-async-timing.json)。
[持续主机时钟短场](docs/motion-v8-host-clock.json)取得23源/119转弯Drive、最大poll间隔21 ms；末源2200、原期限2450、
2460 ms输出Stop，2687 ms完全停稳回中，无碰撞/越界。这仍不是完整比赛实时性或目标WCET验收。

历史v9最终默认异步计时开启前后功能字段逐值一致，导航阶段平均/实测最大为PP6.216/59.705ms、LQR4.346/11.099ms，
见[v9分段计时](docs/motion-v9-async-timing.json)。这些包括非Drive计划，功能钟等待worker时冻结，不是目标板截止时间结论。
[v9持续主机时钟短场](docs/motion-v9-host-clock.json)取得23源/119转弯Drive，poll最大21ms；末源2200、原期限2450，
2460ms输出Stop，2686ms完全停稳回中，无碰撞/越界。v9没有新做多样本同步profile，不沿用更早profile当作v9测量。

历史 v7 顺序主机计时各预热 1 次、正式 3 次：同步整场平均墙钟 PP 1609.081→1621.487 ms（+0.77%），
LQR 1494.640→1484.827 ms（−0.66%），模拟时长/路程保持。小样本差异不足以认定加速或退化，见[完整样本](docs/motion-v7-host-profile.json)。
默认异步导航计时段 PP 平均/最大约 4.557/11.875 ms、LQR 4.809/20.414 ms；包含已完成计划中的非 Drive 情形，
[开启计时的完整功能结果](docs/motion-v7-async-timing.json)除计时字段外与默认运行逐值相同。
[持续主机时钟](docs/motion-v7-host-clock.json)取得 23 份源、119 次转弯 Drive；最大 poll 间隔 26 ms。
末源 2201 ms 的原期限为 2451 ms，2460 ms 轮询输出 Stop，2687 ms 完全停稳回中。均为本机观察，目标板时限仍待实测。

灯前禁越线由任务和导航共用有向半平面，支持任意接近方向；锥桶阶段预查灯前续段时即启用，
原有停稳和绿灯新帧确认通过后才解除。全局搜索、局部扫掠、停车圆盘与到达判断都受约束，
避免先越线再绕回合法目标；车体外接圆和网格余量可能保守拒绝贴线姿态。

比赛目标显式区分必须停车的 `Stop` 与锥桶中间点的 `PassThrough`。通过点仍按当前首段独立跟踪、
按原目标位置验收并推进任务顺序；只有从首段实际末端位置/朝向/曲率出发、经过碰撞检查的续段才能提供制动余量。
续段不会进入当前点的前视和进度搜索，不能通过长前视跳过必经点；续段不可行时回到停车策略。
`PassThrough` 同时传递下一阶段的 `next_max_speed_mps` 和任务真实的 `admission_radius_m`；
原配置任务验收 0.065 m 与导航停车容差 0.045 m 分别使用，在进入真实验收圈前提前按减速能力限速，
避免任务切换一拍内突然把速度上限从巡航值降到接近值。
它限制候选目标，不保证进入半径瞬间的测量速度必定低于下一阶段上限；外部超速或测量偏差仍可能触发原有空速度区间 Stop。

执行状态采用 `SteeringEstimate { at, commanded_curvature_per_m, applied_curvature_per_m }`。
`advance_to` 按旧已采用目标推进估计，`adopt` 先推进再切换新目标；Stop 把目标置零，执行估计继续逐步回中。
直接导航的 `plan_with_arrival`/`plan_stop` 只规划，由输出方采用最终命令；同步 `AutonomyController::tick`
在 Safety 检查后采用最终结果。异步 `tick_with_execution_state` 只规划，后台丢弃的结果不能改变执行历史。

使用 `AutonomyWorker` 时，输出线程须持续调用 `poll` 并使用 **`ControlPoll.command`**；`latest.command` 仅供诊断。
poll 先检查原 lease 和锁存故障。固定容量历史只记录最终采用的命令变化，同一命令持续保持不消耗新槽位。
提交方按历史恢复源时刻转向，再将源位姿/速度/转向分段推算到规划时刻；原测量及其 captured_at 保持不变，
仍用于任务验收、新鲜度、Safety 和障碍世界投影。源早于最新 poll 不再自动拒收，超出保留历史则明确返回状态。
后台生成有界采用窗和命令序号证书，覆盖原期限内运动及停车空间、窗口内正常速度与转向/侧向约束；
poll 无需重跑规划。过晚或前提改变的计划不采用，同源重复不续租，晚到 Drive 不解除 Fault。
异步证书现在使用源车体系中的有向矩形，包含历史最大速度/曲率、到原期限再加一拍的运动和完整制动，
不会用新低速命令抹掉历史高速。车体角点旋转、侧向位移和大转角后的向后位移均计入，地图/灯前边界和障碍半径保持。
完整异步默认时序为100ms采样、60/80ms输入延迟、20ms输出及3/7/9ms采用偏移。
历史 v6 同一比赛中 **PP 57.663秒完成；实验LQR在29.463秒因普通停滞超过10秒失败**，两者最终速度和曲率均为零。
LQR经历证书拒绝、短暂恢复、路径搜索失败后持续停车；没有通过放宽证书或比赛期限使其完成。
v6 的这段绕桶过程未启用末端预算预留，详见[异步结果](docs/motion-v6-async-comparison.json)及[失败复核](docs/motion-v6-async-failure-review.json)。
功能时钟等待后台时暂停，因此完整场景不能证明计算按时完成；连续推进时钟的专项测试和实际主机观察单独记录。
`observed_plan.report` 包含未采用或故障报告，仅供诊断；与 `latest.command` 一样不能直接发给底盘。
模型仍可能保守拒绝转弯狭小空间，也不代替真实反馈、共同单调时钟、扫描去畸变或动态障碍处理。

模拟车辆现在按不超过 1 ms 子步及速度/曲率到达目标的分界点积分；航向保留速度与曲率同时变化的交叉项，
每个子步检查实际模拟足迹、锥桶和禁越线，Fault 后制动采用同一逻辑。它提高离线模型一致性，仍不证明采样间绝对无碰撞或实车响应一致。

`NavigationDecision.diagnostics` 还记录当前、选中候选和下一完整周期的停车余量及限制来源。
v4 曾按下一拍停车余量直接排序，造成绕桶网格角阻塞，该排序已撤销，历史余量字段继续用于诊断。
v7 另外加入当前采用约束和有界多拍后备检查，不能把它等同于恢复旧的余量排序。历史实验和独立复算见[停车余量实验](docs/motion-v4-stopping-experiment.json)。
`SimulationSummary.statistics` 由 Rust 在内存累计每阶段时长、实际路程、非预期 Stop tick/连续次数/命令时长，
以及路径生成次数/长度、拒绝原因、终端求解额度总数/每拍最大值；结束后制动单列，每阶段仅保留最近 16 次路线事件和总数。
路径生成包括首次和目标变化，不能一律当作重规划；Stop 命令时长也不是物理静止时长。

`NavigationDecision.diagnostics` 记录路径版本/进度、跟踪误差、曲率请求与实际候选、拒绝原因计数。
模拟摘要的 `first_navigation_failure` 最多保存异常前 8 帧、触发帧、后 8 帧；先前正常任务停车不触发，
首次可恢复阻塞也可能被记录，因此不能把它直接当作最终失败根因。该窗口只在内存维护，结束才随摘要输出，
不默认逐帧刷写 eMMC；目前接入 `autonomy-sim` 和整场 A/B，快照回放未接入此窗口。

当前激光定位是 ICP，**尚无 EKF/ESKF、速度 PI 或真实执行器闭环**。没有轮编码器，不能把低频激光速度重复读取当成高速轮速；
后续先补传感器共同时间轴、扫描去畸变、IMU 标定/融合，再评估前馈表加限幅抗饱和 PI。
舵机标称 `0.16～0.18 s/60°` 不直接等于前轮转向响应；轴距、转向曲率/PWM、制动能力与 MCU 断链停车仍须实测。
Stop 是停止请求，车辆模型继续减速并逐步回中；物理 ESC 中位是否制动尚未验证。
导航保存已采用曲率目标与模型执行估计，尚无真实转向反馈；解析过渡峰值也依赖配置的执行器模型，
不能据此声称实际侧向加速度已受实测保证。保守停车圆盘覆盖中间转向，但仍依赖实际制动能力达到配置假设。

## 配置与样例索引

| 位置 | 用途 |
|---|---|
| `config/yolo26n.json` | 模型尺寸、输出契约、检测阈值 |
| `config/online-example.json` | 在线场景模板：模拟真值、实验标记开关与有限遮挡 |
| `config/online-sim.json` | 已编译在线模拟场景，交给 autonomy-sim |
| `config/online-controller-sim.json` | 跨场地相同的在线控制器配置；快照需包含 road.elements |
| `config/competition-sim.json` | RGB + 雷达 + 车辆反馈的完整比赛模拟场景 |
| `config/competition-controller-sim.json` | 同步传感器快照回放控制器，非旧事件回放配置 |
| `config/road-perception-sim.json` | 道路视觉、灯 ROI、未标定的地面投影示例 |
| `config/scan-assembly-sim.json` / `config/laser-localization-sim.json` | 整圈覆盖和 ICP 库接口参数；没有独立设备 CLI |
| `config/robot-sim.json` + `examples/robot-sim.jsonl` | 多传感器、安全状态机、超时和急停的合成回放 |
| `config/imu-replay.json` + `config/robot-imu-replay.json` + `examples/robot-imu-replay.jsonl` | 显式零偏、frame、分量年龄，以及坏 IMU 帧不刷新状态的样例 |
| `config/n10-replay.json` + `config/robot-n10-replay.json` + `examples/robot-n10-replay.jsonl` | N10 合成分片、坏校验与雷达超时样例；有效包仅 16 束局部数据 |
| `config/chassis-calibration-sim.json` | 人工合成的分段标定表，明确 `simulation_only=true`、`measurement_status=unverified` |
| `config/serial-imu-capture.json` / `config/serial-n10-capture.json` | `/dev/imu` 115200 / `/dev/laser` 230400 的厂商参考采集计划，实际设备身份待核实 |

不要把示例超时、零偏、frame 或 PWM 中性值作为实车校准结果。N10 局部包的接收健康不能证明全圈覆盖或避障有效。

## 本机运行

在仓库根目录执行。以下命令不需要模型或设备：

```bash
cargo run --locked --offline --bin xt-stcar-robot -- autonomy-sim --config config/competition-sim.json
cargo run --locked --bin xt-stcar -- self-check
cargo run --locked --bin xt-stcar-robot -- replay --events examples/robot-sim.jsonl
cargo run --locked --bin xt-stcar-robot -- replay \
  --events examples/robot-imu-replay.jsonl --config config/robot-imu-replay.json \
  --imu-config config/imu-replay.json
cargo run --locked --bin xt-stcar-robot -- replay \
  --events examples/robot-n10-replay.jsonl --config config/robot-n10-replay.json \
  --n10-config config/n10-replay.json --chassis-calibration config/chassis-calibration-sim.json
```

自主模拟默认 stdout 只输出摘要，`--output work/competition-run.jsonl` 保存阶段/故障/终态；`--trace` 开启有界调试细节。
原有回放输出 step + summary JSONL，可加 `--output work/replay.jsonl`。每条运动记录标明
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
| `scripts/onnx_worker.py` | 仅供显式 Python 参考后端的一次性推理工作进程；配置、模型和张量受实际读取量限制 |
| `scripts/review-motion-ablation.py`、`scripts/review-motion-ablation/` | 固定旧提交、检查顺序补丁和只读观察器；每个变体独立主机编译目录，复现三版本消融 |
| `scripts/capture-online-first-rejection.py`、`scripts/capture-online-first-rejection/` | 固定提交的两份独立主机构建，补采小场异步首拒的原始输入/缓存/终端账；核对439条计划一致并保留比赛失败 |
| `scripts/build-riscv.sh`、`inspect_elf.py` | fmt/test/clippy、两程序交叉构建、独立 ELF/GLIBC 报告和源码哈希 |
| `scripts/delivery.py`、`package.sh`、`upload.sh` | 白名单打包与哈希核验；上传默认 dry-run，显式新账号/IP/release 目录 |
| `scripts/check-vehicle.sh` | 车端系统/设备/服务的只读检查脚本，不启动驱动或运动 |
| `target/riscv64gc-unknown-linux-gnu/release/` | 两个目标程序与 build/ELF 证据（本地产物，不进 Git） |
| `dist/` | core 或含模型的独立部署包（不进 Git） |

本轮冻结源码后的最终在线矩阵为 4/19 符合检查、正常完成 0/15；通过项仅预定缺失场景的安全停止。最新 `legacy-integration` 旧八场 8/8 完成、原性能门通过，与 v9 的九项关键指标、输出计数和单因素比较一致，不能代替在线整场通过。本轮不宣称已完成在线比赛验收。

原正式部署包对应 Rust 工作区 **565 通过、0 失败、3 项按设计忽略**；实际日志另记录 tracking/timing/online example 分别 **2/5/2 通过**，Python 交付 **37 通过**。fmt、all-targets clippy `-D warnings`、双 RISC-V 链接与 ELF 检查通过；[构建报告](docs/online-mission-build.json)记录并核对 **150 份**编译输入。例程与 Python 数量来自各自实际日志，不另推断这些日志的源码同一性。后续首拒回放增加2项测试，最新主机工作区为567通过、3忽略，见[首拒验证](docs/online-first-rejection-validation.json)；本次实车检查未修改Rust算法或重做该主机测试计数。
两个程序均为 RISC-V64、LP64D、RVC、PIE，加载器 `/lib/ld-linux-riscv64-lp64d.so.1`；构建基线 **GLIBC 2.38**、实际最高引用 **2.34**。`xt-stcar` 为 **1,217,056 字节**、依赖 libc；`xt-stcar-robot` 为 **2,376,448 字节**、依赖 libm 和 libc。上述构建报告生成时未接车；2026-09-16已使用同一哈希的程序执行车端模块测试，结果见顶部及实车检查记录。
[在线验证](docs/online-mission-validation.json)、[在线矩阵](docs/online-mission-matrix.json)及[开发实验](docs/online-mission-experiments.json)保留成功与失败；core/含模型两包的本地路径、哈希和核验见[包外交付报告](docs/online-mission-delivery.json)。包内 `build.json` 是本轮构建报告，`elf.json`、`robot-elf.json` 对应两程序；交付报告留在包外避免循环哈希。Git 提交与远端同步状态以实查为准。

历史场地适配轮 Rust 工作区 **409 通过、0 失败、2 项按设计忽略**；跟踪 example 2 项、旧时序矩阵 example 5 项、Python 交付 36 项及 39 个子测试通过。
fmt、all-targets clippy `-D warnings`、双 RISC-V 链接与 ELF 检查通过；[构建报告](docs/field-adaptation-build.json)记录 **112 份**编译输入，
GLIBC 基线 **2.38**、实际最高引用 **2.34**，robot 为 **2,090,544 字节**。
[验证汇总](docs/field-adaptation-validation.json)区分 PP 18/18、LQR 17/18、旧八场回归及三个实际 CLI 完整运行，
[本地交付报告](docs/field-adaptation-delivery.json)记录本地包；目标程序尚未在车上或模拟器执行。
新场地改动还包含有限禁带整包络检查、网格误拒后的原预算连续恢复、定向通过点短连接及原到达区域后备连接。
后备连接只在原精确解/搜索失败后使用，终点保留实际积分值，位置/朝向误差含累计误差须进入原容差一半；不把邻近位置假作目标中心。
开发失败与 22/36、34/36 中间矩阵见[试验记录](docs/field-adaptation-experiments.json)。同步末曲率未单独观测；全部异步场末速度/曲率归零。

历史 v9 为 371 Rust、2 跟踪 example、5 矩阵 example、35 Python 交付测试通过；108 份编译输入。
[原验证](docs/motion-v9-validation.json)、[连续性回归](docs/motion-v9-route-regressions.json)和[原试验记录](docs/motion-v9-experiments.json)保留，不能作为新二进制证据。

历史 v7 Rust 常规测试 **350 通过、0 失败、2 项按设计忽略**；跟踪 example 2 项、Python 交付 35 项通过。
fmt、all-targets clippy `-D warnings`、两个 RISC-V 交叉链接和 ELF 检查通过。
[构建报告](docs/motion-v7-build.json)记录 98 份 Rust 源码、清单及编译嵌入 fixture 的哈希，GLIBC 基线 2.38、实际最高引用 2.34。
`xt-stcar-robot` 为 2,020,984 字节；最终二进制证据见[验证汇总](docs/motion-v7-validation.json)，本地包状态见[交付报告](docs/motion-v7-delivery.json)。

历史 v6 Rust 常规测试 **322 通过、0 失败**；原生ORT opt-in和独立证书性能基准各1项按设计忽略。
跟踪example 2项、Python交付34项通过；fmt、all-targets clippy `-D warnings`和两个RISC-V交叉链接通过，
见[验证汇总](docs/motion-v6-validation.json)与[构建报告](docs/motion-v6-build.json)。89份Rust源码/清单哈希已核对，
GLIBC构建基线2.38、实际最高引用2.34。本地包的生成/核验状态、路径和哈希见 `docs/motion-v6-delivery.json`。
v6 另测完整异步分段耗时与持续主机时钟；其 LQR 异步失败保留在历史报告中，测试通过并不表示两种算法全场均完成。
[前轮 v2 验证](docs/motion-v2-validation.json)的237项Rust常规测试、2项跟踪example测试、34项交付测试与二进制证据保留为历史。
更早的[运动控制验证](docs/motion-control-validation.json)也保留，不作为当前二进制证据。
模型/原生推理模块的历史结果单独记录；本轮是否重跑及最终打包证据以在线验证报告为准。
前轮 1 项原生 ORT、48 项模型测试记录见 [历史比赛验证](docs/competition-validation.json)。
构建脚本完整执行 fmt、主机测试、clippy `-D warnings`，再检查两个 RISC-V ELF 的架构/ABI/加载器/GLIBC。
ELF 检查核对动态加载表与 section 映射及版本需求，不验证所有机器指令或代替目标机运行。
交付包不含工具链、虚拟环境、Mac dylib 或缓存；带模型包另带 provenance 与模型许可。
源码仓库也排除模型、厂商大包、镜像、`target/`、`dist/`、`work/` 和缓存。

## 已完成与后续实车工作

已完成上述 Rust 算法、线程接口和离线集成，不等于无人驾驶系统已实车验收。
比赛状态机、元素短期身份/几何、局部可通行检查、斑马线/灯色/锥桶视觉、全圈组帧、ICP、导航与模拟闭环已有代码；
已完成USB相机短时取帧和雷达/IMU串口解析；仍需确认实际停车/终点标记、持续采集接入、共同时间轴/扫描去畸变、TF/初始定位、物理标定、定位融合、
必要的 ROS 2 接口及 MCU 反馈/watchdog。没有物理 MotionSink，也没有实时带动力 CLI。
厂商教程明确没有轮编码器；模拟反馈不能被写成实测轮速或定位精度。

比赛初稿没有禁止 Rust，也没有明文强制 ROS2，但明确禁止 ROS1 和 bag 工具。
普通 80 类 YOLO 仍只提供通用交通灯框，Rust 道路算法补充斑马线、灯色和锥桶候选；尚需真实数据标定和测量。

eMMC 5.1 策略以正常运行/比赛为先：计算与队列在内存，默认只在结束后保存阶段、故障和摘要，
不逐帧落盘、不每 tick 同步刷盘。需要诊断时可显式 trace/采集；日志预算不降低感知频率、不触发控制 Fault。
不会为了省擦写去修改车端系统或牺牲比赛性能。

2026-09-16已在车端用独立目录的官方SpacemiT 2.0.6所带ORT1.24.2运行Rust原生YOLO26n，证明本次API和模型路径可执行；系统自带ORT1.18.1不兼容，失败证据保留。
尚未启用或验证厂商EP、持续帧率和量化精度。候选库要求GLIBC2.38，当前车端实际2.39；EP另要求GLIBCXX3.4.32 / CXXABI1.3.15。
相关说明见 [原生库核验](docs/SpacemiT原生运行库核验.md)。

进一步阅读：[机器人事件与状态机](docs/机器人模块.md)、[厂商协议适配](docs/厂商协议Rust适配.md)、
[N10 协议依据](docs/N10协议依据.md)、[底盘标定映射](docs/底盘标定映射.md)、[YOLO26 接口](docs/YOLO26接口.md)、
[部署包使用说明](docs/部署包使用说明.md)、[官方整车资料核对](docs/无人车2026资料核对.md)、[资料索引](资料/资料索引.md)。

官方 `3.实验案例` 的14份文档与原始出厂源码对照见 [实验案例对照](docs/实验案例对照.md)：记录可借鉴的运动学候选思路、教程与ZIP配置差异，以及不能直接搬入比赛控制的参数。

2026-09-16 用户确认车辆到场并授权架空模块检查；最新接入状态和逐项结果见 [车辆到场模块检查](docs/车辆到场模块检查.md)。此前目标端未执行的交付报告保留为历史证据，实际通过项目以新记录为准。
