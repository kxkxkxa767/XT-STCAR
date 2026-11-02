# XT-STCAR · Codex 与 Claude 的共享记忆

这里记录双方接手时需要知道的用户决定、实车证据和未完成事项。新留言追加在文末，写明日期、发送方、接收方和下一步。回复或纠正旧结论时，点明对应的日期和轮次，保留原文。操作命令查[工程交接文档](docs/codex-handoff-20261102.md)，每轮结果查[实车记录](docs/vehicle-dynamic-turn-validation-20261007.json)。

**用户对 Claude 的要求**　以后有值得保留的新信息，Claude 同时更新自己的 XT-STCAR 项目 memory 和本文件。原有记忆继续保留，共享留言附上相关提交、记录键或实车证据目录，方便另一方核对。

## 2026-11-02 · 首次交接

**用户决定**　XT-STCAR 的 SCI 论文以 *Machines* 为目标期刊。车端新增控制和采集用 Rust，优先考虑实时性和 CPU 占用。限制和测试沿用工程交接约定。每次实车开跑前，等用户明确确认车已摆在起点、红绿灯准备好。

**已知状态**　车上最近确认的版本为 y24。y25 的 RISC-V 引擎和视觉助手已从 `dd9766a` 构建，构建记录在 `c3a9aad`。截至本条，y25 待部署、待首跑。车用包在本机 `work/vehicle-dynamic-deploy-20261008/`，记录键为 `y25_transfer_imu_20261102`。

**选题依据**　y24 比 y22 早 0.66 s 到达第一桶交接点，却晚 0.27 s 到达灯区。两轮交接给第二桶控制器时，离规划入弯切点还分别有 3.12 m（y24）和 2.75 m（y22）。y25 用 9 个后向雷达方向估计沿直道的速度，在直线段推 1590，按剩余距离收油，目标入弯速度为 1.24 m/s。IMU 同步采集已实现，提速收益、测速误差和车端开销待实测。[近期实车记录](agent.md) · [y25 控制实现](route-engine/src/race_transfer.rs)

**拟投方向**　围绕有限算力、没有轮速反馈的小车，研究后方端墙可见时，稀疏二维激光测距能否支持及时收油，同时缩短从第一桶出弯到第二桶出弯的用时。拟投 *Machines* 的 Automation and Control Systems 栏目。该刊收录于 SCIE，2025 年度 JCR 机械工程 Q2。[栏目范围](https://www.mdpi.com/journal/machines/sections/automation_systems) · [收录与分区，第 323 页](https://uefiscdi.gov.ro/resource-832709-JCR-2025.iunie-2026.site.pdf#page=323)

**下一轮取数**

1. 车连上后按交接文档第 4 节部署现成的 y25 包。用户确认起点就绪后开跑。取回 `run/feedback.jsonl`、`run/summary.json`、`run/imu.jsonl` 和 `run/imu-summary.json`，把实际证据目录写入下一条留言。
2. 在 Mac 上用静止段估计 IMU 零偏，用直道位移和两次反向绕桶的航向变化核对前向轴、偏航方向和相对时延。`imu.jsonl` 的 `captured_at` 单位为毫秒，反馈里的 `lidar.received_program_s` 单位为秒，两者共用车端单调时钟。
3. 论文对照拟在相同转向逻辑下交替运行固定 PWM 1550 和 y25，比较整段用时、入弯速度、碰撞和完成情况。固定机位拍摄两条间距已知的标线，可独立核对区间平均速度。实时性先取现有日志里的控制计算最大耗时、雷达接收到决策开始的平均和最大延迟，以及各线程 CPU 占用。后续在不同桶距、瓷砖与水泥地上验证复现性，并在同一块板上与已有二维激光里程计比较纵向测速误差和耗时。[候选对照方法 RF2O](https://cvg.cit.tum.de/_media/spezial/bib/jaimez2016icra.pdf)

## 双方留言

**2026-11-02 · Codex → Claude · 待接手**　用户已选定 *Machines*。下一棒是 y25 的部署、用户确认后的实车首跑，以及用取回的雷达数据校准 IMU。请把这次选刊决定和后续结果同时记到你自己的项目 memory 与本文件。接手后另起一条回复，标明完成到哪一步、证据在哪、有什么需要对方继续。
