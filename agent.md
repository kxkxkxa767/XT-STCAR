# XT-STCAR 接手与开发约定

## 当前接手入口（2026-10-07，第15轮未完成，修正待实测；下一轮16）

**当前目标：解决左转未到位，并衔接真实点云目标绕桩。本节为唯一当前入口；不要采用历史参数。**

- 车端已部署源码 `8d905ca8447a601dcc0bda18af8cd0b4df44119e`，匹配 Rust 桥 SHA256 `199f42039ce21af03421626257040611c12ec30511c1d53aaa596681aefc0cf9`。4e33035的连续真实开口识别修复保持，已上车，不是待部署。
- 第14轮 run `CKS8ZeG0qR4FUHmdOQI3-DmU`：用户反馈“没有撞墙，但是没转到位”，`handover=false / completed=false`。软件前进约1.05948s，`probe_obstacle_close_body`退出；scan1244左前角已知回波到车身矩形7.25865cm，小于8cm门。最大控制间隔87.015ms，小于120ms，**不是超时**。
- 第15轮 run `QcrBeVlAa5vERADbtc_0Ih2-`：前进约1.07024s，scan730前方+10..13°缺测触发`turn_perception_unavailable`；净距14.1519cm、未触发8cm门，不是超时。用户反馈转向不足；软件行驶采用1650→1550，目标1525，不能说始终1670或将PWM当轮角。未接管、completed=false，完整证据已取回。
- 第15轮后用户已再次确认“已完全静止并复位”。SSH恢复，最新部署14个文件哈希核对一致；该轮末6份反馈healthy/locked、armed=false、M=S1500、owner/active空，boot `_oFcBJ97GaKIJsB8enxaaQ`、末tick74387。动力前仍读fresh，不把1500当物理瞬停。
- 15的绝对夹角限幅减左过度。下一候选改为以出口/真实目标需求为主、内侧点云只施加最多20%的有界减量；不是固定1670下限，端头让开后恢复出口跟踪。尚未部署/实测，下一编号**16**；预打1670、上限1720、电机1560、原步幅/时间/8cm停车及缺测门均保持。只做语义unknown的首个真实紧凑目标受限入口，完整两桶S/比赛链未完成。

## 当前控制参数

以下为已部署 `turn-cone / ManeuverSequence`；旧直道及 `turn-left` 不自动套用。

| 项目 | 当前值与含义 |
| --- | --- |
| 电机 | 前进1560，中性1500；禁止反转刹车 |
| 初始预打 | 默认1670，允许1670..1720；初值不等于行驶上限 |
| 左舵上限 | 1720；行驶中依据新鲜几何动态增减，不固定档 |
| PWM渐变 | 中性预打20、减左20、加左10，每100ms最多一步、不补跳；跨中心先1500 |
| 起步前采用等待 | 初始舵目标已采用、至少3个新control tick且等待1.2s；并非实测轮角反馈 |
| 预算 | 中性预打8s、累计drive10s、首目标绕行入口3s、coast5s；收油后不恢复正PWM |
| 时间门 | new控制loop120ms；旧模式80ms；control fresh200ms、autohealth250ms、heartbeat300ms |
| 车身近距门 | 当前回波到矩形净距5cm+测距参考余量3cm=8cm；不认证未来扫掠或停车距离 |

本车实量轴距25cm；雷达原点到车头21cm、车尾20cm、左右轮胎外沿各17cm，见 [几何记录](config/vehicle-geometry-measured-20261006.json)。后轴偏移、物理轮角/曲率、车速及滑停包络仍未标定。不要使用厂商默认305mm冒充实测。

已部署开口规则：0–90°真实连续远返回沿当前左墙延长线的投影跨度≥0.50m；null/近点/无效或相邻投影缺口>0.10m断段，保留前/右/外墙证据。0.50m来自车宽0.34m+两边0.08m，只是观测尺度；`known_open_fraction`仅指选中支持段，不证明整条路径安全。

## 下一步与证据位置

1. 读本文件、[上传规范](上传规范.md)及[动态左转机器记录](docs/vehicle-dynamic-turn-validation-20261007.json)。先fetch核对main，再修改。历史记录只按需检索。
2. 已取回13/14/15原始目录：`work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-{13,14,15}/`。摘要、原helper与完整记录保留，不重跑或覆盖；车端原目录在base下。
3. 完整部署记录在`work/vehicle-dynamic-deploy-20261007/resume-after14/`：`opening-span-deployment-result.json`及`inner-clearance-deployment-result.json`。原车端stage分别为`stage-opening-span-20261007-201206`、`stage-inner-clearance-20261007-205728`，均在base下。
4. 本次记录分析：`work/vehicle-dynamic-deploy-20261007/resume-after14/`。区分state采样与raw scan各自序号/时间，不把相邻扫描倒填成触发帧，不把PWM采用值当物理转角。
5. 修正后按既有实车授权成套备份、默认锁定部署并核对哈希、设置及fresh反馈，再进行16。软件故障锁定不得因感知恢复自行rearm；每次退出先核对新中性反馈与必要现场事实。
6. 每次修改更新本文件的当前入口及机器记录；按上传规范提交并推送main，不强推。文档整理不改变车端代码或运动结果。

## 连接与部署

- 工程 `/Users/yuhaojin/Documents/XT-STCAR`；GitHub `https://github.com/kxkxkxa767/XT-STCAR`，直接main开发，不另建分支/PR。
- SSH `bianbu@192.168.0.156`；复用socket `/Users/yuhaojin/Documents/XT-STCAR/work/vehicle-test-ssh.sock`。失效时让用户在终端重连，不向聊天索取密码，不将旧反馈写成实时状态。
- 车端base `/home/bianbu/xt-stcar-console`；活动目录 `20260917`，用户服务 `xt-stcar-console.service`。仅执行硬件的主agent持有控制；共享驾驶台采集与唯一串口所有者，不另开相机/雷达/底盘抢设备。
- 更新须备份完整活动目录、入口、运行设置及私有配置，传齐8个Python模块、匹配Rust桥、入口、三前端和目标配置例；核对依赖、运行路径、安装哈希及锁定反馈。模型、视觉配置、雷达校准、访问码和启动命令保留。失败恢复整套。
- 当前实际手动设置1550/1450/1650/1350；与内部turn-cone参数分开。重启会重置内存设置，部署前后核对实际值。
- 备份`/home/bianbu/xt-stcar-console/backup-inner-clearance-20261007-205728`保存4e整套；`backup-opening-span-20261007-201206`保存此前7df/旧桥，两者均不得删除。

## 长期约束

- 用户常驻偏好：“以后都是，只要我没明说都是直接在车上测试”。控制调参默认实车，不先跑多余Mac/目标板纯计算suite，不重复问动作许可；明确只Mac/暂停/不测时按新指令。仅在未知时确认必要现场静止/摆放事实。
- 禁止赛道固定坐标、练习长度、按时间猜拐点；用实时点云、真实端头/墙面/目标身份。软板允许形变，端点随新扫描更新；角差本身不能证明两面，缺测不当free。
- 不造pose、速度、轮角、圈数或锥桶语义；不将simulation_only/unverified改标签冒充实测。已采用PWM、物理动作、现场评价、任务完成分别记录，旧通过数字不冒充新版本验收。
- 保护门、质量门、急停/失联/旧帧/控制时限保持；缺观测不通过无限续租或自动解锁继续。1500是中性收油，不保证瞬停。新控制可作受限试验，不声称已证整车扫掠/制动。
- 直道正式停车服从传入停车点，不能绑定练习左路口。任务次序为斑马线停车→第一桶逆时针→第二桶顺时针→灯区→终点；是S形通过，不是每桶一整圈。完成/身份账本防旧元素重入。
- 两桶以雷达独立确认、跟踪和规划为主，YOLO辅助同身份；其余任务语义由YOLO。训练交队友；最终标签固定 `0:crosswalk / 1:blue_cone / 2:red_light / 3:green_light`。车上仍单类blue_cone，不冒称已四类。完整视觉任务单见历史归档。
- 新核心算法优先Rust，现有实时Python模块按现有边界维护；不为统一语言无故重写。Mac交叉构建基线GLIBC2.38，沿用项目工具链；不改车端libc/系统固件，不恢复暂停的STM32读取/刷写，不读厂商视频。
- 正常运行优先于eMMC寿命：减少无用落盘，不降感知/控制频率或因可选日志额度触发故障。必要诊断和全部失败证据保留。
- 源码/配置/必要记录进Git；密码、私钥、access文件、原始影像、模型、工具链、环境、构建产物与部署包不进Git。保留队友原有SUMMARY、ZIP、文本与比赛PDF，不暂存无关文件。

## 必须保留的直道好版本

- 用户认可的直道源码 `8efc47b`：本机 `work/vehicle-junction-coast-test-20261006/release-process/` 及完整 `run04/`；车端 `/home/bianbu/xt-stcar-console/backup-dynamic-turn-20261007-102438`。**绝不能删。**
- `backup-coast-process-20261006-161139` 是部署8ef之前的旧备份，不是run04源码。run04现场效果获认可，但软件低运动/停稳未确认，`completed=false`仍保留。
- 其余历史失败备份、原始扫描与队友原件同样保留，不用新成功覆盖旧失败。

## 代码与历史索引

| 入口 | 职责 |
| --- | --- |
| `web/vehicle-console/maneuver_sequence.py` | 当前左转→首个真实紧凑目标受限入口 |
| `turn_motion.py`、`compact_target.py`（同目录） | 墙/通道关联与预打控制、真实点簇多帧身份 |
| `server.py`、`autonomy_live.py`、`coast_motion.py`（同目录） | 所有权/控制时限、车身近距、滑行/低运动 |
| `autonomy-control.py`、`stop_goal.py`（同目录） | CLI、正式停车目标契约 |
| `crates/robot-core/src/junction.rs`、`junction/turn_goal.rs` | 原生开口/墙线/端头几何 |
| `crates/robot-core`、`crates/runner` | Rust任务/导航、桥与执行；离线比赛结果不等于实车闭环 |
| [README](README.md)、[命令手册](Mac与车端命令手册.txt) | 代码结构、模块入口、Mac/车端命令 |
| [环境](资料/环境.md)、[资料索引](资料/资料索引.md) | 工具链、厂商文字/源码来源 |
| [动态接口](docs/动态左转与测试目标接口-20261006.md) | 实车左转及实验目标边界 |
| [完整历史归档](agent-history-20261007.md) | 精简前1614行原文、全部旧参数/失败记录/视觉任务单；仅按需查阅 |

维护方式：直接改写当前入口，不再层层追加“新窗口接手入口”；详细逐轮数据写入docs/机器记录，原始材料留work/。历史归档正文逐字节保留，归档头列原SHA256；历史中的“最新”“待部署”“暂停”等不能覆盖当前用户指令。
