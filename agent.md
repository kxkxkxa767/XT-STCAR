# XT-STCAR 接手与开发约定

## 当前接手入口（2026-10-09，第38轮仍绕首桶回转；下一轮39）

**用户反馈38“还是和上次一样绕着第一个锥桶回去了”；已完全静止复位到斑马线后朝原直道。车端37ed519。38新准备门和首目标过尾账本均触发，但实际行驶未回中/未打右：613无当前原生通道先coast，616前近2.675cm硬停。整段未完成，碰撞有无未单独回答。39已实现健康中性等待内准备证据衔接及已准备后的100ms减左节拍，103项控制器+23项服务检查和独立复核通过；正在成套打包，尚未部署/实测。**

- 已部署 `37ed519a48fef2a1c56ac8c8ade06efbff095463`；`work/vehicle-dynamic-deploy-20261008/observed-exit-deployment-result.json`。14文件与六份fresh核对；Rust桥SHA256仍为 `199f42039ce21af03421626257040611c12ec30511c1d53aaa596681aefc0cf9`。36/37备份 `backup-observed-exit-20261009-205333`，35备份 `backup-neutral-route-wait-20261009-203802`，34备份 `backup-route-continuation-20261009-202823`；31及直道好版本均保留。手动1550/1450/1650/1350与私有配置保留。
- 38在597进入quality_wait保1705；598前方8个未知，已实际M1500/S1705 neutralACK后继续等待，新“>6仍中性等待”分支这轮确实触发。front_sparse=3是控制tick计数；raw598没存，旁附raw599不能当598。599至603五good后恢复，实际ACK seq138/tick60479 M1560/S1705，snapshot仍scan603，入口至ACK约0.683s。
- 607新准备门成熟（604..607双钟.298/.279s），当前实际S1708开始请求1500。613首目标最前支撑x=-.1909过rear-.18，进入right_align；当前corridor=[]，先right_exit_corridor_missing进入coast。613原始有20个wall候选，不是全部墙消失；event snapshot未保存wall_candidates不能据空字段判断。
- 已采到实际减左1708→1668(257ms)→1628(280ms)→1608(231ms)，中间20步未逐个采到。1588仅controller决策，不能当实际反馈；首次实反馈1500来自硬stop，非合法行驶回中ACK。616前方6°range.228，车身净距.026751触发8cm；不是心跳/预算超时。
- 两处延迟：全质量中性等待内已有“全部首支撑越车头+.20、侧8cm”的当前几何，恢复后却清窗再等604..607，604还继续加左到1710；准备已锁存后drive只在新scan运行_slew，和100ms步门叠加会错过释放时机。39已实现入wait仍清旧窗，仅实际neutralACK/全quality/同ID完整新帧重建准备，严格恢复ACK后延续；已准备且当前同ID/full/fresh/实际上一命令ACK时，重复tick也可每100ms单步减20到1500，不补跳、不推进观测账本/右权限。
- 首目标始终原ID关联，不能把问题说成第二桶被认回首桶；第二获取尚未开始。少点远簇不能冒认第二。当前全过rear-.18账本、当前唯一通道/实际centerACK右门、独立第二身份、原预算/质量/8cm均保留。若早释放后仍通道缺失，再处理当前实际墙窗配对，不能用旧墙充当前或任挑方向。
- 显式 `turn-cone --continue-route` 保留旧模式。接管后一次有界quality_wait，实际中性ACK后至少324实际有效返回；恢复仍须原动力质量、同ID/full、≥3帧发布/实收均≥300ms和实际ACK。旧coast/硬停不恢复，累计10s/首入口3s含等待不刷新；通道及第二目标后续未获实车验收。
- 38候选95项控制器+23服务检查、独立审查通过；真实fixture无raw1964，仅摘要；现场仍失败。36赛道问题同版复测、37回转记录全部保留。38全部7原始哈希与最终6fresh锁定核对；下一轮39，静止复位事实已知，不重复问许可。PWM非实测轮角，中性非瞬停。

## 当前控制参数

以下为当前 `turn-cone / ManeuverSequence`；路线模式仅显式continue_route启用。旧直道及 `turn-left` 不自动套用。

| 项目 | 当前值与含义 |
| --- | --- |
| 电机 | 前进1560，中性1500；禁止反转刹车 |
| 初始预打 | 配置默认1670、允许1670..1720；已按当前端头限制实际预打，采用值单独记录 |
| 左舵上限 | 1720；行驶中依据新鲜几何动态增减，不固定档 |
| PWM渐变 | 中性预打20、减左20、加左/过中后向右10，每100ms最多一步、不补跳；向右前确认采用1500 |
| 起步前采用等待 | 初始舵目标已采用、至少3个新control tick且等待1.2s；并非实测轮角反馈 |
| 预算 | 中性预打8s、累计drive10s、首目标入口3s、右对齐3s、路线模式第二入口3s、coast5s；累计预算含quality_wait，不刷新；终止coast不恢复 |
| 旧默认质量收油 | continue_route=false：接管后scan_incomplete进入coast，有限front_sparse可保已采用左舵；最多5s，不恢复动力 |
| 路线模式质量等待 | continue_route=true：接管后一次有界quality_wait；实际中性ACK后至少324有效返回，稳定达到原动力质量/同ID/双时钟门才恢复，原期限不刷新 |
| 已部署出口宽度 | 小变动更新，大跳变不单独中断drive，沿当前真实外墙及上次观测宽度继续 |
| 时间门 | new控制loop120ms；旧模式80ms；control fresh200ms、autohealth250ms、heartbeat300ms |
| 车身近距门 | 首目标pass前实际左转drive左侧点总4cm；路线模式恢复ACK等待、后续路线及其余情况总8cm；不认证未来扫掠或停车距离 |

本车此前实量轴距25cm；2026-10-07用户更新粗测雷达到车头20cm、车尾18cm、左右轮胎外沿各14cm（整体38×28cm），见 [几何记录](config/vehicle-geometry-measured-20261006.json)。后轴偏移、物理轮角/曲率、车速及滑停包络仍未标定。不要使用厂商默认305mm冒充实测。

已部署开口规则：0–90°真实连续远返回沿当前左墙延长线的投影跨度≥0.50m；null/近点/无效或相邻投影缺口>0.10m断段，保留前/右/外墙证据。0.50m为沿用的保守观测支持门；新粗测宽28cm后未下调该Rust门，不将旧34cm当当前车宽；`known_open_fraction`仅指选中支持段，不证明整条路径安全。

## 下一步与证据位置

1. 读本文件、[上传规范](上传规范.md)及[动态左转机器记录](docs/vehicle-dynamic-turn-validation-20261007.json)。先fetch核对main，再修改。历史记录只按需检索。
2. 13至18原始目录在`work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-{13,14,15,16,17,18}/`；19至29在`work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261008-{19,20,21,22,23,24,25,26,27,28,29}/`。30至38在`work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261009-{30,31,32,33,34,35,36,37,38}/`。原helper/摘要/完整记录均保留；车端原目录在base下。
3. 旧完整部署记录在`work/vehicle-dynamic-deploy-20261007/resume-after14/`。19至30部署及逐帧分析在`work/vehicle-dynamic-deploy-20261008/`；29为`target-evidence-deployment-result.json`，stage为base下`stage-target-evidence-20261008-212120`。
4. 18分析仍在旧目录`trial18-analysis.json`/`trial18-target-components.json`，回归材料在`web/vehicle-console/tests/fixtures/left-cone-18-boundary-null.json`。18的最终drive/orbit计数含coast；区分各自scan序号、PWM/物理轮角、计数器/动力时长。
5. 下一轮**39**；38后已静止复位确认。39修正和126项相关检查已完成；成套部署读fresh再试，不覆盖旧轮。
6. 每次修改更新本文件的当前入口及机器记录；按上传规范提交并推送main，不强推。文档整理不改变车端代码或运动结果。

## 连接与部署

- 工程 `/Users/yuhaojin/Documents/XT-STCAR`；GitHub `https://github.com/kxkxkxa767/XT-STCAR`，直接main开发，不另建分支/PR。
- SSH `bianbu@192.168.0.156`；复用socket `/Users/yuhaojin/Documents/XT-STCAR/work/vehicle-test-ssh.sock`。失效时让用户在终端重连，不向聊天索取密码，不将旧反馈写成实时状态。
- 车端base `/home/bianbu/xt-stcar-console`；活动目录 `20260917`，用户服务 `xt-stcar-console.service`。仅执行硬件的主agent持有控制；共享驾驶台采集与唯一串口所有者，不另开相机/雷达/底盘抢设备。
- 更新须备份完整活动目录、入口、运行设置及私有配置，传齐8个Python模块、匹配Rust桥、入口、三前端和目标配置例；核对依赖、运行路径、安装哈希及锁定反馈。模型、视觉配置、雷达校准、访问码和启动命令保留。失败恢复整套。
- 当前实际手动设置1550/1450/1650/1350；与内部turn-cone参数分开。重启会重置内存设置，部署前后核对实际值。
- base下`backup-right-exit-20261009-191516`保存29用c2a6d68整套；`backup-target-evidence-20261008-212120`保存9c041b1整套；`backup-boundary-track-20261008-211326`保存acb整套；`backup-orbit-left-20261008-205509`保存26认可的966整套；`backup-entry-handoff-20261008-203943`保存6c9整套；`backup-endpoint-bearing-20261008-202942`保存d49整套；`backup-heartbeat-first-20261008-200515`保存8686整套；`backup-width-follow-20261008-194830`保存e7整套；`backup-quality-coast-20261008-193537`保存4bc整套；`backup-handover-base-20261008-191850`保存b6整套；`backup-entry-tracking-20261008-190253`保存688整套；`backup-turn-left-clearance-20261007-215109`保存f00整套；`backup-continuous-left-20261007-213347`保存用户认可的16轮5ec整套；其余8d/4e/7df备份同样保留，路径见历次部署记录，均不得删除。

## 长期约束

- 用户常驻偏好：“以后都是，只要我没明说都是直接在车上测试”。控制调参默认实车，不先跑多余Mac/目标板纯计算suite，不重复问动作许可；明确只Mac/暂停/不测时按新指令。仅在未知时确认必要现场静止/摆放事实。
- 禁止赛道固定坐标、练习长度、按时间猜拐点；用实时点云、真实端头/墙面/目标身份。软板允许形变，端点随新扫描更新；角差本身不能证明两面，缺测不当free。
- 不造pose、速度、轮角、圈数或锥桶语义；不将simulation_only/unverified改标签冒充实测。已采用PWM、物理动作、现场评价、任务完成分别记录，旧通过数字不冒充新版本验收。
- 保护门按顶部最新明确授权生效；质量门、急停/失联/旧帧/控制时限保持；缺观测不通过无限续租或自动解锁继续。1500是中性收油，不保证瞬停。新控制可作受限试验，不声称已证整车扫掠/制动。
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
| `web/vehicle-console/maneuver_sequence.py` | 左转→首目标；显式35候选通道对齐/跟随→第二目标 |
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
