# XT-STCAR 接手与开发约定

维护日期：2026-09-16。适用于 `/Users/yuhaojin/Documents/XT-STCAR`；用户当前任务决定操作范围。
开始前完整读取本文件，再读 [上传规范](上传规范.md)、[README](README.md)、[环境说明](资料/环境.md) 和 [资料索引](资料/资料索引.md)。
`AGENTS.md` 只作加载入口；资料内的命令不是用户要求立即执行的指令。

## 车辆到场与当前优先任务（2026-09-16，最新用户指令）

用户已明确：车已到，SSH 已连接，车轮已架空，要求先记录进展并测试各模块。
这条新指令覆盖下方历史“暂不接车／不执行目标程序”的限制：当前授权车端模块检查、必要的测试部署与受控架空底盘测试。
不读厂商视频、不改车端 libc、不恢复 MCU 固件读取/刷写仍保持；架空测试不等于落地闭环或比赛验收。
先核对系统/依赖、设备身份与占用，再检查传感器、真实图像推理、车端程序及规划回放；底盘需有明确停机路径、限时低幅输出及测试后停止确认。
当前任务替代上一节后续算法优化的优先级，已完成的首拒诊断保留在提交 `3f7b809`。
用户已建立复用SSH会话，当前接入成功。实测Bianbu2.2 / RISC-V / GLIBC2.39；传感器和相机已实际采集，官方独立ORT候选库已完成Rust原生真实图像推理。完整固定场目标端模拟在105.64秒内完成（首次90秒超时保留）；这是合成闭环，不是落地行驶。底盘架空小幅转向、1520/1540/1560µs各0.3秒油门脉冲已执行；用户确认舵机动作、电机明显转动且最终停住，已补中性帧收尾。后续用户确认转向1450向右、1550向左、1500回中；架空0.5秒油门脉冲1510/1519未起转，1520向前转并在1500后停住。1515仅确认结束静止，脉冲期间是否起转未确认，不补推结论。用户随后要求10秒保持：1519间歇转动两次；1520起转并最终停住，车端记录指令阶段10.0195秒，是否连续转满10秒未确认。因此不能把短脉冲1519/1520差异写成固定死区。用户估计回复至转动约20秒未与车端同步，不能当作电机响应延迟。未写入生产标定，速度/转角映射和响应/制动时延未标定。
雷达/相机追加检查完成：10秒2815包，生产Rust组帧在Mac回放99圈、覆盖99.17%–100%、拒绝0；相机MJPG传输约29.8fps，OpenCV BGR读循环640×480约14.91fps、1080p约9.93fps，读取失败0，不能作为推理帧率。见 `docs/vehicle-sensor-followup-validation.json`。
新增 `scripts/camera-preview.py`，使用车上已有OpenCV及标准库提供8080网页/MJPEG/单帧，已部署到车端独立测试目录。实际640×480单帧与连续MJPEG验证通过，SIGTERM退出0；测试后服务已停止，用户尚未要求常驻运行。启动方式见模块检查文档。
用户随后明确要求更新车端ONNX Runtime：已将官方SpacemiT 2.0.6（ORT1.24.2+spacemit.a1）安装到 `~/.local/opt/xt-stcar/onnxruntime/spacemit-2.0.6`，`current`指向它；`~/.local/bin/xt-stcar`为工程默认启动入口，配置 `~/.config/xt-stcar/runtime.env`。已实测API22、安装目录及启动入口两次真实图片推理、新登录PATH可见。系统原生1.18.1/Python1.18.0保留，当前sudo需密码，未修改系统包/libc、未启用EP。详情 `docs/车端ONNXRuntime升级.md`；不要把用户级安装写成系统/Python全局升级。
用户继续要求车上能更新的都更新：已在用户审计目录重新获取并验证当前Bianbu2.2配套源全部索引（29.1MB，apt update退出0）。普通upgrade模拟为0升级/0新增/0删除/0保留；dist-upgrade模拟却删除当前linux-image-6.6.63并降级bianbu-esos0.0.10→0.0.9，未执行任何系统包事务。不得建议直接full-upgrade/dist-upgrade。系统sudo仍需密码，无需为零更新额外授权；不改源/优先级/系统索引/libc/内核。发现realtek-bt.service自9月15日开机即bring up hci failed，当前无HCI，软硬阻断均否，尚未修复。详见 `docs/车端系统更新检查.md`。
用户随后要求验证蓝牙：BlueZ主服务运行，但无HCI控制器，实际限时扫描报No default controller available（命令退出0不代表通过）。厂商8852bs分支需要rtk_hciattach，标准程序路径和dpkg文件记录均缺失；这是明确初始化阻碍。内核HCI UART/H4/H5已注册、rfkill未阻断，不判硬件损坏。只验证，未安装工具/重启服务/配对，见 `docs/vehicle-bluetooth-validation.json`。
用户授权补齐蓝牙，并要求先确认可以再装系统：已取得官方rtk_hciattach固定源码及RTL8852BS固件/配置，Mac交叉编译PIE/GLIBC2.38、车端-l/ldd通过。已部署 `~/xt-stcar-tests/20260916-bluetooth-repair/repair.py`，等待用户在SSH执行sudo；脚本临时mount namespace内验证控制器及收到扫描事件才安装三文件，安装后复测失败则回退。第一轮临时测试因Zig串口头文件B115200常量与车端GLIBC2.39不匹配失败，未安装系统；已用车端termios头文件-I覆盖重编译，无设备C探针对照通过，修正版hash为1a807e8408bd27212c7414b91072591c2168ddcd55cebf338f6afbbf150f65d8，等待用户sudo重试。尚未实际扫描成功或安装系统。见 `docs/车端蓝牙修复.md`，不能把准备完成写成修复成功。
蓝牙后续：第二轮临时初始化已成功识别RTL8852BS、加载固件并创建hci0，但脚本未等BlueZ就绪就退出；第三轮因缺少蓝牙上电复位，芯片保持上次1.5M链路状态而H5_SYNC超时。已补BlueZ实际就绪等待及与厂商一致、校验type/name的蓝牙rfkill关闭/开启各1秒，模拟等待/中断恢复测试通过；修正版上传，等待用户第四轮sudo执行。前三轮均未安装系统，记录详见蓝牙修复文档。
最新蓝牙第四轮：临时扫描已收到8设备；三文件系统安装后，原厂服务FIFO收0/1导致控制器停止重启，第二次扫描撞上切换而失败，脚本已回退系统文件。已把系统阶段扫描改为最多3次真实事件验证并保存错误，模拟回归通过；等待sudo重试。用户明确全权继续、要外出，不需重复确认授权，但实查sudo -n仍需密码，不能绕过认证。当前没有进行任何无人看护底盘测试。
最新第五轮：临时扫描收到9设备，厂商服务随后收到FIFO关闭指令，3次系统验证失败并回退。已定位车端Blueman KillSwitch插件调用realtek_bt.sh hci_start/hci_stop；插件子进程阻塞FIFO并拖住applet。用户级org.blueman.general plugin-list已由[]改为["!KillSwitch"]，原值备份在车端蓝牙修复目录blueman-plugin-backup.json；结束阻塞的用户子进程、重启用户applet后，D-Bus QueryPlugins正常且不含KillSwitch，PowerManager保留。未修改厂商系统脚本，仍待sudo安装复测；此插件调整尚不能替代实际系统扫描验收。第五轮原始报告归档到本机work/vehicle-bluetooth-repair/attempt5-result.json，附近设备地址不上传。
最终第六轮蓝牙修复完成（覆盖上面各轮待安装状态）：临时扫描收到8设备，系统安装后扫描收到7设备，repair-result.json记录system_install_completed=true。独立复核厂商realtek-bt.service为active/enabled，控制器Powered=yes，三份系统文件SHA256与已验证候选完全一致。用户级KillSwitch禁用保留、原配置已备份。未验证重启后恢复、配对或音频/数据协议，不重复执行拒绝覆盖已有文件的安装脚本；公开结果见docs/vehicle-bluetooth-repair-validation.json。
实际结果逐项记录到 `docs/车辆到场模块检查.md`，不得将准备或连接尝试写成验证通过。

## 新窗口接手入口（2026-09-16，上传完成后）

**本节优先于下方历史轮次和 `work/` 内旧暂停快照。请完整读完本文件再继续，不要重新从交叉编译环境安装开始。**

- 工程根目录 `/Users/yuhaojin/Documents/XT-STCAR`，分支 `main`。上一轮实现与交付提交为
  **`ab323a277ca1aa1bc195c221333a1948a24b8e35`**（`新增观测驱动任务与局部导航并归档验证`），
  已成功推送；上传后本地 HEAD 与 GitHub `refs/heads/main` 完全一致。之后的交接文档提交不改变这个实现基线，
  不要把上述实现提交号硬当作最新 HEAD。新窗口修改前仍先 fetch、核对并按上传规范同步。
- 当前没有待合入的代理补丁或待上传的源码；本机 core/含模型两包也已生成并独立核验。
  用户原件 `XT-STCAR_a623638_review.zip` 与根目录比赛规则 PDF 故意保持未跟踪，不删除、不暂存。
  新窗口不能依赖旧窗口的子代理、变量或命令会话；以磁盘文件和 Git 为准。
- 当前目标仍是**任务顺序固定、元素位置在线估计、持续更新短目标**。默认 PP，LQR 为实验选项；
  保留旧 Fixed 模式作回归，不能退回用场景真值生成在线必经坐标。最新方案正文已保存在
  `work/online-mission/new-plan.txt`，无需仅因换窗口重新获取历次分享或重读所有旧复核包。
- **构建交付完成，在线比赛功能尚未验收。** 最终 Rust 565 通过、3 项按设计忽略；example 2/5/2、Python 37 通过，
  fmt/clippy、双 RISC-V GLIBC 2.38 交叉链接与 ELF 检查通过，150 份编译输入已核对。旧固定目标八场 8/8 通过原性能门。
  在线最终 19 场仅 4 个预声明缺失场景符合安全停止检查，正常整场完成 **0/15**；14 场证明两桶后缺灯区几何，
  另三个小场在第一桶尚未通过。不能把“测试通过、上传完成”写成“比赛已经跑通”，也不能说只剩实车验证。

### 下一步从哪里继续

0. **最新诊断进展（2026-09-16，本次接手）**：已补齐下述 33180/33260/33380 ms 的真实原始输入、
   Navigator 边界更新前/plan 前后全部缓存、terminal seed、障碍与共享账本；夹具为
   `crates/robot-core/src/navigation/fixtures/online33260_first_rejection.json`。
   详见 [小场异步首拒快照](docs/小场异步首拒快照.md) 和 [本次验证](docs/online-first-rejection-validation.json)。
   两套独立源码/target 的原版与观测版均保留小场失败，439 条计划及全部功能 JSON 逐值一致（仅主机耗时不同）；
   固定提交复采脚本又生成了逐字节相同的夹具。生产控制算法没有修改，原在线矩阵与部署包未替换。
   33260 ms 的17次运动段拒绝均定位到有限停车线侧带：15个搜索段端点车体余量小于40mm净空，
   两次终端尝试约42.802mm，低于原净空加起步制动预留44.333mm；即使将扫掠管半径降至0，这些候选也不能通过。
   本轮完整 Rust 测试 **567通过、3忽略**，fmt/clippy通过；与上次交叉构建证据分开记录。
   用户追加的7组14份实验案例已读，并直接核对出厂ZIP；见 [实验案例对照](docs/实验案例对照.md)。
   RPP固定前视和Smac/MPPI运动学候选可供后续参考，但教程存在关闭碰撞检测、配置版本不同和默认配置文件缺失，不能照搬参数。
   **下一步检查同约束下的局部目标/路线形状、终端初猜及候选覆盖，不再重复补抓，也不继续仅靠收紧扫掠管证明。**
   这不是物理无解证明，不删除原净空、起步预留、source/history/lease 门。同步33600及灯区正几何缺口仍另行处理。
1. 先读 [在线任务说明](docs/在线元素驱动任务.md)、[验证报告](docs/online-mission-validation.json)、
   [最终矩阵](docs/online-mission-matrix.json) 和 [交付报告](docs/online-mission-delivery.json)。
   [开发实验](docs/online-mission-experiments.json) 保留 31 条历史/最终记录，不覆盖旧失败以追成绩。
2. 原接手待办（现已由第0项完成捕获）：**小场异步 planned=33260 ms、source=33200 ms、delivery=33269 ms** 的完整首拒快照。
   已知该帧因 `boundary_changed` 重建，恢复搜索 `open_empty`，6 节点/5 展开/25 次 primitive 尝试，
   terminal 2 solver/2 iteration/48 samples，预算未耗尽且 rollout=0；尚未进入 Drive 的 `permits_command`。
   提示 cap≈0.23443 高于可执行下界≈0.19341，不是 `empty_speed_interval`。
   当时缺少完整 Navigator before/cache、terminal seed 和障碍快照；本次已真实补采并建立回放 fixture，
   仍不能证明物理无解。不要从最终位姿重造缓存，也不要拿之后 Stop 回舵的次生失败继续堆连接候选。
3. 小场同步的新首个来源包络拒绝在 **33600 ms**：前段已连续降速，新观测线几何下假设 cap≈0.10781 可证，
   但真实 source 速度≈0.13711，最终门仍必须使用真实/历史速度而拒绝。不能把较低目标速度当作已经实现的车速。
   上述同步/异步定点捕获来自末轮工作副本，完整最终场结果另见正式矩阵；不得混用不同运行的时刻或源码证据。
4. 灯区是另一问题：灯色不提供停车区域距离，官方真实停车/终点可见图案仍未确认；当前白条协议只是显式实验协议。
   保留正几何期限与负约束，不能靠延长 TTL、猜距离、场景真值或删除停车门让整场通过。
   小场仍需算法/几何诊断，不能把它归入灯区信息缺口。

### 本机证据与复跑入口

- `work/online-mission/source-speed-hint-independent-review.md`：当前来源降速提示的独立审查与首拒证据边界。
- `work/online-mission/sixth-small-review/source-hint-sync-capture/gate-33600.json`、
  `work/online-mission/sixth-small-review/source-hint-async-capture/hint-33260.json`：真实来源门与异步提示快照。
- `work/online-mission/sixth-small-review/source-hint-sync-trace.jsonl`、`source-hint-async-trace.json`（同目录）：对应定点捕获的运行记录。
- `work/online-mission/final-full-matrix.json`、`work/final-matrix-run.json`：最终原始结果及运行前后源码/二进制/输出凭证；
  `work/online-mission/final-independent-classification.json`：各场最后非 fault 行的真实原因。
- `work/online-mission/final-independent-review.md` 保存各次独立复核；`INTEGRATION-HANDOFF.md`（同目录）顶部已标交付完成，
  下方的暂停、未构建、未上传内容均为历史。`work/` 被 Git 忽略，只在这台 Mac 上可用；换电脑应以仓库报告/fixture为准，缺材料须重新真实捕获。
- 小场现状复跑：在工程根目录执行 `cargo run --release --locked --offline -p xt-stcar-robot-runner --example online_matrix -- both small_5x4`。
  目前返回非零是如实保留失败，不能改预期将其转成通过。正式复跑/构建步骤见下方说明与命令手册。

上一次交接仅改文档；本次补充主机捕获脚本、真实fixture及测试，未修改生产算法或替换上一轮交付包。继续开发时按实际改动重验；
暂不接车、不读视频、不动 MCU/车端 libc、不执行目标程序或模拟器的边界继续生效。

## 当前交接快照（2026-09-16）

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

### 当前整合状态（2026-09-16，最新）

- 用户已继续统一整合与复核；本轮源码、最终矩阵与本地交叉构建已完成整合，实现提交 `ab323a2` 已推送 main。
  上传后已核对远端 HEAD；后续文档提交和最新同步状态仍以 Git 实查为准，不替用户修改模型设置。
- 冻结源码后的最终完整矩阵：**4/19** 符合预声明检查；通过的只是 missing_markers 与 missing_cones 各同步/异步，
  正常场 **0/15 完成**。独立裁判共14场证明两桶（12个正常场、2个缺标记场），随后都在 ApproachLight
  因看见灯但缺确认的停车区几何而停止。三个小场均未通过第一桶：同步46.200s、异步43.863s、扰动异步45.045s结束。
  全部19场最终都未完成比赛，故障为普通停车超过10s。**在线比赛闭环尚未验收，不能把所有失败都归为灯信息不足**。
  11个预声明输入与运行前后源码账、真实命令、主机可执行文件、stdout/stderr哈希均已核对；各轮开发失败保留。
- 生产新增：仅 OrbitCone 的实际姿态+5个观测短弧姿态/8步停车包络选速提示；原 source/history/负线/lease 门保持。
  StopLine/FinishMarker 槽内身份在真实唯一重观察时可续接，过TTL首新帧重新计1、第二不同采集帧才恢复 confirmed；
  observations表示当前确认周期的独立视觉帧数；invalid/歧义不能复活，processed保持，原TTL不延长。
  runner显式pin至多2个当前引用的StopLine/FinishMarker槽，避免其身份被过期槽回收；pin不续期、不伪造fresh或确认。
  未被保留且已回收的槽不能找回旧ID。锥桶规则不变。
- RollingLocal 的 <0.35m 定向目标按原严格双弧→原严格单弧→原半容差区域→lattice，在新boundary下同账重证，实际18100/18800大圈反例已修；
  Fixed 不改。新增 fixture `online18100_boundary_short_arrival.json`。Finish 完成显式要求原全车清灯线，重叠区域未清尾则 Fault。
  最终workspace为565通过、0失败、3项按设计忽略；早期定向计数不再作为当前总数。
- 新增仅RollingLocal的完整停车后备证明：原disk失败后，用StoppingEnvelope覆盖原2控制周期+全制动、历史速度上界、
  Kmax下任意变向和完整车体；不是固定转向名义轨迹，原rollout/误差/采用门保留，Fixed不变。
  零sweep仍按实际切向证明整车体KnownFree，不直接判Unknown或自动放行；真实出桶完成门不变。
  第四轮出口交接反例及后续修正均保留；当前结果以最终矩阵为准，不直接套用前轮trace推断当前故障。
- 三处追加生产修正已纳入最终构建，开发与最终矩阵分别记录：Rolling原region及zero候选全失败后可同账认证
  “实际κ保持短段→原rate回中→直行”，三段连续传递误差、真实终点满足原half位置/航向容差。38300原成功及Fixed三帧保持，
  38400/38500短接定向回归通过，真实越线/障碍/误差/预算/非法输入负例拒绝；不据此宣称整场完成。
  原strict continuation动作候选全部失败后，仅Rolling从该候选完整周期预测姿态/曲率/error调用旧region证明，
  同账保留原运动/碰撞/制动/采用门，不调用新三段候选，也不直接报告任务到达或adopt。
  仅Rolling+recovery+有限侧带边界新增整车扫掠二证：端点车体凸包加L/2×(1+Kmax×body_radius)、原净空/重启和累计误差，
  所有点须由同一允许半空间分离；world bounds/obstacles仍原胶囊，Fixed和point-only参考检查保持。
- 后续增加仅该同一范围内的第三份时间域证明：原胶囊与一阶扫掠仍失败时，以实际MotionTransition给出
  M=A+V²K+r(AK+VS+V²K²)，端点完整车体凸包加M×dt²/8覆盖全段。V/K取initial/target上界，A取原accel/decel较大者、
  S取原slew；饱和处v/κ及车体点一阶导数连续，恒速/恒曲率也不削减A/S上界。
  primitive的dt=同次ds/max_speed，rollout用与predict一致的实际缩尾duration.min(dt)，不除以实测低速；
  保留endpoint位置/heading累计误差、clearance/restart、roundoff及共同分离侧，world/obstacles仍原胶囊。
  Fixed在新界计算前退出，无额外samples/solver/node预算。真实五帧fixture为online32000_body_chord_tube.json，
  不能把32100历史停舵状态的短route当Drive，也不宣称31700..31900原region路径与现在优先strict完全相同。
  独立数学审查已追加final-independent-review。小场32500新鲜StopLine负向门已实查并追加当前源选速提示，
  生产已冻结、11场真实CLI编译完成且控制器逐值相同；最终19矩阵与完整build均已完成。
- 最新旧8证据为 `work/online-mission/legacy-integration-timing-final.json` 与 `legacy-integration-comparison.json`：
  8/8完成且原性能门通过；九项关键指标、序列化输出计数及八项单因素比较与v9精确一致，未声称逐命令或完整轨迹逐点相等。
- 最终Rust工作区565通过、0失败、3项按设计忽略；实际日志另记录tracking/timing/online三个example为2/5/2通过，Python交付37通过。
  fmt、all-targets clippy -D warnings、双RISC-V交叉链接与静态ELF检查通过；150份编译输入与当前源码核对一致。
  目标基线riscv64gc-unknown-linux-gnu.2.38，实际最高引用GLIBC2.34，RISC-V64/LP64D/RVC/PIE，加载器/lib/ld-linux-riscv64-lp64d.so.1。
  xt-stcar为1,217,056B，SHA256 b83012928876ff87f322c9b4bac8e51c9a338b4c56c495c1a3598e8bc32b11d5，仅需libc；
  xt-stcar-robot为2,376,448B，SHA256 259b419d6f9061ccb6d56b746d5c23d80a2d0a23d51bcb337e97514b0b793042，需libm+libc。
  正式证据为docs/online-mission-{matrix,build,validation,experiments}.json；本地两包路径、哈希及核验见包外online-mission-delivery.json。
  未运行RISC-V程序、未接车。测试与链接通过不代表在线比赛完成；开发版489测试/旧二进制保留为历史。

### 在线元素驱动（2026-09-16，尚未通过整场验收）

- 用户最新授权按分享 `https://chatgpt.com/share/6aa93bcc-f29c-83ee-996b-865058fcd348` 直接改；
  修改前 fetch 确认 main/origin=`4dfd201db977ffa5c18c1eca9f896dade4ad7875`、0/0。
  分享正文已完整读取，本地开发记录 `work/online-mission/new-plan.txt`：任务顺序固定、位置在线估计、
  持续更新短目标，只执行一段再观测。不是再调 LQR 或恢复整场固定坐标。
- 新增 Rust `local_world.rs`（有界元素身份/视觉雷达关联/空闲障碍未知）和 `online_mission.rs`（观测驱动任务）。
  `runner/online.rs` 接入旧 Autonomy、PP、完整车体制动包络和异步最终证书；旧任务几何不进入在线目标。
  `online_simulation.rs` 隔离场景真值，只供渲染与独立裁判；改变场景时控制器配置逐值相同。
  CLI `online-example/online-compile`，内存感知 `RoadPipeline::new_online`；旧入口保持兼容。
- 已确认锥桶的视觉语义与当前几何分开：`last_visual_at` 最长20s、`last_geometry_at` 无更新满1.8s失效。
  360°雷达只维护已视觉确认的唯一圆簇身份，不新建桶、不猜颜色、不增加确认次数；同刻位姿、外参、半径及误差门保持。
  独立拟合误差保留视觉下限，不逐帧重复累加；仅纯roundoff保留旧浮点表示，位置上限1nm，真实毫米变化仍更新。
- `KnownFree` 已改为完整面积的自适应证明：原外接圆成功路径保留，否则用固定栈按长边二分定向外包矩形，
  每个接受叶矩形完整包含于已证空闲圆，最多64次原空间查询；未知、障碍或额度耗尽不能放过未覆盖区域。
  `Obstacle` 独立检查同源真实有限回波到最多32点凸包的欧氏距离，包含原量程/位姿/年龄误差；
  覆盖圆多出的面积不制造碰撞。未来短弧Unknown只能作规划假设，当前目标和整个执行停车包络仍须KnownFree。
- 绕桶R按实际车体半宽、桶几何误差与原曲率能力取下界，前后悬保留在分段扫掠证明；仍可行的R保持。
  入口必要时用原Stop的位置/航向/停稳条件对齐，无新增hold；稳定带航向目标可用两个horizon（默认1.6m），
  更远是0.8m无航向临时引导，搜索仍单horizon。未来短弧未知时先用原approach限速.18，完整空闲才允许cruise .3。
  出口目标沿切线延伸“桶位置误差+2×原目标位置容差”，实际出口投影须越过位置误差，同时满足原半圈/外侧/near条件。
  圆顶尚差5–6mm过中线就提前processed的缺陷已修，圆弧进度仍封顶π，未虚增角度或放宽容差。
  剩余转角为零仍按该绕行方向的切向做完整车体和净空KnownFree证明，不能直接当Unknown，也不免去出口完成门。
- runner的OrbitSpeedHint仅在OrbitCone用实际pose+5个当前观测短弧参考姿态检查完整StoppingEnvelope，
  采用原Kmax及感知年龄+控制周期horizon；原upper已证则保持，否则8次二分只选已证正下界，target/continuation同步限速。
  这不是连续采样执行证书，无证明不伪造零速目标、不改写真正较高速度；source/history/negative/lease与实际执行门全保留。
- 仅绕桶且负线未获绿灯释放时，在Nav前对current-source姿态的完整StoppingEnvelope加pending/confirmed负线证明，
  线误差沿原source/checked时钟增长，最多8次二分已证正速度上限，target/continuation同步限速。
  下界用同源测量或planning投影速度按原max_decel与实际tick间隔（不超原period）计算，再与有效adoption速度区间下界取max；
  source/planned时间须一致。几何cap低于可执行下界则不改提示，不抬高未证cap、不伪造当前低速，原actual-source/history门继续拒绝不安全Drive。
  新fixture/tests为online_source_stop_speed_hint；无新车体/控制/TTL参数，不刷新线身份或阶段，不解除原负约束。
  开发async33260的boundary_changed后OpenEmpty只有有限小搜索，未耗预算且rollout=0，尚未进permits；cap高于采用下界。
  缺完整缓存只能称原约束下有界规划未找到路径，不能称几何无解/节点耗尽/source gate拒绝；同步局部改善同样未完成整场。
- 导航三个真实快照问题已有修正和专项回归：26580ms同目标短缓存漂移在原账内优先重证，
  49580ms滚动无航向点在[.35,min(2×lookahead,2m))内先试短单弧，严格失败后可认证半位置容差内真实端点；
  21380ms定向斑马线接近在原单/双弧后，另认证实际初始曲率按原模型向零渐变的短前进候选，包含完整累计误差及半位置/航向容差。
  `TargetPolicy::Fixed` 为默认，在线统一`RollingLocal`；不按场地开关，不重置预算，不伪造端点或已采用命令。
  <0.35m定向RollingLocal目标按strict2→strict1→region→lattice同账重证新boundary，18100/18800真实大圈修正，邻帧strict成功保持。
  仅RollingLocal原停车disk失败时可追加完整StoppingEnvelope，涵盖2周期反应+全制动、历史速度、Kmax任意变向；Fixed不改。
  最新 `work/online-mission/legacy-integration-timing-final.json` 旧八场全部完成、原性能门通过；九项关键指标、序列化输出计数及八项单因素比较与v9精确一致。
  初版通用点连接造成的旧八场退化保留在 `legacy-point-timing-final`，不能删除中间失败。
- 灯色不提供地距。`vision/ground_markers.rs` 仅实现显式实验白条协议，库默认禁用，演示场景显式启用；
  不宣称官方现场有这些标记。官方资料只给光电到灯约1m及终点80cm，未确认可见线图案。
  规则中的随机布置没有提供左桶出口到灯的正距离下界；不能用旧 FieldSpec 布局检查冒充规则先验。
- 两个独立观测缺口仍在：固定灯色ROI可先看到灯而无停车区距离；已确认的地面标记又可能在清尾前退出前向ROI。
  默认直行的2.44s/计既有误差约4.89s仅为特定配置估算，不是全场不可行证明。缺几何有灯色仍Stop，不增加stale正向授权。
  已processed的线有真实观测可刷新但不能重选；过期StopLine负约束按来源时钟继续扩张误差，完整包络仍可在线前或侧带外通过，
  不因TTL瞬间封死整个侧带。过期证据不能支持新任务、绿灯或清尾；Crosswalk目标也减含航向项的完整region_error。
  区域过TTL后真实唯一重观察首帧重计1、默认第二独立采集帧恢复确认；ID/first_seen/processed保留，invalid/歧义不复活。
  任务显式pin≤2个当前引用区域只防槽回收，不延长TTL或跳过确认。终点完成显式要求light_cleared；合法区域重叠而未清尾则Fault且不mark_processed。
- 首轮2/19、第二轮3/19、第三至第五轮各4/19仅为历史阶段矩阵。冻结源码后的最终矩阵4/19，正常完成0/15；
  14场通过两桶但灯前缺几何，三个小场尚未通过第一桶，另两场缺桶有限搜索后安全停止。通过项仅四个预声明缺失场景。
  独立`online_referee`检查真实轨迹/顺序/半圈，任务自报processed不能替代裁判。在线比赛闭环尚未验收，不能用旧35/36或旧八场代替。
  82.8s同步两桶后缺几何Stop、84.383s异步第二桶回环耗尽语义期限、早期搜索/覆盖与移动carrot附heading失败均保留。
- 停车包络速度提示已接入最终源码；565 Rust/3按设计忽略及本轮双ELF是当前构建证据，早期`development-build-1.*`的489/2仅为历史。
  三份online配置已生成，11种真实CLI编译记录在`work/online-mission/cli-compilation-final.json`，配置、控制器一致性、源码与报告已核对。
- 预声明11输入（含3异步时序变体）、共19同步/异步 PP 场次；包含物体移位、5×4、上下道不等宽、
  斑马线移位、短遮挡、缺桶、缺标记。输入已在首跑前冻结，不能删除失败或改场地追成绩。
  原车体、限速、净空、3s/300ms、普通停车10s、节点40000、终端256/1024/65536保持。
  几何TTL1800ms仍保留；新增雷达维护不能新建颜色身份或增加视觉确认，并有独立有限语义年龄。
- README、`docs/在线元素驱动任务.md`、命令手册已同步最终入口与结果；正式 `online-mission-*` 矩阵/build/validation已归档。
  两包及核验以包外delivery报告为准，实现提交 `ab323a2` 已推送并核对远端；后续状态以Git实查为准。
  本轮继续直接main、GLIBC2.38本地交叉基线、暂不接车、不读视频、不执行RISC-V/模拟器。

### 赛场规格自适应（2026-09-15，历史）

- 用户已授权“你来补齐自适应赛场规格的吧”；开工前 fetch 确认 main/origin 为 `98368200f6c0eb8189c3e1d79941469c821a10b9`、0/0。
- Rust `robot-core/field.rs` 读取显式米制 FieldSpec，按规则图5外场5–8×4–6m、上下直道1–2m等检查范围与组合，生成官方右桶→左桶→上方直道拓扑。
  0.8m发车/终点区、纸条0.105×0.297m及间距0.105m、0.28m方底锥桶（外接圆半径0.14√2）保持，不缩放车辆/控制限值。
- `FieldPlanning` 从原导航参数取得footprint/clearance/曲率/网格尺寸；首选绕行半径按侧墙间隙和经原Grid膨胀的灯禁带角点限制。
  输出右/左实际半径、每桶5个45°间隔切向通过点和上直道朝向0入口；参考几何不是动态可行性证明。
- `runner/field.rs` 在启动前编译并冻结配置；CLI `field-example/field-layout/field-compile`，模板 `config/field-example.json`。
  校验派生几何与规格一致；仅浮点叶允许8EPS×max(1,abs)的JSON读回舍入，整数/结构严格，厘米篡改拒绝。
  measured只标来源，仍simulation_only/unverified；没有自动测全场、定位灯/终点或重分左右桶身份的实现。
- Mission可选 `cone_waypoint_headings_rad` 默认空，保持旧配置序列化；新通过点要满足原位置+朝向容差。
  定向PassThrough复用原Stop的有界短连接保护，仍有真实任务验收/后段认证，不强制停车或假造到达。
- `light_boundary_region` 限定上直道禁带；整个车体/凸包/胶囊须位于同一允许半空间，不能只验各角/端点。
  原Grid失败后可用同账连续车体恢复，旧Grid成功路径保持；切域重建真实误差路径，不沿用未认证缓存。
- 九种输入在首次完整矩阵前冻结，不删除失败/不改尺寸追成绩；新场统一180s时限，原普通停车10s/停稳3s/绿灯300ms及40000/256/1024/65536预算保持。
  最终36场35通过：默认PP18/18，实验LQR17/18；tall_5x6同步LQR在左桶阶段节点耗尽/普通停车超过10s，失败完整保留。
  22/36与34/36等开发失败见 `docs/field-adaptation-experiments.json`，输入未删改。大场异步PP原灯前卡在目标6.7cm外，最终99.787s完成。
- 旧八场时序回归已复跑，八场完成且固定性能门通过，时间/路程逐值保持v9；最终源码的受影响检查和完整构建均已通过。
  原预算fixture字节保持。封墙测试仍Stop、验新node耗尽原因与总节点上限；定向直线through现在有1次短连接检查，原连续交接断言保留。
- 原精确中心连接及普通/连续搜索失败后，可用同一终端账尝试真实积分端点进入原位置/朝向容差一半；包括完整累计误差/车体/边界认证。
  不重置预算、不替换adopt、不假造到中心。节点用满后该后备不新增搜索节点，耗尽诊断保留；终端额度耗尽仍拒绝。
  `arrival_region_attempts/accepted`仅诊断，零值旧JSON省略；大场灯前最终尝试3/接受1。独立只读复核无阻塞，范围见验证报告。
- 本轮技术说明 `docs/赛场规格自适应.md`，正式证据前缀 `field-adaptation-*`；README/TXT已接入新模块/命令，最终409 Rust/2跟踪example/5矩阵example/36 Python交付（39子测试）通过，2项按设计忽略；fmt/clippy/双交叉链接与ELF通过。
  112份编译输入，GLIBC2.38基线/实际最高2.34；app SHA ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9 /1,217,056B，
  robot SHA ee6de9fc7f6337304e103c45fef9f0d937b94a0fbe5170d7b8ef8e9303011a8f /2,090,544B；分别需libc和libm/libc。
  三种真实CLI生成/读回/整场完成：5×4 74.9s、7×5 109.2s、8×4 95.0s；与内存矩阵独立记录，不宣称全轨迹逐值相同。
  持续主机钟23源/119转弯Drive，poll最大21ms，末源2200、2460到期Stop，终态v/k0无碰撞越界；非目标WCET。
  本地包见 `docs/field-adaptation-delivery.json`，Git实际提交/同步状态以HEAD与origin为准。
  暂不接车、不执行RISC-V/模拟器、不读视频，PP默认/LQR实验，GLIBC2.38本地交叉基线保持。

### 第九轮路径连续性修正（2026-09-15，v9历史）

- 用户直接授权第九份分享 `https://chatgpt.com/share/6aa89fa3-3894-83e8-afe8-45451cbbc42f` 和
  `XT-STCAR_a623638_review.zip` 修正；修改前fetch确认main/origin=`a6236380a68431342ab9e76f468c587c9dd775f3`、0/0。
  本轮按上传规范直接main；实际提交/远端状态通过HEAD/origin核对，不把构建基线当最终提交号。
- 4630字符正文完整读取；ZIP SHA256 `5ac964c02693734ebe2dfdd8931c61b0631a62a405054c9ca6ea4cc54ae1c718`，
  Python算术结果与提供文件一致。外部材料不包含Rust/实车执行；原ZIP和规则PDF保留、不暂存。
- 三版本Rust离线消融：7d原版和检查顺序-only均54.447s/10.317616m，285条实际命令和状态逐值相同，前瞻预扣区间减少2.62%；
  v8坏LQR88.647s/16.515558m，实际Stop都4513ms。独立Cargo target避免历史mtime复用旧产物，首次无效共享target试验已剔除记录。
  复现脚本 `scripts/review-motion-ablation.py` 固定commit/源hash/补丁，只在work隔离主机运行，不恢复旧时基到生产。
- 原45360ms灯前同目标、同当前姿态旧剩余0.502864576→新4.910317092m，44候选接受，无节点/终端耗尽、无连续恢复。
  原始四帧、传感源、任务、每帧128真实history、完整缓存和176候选首因在
  `crates/robot-core/src/navigation/fixtures/lqr45360_terminal_seed_origin.json`。
  原45040候选认证+100ms状态，而下一规划45160为+120ms，旧初值未推进导致冷/旧warm各8迭代失败。
- Rust `navigation/continuity.rs` 缓存候选预测下一状态，原方法失败且额度剩余时按正向位移推进数值初猜，再从当前pose/k/Grid/误差完整认证。
  提前/负向不虚构行程，无效/耗尽提示弃用；预测anchor不是实际采用时钟。恢复成功才延续该连接族，普通成功路径保持原顺序。
  **恢复族的候选优先级有意改变**：完整原硬门通过后，先认证接续，再接近已认证首段曲率，最后原评分。
  used_seed来自实际求解分支，不用诊断计数做控制。目标/朝向、到达策略、边界和任务停驻变化清理提示，规划不替代真实adopt。
- 新可选 `route_length_change` 仅用于同目标、旧缓存可用且footprint drift重建成功的同姿态剩余路线比较；不据缺失判断没有绕路。
  所有原共享时基/最终poll/lease/完整车体/边界/误差包络/Q/R/任务门保持；节点40000、终端256/1024/65536、单次8迭代保持。
- 性能预算在试改前固定于 `crates/runner/examples/fixtures/motion-v9-performance-budget.json`，最终逐字节相同；控制器不读取预算。
  v7/v8同工况较快成功场为参照：整场时间/距离110%，阶段+max(1000ms,10%)，实际Stop+1000ms；缺失/重复/无效指标也失败。
  `runner/examples/support/performance_gate.rs` 仅在8场完成后验收，超限保留JSON并非零退出；不调门追成绩。
- 最终固定顺序input[60,80]×adopt[3,7,9]/[5,9,11]、input[40,60]×adopt[3,7,9]/[5,9,11]：
  PP75.467/54.985/56.443/54.449秒，LQR54.663/55.265/54.249/53.445秒，8/8完成且原安全与固定性能门通过；
  PP四组时间/路程逐值保持v8，坏LQR少34.398s/6.224552m，灯前59.400→24.994s、实际Stop4513ms保持。
  全场证书/曲率slew拒绝和终端总额度耗尽均0；原3000ms停稳/300ms绿灯/整车终点/终态v/k0保持。
  同步PP58.8s/11.508841m、LQR54.3s/10.249675m保持；默认PP仍有174恢复活动tick，不能称任意工况收敛。
- 最终371 Rust通过/0失败/2忽略，跟踪example2、矩阵example5、Python交付35通过；fmt/clippy/双RISC-V静态链接通过，108编译输入。
  新7项continuity测试检查原始history一致性并重放记录投影状态的Navigator，不是独立重跑全部worker历史重建；全链路由真实worker矩阵另覆盖。
  第二份45180 fixture是标注的数值试验，不能称a623原始数据。独立只读复核无阻塞问题，范围见route-regressions。
- 默认异步开关计时功能字段逐值相同，导航平均/最大PP6.216/59.705ms、LQR4.346/11.099ms，功能钟等待worker时冻结，非目标WCET。
  持续主机时钟23源/119转弯Drive，poll最大21ms；末源2200、原期限2450、2460 Stop、2686完全停稳回中，无碰撞越界。
- 该轮app SHA `ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9` /1,217,056B、需libc；
  robot SHA `365e34dcba4d7dee93610cc97f96b4edc094370af1fe7c2570c6552fa2344c93` /2,029,552B、需libm/libc。
  GLIBC2.38基线、实际最高2.34；该轮构建/验收/本地包见 `docs/motion-v9-build.json`、`motion-v9-validation.json`、`motion-v9-delivery.json`。
  本轮技术说明 [路径连续性与性能回归](docs/路径连续性与性能回归.md)，README/TXT已同步；模型/ORT未改、不重跑原生推理。
  PP默认/LQR实验、暂不接车、不执行RISC-V、不读视频、正常运行优先于eMMC寿命约束保持。

### 前轮采用时基与交叉时序修正（2026-09-14，v8历史）

- 用户直接授权按第八轮复核修改；修改前 fetch 成功，`7d28761c40f7d7e69a8ac77977f5b6710dd5a0c6`
  与 origin/main 一致 0/0。最终提交/推送状态以实际 HEAD 与远端为准，继续直接 main。
- 分享 `https://chatgpt.com/share/6aa7e186-f7a4-83e8-8cad-c7d07e02d3e1` 已从服务器 hydration 提取并完整读取4792字符正文。
  原 ZIP `XT-STCAR_7d28761_review_reproduction.zip` SHA256
  `d130cc0096adc5cee61d89e9a8fbde74dcf1a275acc8f4901739cc2cdb5e7d04`；两份Python脚本在work隔离副本执行，生成结果与提供文件一致。
  它们是算术/仓库数据复核，不能写成Rust或实车验收；原ZIP与规则PDF不修改、不提交。
- `robot-core/admission/slew.rs` 共享曲率目标限幅时基：从同一seqlock历史取得真实命令变化时刻及revision，
  导航/当前采用检查/最终证书均以规划窗起点减去该时刻计算；poll仍按实际now、revision、原lease独立复核。
  相同整条命令重复输出不刷新，只有速度变化也算新命令；Stop目标归零、物理模型继续回中。
  revision0且无真实变化才用原启动周期，伪造/缺失/未来时刻拒绝。后备前瞻保留相对秒和半毫秒，速度变化同步推进虚拟采用时钟。
  原真实源历史V/K不清除、不延长lease、不在poll临时裁剪动作。手工processor越界候选现在被最终证书提前Stop，
  正常Navigator已在生成时避免该候选；不能把该测试中的行为变化说成旧Drive保持。
- 多源后备前瞻移到原基础rollout成功后、终端认证/选择之前；全部原门仍保留，rollout不重算。
  原PP19380ms完整361障碍/285点含续段缓存已从隔离基线取得；44个Grid拒绝、accepted0保持，
  forecast 132场景/3850投影/167398区间/1650源检查降为0。基线未记录该帧真实last_change，
  排序专项显式用一周期合成时基保持旧候选区间；真实采用时基另用独立Rust/worker回归，不能混为原始完整历史。
- 新可选route_rebuild_reason记录首次缓存失效原因，失败搜索续留，成功后清除，复用缓存不重复输出；
  原候选评分、完整车体/净空、连续恢复/累计误差界、共享节点和终端预算、Q/R均保留。
- `runner::async_simulation::simulate_async_observed` 只给离线模拟提供输出后观察回调，不进入worker/poll或替换命令。
  `runner/examples/async_timing_matrix.rs` 固定2×2输入/采用时序×PP/LQR，内存限额16384命令变化/2048规划/每规划2048路径点，
  超额显式标记；只在该例子启用，默认无逐tick落盘。真实输出按时刻比较、规划按同source比较；
  返回路径含当前投影位置及剩余参考，坐标或点数首次差异不证明拓扑分叉，需结合revision/重建原因/任务阶段。
  实际Stop时长/段数、导航Stop、被拒绝时保持Drive、采用拒绝分开，Stop时长含启动/终态制动且不等于物理静止。
- 隔离原版完整8场7完成1失败；新交叉input[60,80]×adopt[5,9,11]的LQR在91.669秒普通停车超时，终态v/k0。
  baseline矩阵加入只读observer前后全部summary除wall_time_ms外一致；原4场与v7仓库结果一致。
  本轮技术说明 [采用时基与时序交叉验证](docs/采用时基与时序交叉验证.md)，证据统一 `docs/motion-v8-*`。
- 最终固定顺序input[60,80]×adopt[3,7,9]/[5,9,11]、input[40,60]×adopt[3,7,9]/[5,9,11]：
  PP75.467/54.985/56.443/54.449秒，LQR85.863/86.269/88.647/53.445秒，8/8完成且cert/slew拒绝全0；终端预算耗尽全0。
  全部原3000ms停稳/300ms绿灯/整车终点/终態v/k0保持。同步PP58.8秒、LQR54.3秒保持。
  原失败LQR完成86.269秒；但早input早adopt LQR54.447→88.647秒、10.317616→16.515558m退化，默认PP73.863→75.467秒。
  不宣称普遍鲁棒或性能全面改善，不进一步改Q/R/限值来追成绩。PP默认恢复4次/174活动tick；LQR四场均无恢复，路线仍时序敏感。
- 最终364 Rust通过/0失败/2忽略，跟踪和矩阵example各2通过，Python交付35通过；fmt/clippy/双RISC-V静态链接通过，103份编译输入。
  节点/终端旧回归与nav42项保持；独立审查修复临时不可达目标切回原缓存时的pending重建原因残留，控制/缓存行为不改。
  真实controller旧测试原本期待输出拒绝，已改验证10ms=.04区间、source100于350ms到期Stop及500ms模型归零，6项期限测试通过。
- 主机同步profile预热1/测3：PP1619.924→1632.190ms(+.76%)、LQR1491.721→1513.593ms(+1.47%)；外部负载未控制，非普遍加速证据。
  默认异步导航均值/峰值PP6.395/59.369ms、LQR4.573/13.166ms，含非Drive计划；开启计时后功能字段一致。
  本轮PP有恢复，不能把局部省下预测推为整场加速；功能时钟等待worker会冻结。持续Instant短场23源/119转弯Drive、poll最大21ms，
  source2200原期限2450、2460ms Stop、2687ms停稳回中，无碰撞越界；都不是目标WCET。
- 该轮app SHA ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9 /1,217,056B、需libc；
  robot SHA ed420aa4615948ced31bccfa61bb2f8081b4c8005224646edad101ba0fbd8bd6 /2,026,384B、需libm/libc，最高实际GLIBC2.34。
  最终测试/构建/包分别见motion-v8-validation/build/delivery，包只在本地dist、未上传车端。
- 暂不接车、不读视频、不执行RISC-V目标，PP默认、LQR实验、GLIBC2.38本地交叉基线/模型/工具链保持；STM32任务仍暂停。

### 前轮采用约束与前向恢复（2026-09-14，v7历史）

- 用户已授权“你修改吧”，按第七轮意见实施；修改前 `git fetch origin` 成功，
  `c440abf26eb159e8ec742dfcd3663fb9d2db701b` 与 origin/main 一致 0/0。最终提交状态以实际 HEAD/远端为准。
- 最新分享 `https://chatgpt.com/share/6aa7b111-c61c-83ee-9d3d-437a65ddce42` 已读取完整正文。
  用户原 ZIP `XT-STCAR_c440abf_review_reproduction.zip` 的三个文件已在 work 隔离复算；
  SHA256 `4d22c46cdf693590cb2739ac9718aeb214d00831d0a50cc92370d614a47f3010`。原 ZIP 与规则 PDF 不修改、不提交。
  分享把 246 组称为合法目标组合不够精确：它们只满足绝对限值，不全满足该拍加减速/转向变化限制，
  也不是完整 Rust 认证。本轮 Rust fixture 单独检查正常运动合法子集，当前真实历史 V/K 不清零。
- `control_execution::certify` 改为结构化 `Result`，保留原早退顺序和独立最终证书；首因包含障碍索引、符号余量、
  源/规划/lease/采用窗、历史与最终 V/K 和停车矩形。Worker 与 observed_plan 用 Arc 分享诊断，异步报告保留有界窗口。
  最终仍只能执行 `ControlPoll.command`，不把被拒绝 report 当执行输出；默认无逐拍落盘。
  新增诊断 JSON 全默认/None 省略，缺失=对应 Rust 默认值，有值完整保留；默认PP事件输出9行/99310字节，17帧首失败窗口保持。
- `robot-core/admission.rs` 共享原源车体系停车矩形，`runner/control_admission.rs` 从真实历史与 runtime lease 每源准备一次约束。
  当前候选前置检查保留全部历史峰值；下一阶段用 3 个固定采用延迟、实际采集相位和本次源年龄预测正常减速后备。
  预测最多 64 个规划时刻、2 秒，额度不足或未完整停稳直接拒绝；不改原源期限、不缩小当前停车覆盖。
  这只是有限模型前瞻，不是任意采用延迟序列或递归可行性证明；最终连续采用窗证书和新观测不可省略。
- `navigation/recovery.rs` 在普通前向 OpenEmpty 后使用剩余节点预算做连续胶囊恢复；完整外接圆、原净空、地图和半平面保持。
  连续域额外预留 .5×min(vmax,aT) 最小起步候选所需停车距离，修复中间版本的原双锥回归；原实际停车门不变。
  `primitive_envelope.rs` 按 v/k 饱和时刻分段，解析朝向/路程、固定至多 12 个位置积分子段与导数余项。
  胶囊使用真实弧长的 K L²/8 与累计位置界；误差经过节点、终端两段、PassThrough、候选下一拍与缓存续段重验。
  恢复末端以真实积分姿态连同误差在原到达容差内验收，不传送到目标。普通模式数值路径、v6 终端预留与 Q/R 保持。
  普通/恢复/后续路段共享原节点总账，节点/终端预算分别诊断；这些计数不是 CPU 时间。
- 本轮入口 [采用约束与前向恢复](docs/采用约束与前向恢复.md)，构建和验证证据统一 `docs/motion-v7-*`。
  同步 PP58.8秒/11.508841m、LQR54.3秒/10.249675m 保持；默认异步 PP73.863秒/14.173789m、LQR89.069秒/15.807183m，
  扰动异步 PP56.849秒/11.124416m、LQR53.849秒/10.217582m 均完成。4场最终证书拒绝0；默认/扰动每tracker曲率slew拒绝仍5/2，
  与实际Stop计数分开。原3000ms斑马线、300ms绿灯、整车终点和终态v/k0门保持；默认时序PP较v6慢16.2秒，不宣称全面改善。
  原停稳源与新增中间灯前源的8项恢复专项通过，265/12节点，42.344494/1.876981µm位置界。
  中间尝试的首次恢复停稳gate会使LQR79.263秒失败，已撤去；起步预留自身通过原双锥回归。
- PP 继续默认，LQR 仍实验。GLIBC 2.38、工具链、模型均不变；暂不接车、不读视频、不执行 RISC-V 目标。

### 前轮恢复预算与完整异步验证（2026-09-13，v6历史）

- 用户在第六轮复核后明确“你改吧”，已实施。修改前fetch成功，`b9f75c7427411c12285411efa4af793fa167b8c7`
  与origin/main一致0/0；提交/推送状态始终以当前HEAD与远端核对，不把构建基线当本轮提交号。
- 第六份 `https://chatgpt.com/share/6aa6b2fa-86bc-83ee-8424-04afe06f7d3c` 正文已完整读取；
  通过服务器hydration同时取得上一轮正文。v5“未取得正文”仅描述当时情况，当前不再阻塞。
  根目录 `XT-STCAR_b9f75c7_review_reproduction.zip` SHA256为
  `4cd3aa49b07b455c7841576cb827a35f786d4c20e35e199c6fb7654e86e569dd`，仅含README/Python几何及预算复算/JSON，
  不当成Rust补丁或完整异步验收。只在隔离work副本执行；原ZIP与规则PDF不修改、不提交。旧f83898a ZIP现已不在根目录，不恢复。
- `navigation/terminal.rs`在既有256求解/1024迭代/65536预扣样本内，为已认证短连接接续保留一次求解机会；
  普通候选触及暂时上限记cold deferral，不锁死全局预算。恢复前解除预留，普通已认证最佳仍优先，原rollout不重跑。
  前置route/lattice开销已计入；不足完整预留时只保留剩余额度，仍允许完整认证成功后执行，不把不足最坏估算当物理不可达。
  原42.2秒21/41/61/81档×镜像均Drive；21档51541样本保持，41档以上63799样本，新障碍仍拒绝旧初值。
  42.3秒真实lattice的194求解/604迭代/18087样本保持；该帧未同时触发短连接恢复，不能混写证据。
- 同步完整比赛PP58.8秒/11.508841米、LQR54.3秒/10.249675米，与v5共有报告字段逐值一致。
  PP保留2次可恢复Stop；LQR权重、默认跟踪器、原期限/碰撞/容差均未调整。
- `async_simulation.rs`新增完整RGB/雷达/同源位姿→真实AutonomyWorker→certify→poll.command→渐变转弯plant。
  默认100ms采样、60/80ms源延迟、20ms输出、3/7/9ms采用偏移；PP57.663秒完成，LQR29.463秒失败，终态v/k均0。
  LQR首次16.863秒证书拒绝后曾恢复，19.263秒lattice预算耗尽，19.469秒起最后停车；随后无前向路线，
  29.463秒“ordinary stop exceeded 10 seconds”锁存Fault。不能把第一张拒绝直接当最终唯一根因。
  此绕桶过程未启用新增短连接预留。保留失败且LQR仍实验；不放宽证书/延长10秒期限/加大全局预算来制造完成。
  下一轮若继续此问题，先细分证书拒绝原因，核对规划与可采用停车空间，单独复现停住后的前向搜索；不直接调Q/R。
- `control_diagnostics.rs`显式开启主机计时；排队、投影、processor、纯导航、终端求解、证书、发布/观察分开记录。
  原逻辑source/planned/lease不受Instant影响。默认不读钟、不逐tick写盘；共享Arc保留最新发布尝试（包括未采用/故障）。
  `observed_plan.report`与`latest.command`都只是诊断，车辆只能使用最终`ControlPoll.command`。
  调度钩子仅离线故障注入，在worker侧/锁外执行；默认关闭，不在poll里回调。
- 完整异步功能测试等待后台时冻结逻辑钟，不能用于deadline结论；`async_deadlines.rs`在后台阶段挂起时持续推进
  poll和车辆，覆盖过窗、过期、latest替换、revision/曲率变化率与完整停车。`async_host_clock`另跑一次持续Instant时钟短转弯/断流，
  使用独立短场景（斑马线尚未可见），记录真实主机poll间隔与各段耗时，不声称完整比赛实时性或目标WCET。
- 本轮入口 [恢复预算与完整异步验证](docs/恢复预算与完整异步验证.md)，机器证据统一`docs/motion-v6-*`；
  当前测试/构建/源哈希/包状态分别以validation/build/delivery报告为准，旧轮报告保留历史。
  正式主机同步顺序profile：PP平均1606.468→1612.415ms，LQR1480.943→1480.708ms，未见明显总耗时变化。
  持续Instant短场景23源/119次转弯Drive、poll最大间隔21ms，末源2200ms于2461ms轮询Stop，2687ms完全停稳回中。
  暂不接车；未改变工具链/模型/GLIBC基线，未执行RISC-V目标。

### 前轮终端接续、计算额度与异步停车空间修正（2026-09-13，v5历史）

- 用户要求根据第五份运动意见和工程根目录复现ZIP继续修正，拟定计划后直接实施。修改前网络恢复后fetch成功，
  `f83898a050649e03f25ad51b9c18c0bb12c743b2` 与origin/main一致0/0；最终提交/推送状态以HEAD和远端核对。
- 分享链接 `https://chatgpt.com/share/6aa6a3ce-6304-83ee-b875-2298fc1e846` 网络恢复后仍由服务器返回
  “Can't load shared conversation”，未读到第五份完整正文。已请求新的分享/文字，本轮依据已核实本地ZIP实施，不能声称逐条读完分享。
  `XT-STCAR_f83898a_review_reproduction.zip`只有README、Python几何探针和两个JSON，无Rust补丁/原点云/整场验证。
  原ZIP和比赛PDF不修改、不暂存；ZIP哈希与来源见 `docs/motion-v5-before.json`。
- 原提交隔离重放额外仅加三条内存记录，取得42.1/42.2/42.3秒完整输入与每帧360世界障碍，
  整场和三帧导航诊断与v4逐值一致，证据 `docs/motion-v5-original-input-window.json`。
  原42.2秒44个冷候选未被八次迭代等长双段求解认证；几何失败不能证明不可达。
  下一拍新路线较旧剩余路线长3.523601m。ZIP35:65几何见证须经Rust完整Grid/车体/运动/灯前边界重新验证。
- 新 `robot-core/src/navigation/terminal.rs` 提取原单/双段渐变曲率求解，缓存已认证双段的剩余段比与数值初值，
  同目标/朝向时每次重求解并检查当前网格。原冷候选全部失败时，恢复轮按接近已认证首段曲率顺序，采用首个完整认证动作；
  原冷候选有解时保留原评分选择。缓存不是执行凭证，不改变已采用命令状态，不跳过任何安全门。
- 整次导航共享256次域内求解、1024次迭代、65536份预扣采样额度，单次最多8轮；lattice原节点上限另保留，
  终端额度已耗尽则结束不能产生结果的剩余lattice搜索。已有完整认证候选可用，未认证不得因为预算放行。
  `terminal_work.solver_iteration_exhaustions`区分单次迭代用尽；`budget_exhausted`为整拍共享额度不足。
  `primitive_samples`为primitive前预扣额度，可能大于障碍早退时实际执行样本，非CPU操作计数。
  `terminal_budget`表示受整拍额度限制而未继续认证的候选，不推断加额度即可行；`terminal_unreachable`是有限认证失败分类，非数学无解。
- 中间“所有恢复候选仍按旧评分选最好”消除了42.2秒Stop却使LQR退化79.7秒/15.057722m，已撤销。
  最终首认证接续方案：PP58.8秒/11.508841m保持；LQR54.3秒/10.249675m，比v4快16.3秒、少3.190640m。
  LQR灯前26.2秒/4.390629m、5次路线生成、全场无非预期Stop；PP原两次短Stop保留。
  二者终速0，斑马线3000ms、绿灯300ms、最小锥桶间隙PP.242509m/LQR.273528m；无共享终端额度耗尽。
  中间失败与保留策略见 `motion-v5-terminal-experiments.json`，完整结果 `motion-v5-competition-comparison.json`。
- `runner/src/control_execution.rs`异步采用证书由全向圆换为源车体系有向矩形：V/K保留源到规划的实际采用历史上界，
  总弧长S=V*T+V²/(2b)，覆盖原lease再加一拍和完整刹停；横移min(S,KS²/2)、角点旋转min(RKS,2R)，
  KS>=π/2时额外覆盖后向S。地图和灯前半平面查四角，激光TF和障碍圆盘半径保留，相切拒绝。
  采用窗/命令序号/原源期限、正常速度和slew/侧向过渡检查保持；同步导航原紧急停车检查不缩小。
- 0.9m合成直道真实默认Worker：60/80ms源延迟、3/7/9ms采用抖动、23帧连续Drive，默认任务限速.18m/s，
  超过.17m/s并前进>.25m；最后源2200ms按2450ms原期限Stop并制动至0。另直接覆盖.30m/s直道证书和前墙拒绝。
  该宽度不是官方赛道测量。独立.5ms积分覆盖101种采用时刻×4轨迹、历史高曲率/Stop回中、大角度后向运动及TF/旋转边界。
  仍依赖静态障碍、无侧滑/超调和制动能力下界；完整异步比赛/实车/目标板时限未验。
- `runner/src/phase_statistics.rs`追加终端工作总数/每拍最大值与额度拒绝，固定内存/饱和计数，不影响控制和日志策略。
  新 `runner/examples/motion_profile.rs` 显式预热一次+重复1..20次整场simulate主机墙钟，含合成传感器、控制、车辆和sink遥测，
  报告序列化不计入；不是单solver时间、每tick延迟或板卡WCET。基线正式测量恢复原simulation.rs字节，仅加入同一profile示例。
- 本轮说明 [终端连接延续与异步停车包络](docs/终端连接延续与异步停车包络.md)；README模块结构/运动控制和命令TXT同步。
  当前构建、测试、包哈希以 `motion-v5-validation.json`、`motion-v5-build.json`、`motion-v5-delivery.json` 为准；
  v4及之前证据仅保留历史，不能沿用旧二进制哈希。GLIBC2.38仅本地交叉目标，默认PP、LQR实验，Q/R和比赛门限保持。
  暂不接车、不读视频、不执行RISC-V目标，STM32固件任务暂停。工具链/环境/模型/产物不进Git。

### 前轮任务交接、阶段统计与异步运动修正（2026-09-11，v4历史）

- 修改前 fetch 确认 `aac7421` 与 origin/main 一致（0/0）。用户已授权修正第四份运动控制意见，
  继续直接 main 验证、提交、推送；最终同步状态以 Git HEAD/origin/main 核对。
  本轮说明为[任务交接与异步运动修正](docs/任务交接与异步运动修正.md)，证据统一使用 `docs/motion-v4-*`。
- `PassThrough.admission_radius_m` 由 Mission 传真实 `goal_tolerance_m`，修复 .065 m 任务验收与
  .045 m 导航停车容差混用导致的提前减速过晚。新增半径/距入圈余量诊断；从较早帧连续减速，
  已来不及满足正常减速约束仍 Stop，不能通过放宽 epsilon 修复。原 LQR 22.5 s 空速度区间已核验为真实遗漏。
- 缓存参考偏离采用完整车体对应四角的最大位移，同时纳入位置和航向；不只查中心横向偏差。
  保留原目标容差一半的重规划数值门限和原 Q/R。对必停定向目标，若已有单弧求解器未认证而双弧可认证，
  新候选在原安全门通过后还需验证完整下一周期仍有短连接/严格入圈；这是有界连接族检查，不是全局可达证明。
  每拍最多165次状态查询，每次最多单/双弧各8迭代，有短Vec分配，不重跑lattice；目标板时限未验。
  既有变动点云测试的拍末值Euler模型改为独立1ms连续积分；关闭新guard仍复现原失败，原场景/停车/碰撞门限保留。
- 最终完整合成比赛 PP58.8秒/11.508841m、LQR70.6秒/13.440314m均完成，终速0；
  斑马线3000ms、绿灯确认300ms。对比aac7421，PP慢3.2秒、多走.38345m，LQR快13.3秒、少走2.34996m。
  PP最小锥桶间隙.24251m、LQR.27353m；变化不全是改善。PP两次短Stop保留；LQR原交接空区间已消除，
  另在42.2s有一次可恢复终端门控Stop。原始阶段/候选数据见本轮competition-comparison，不能把候选拒绝数写成停车次数。
- `motion_transition::project_motion` 提供无堆分配、有时域/数值预算的分段车辆模型投影，
  区分速度/转向渐变与饱和点，解析积分航向和距离；独立精细积分回归不冒充真实反馈。
- 导航新增当前、选中候选和下一完整周期的停车余量/来源诊断。保留原硬停车包络与候选评分。
  曾尝试按未来余量优先排序，但 PP 在 19 s 出现所有候选都切同一占用格角的回归，已撤销该排序。
  不是新扫描让当前格变为占用；旧网格独立复算同样拒绝。详见 `docs/motion-v4-stopping-experiment.json`。
- 新 `runner/src/phase_statistics.rs` 将阶段实际时长/距离、路径生成/长度、非预期Stop次数/命令时长和
  固定导航/候选原因累计进 `SimulationSummary.statistics`；每阶段仅留最近16次路线事件与完整计数。
  最后制动单列 `final_braking` 并与总时长/距离对账。无新增逐tick落盘，不降控制/感知频率。
- 新 `runner/src/control_execution.rs` 固定128条已采用命令变化历史；同命令poll不耗槽位，
  源时刻早于最近poll仍可查历史，历史不足返回 `HistoryUnavailable`。分段推算位姿、速度及曲率到规划时刻。
  `PlanningContext`/`tick_with_projection` 保留源测量用于任务、Safety和世界障碍投影，导航另用模型规划状态。
  后台证书约束采用时间窗、真实采用前提、原源期限、正常运动限幅、侧向峰值和静态停车空间；
  `poll` 不重做搜索/不等待锁。同源不续租、原期限先验、晚到Drive不能清Fault等约定保持。
  poll还按真实命令变化时间检查曲率slew，不能借用更长规划间隔。原始源期限受Mission/Nav/Safety最短限制。
  投影/证书共享原导航±1e-6测量边界，归一化仅用于模型副本；真实越界仍InvalidInput，命令上限仍严格。
  默认spawn已有空旷直路异步集成：10Hz采样、60/80ms延迟、50Hz输出，13份源快照Drive后按原期限Stop并减速到0；
  另有固定60ms源延迟+3~9ms采用抖动、窗口过期/前提变化等专项，不把冻结模拟时钟的线程测试写成目标耗时测量。
  证书有可能保守拒绝狭窄通道；未验证完整异步比赛，真实多传感器时钟、动态障碍与执行模型仍待车到后验证。
- `README.md` 与 `Mac与车端命令手册.txt` 同步模块和验证入口；新构建/包以
  `motion-v4-validation.json`、`motion-v4-build.json`、`motion-v4-delivery.json` 为准。
  v3/v2和更早数据保留历史，不能沿用其二进制哈希或测试数量当本轮实测。
- 当前仍暂不接车、不读视频、不执行RISC-V目标，GLIBC2.38仅本地基线。PP默认，LQR实验。
  工具链/环境/模型/产物不入Git；原比赛PDF不修改或暂存，STM32读取任务继续暂停。

### 前轮运动执行状态、过渡峰值、通过点与模拟积分修正（2026-09-10，v3历史）

- 修改前 fetch 已确认 `9b59125` 与 origin/main 一致；继续按用户授权直接 main 开发/验证/提交/推送。
  本轮已完成下述主机验证与交叉链接，提交/推送状态以 Git HEAD/origin/main 核对；不沿用 v2 的数字或哈希。
  该轮说明为[运动执行与通过点修正](docs/运动执行与通过点修正.md)，历史证据使用 `docs/motion-v3-*`。
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
  该轮仅改README和交接文档，未改变代码/配置。旧0f3ab64、motion-control及motion-v2构建证据保留历史，该轮证据使用motion-v3前缀。

### 当前代码：5 个 crate、2 个程序

| 模块 | 已实现 |
|---|---|
| `crates/vision` | 纯 Rust 模型契约、RGB/letterbox/NCHW、阈值与坐标解码；road.rs 斑马线/灯色/带色锥桶及地面投影，ground_markers.rs 显式实验地面标记 |
| `crates/app` → `xt-stcar` | self-check/preprocess/replay/infer；原生 ORT C API 动态加载、模型来源及元数据验证、常驻 Session；Python 参考后端须显式选择；file_io 提供两套 CLI 共用文件边界 |
| `crates/robot-core` | 强类型传感器语义、frame/时间/单位校验、急停/deadman/超时/限值状态机、仅记录的 MotionSink；底盘/WIT IMU/N10 协议与标定表；固定及在线任务、LocalWorld有界身份/几何/面积证明、Stop/PassThrough、灯前半平面、整圈扫描、ICP、执行曲率估计、解析过渡峰值、车模型导航/局部参考/PP与实验LQR |
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

- 历史v9：371 Rust/2跟踪example/5矩阵example/35 Python交付通过，2项按设计忽略；fmt/clippy/双交叉链接与ELF通过。
  108份编译输入、GLIBC2.38基线/最高实际2.34。该轮结果与本地包为 `motion-v9-validation/build/delivery`；当前见开头在线元素驱动快照。
  8组时序性能门全部通过；原生模型推理和目标执行未验证，以下旧轮记录仅为历史。

- 历史 v7：Rust常规350通过/0失败/2忽略，跟踪example2项、Python交付35项通过；fmt、all-targets clippy -D warnings、双RISC-V交叉链接通过。
  98份源码/清单/嵌入fixture记录于 `docs/motion-v7-build.json`；基线GLIBC2.38、最高实际引用2.34。
  app SHA256 `ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9`，1,217,056字节，需libc；
  robot SHA256 `3b30c966de36d4a28c8054794dbf1119cc92bda7930e3c7de5ae805033a5f673`，2,020,984字节，需libm/libc。
  本轮精确功能/性能范围与本地包分别见 `motion-v7-validation.json`、`motion-v7-delivery.json`；原生ORT和模型执行未重跑。
  正式顺序同步profile：PP平均1609.081→1621.487ms（+0.77%），LQR1494.640→1484.827ms（−0.66%），各预热1次/测3次；不作目标WCET结论。
  默认完整异步导航阶段均值/峰值 PP4.557/11.875ms、LQR4.809/20.414ms；含非Drive计划，开启计时后功能字段逐值一致。
  持续Instant短场23源/119转弯Drive、poll最大26ms；末源2201、原期限2451、2460 Stop、2687完全停稳回中，无碰撞越界。

- 历史v6：Rust常规322通过/0失败/2忽略（原生ORT与独立证书性能样例未重跑），跟踪example2项、交付34项通过；
  fmt、全目标clippy -D warnings、两个RISC-V交叉链接通过，89份Rust源码/清单哈希核对。
  构建、完整同步/异步成功与失败及主机时钟观察见 `docs/motion-v6-validation.json`；
  新增预算预留和可选内存诊断，默认PP保持。实际ELF、全部Rust源码哈希与包核验分别见v6 build/elf/delivery，
  完整异步LQR尚未通过，不能将同步或构建通过写成实车可参赛。

- 历史v5：Rust常规305通过、0失败，默认忽略原生ORT与显式性能基准各1项，性能基准另跑1项通过；
  跟踪example 2项、Python交付34项通过，fmt/all-targets clippy -D warnings/双RISC-V交叉链接通过，82份源码哈希核对。
  app哈希 `ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9`；
  robot哈希 `f286246369cd068271ba6bd4c987b12a40aecade7c458ebda9c62f965c2f8b9f`，1,975,968字节。
  GLIBC基线2.38、实际最高引用2.34，动态依赖仍为app/libc，robot/libm+libc。
  主机release整场预热1次再各3次：PP平均1.621→1.614s；LQR平均1.921→1.496s（任务变短，非每tick加速证明）。
  证书360/2048点各2000样本，平均8.862/19.234µs、峰59.791/56.875µs；默认Worker23帧平均2.626ms、峰3.790ms。
  都是本机采样，不是目标板WCET；资料入口为motion-v5-validation/host-profile/async-validation/build/delivery。

- 历史 v4：Rust 常规 294 通过、0 失败，原生 ORT opt-in 1 项忽略；跟踪 example 2 项、Python 交付 34 项通过。
  fmt、all-targets clippy -D warnings 和两个 RISC-V 交叉链接通过。该历史阶段入口为 `docs/motion-v4-validation.json`；本轮见开头在线元素驱动快照。
  两者实际最高 GLIBC 引用 2.34，构建基线 2.38；robot 需要 libm 与 libc，xt-stcar 需要 libc。
  构建源码哈希已核对，该轮二进制哈希见 `motion-v4-build.json` 和两份 ELF 报告。
  模型测试套件和原生 ORT 推理未重跑；打包另做模型格式、来源与包内文件校验，包状态见 `motion-v4-delivery.json`。
  本地 core 包 54 个文件、含模型包 58 个文件均已核验；包内文档、两个 ELF 及 80 份源码哈希与最终文件一致。
  两包仅在本地 dist，未上传或执行车辆；Git 仅同步源码、脚本、文档和验证记录。

- 前轮 v3 运动执行修正（历史）：Rust 常规 266 通过、0 失败、原生 ORT opt-in 1 项忽略；跟踪 example 2 项、Python 交付 34 项通过。
  fmt、all-targets clippy -D warnings、两个 RISC-V 交叉链接及 PP/LQR 完整场景复验通过。
  该轮汇总入口为 `docs/motion-v3-validation.json`，构建与交付为 `docs/motion-v3-build.json`、`docs/motion-v3-delivery.json`。
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
  加载器 `/lib/ld-linux-riscv64-lp64d.so.1`。v2最高 GLIBC 引用 2.34；当前实际引用和依赖以本轮 online-mission 最终构建/ELF报告为准，不复用历史哈希。
  前轮 robot 程序额外需要 `libm.so.6`，不能声称两个程序都只依赖 libc。
- **当前构建证据见本轮快照；`docs/motion-v9-*`、`docs/motion-v8-*`、`docs/motion-v7-*`、`docs/motion-v6-*`、`docs/motion-v5-*`、`docs/motion-v4-*`、`docs/motion-v3-*`、`docs/motion-v2-*`、`docs/motion-control-*`、[Rust比赛自主闭环](docs/Rust比赛自主闭环.md) 与 `docs/competition-*`保留历史证据。**
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
