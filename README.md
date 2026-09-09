# XT-STCAR

Muse Pi Pro / RISC-V 无人车工程：**Rust 为主，在 Mac 交叉编译，视觉使用 YOLO26n Detect。**
工程目录为 `/Users/yuhaojin/Documents/XT-STCAR`，源码仓库为
[github.com/kxkxkxa767/XT-STCAR](https://github.com/kxkxkxa767/XT-STCAR)。

目前有 **5 个 Rust crate、2 个可执行程序**。视觉预处理/后处理、原生推理调用、传感器协议、
串口传输、比赛状态机、道路识别、激光里程计、路径规划、底盘标定映射和调度均由 Rust 实现。Python 用于离线模型导出、校验与参考对照；
默认推理由 Rust 直接调用 ONNX Runtime C API，不启动 Python 进程，推理引擎仍是上游原生库。

用户当前明确“暂不接车”：本轮没有连接车辆、操作电机、升级车端 GLIBC 或读取视频。
串口在 Mac 的伪终端上验证；运动输出只记录和预览。
新增比赛模块与验证边界见 [Rust 比赛自主闭环](docs/Rust比赛自主闭环.md)，完整命令见 [Mac与车端命令手册.txt](Mac与车端命令手册.txt)。
前轮 [总体架构审查](docs/总体架构审查-2026-09-08.md) 保留作历史，交接状态见 [agent.md](agent.md)。
已完整核对用户提供的 10 页比赛规则初稿，见 [赛项三规则与工程差距](docs/赛项三规则与工程差距.md)。
日常直接在 `main` 开发和推送；修改前检查并拉取远端更新，提交信息简述本次改动，详见 [上传规范](上传规范.md)。

## 整车与硬件详细参数

核对日期：2026-09-08。以下是**资料标称与出厂代码参考**，当前尚未接车，不代表本车批次已验收。
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

本次只补文档，不读取视频、不连接设备，不将任何标称或示例数字写入当前比赛控制参数。

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
| [`crates/robot-core/src/autonomy.rs`](crates/robot-core/src/autonomy.rs) | 米制坐标、位姿质量、车体足迹和道路观察的共享类型 |
| [`crates/robot-core/src/mission.rs`](crates/robot-core/src/mission.rs) | 斑马线实际停止计时、锥桶顺序、灯前全车停车/绿灯确认、终点与故障锁存 |
| [`crates/robot-core/src/scan.rs`](crates/robot-core/src/scan.rs) | N10 整圈组帧、覆盖/盲区/时间检查；不同于旧局部包回放 |
| [`crates/robot-core/src/localization.rs`](crates/robot-core/src/localization.rs) | 有界 ICP 激光里程计；退化/跳变/过期门控，不假造编码器 |
| [`crates/robot-core/src/navigation.rs`](crates/robot-core/src/navigation.rs) | 带车体和转弯约束的路径搜索、跟踪、障碍/制动检查及停车朝向 |
| [`crates/robot-core/src/tracking.rs`](crates/robot-core/src/tracking.rs) | PathTracker 接口、默认 Pure Pursuit、实验性曲率前馈 + LQR；只给导航期望曲率 |
| [`crates/robot-core/examples/tracking_comparison.rs`](crates/robot-core/examples/tracking_comparison.rs) | 直线/弯道的 Rust 跟踪 A/B 实验，输出误差与合成转向响应指标 |
| [`crates/runner/src/laser_pose.rs`](crates/runner/src/laser_pose.rs) | 整圈检查、雷达到车体外参与 ICP 桥接，保留源时刻 |
| [`crates/runner/src/perception.rs`](crates/runner/src/perception.rs) | 同图 RGB + 常驻原生 YOLO + RoadDetector；后台感知只留最新待处理帧 |
| [`crates/runner/src/autonomy.rs`](crates/runner/src/autonomy.rs) | 位姿/雷达/道路观察校验 → 任务 → 导航 → 安全控制器 |
| [`crates/runner/src/control_runtime.rs`](crates/runner/src/control_runtime.rs) | 后台规划、最新快照队列、独立周期轮询和源时间命令看门狗 |
| [`crates/runner/src/simulation.rs`](crates/runner/src/simulation.rs) | RGB/雷达/位姿反馈与有加减速车辆模型的合成闭环，掉线注入和碰撞检查 |
| [`crates/runner/examples/motion_comparison.rs`](crates/runner/examples/motion_comparison.rs) | PP/LQR 进入同一完整模拟比赛，报告成功与失败结果 |
| [`crates/runner/src/autonomy_replay.rs`](crates/runner/src/autonomy_replay.rs) | 同步传感器快照回放，不接受人工 Motion；结束明确 Stop |
| [`crates/runner/src/telemetry.rs`](crates/runner/src/telemetry.rs) | 有界内存事件日志；调试预算耗尽不影响比赛控制 |

原有诊断流为：文件回放 → 传感器/视觉模块 → 安全状态机 → 可选 PWM 映射校验 → 运动记录/预览日志。
新增自主流为：RGB/雷达/位姿 → 道路识别 → 比赛任务 → 路径规划/跟踪 → 安全控制 → 车辆模型 → 下一帧反馈。
独立串口采集先生成可回放的原始字节文件；采集没有 Arm/Start/Motion 事件。
目前没有把采集与电机发送接成实时闭环。

## 运动控制

已对照[用户分享的算法建议](https://chatgpt.com/share/6aa0bf59-c8f8-83ee-b001-13dd5945ce14)审查代码。
当前采用前进车辆运动学：比赛任务给目标/限速 → A* 连通与带航向/曲率的车模型路线 → PathTracker →
候选轨迹预测、碰撞和制动检查 → 安全状态机。默认 **Pure Pursuit** 保留，新增 **曲率前馈 + LQR** 供离线比较。
不是单个 PID 直接把图像误差变成舵机 PWM，也没有实现 MPC 求解器。

`tracking.rs` 只输出期望曲率；加减速、横向加速度、曲率及其变化率、完整车体和停车可达域仍由 `navigation.rs` 检查。
两种跟踪器均走同一安全通路。命令单位为线速度 m/s、曲率 m⁻¹，模型 `yaw_rate = speed × curvature`；
转向正方向为左转。静态 PWM 映射在 `protocol/calibrated_chassis.rs`，不把厂商 Twist 的经验系数当成物理单位。

LQR 同时使用相对真实参考路径的横向和航向误差，以相邻路径几何作曲率前馈。
这是连续时间、固定正速度的直线附近误差模型，低速回退 PP，异常输入/超出误差域请求 Stop；
它没有证明在弯道、执行器延迟和所有离散周期下优于 PP。旧 JSON 省略 `navigation.tracking` 时仍选 PP。
选择方法、模型假设和逐项评估见 [运动控制设计与对比](docs/运动控制设计与对比.md)。

```bash
# Mac：依次为跟踪层实验、完整比赛对比；均无设备读写
cargo run --release --locked --offline -p xt-stcar-robot-core --example tracking_comparison
cargo run --release --locked --offline -p xt-stcar-robot-runner --example motion_comparison
```

本轮跟踪层16组全部进入终点容差，但当前LQR权重的横向RMS高于PP；完整比赛中PP仍55.3秒完成，
LQR绕锥桶阶段停住并在27.9秒触发普通停车超时。因此 **LQR尚未通过全场验收，不用于替换默认算法**。
原始结果和条件见上面的运动控制说明；模拟秒数不代表板卡计算耗时。

当前激光定位是 ICP，**尚无 EKF/ESKF、速度 PI 或真实执行器闭环**。没有轮编码器，不能把低频激光速度重复读取当成高速轮速；
后续先补传感器共同时间轴、扫描去畸变、IMU 标定/融合，再评估前馈表加限幅抗饱和 PI。
舵机标称 `0.16～0.18 s/60°` 不直接等于前轮转向响应；轴距、转向曲率/PWM、制动能力与 MCU 断链停车仍须实测。
Stop 是停止请求，车辆模型继续减速并逐步回中；物理 ESC 中位是否制动尚未验证。
导航保存的是上一目标曲率，尚无真实转向反馈；侧向加速度检查约束候选 `v²|curvature|`，
不能据此声称执行器动态过渡的实际侧向加速度已受实测保证。保守停车圆盘覆盖中间转向，但仍依赖实际制动能力达到配置假设。

## 配置与样例索引

| 位置 | 用途 |
|---|---|
| `config/yolo26n.json` | 模型尺寸、输出契约、检测阈值 |
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
| `scripts/build-riscv.sh`、`inspect_elf.py` | fmt/test/clippy、两程序交叉构建、独立 ELF/GLIBC 报告和源码哈希 |
| `scripts/delivery.py`、`package.sh`、`upload.sh` | 白名单打包与哈希核验；上传默认 dry-run，显式新账号/IP/release 目录 |
| `scripts/check-vehicle.sh` | 车端系统/设备/服务的只读检查脚本，不启动驱动或运动 |
| `target/riscv64gc-unknown-linux-gnu/release/` | 两个目标程序与 build/ELF 证据（本地产物，不进 Git） |
| `dist/` | core 或含模型的独立部署包（不进 Git） |

本轮209项Rust常规测试、2项跟踪基准测试、34项交付测试通过，两个RISC-V程序交叉链接通过；
默认PP完成合成比赛，LQR的失败结果一并保留，详情见 [运动控制验证汇总](docs/motion-control-validation.json)。
模型/原生推理模块未改，本轮不重复运行其验证；前轮1项原生ORT、48项模型测试记录见 [历史比赛验证](docs/competition-validation.json)。
构建脚本完整执行 fmt、主机测试、clippy `-D warnings`，再检查两个 RISC-V ELF 的架构/ABI/加载器/GLIBC。
ELF 检查核对动态加载表与 section 映射及版本需求，不验证所有机器指令或代替目标机运行。
交付包不含工具链、虚拟环境、Mac dylib 或缓存；带模型包另带 provenance 与模型许可。
源码仓库也排除模型、厂商大包、镜像、`target/`、`dist/`、`work/` 和缓存。

## 已完成与待接车工作

已完成上述 Rust 算法、线程接口和离线集成，不等于无人驾驶系统已实车验收。
比赛状态机、斑马线/灯色/锥桶视觉、全圈组帧、ICP、导航与模拟闭环已有代码；
还需要真实相机采集、共同时间轴/扫描去畸变、TF/初始定位、物理标定、定位融合、
必要的 ROS 2 接口及 MCU 反馈/watchdog。没有物理 MotionSink，也没有实时带动力 CLI。
厂商教程明确没有轮编码器；模拟反馈不能被写成实测轮速或定位精度。

比赛初稿没有禁止 Rust，也没有明文强制 ROS2，但明确禁止 ROS1 和 bag 工具。
普通 80 类 YOLO 仍只提供通用交通灯框，Rust 道路算法补充斑马线、灯色和锥桶候选；尚需真实数据标定和测量。

eMMC 5.1 策略以正常运行/比赛为先：计算与队列在内存，默认只在结束后保存阶段、故障和摘要，
不逐帧落盘、不每 tick 同步刷盘。需要诊断时可显式 trace/采集；日志预算不降低感知频率、不触发控制 Fault。
不会为了省擦写去修改车端系统或牺牲比赛性能。

车端真实推理需核实 RISC-V 标准 ORT C 库。官方 SpacemiT ORT/EP 2.0.6 已静态核验，仍未测试目标
`GetApi(22)`、模型算子或 EP 初始化。候选库要求 GLIBC 2.38，EP 另要求 GLIBCXX 3.4.32 / CXXABI 1.3.15。
相关说明见 [原生库核验](docs/SpacemiT原生运行库核验.md)。

进一步阅读：[机器人事件与状态机](docs/机器人模块.md)、[厂商协议适配](docs/厂商协议Rust适配.md)、
[N10 协议依据](docs/N10协议依据.md)、[底盘标定映射](docs/底盘标定映射.md)、[YOLO26 接口](docs/YOLO26接口.md)、
[部署包使用说明](docs/部署包使用说明.md)、[官方整车资料核对](docs/无人车2026资料核对.md)、[资料索引](资料/资料索引.md)。
