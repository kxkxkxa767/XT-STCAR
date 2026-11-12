# y85／192：ROS2 + Rust 迁移复核入口

用户要求交接只保留关键内容，具体实现由Claude自行查阅。

- 正式车载仍y83／190；算法保持已建未跑的y84／191。新候选y85／192采用Humble＋rclrs0.8.0＋FastDDS，未升级OS、未卸载软件、未实车发车。省赛新赛道仿真仍暂停。
- ROS2构建实际运行 `xt_stcar_race`、`xt_stcar_chassis`、`xt_stcar_lidar`。数据和指令走ROS2话题；原bridge不执行，保留原Guard/N10/PWM/标定实现。独立硬件进程和EOF连接保留故障隔离，EOF管道不传业务数据。
- 原20ms控制节拍、新扫描唤醒及绕桶/刹车算法不改。命令发布在独立线程，队列有界；executor长期spin一次。当前消息为String/JSON，不是零拷贝。IMU/视觉原流程保留。
- `--start ros`支持非交互launch等待start话题；`--start never`不会因start话题解锁。新增 `run/clock.json`记录真实程序CLOCK_MONOTONIC零点，分析视觉时不要把helper进程启动时刻当成该零点。

主要入口：
- `route-engine/src/race.rs`、`race_ros2.rs`、`race_hardware.rs`、Cargo配置。
- `scripts/race-trial.py`只增加ROS环境启动包装；`ros2/xt_stcar_race/`是标准ROS包。
- `scripts/ros2-generate-rust.py`、`build-ros2-race.py`、`package-ros2-round.py`；现有工程内build-engine.py增加显式ros2 feature/profile，包装器核对产物。

验证：968参考回归、29Rust检查通过；默认及ROS2 clippy均32。RISC-V真实Humble链接、三节点launch、start never/stop通过。软件PTY（不是车串口）确认原Guard心跳失效约325ms中位，父连接关闭后退出，末20个UART包为中位。没有完整ROS2闭环/实车圈时证明。

**性能不能写成不变。** 预热后的两进程Reliable echo：128B@50Hz平均RTT为管道0.245ms／ROS2 2.914ms；12KB@10Hz为0.325／2.993ms，均无丢样。这不是单向延迟或完整控制器负载，实际扫描/反馈为BestEffort。重点复核DDS时延、队列/回执、时间戳、退出和故障路径。

版本取舍：Humble在本板已有真实库并验证能用。Jazzy的rclcpp执行器改进、Kilted的rclpy事件执行器/Zenoh、Lyrical的C++EventsCBGExecutor/Buffer不自动改善当前Rust链；没有跨发行版上板排名。Humble默认同步发布和共享内存传输已存在，不能声称重新设置一次就提速。

证据/缓存：`work/ros2-migration-20261112/`，重点 `transport-benchmark-v2.log`、`guard-pty-run.log`、`launch-validation-v3.log`；v1基准预热不对等、launch v1/v2失败已排除并保留记录。目标库在sysroot/opt/ros/humble，Rust接口在interfaces。生成器提交：rosidl `02a3de88b75af9c8d4efe167db4b14504e81cb90`；rosidl_rust `19d57818dab3b51e418c0893b70b3a1a8b64c495`。

机读键 `y85_ros2_rust_hardware_migration_20261112`。最终源提交和包状态见共同记忆文末。ROS overlay附同提交源码归档。车端ros2bag/rosbag2仍在，环境合规问题未随迁移自动解决。

参考：[rclrs](https://github.com/ros2-rust/ros2_rust)、[Fast DDS Humble](https://github.com/ros2/rmw_fastrtps/blob/humble/README.md)、[Jazzy](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Releases/Release-Jazzy-Jalisco.rst)、[Kilted](https://github.com/ros2/ros2_documentation/blob/kilted/source/Releases/Release-Kilted-Kaiju.rst)、[Lyrical](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst)。
