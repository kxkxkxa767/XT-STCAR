# y85／192：ROS2 + Rust 迁移复核入口

用户要求交接只保留关键内容，具体实现由Claude自行查阅。

- 正式车载仍y83／190；算法保持已建未跑的y84／191。新候选y85／192采用Humble＋rclrs0.8.0＋FastDDS，未升级OS、未卸载软件、未实车发车。省赛新赛道仿真仍暂停。
- ROS2构建实际运行 `xt_stcar_race`、`xt_stcar_chassis`、`xt_stcar_lidar`。数据和指令走ROS2话题；原bridge不执行，保留原Guard/N10/PWM/标定实现。独立硬件进程和EOF连接保留故障隔离，EOF管道不传业务数据。
- 原20ms控制节拍、新扫描唤醒及绕桶/刹车算法不改。订阅统一采用 `worker.create_subscription` 在 `WaitSet::wait` 内同步回调（消除 `node.create_subscription` 的 `BoxFuture`/`Arc<Task>` 分配与二次线程唤醒）；硬件节点直接发布 `BestEffort` 状态/扫描（移除冗余 `ros2-publish` 线程与 `status_backpressure` 断车分支）；启动环境改为 `ROS_DOMAIN_ID=217` 保留 Fast DDS 默认 SHM 共享内存传输（修复 Humble 下 `ROS_LOCALHOST_ONLY=1` 禁用 SHM 退化为 UDP 回环的问题）；修复 `build-ros2-race.py` 复制 `payload/route-engine` 时缺少 `0755` 执行权限导致 `ros2 launch` 找不到可执行文件的问题。

主要入口：
- `route-engine/src/race.rs`、`race_ros2.rs`、`race_hardware.rs`、Cargo配置。
- `scripts/race-trial.py`只增加ROS环境启动包装；`ros2/xt_stcar_race/`是标准ROS包。
- `scripts/ros2-generate-rust.py`、`build-ros2-race.py`、`package-ros2-round.py`；现有工程内build-engine.py增加显式ros2 feature/profile，包装器核对产物。

验证：968参考回归、29Rust检查通过；默认及ROS2 clippy均32。RISC-V真实Humble链接、三节点launch、start never/stop通过。软件PTY（不是车串口）确认原Guard回执19.0ms、心跳失效约328ms中位，父连接关闭后退出，末20个UART包为中位。

通信时延优化结果（实车板端 `stcar-transport-bench` 实测）：
- 原始管道：128B@50Hz 平均 RTT 0.245ms（单向约 0.12ms），12KB@10Hz 平均 RTT 0.325ms。
- Codex 初版（`node.create_subscription` + `ROS_LOCALHOST_ONLY=1`）：Reliable 128B RTT 平均 2.914ms（p95 5.775ms，max 9.915ms，CPU 15.4%）；12KB RTT 平均 2.993ms。
- 优化后跨进程（`worker.create_subscription` + Fast DDS SHM）：Reliable 128B RTT 降至 1.614ms（p95 2.222ms，max 4.045ms，CPU 10.3%）；实际传感器/状态使用的 BestEffort 128B RTT 降至 1.459ms（单向约 0.73ms，p95 2.359ms，CPU 7.6%），12KB RTT 降至 1.864ms（单向约 0.93ms，p95 2.330ms）。同进程多节点 `intraprocess_delivery=FULL` 下 RTT 进一步降至 0.985ms（单向约 0.49ms）。

版本取舍：Humble在本板已有真实库并验证能用。Jazzy的rclcpp执行器改进、Kilted的rclpy事件执行器/Zenoh、Lyrical的C++EventsCBGExecutor/Buffer不自动改善当前Rust链；没有跨发行版上板排名。Humble默认同步发布和共享内存传输已存在，不能声称重新设置一次就提速。

证据/缓存：`work/ros2-migration-20261112/`，重点 `transport-benchmark-v2.log`、`guard-pty-run.log`、`launch-validation-v3.log`；v1基准预热不对等、launch v1/v2失败已排除并保留记录。目标库在sysroot/opt/ros/humble，Rust接口在interfaces。生成器提交：rosidl `02a3de88b75af9c8d4efe167db4b14504e81cb90`；rosidl_rust `19d57818dab3b51e418c0893b70b3a1a8b64c495`。

机读键 `y85_ros2_rust_hardware_migration_20261112`。最终源提交和包状态见共同记忆文末。ROS overlay附同提交源码归档。车端ros2bag/rosbag2仍在，环境合规问题未随迁移自动解决。

参考：[rclrs](https://github.com/ros2-rust/ros2_rust)、[Fast DDS Humble](https://github.com/ros2/rmw_fastrtps/blob/humble/README.md)、[Jazzy](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Releases/Release-Jazzy-Jalisco.rst)、[Kilted](https://github.com/ros2/ros2_documentation/blob/kilted/source/Releases/Release-Kilted-Kaiju.rst)、[Lyrical](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/Release-Lyrical-Luth.rst)。
