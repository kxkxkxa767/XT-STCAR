# 省赛 bag 组件清理

2026-11-13，Codex（GPT-6）。用户回来后已实际完成清理：10个bag包、ros-base聚合包及未使用的cartographer-ros共12包移除。ROS2 launch/run正常，route-engine无缺失库；本页下方保留准备时的包清单。

`规则/赛项三-RISC-V轻量无人车赛补充细则.pdf` 第3页二8禁止使用ROS1与bag工具，三3进一步规定检录检查环境中是否存在这些工具。ROS2自带的bag没有明文例外。文件没有指定Linux发行版；已有Bianbu2.2、MUSE Pi Pro和ROS2 Humble不因此需要更换。

11月12日只读安装记录 `work/provincial-rules-20261112/platform/live-audit.json` 确认以下10包存在，版本均为0.15.13-1jammy：

```text
ros-humble-ros2bag
ros-humble-rosbag2
ros-humble-rosbag2-compression
ros-humble-rosbag2-compression-zstd
ros-humble-rosbag2-cpp
ros-humble-rosbag2-interfaces
ros-humble-rosbag2-py
ros-humble-rosbag2-storage
ros-humble-rosbag2-storage-default-plugins
ros-humble-rosbag2-transport
```

实际赛车通过Rust ROS2节点、rclrs及Fast DDS运行。`route-engine/src`、`scripts`、`ros2`、`config`的运行代码未发现bag调用或依赖；继续使用既有JSONL、IMU/雷达日志与视觉图片，无需替换控制通信链路。原始安装记录的检查范围内未检出ROS1。

用户回来唤醒后，先将 `scripts/remove-bag-tools.py` 上传到车端工程的 `scripts/` 目录，再运行：

```sh
python3 scripts/remove-bag-tools.py --allow-dependent ros-humble-cartographer-ros
sudo python3 scripts/remove-bag-tools.py --apply --allow-dependent ros-humble-cartographer-ros
```

脚本读取届时实际安装包名，先保存apt卸载计划，再移除bag包。ROS的ros-base/desktop等聚合包可能因依赖bag而随之移除；它们不是运行库。若计划包含其他额外组件，脚本不执行卸载，先处理依赖。整个过程不执行autoremove，也不启动车辆。

清理后用包清单、`ros2 --help`及已知ROS安装前缀确认bag命令与包消失，并确认原有ROS2赛车launch依赖仍在。若存在apt不拥有的手工安装副本，再按其准确安装位置处理；不要通配删除整个ROS目录。`/opt/ros/humble`和`/opt/bros/humble`都在已知检查范围内。只有实际清理并完成车端核对后，才把状态记成已整改。

本文件只处理此次授权的软件环境问题；现场硬件与其他赛事条款仍按原规则检录。

实际apt计划额外依赖为ros-humble-cartographer-ros，确认比赛源码未使用它后通过精确的 --allow-dependent 参数放行；不忽略其他未知依赖。用户执行sudo卸载，随后Codex清除了dpkg不拥有的rosbag2_py缓存空目录。车端核对证据：`work/codex-y90-deploy-20261113/bag-cleanup-verification.json`。未执行autoremove或实际发车。
