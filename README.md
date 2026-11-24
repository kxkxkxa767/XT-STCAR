# XT-STCAR · 1 号赛场版

本分支维护 Muse Pi Pro 比赛车的 **1 号程序**。Rust 控制器用雷达实测锥桶、挡板和终点直道开口，结合相机与 IMU 完成斑马线停车、绕桶、灯区停车和终点停车。实车雷达为 N10P，串口 460800 波特率，相机直接由 Rust 采集，红绿灯识别灯罩内的灯珠和过曝白芯周围的色晕。

| 构建 | 状态 |
| --- | --- |
| `c5e8c05` | 2026 年 11 月 22 日实跑版本 |
| `7b9e076` | 11 月 23 日已编译的灯区停车候选，待装车验证 |

新候选以终点直道内侧隔板开口作为灯架。车端估算前轮越过灯架上游 1.00 m 光电线的时刻，从这里开始倒计时。车头目标距灯架约 0.55 m，给车尾越过入口线预留约 0.06 m。车辆先在区内停稳，再根据两帧新鲜有效绿灯或过线倒计时放行，等灯设为 0 秒也须停稳。[1 号灯区控制](route-engine/src/maneuver_light.rs)

在车端 Bash 使用 `start_1_7 --show` 回读版本、Rust 视觉和等灯秒数。`start_1_7 --check-only` 在锁定状态下检查相机、雷达和控制程序。每轮现场准备完成并获得单独发车确认后，使用 `start_1_7` 运行一轮。

候选通过 75 项 Rust 单测和 970 项控制台测试。七个闭环工况均完成灯区停车后放行，其中五个正常完成全程。另两个在终点段触发 8 cm 保护，一个模型随后接触端板。

11 月 24 日的纯 Rust 视觉候选完成赛场与学校实拍回放。[灯珠验证结果](docs/rust-led-field-20261124.md)

2 号建图版由 `main` 维护。两个程序的比较、构建、仿真续跑和归档方式见项目首页。[主仓库 README](https://github.com/kxkxkxa767/XT-STCAR/blob/main/README.md) · [编号入口与安装](https://github.com/kxkxkxa767/XT-STCAR/blob/main/docs/numbered-race-start.md) · [本分支原版完整 README](https://github.com/kxkxkxa767/XT-STCAR/blob/41337e111e8dc0bcb442d306b1aa5cec9b353f85/README.md)
