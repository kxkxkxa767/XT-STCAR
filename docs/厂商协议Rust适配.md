# 厂商协议 Rust 适配与修正

底盘帧编码、厂商映射预览、IMU 流解析已用安全 Rust 实现，代码在 `crates/robot-core/src/protocol/`。
它们接入 `xt-stcar-robot` 的本地命令与回放，不会打开串口、连接 ROS 或操作电机。

## 底盘

`PwmCommand` 接受带单位的电机/舵机 PWM，先检查厂商给出的 500–2500 µs 协议范围，再编码为
`AA motor_low motor_high servo_low servo_high sum 55`。校验为四个载荷字节求和模 256。
这个范围是协议边界，不是车辆的安全标定范围。

厂商不同入口用不同的 `FactoryProfile` 显式区分：

| profile | 电机转换 | 舵机转换 |
|---|---|---|
| navigation1300 | 1500 + linear × 100 | 1500 + angular × 1300 |
| navigation_one1200 | 1500 + linear × 100 | 1500 + angular × 1200 |
| teleop_pwm_degrees | linear 直接为微秒 | 2500 − angular × 2000 / 180，角度限 0–180 |

所有浮点输入先校验有限性和结果范围，再沿用厂商整数截断规则；不默默裁剪或饱和转换。
这些映射仅供参考预览，不能接收现有物理单位 `MotionIntent`，没有默认 profile，也不假定 1500 已实车验证为停车值。

```bash
cargo run --locked --bin xt-stcar-robot -- chassis-preview \
  --profile teleop_pwm_degrees --linear 1500 --angular 90
```

输出帧为 `AA DC 05 DC 05 C2 55`，JSON 明确标记 `physical_output_enabled=false`。
`PacketWriter<W: Write>` 可供未来串口适配使用，处理短写/Interrupted，写失败后锁存故障并拒绝继续发帧。
它本身不打开设备，也不能保证物理停车；真实端口需另实现配置成功校验、I/O 截止时间、控制源仲裁及独立 watchdog。

## IMU

明确实现与出厂解析代码相符的 WIT normal 11 字节 profile：帧头 0x55，0x51/52/53 分别为加速度、陀螺仪和欧拉角。
前 10 字节累加模 256 与最后一字节比较；依据 WIT 厂商协议文档的[串口协议章节](https://manuals.plus/m/204b2568de142446a45af012464a4a756aded3b6296fbea8db46f5d2ce8e2bce.pdf)。
该协议对应关系不等于实际 IMU 型号已确认。

- 保留跨读取的半帧；每块最多 4096 字节，解析缓存至多 11 字节，支持连续多帧和噪声重新同步。
- 校验失败时丢弃候选帧头重新搜索，同时清空待组合分量，保留错误统计。
- 有符号小端读取；加速度按 16 × 9.8 m/s²、角速度按 2000°/s 转 rad/s、欧拉角转单位四元数。
- 三类分量全部更新后才生成一个样本，生成后清空本轮组合；单独收到加速度不会重复发布旧陀螺仪和姿态。
- 保存帧首字节接收时间，限制分量年龄与相互时间差；样本时间取最早分量时间，避免半帧结束时间掩盖陈旧数据。
- 时间是本次回放的主机接收时间，不能等同传感器测量时刻；未引入未经核实的设备时钟换算。
- frame、加速度/陀螺仪偏移和时间限制都要求显式配置。没有厂商示例的硬编码偏移；零偏配置表示未校准，不表示已完成校准。

## 与机器人回放串联

```bash
cargo run --locked --bin xt-stcar-robot -- replay \
  --events examples/robot-imu-replay.jsonl \
  --config config/robot-imu-replay.json \
  --imu-config config/imu-replay.json
```

新事件格式：`{"at":0,"event":{"type":"imu_bytes","bytes":[85,81]}}`。
原始 IMU 输入必须给出 `--imu-config`，不允许与已有解码 IMU 输入混用；IMU frame 必须与机器人配置一致。
日志保留解码样本、坏校验/丢弃字节/过期分量统计。没有完整有效样本的块只作为 Tick 处理，不刷新传感器时间。
因此已有控制器的急停、deadman、命令/心跳/传感器超时持续生效；EOF 仍记录停止。

示例是合成数据：先发送一组分包 IMU，进入 Running，再输入坏帧，80 ms 时因 IMU 超时进入 Fault/Stop。
它只演示逻辑，不证明串口硬件、车辆运动或实时停车时延。

## 验证与剩余工作

新增测试覆盖固定底盘帧向量、三种映射区别、NaN/无穷/越界、短写/中断/错误锁存、所有单切点分包、粘包、符号及单位换算、校验错误恢复、缺失分量、陈旧半帧、时间差/回退、显式校准、CLI 预览和 IMU 超时回放。

仍需实物数据确认的项目：串口绑定/配置、实际 IMU 型号与协议、MCU 接收/反馈/watchdog、速度与转向标定。
没有把厂商转向系数直接用于 MotionIntent；未移植带旧标定的 C++ 节点，未改变雷达或导航算法，未读取视频。
