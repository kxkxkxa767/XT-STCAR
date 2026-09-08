# Rust 串口采集与回放

`crates/device-io/src/lib.rs` 负责底层串口，`crates/runner/src/capture.rs` 负责传感器记录。
当前支持 Linux 目标和 macOS 主机测试。测试只使用操作系统创建的伪终端（PTY），没有打开真实车端设备。

## 模块行为

`SerialPort::open` 要求调用者明确给出字符设备及 38400、115200 或 230400 波特率，不扫描或猜测端口。
采用安全 Rust 和锁定的 rustix 1.1.4：非阻塞打开、独占标记、8N1、raw、关闭软硬件流控；设置后读回
波特率、字符格式、输入/输出/本地模式及 VMIN/VTIME。Drop 尝试恢复原始 termios 并解除独占。
恢复在进程正常退出/返回错误时执行；强制终止进程或硬件断开时不能保证恢复成功。

`read_until` 的超时返回 `None`；`write_packet_until` 为整个包共用一个截止时间，处理短写和系统调用中断。
断开、I/O 错误或写超时会锁存故障，后续调用拒绝继续；不会自动重发部分写出的帧。
底层有写方法用于完整传输实现和 PTY 测试，但当前总调度没有向底盘发送运动指令的入口。
`robot-core::protocol::chassis::PacketWriter` 是通用 `Write` 适配器，本身没有截止时间；不能用它替代有界串口方法。

`serial-capture` 默认只输出计划：读配置、校验参数和输出路径，不打开设备、不创建或覆盖输出。
显式 `--execute` 才打开配置中的 tty、设置串口并采集。它读取 IMU/N10，不调用写方法，不发送设备命令。
打开真实 tty 仍会改变串口配置，可能影响调制解调器控制线；默认计划不产生这类操作。

## 配置与命令

| 配置 | 厂商参考端口 | 协议 / 波特率 |
|---|---|---|
| `config/serial-imu-capture.json` | `/dev/imu` | WIT 11 字节 / 115200 |
| `config/serial-n10-capture.json` | `/dev/laser` | N10 58 字节 / 230400 |

这些端口来自出厂 launch/源码，尚未在新车核实；应在取得设备身份后使用实际稳定路径。
配置严格拒绝未知字段，必须给出绝对 `/dev/...` 路径，禁止 `..`；持续时间 1–60000 ms，单次读等待
1–1000 ms，字节上限 1–524288。记录最多 10000 次读取事件加一个结束 Tick，达到时间、字节或记录
上限时正常完成，stdout 摘要用 `ended_by` 区分原因。

以下命令仅生成计划，可在 Mac 执行：

```bash
mkdir -p work
cargo run --locked --bin xt-stcar-robot -- serial-capture \
  --config config/serial-n10-capture.json --output work/n10-capture.jsonl
```

用户当前明确暂不接车，因此本轮没有为上述配置加 `--execute`。
执行模式先将记录保存在有上限的内存缓冲区，成功后用同目录临时文件原子替换目标；
采集失败不发布不完整记录，也不覆盖此前日志。输出不得指向配置、字符设备或目录。

## 时间与事件

采集启动后以同一个 `Instant` 纪元产生毫秒时间，原始数据为 `imu_bytes` 或 `n10_bytes`，
读等待超时产生 `tick`，结束也产生 `tick`。日志仅包含回放输入事件，统计摘要单独输出到 stdout。
捕获时间是主机接收时间，不能当作设备测量时刻。
采集文件没有 Arm、Start、Motion 等控制事件，因此单独回放保持 Disarmed；不会自动开始运动。

采集到 N10 字节后，可使用下列格式离线解析：

```bash
cargo run --locked --bin xt-stcar-robot -- replay \
  --events work/n10-capture.jsonl --config config/robot-n10-replay.json \
  --n10-config config/n10-replay.json --output work/n10-decoded.jsonl
```

若采集期间完全无数据，文件只有 Tick，应省略 `--n10-config`；原始协议配置参数仅在相应字节事件存在时接受。
IMU 对应 `--imu-config config/imu-replay.json` 与 `config/robot-imu-replay.json`。
不同采集文件各有自己的时钟纪元，不能直接拼接或冒充同步采集；多传感器实时调度与时钟对齐尚待实现。
