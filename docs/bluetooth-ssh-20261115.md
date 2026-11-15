# 蓝牙 SSH 与现场连接

Mac 保留原来的 Wi-Fi 上网，车辆通信走蓝牙；不要求 Mac 同时连接两个热点。
车端比赛版本仍为 y94／201（0acb85a），本次没有改 Rust 控制或发车。

## Mac 终端连接

下面命令在 **Mac 本地终端** 执行，提示符应为 `yuhaojin@…`，不是 `bianbu@bianbu`。
如果已经在车端 SSH 内，按 ⌘N 新开一个 Mac 终端即可。

```bash
cd /Users/yuhaojin/Documents/XT-STCAR
python3 scripts/connect-vehicle-bluetooth.py
```

首次连接输入车辆现有 SSH 密码。如果 macOS 请求终端使用蓝牙，允许后继续。
进入车端终端后，重新摆好车、复位光电和灯，再输入 `start`。每轮结束后可再次摆放、复位并输入 `start`。

只想建立后台连接供传包和取数据使用：

```bash
python3 scripts/connect-vehicle-bluetooth.py --background
```

连接通过项目原有 `work/vehicle-test-ssh.sock` 复用，现有传包脚本仍可使用。即使命令中保留 `192.168.0.156`，它也只是原 SSH 主机身份；实际字节由蓝牙代理提供。可由 `SSH_CONNECTION` 的两端回环地址与车端蓝牙服务收发计数核实。

## 当前实现

- 车端 `vehicle-bluetooth-ssh.py` 通过 BlueZ 注册专用 RFCOMM 服务，转发到本机现有 SSH 服务。只接受配置中的已配对 Mac；SSH 仍使用原账户鉴权。
- Mac 的 `bluetooth-ssh-proxy.m` 使用 IOBluetooth 异步通道，不使用已经出现重连失效的 `/dev/cu.XT-STCAR` 虚拟串口。
- 服务 UUID 为 `78de4e4b-909b-45e1-a987-40c6d40c0b4e`，RFCOMM 通道 23。Mac 本地硬件地址配置在 `work/bluetooth-client/vehicle.json`，构建产物和缓存也在该目录。
- 车端用户服务 `xt-stcar-bluetooth-ssh.service` 已启用，开机启动；它不打开底盘、雷达或摄像头。
- `vehicle-console-entry.py` 已安装为车端 `run-service.py`。缺少默认路由时允许本地地址或仅回环地址启动，避免 CPE 断电后控制台启动失败。原启动器保存在车端工程 `work/bluetooth-field-20261115/run-service-before.py`。

先前配置的蓝牙 PAN 在这台 Mac 上没有可用网络接口，`10.77.0.1` 不作为当前连接入口。

## 当前验证边界

已完成重新配对、原虚拟串口的 SSH 握手，以及专用 RFCOMM 通道读取 SSH 响应。Mac 关闭连接后仍出现过通道残留；关闭再开启一次 **Mac 蓝牙** 可以恢复下一次连接，不需要切换 Wi-Fi。当前采用持久 SSH 主连接，后续终端、传包和多轮 `start` 复用同一连接。

11月15日已通过蓝牙完成密码登录，`SSH_CONNECTION` 两端均为回环地址，查询版本为 y94。1 MiB 文件上传耗时 12.171 s，84.13 KiB/s；随后的下载中断，不能记成双向传输已通过。车端日志确认 BlueZ 传入的非阻塞描述符在缓冲区满时返回 EAGAIN；已将转发套接字改为阻塞等待缓冲区空间，并更新车端服务。

用户随后暂停蓝牙传输测试，优先准备现场测试版本。该缓冲区修正未复测；蓝牙暂按发车用途继续准备，文件走 CPE。原蓝牙 SSH 主连接已在下载失败时断开，CPE 复用连接已恢复。没有通过蓝牙实际发车。
