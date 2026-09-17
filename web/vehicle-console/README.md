# XT-STCAR 网页驾驶台

该页面用于现场有人看护的手动遥控，独立于比赛导航。按住方向键或WASD持续输出、松开该方向键回中；前后键与转向键可组合。空格/停止按钮立即请求中性并锁定，切换标签、失焦或断网停止。没有固定2秒动力时限，这是用户明确要求的按住持续模式。

## 安卓 / Windows / Mac 通用入口

在同一局域网中，用浏览器打开 `http://192.168.0.156:8081/`，首次输入车辆访问码；也可打开工程根目录 `打开车辆驾驶台.html`，填写地址和访问码进入。安卓使用屏幕方向按钮，电脑可使用键盘。推荐当前版本 Chrome / Edge / Safari；未在实体安卓或 Windows 设备验收。

访问码在车端 `~/xt-stcar-console/access.json` 的 token 字段，由已有SSH连接读取；不写入Git。完整入口可在网址后加 `#token=访问码`，认证后浏览器移除地址中的码并存入当前标签会话。安卓可将车辆网址添加到主屏幕，Windows可收藏或创建浏览器快捷方式。打开网页需要车端服务运行，但日常访问不依赖Mac或SSH隧道。车辆IP变化时修改入口地址并更新服务 `--lan-bind`。

解锁前必须传感器在线。底盘锁定/心跳超时后会释放页面控制权；连续点击解锁只发一次请求。已有其他页面控制时先按“停止并锁定”，再解锁。后台不自动恢复解锁。

## 双击打开（Mac，可选）

在工程根目录双击 `打开车辆驾驶台.command`。首次可能在终端询问SSH密码，直接在自己的终端输入；密码不会保存。它优先复用当前SSH会话，检查车端已部署驾驶台，必要时启动服务，建立本机8081转发，再用默认浏览器打开完整入口。启动服务只会保持中性，不自动解锁或发送行驶指令。打开成功后终端窗口可关闭。

入口脚本和网页源码都在工程中；访问令牌实时从车端读取，不写进提交。车辆需要开机并能通过网络访问。车端程序尚未部署、端口被其他服务占用或现有进程异常时会给出错误，不强杀旧进程、不自动重新部署。

诊断命令（工程根目录）：

```bash
python3 scripts/open-vehicle-console.py --check-only
python3 scripts/open-vehicle-console.py --no-open --port 8082
```

第一条只检查现有连接，不启动服务、隧道或浏览器；第二条准备8082入口但不开浏览器。正常双击用8081。脚本兼容工程目录含空格；`.command`的可执行权限已保存在Git。

## 当前部署

车端目录：`/home/bianbu/xt-stcar-console/20260917`。
服务同时监听车端 `127.0.0.1:8081` 和局域网 `192.168.0.156:8081`；原Mac SSH入口仍可用。启动后默认锁定，当前网页前进PWM默认1550、上限1620，转向1450–1550、中位1500。用户按“解锁键盘控制”后才允许非中性指令。参数只能在锁定状态修改，重启恢复默认值，不写入比赛标定。

当前部署未启用倒车；↓/S及1480–1500倒车参数的代码和模拟测试已实现，但电调实际倒车/制动行为未验证，不能凭PWM小于1500保证后退。完成现场倒车标定后，停服务并在启动参数中显式添加 `--allow-reverse`。程序不擅自执行“刹车—回中—倒车”的电调序列。

车端启动（SSH终端执行；默认不加倒车开关）：

```bash
cd /home/bianbu/xt-stcar-console/20260917
nohup python3 -u server.py --bridge ./vehicle-bridge \
  --bind 127.0.0.1 --lan-bind 192.168.0.156 --port 8081 \
  --output /home/bianbu/xt-stcar-console/recordings \
  --access-file /home/bianbu/xt-stcar-console/access.json \
  > /home/bianbu/xt-stcar-console/server.log 2>&1 < /dev/null &
```

Mac建立隧道（当前8081转发已建立，不重复占用端口）：

```bash
ssh -N -L 127.0.0.1:8081:127.0.0.1:8081 bianbu@192.168.0.156
```

车端查看本次专用入口（令牌保存在权限0600的本地文件，正常重启复用；每次启动另换控制会话标识，旧解锁请求无效；不要上传access.json或分享入口）：

```bash
python3 - <<'PY'
import json
p='/home/bianbu/xt-stcar-console/access.json'
print('http://127.0.0.1:8081/#token='+json.load(open(p))['token'])
PY
```

在Mac浏览器打开输出的链接。SSH隧道断开后页面离线，需要恢复隧道；服务不自动解锁。页面刷新仍需点击解锁，不能靠浏览器历史恢复行驶。

停止服务：先在网页按停止，随后在车端执行：

```bash
python3 - <<'PY'
import json,os,signal
p=json.load(open('/home/bianbu/xt-stcar-console/access.json'))['pid']
cmd=open(f'/proc/{p}/cmdline','rb').read()
if b'xt-stcar-console' not in cmd or b'server.py' not in cmd:
    raise SystemExit('PID身份不匹配，不发送信号')
os.kill(p,signal.SIGTERM)
PY
```

服务关闭会先请求中性，再关闭控制子进程输入；Rust桥在EOF时补发20帧中性。不要用kill -9结束底盘桥来代替停车。没有安装系统服务或开机自启，不改原厂ROS/串口规则；该页面运行时独占相机、雷达和底盘，其他硬件实验前先正常停服务。

## 图像和保存

- 相机：车上已有OpenCV采集 `/dev/video20`，640×480，最多10fps编码，通过独立JPEG请求刷新。
- 雷达：Rust沿用现有N10校验解析器，以0度跨圈形成360个显示bin。无回波保留null；显示有效回波比例，失效时变灰。这个显示路径不放宽比赛定位的质量门，也不是里程计或自动避障。
- 图片保存：点击“保存相机照片”生成JPG，“保存雷达图片”生成PNG，“保存双画面”生成1280×520的单张合成PNG，并发起浏览器下载，不需要解压。文件同时留在车端，下方图片链接可重新下载；具体本机保存位置由浏览器决定。数据过期时拒绝保存旧画面。
- 可选原始快照：展开“原始数据与录制”，ZIP内有 `camera.jpg`、`lidar.svg`、`lidar.json` 和 `status.json`。模拟模式的相机是SVG演示图。
- 录制：ZIP内有JPEG帧序列、`frames.jsonl`采集时间、`lidar.jsonl`点云、`control-events.json`及元数据；不是MP4编码视频。最多60秒或250MiB自动结束；总目录约2GiB及最低512MiB剩余空间保护。结束后可在网页下载到Mac。
- 保存目录 `/home/bianbu/xt-stcar-console/recordings`。不自动删除旧记录；空间不足时停止保存并提示。异常中断遗留目录保留以便恢复，正常完成才压成ZIP。
- 时间戳为主机接收/采集单调时钟，未硬件同步，不能用录像反推精确电机响应。

## 停止逻辑与边界

核心在 `crates/robot-core/src/teleop.rs`。独立Rust `vehicle-bridge` 才持有底盘串口，20ms循环；浏览器约80ms发送按键状态。300ms未收到有效指令即锁定回中；250ms以上旧时标、逆序指令、非法PWM、控制间隔异常也锁定。停止建立时间屏障，停止前已经发出但延迟到达的解锁请求不能重新启动。输入有长度/队列上限，状态通道阻塞会锁定。服务器不代替浏览器持续刷新动力命令。

单个页面获得控制权，其他页面只能查看、保存或停止；网络/传感器恢复后必须重新解锁。后端独立检查相机、雷达新鲜度，传感器超过1秒无更新会停止；雷达“新鲜”只表示收到了回波，不证明车辆前方无障碍。

这些机制不保证内核挂死、底盘桥被SIGKILL、串口物理故障或MCU失效时停车，仍需现场独立急停。1500为已观察中位，不是经标定的制动距离保证。前进1620上限是本次手动测试边界，不是允许无人看护连续行驶的证书。

## 本机开发与测试

```bash
cargo build --locked --offline -p xt-stcar-robot-runner --bin vehicle-bridge
python3 -m unittest discover -s web/vehicle-console/tests -v
python3 web/vehicle-console/server.py --demo \
  --bridge "$PWD/target/debug/vehicle-bridge" --port 18081 \
  --output work/console-demo-recordings --access-file work/console-demo-access.json
```

模拟模式不打开任何物理设备；PTY测试只连接虚拟终端。部署验证按用户要求未发送任何非1500电机命令，未测试实车键盘行驶或倒车。

访问码支持8–128位英文字母、数字、`-`、`_`；默认仍生成随机长码。手动更换须先停车并正常停止服务，再修改权限0600的车端access.json并重启。旧码失效后页面显示访问码输入框，重新输入即可；具体访问码不提交到Git。
