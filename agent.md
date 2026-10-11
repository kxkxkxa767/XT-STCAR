# XT-STCAR 接手与开发约定

## 当前接手入口（2026-10-11：第53轮候选=整体修改（感知、跟踪、路径检查、慢速、出口减左），已离线验证；第52轮首次进入通道跟随但第二桶未识别撞桶）

**给接手代理（GPT）：先完整读本节、指南第0–7节，再动手。不要只修上一轮失败的那一处：先在第6节“失败类型表”里找到它属于哪一类，检查同类问题在所有阶段是否都已覆盖，并做完第7节的离线验证，才能上车。**

用户最近的话：“绕第二个桶的时候你舵机没有左转撞到第二个桶了”；“为什么不同的轮数你老重复错误”；“你改你整体改啊”；“明天我让gpt接手做，所以你每一个大的变动都要能让gpt知道”。记录键 `after52_holistic_robustness_correction`；第52轮结果 `turn52_actual`。

- **52实车**：首桶确认+交接（52的跟踪器修复生效）、U形弯、等待后改绑恢复、首过、出口对齐、通道跟随（1550）都走通；第二桶在右前−10°（正好在5–17°空回波带里）从1.72m到.72m，严格检测只在7帧里认出2帧（边缘空格、桶内空格、边缘混合像素），第二桶获取逻辑每次漏检都把跟踪器清掉→从未进入second_orbit→直行撞桶。
- **为什么老重复**：每轮只修出问题的那一处（51/52修了首桶跟踪器），同一类问题（检测在边缘空格处闪烁）在下一阶段（第二桶）又出现；修改没有用历史录像回放检验；识别失败时没有不依赖识别的避撞。
- **53整体修改**（仅显式 `--continue-route`；默认模式不变）：
  1. 感知统一：`compact_target._candidates(tolerant=True)` 用于两个路线跟踪器、第二桶获取、首桶改绑：桶内≤2个空格桥接、边缘≤2个空格/混合像素视为未知（外侧须有更远回波，直径+未知弧长≤.35m）、4个回波也可。这样的帧算完整观测。
  2. 跟踪保持：`CompactTargetTracker(tolerant=True)` 在漏检/歧义帧保留身份300ms（`tracker.held`，返回None、不计确认），关联门随时间放宽。
  3. 阶段不因单帧漏检停：首桶绕行/第二桶绕行遇保持帧就保持当前舵；第二桶获取的证明缺口.30s内既不计数也不清跟踪器。
  4. 路径检查（交接后）：每帧用实测曲率（(舵−1500)/160 每米）预测车身扫过的弧，.7m内会碰到已知回波就换最近的可通过舵量；通道/第二桶阶段优先从障碍左侧过（第二桶顺时针规则）；右舵只在路线已授权时。不依赖识别。
  5. 交接后电机1550（原来首过后才1550）。
  6. 首桶车头线到达后的减左目标改为实测出口通道的追踪目标（夹在1500与当前舵之间），只测量、不保存通道身份。
  7. 第二桶绕行回到通道跟随时重新开始通道身份（避免一次多余的停顿）。
- **离线验证结果**：相关五组253项全过；全量574项失败集合与修改前完全相同；`scripts/detector-replay.py` 回放40–52轮：首桶确认帧162→187、身份重置20→16，前方扇区确认94→113、重置33→20，52的第二桶7帧认出5帧（原2帧）；`scripts/route-replay.py` 开环回放真实录像：51的首桶在开车后1.70s确认并交接（当天未交接），52的第二桶在4.79s获取、5.19s确认并开始左转（当天从未获取）。
- **未实现、等用户决定**：“碰到也继续”（8cm车身近距门改非终止）被自动模式安全分类器拒绝，门仍终止。
- **第53轮状态**：见下面提交记录与 `after52_holistic_robustness_correction.deployment`（若无该字段则尚未部署）。按指南第1节从第5步开始。

## 接手指南（给后续代理，逐步照做）

### 0. 必须遵守的原则

1. **不针对练习赛道特化。** 比赛赛道是一个范围（规则：场地约长5–8m、宽4–6m，人行道和红绿灯位置随机；示意图在仓库规则PDF第3页）。转向和阶段切换只能用实时激光几何（原生通道的轴线角/中心偏移/宽度、开口、真实支撑点）和任务规则（首桶逆时针、二桶顺时针、红绿灯停走、到终点）。禁止写赛道坐标、长度、固定角度，或按时间猜拐点。反例：第34轮的“首桶出口总向右”就是特化，第45轮因此撞墙；第46轮改成按实测通道决定方向。
2. **自行决策，不要反复询问用户。** 用户已允许“适度放宽安全规则”，但要保留硬保护：车身近距停车门、旧帧/时钟门、桥锁/失联、期限预算。任何放宽都要有数据依据，并写进机器记录。
3. **每轮流程固定**：修改→相关测试→记录→提交推送→构建部署→新鲜核验→跑一次→取回校验→分析→记录→推送。helper每轮只跑一次；不覆盖旧轮目录；保留所有备份、失败证据和队友文件；直接在main开发，不强推。
4. **只在确实受阻时问用户**：SSH失效、校验失败、turn_ready=false（车不在起点，请用户放回）、凭据问题。
5. **最终目标是整场自动跑完。** 现在的 `turn-cone --continue-route` 是分段试验入口。每个片段（左转、首桶逆时针、二桶顺时针、转入灯区正对红绿灯、灯区慢速、人行横道/红绿灯停走由YOLO负责、到终点）在实车验证后，要合并进比赛用的自动控制程序：车启动后不需人工干预，按任务规则依次执行全部片段。合并时仍不得特化赛道（比赛赛道是一个范围），阶段切换只能依据实时感知和任务规则。
6. **修一类，不修一处。** 每次失败先在第6节归类；同一类问题在所有阶段（首桶、出口、通道、第二桶、灯区）一起处理；修改后必须做第1节第3/3b步和第7节的离线验证（全量测试失败集合不变、detector-replay不变差、route-replay回放51/52关键行出现），才能上车。52轮的教训：只修了首桶跟踪器，同样的问题在第二桶再次出现。

### 1. 一轮的完整命令（在工程根目录 `/Users/yuhaojin/Documents/XT-STCAR` 执行）

记号：上一轮标签N（如46），新一轮M=N+1，记录键K（如 `after46_xxx_correction`），`W=work/vehicle-dynamic-deploy-20261008`，`SSH="ssh -S work/vehicle-test-ssh.sock -o BatchMode=yes -o ConnectTimeout=5 bianbu@192.168.0.156"`。

1. 同步：`git fetch origin && git rev-list --left-right --count HEAD...origin/main`，必须是 `0 0`；远端有新提交时先 `git pull --ff-only origin main`。
2. 改代码：路线逻辑在 `web/vehicle-console/maneuver_sequence.py`，服务端门控/输出校验在 `server.py`，质量与车身近距在 `autonomy_live.py`。新行为只放在 `continue_route=True` 路径，默认模式不变。
3. 测试（必须全过）：`cd web/vehicle-console/tests && python3 -m unittest test_maneuver_route test_route_control test_maneuver_sequence test_compact_target test_compact_target_roles`（53后253项）。再跑全量 `python3 -m unittest discover -s . -p 'test_*.py' 2>&1 | grep -E '^(FAIL|ERROR):|^Ran ' | sed 's/ (.*)$//' | sort -u`，用 `git stash` 前后各跑一次比较，失败集合必须完全一致（目前只有与路线无关的既有失败；53后共574项）。
3b. **离线验证（必须，见第7节）**：`detector-replay.py` 全部轮次的总数不能比上一版差；`route-replay.py` 回放51和52，关键行必须出现（51：开车后≈1.7s内 handover；52：second_orbit且舵量开始向左）。任何一项变差就不要上车。
4. 记录：在 `docs/vehicle-dynamic-turn-validation-20261007.json` 新增键K（implemented=true、deployed=false、physical_trial_executed=false，写change/validation/risks），改写本文件顶部入口；`git add` 只加本次文件，`git commit`（结尾加 `Co-Authored-By` 行），`git push origin main`（已配置gh凭据）。
5. 构建：`python3 $W/build-matched.py claude-turnN claude-turnM M K`。它要求工作区与HEAD一致，所以必须先提交。输出里有stage路径和helper；`diff $W/left-cone-trial-once-N.py $W/left-cone-trial-once-M.py` 应该只差输出目录。
6. 部署前核验：`$SSH 'python3 -' < $W/read-fresh-route.py > $W/beforeM-fresh.json && python3 $W/validate-fresh.py $W/beforeM-fresh.json $W/claude-turnN-package.json`，必须打印 “Six fresh locked ready samples;14 hashes and settings match”。如果报 turn_ready=false（left_corridor_unknown、probe_obstacle_close_body 等），说明车不在起点。
7. 部署：`python3 $W/deploy-matched.py claude-turnM M`（只装文件，不动车）；再取 `afterM-deploy-fresh.json`，用 `claude-turnM-package.json` 核验，并确认 continue_route 预览为true。
8. 跑前检查：车端没有 `left-cone-trial-<日期>-M` 目录；`sha256sum <stage>/trialM.py` 与本地 `left-cone-trial-once-M.py` 一致；没有 autonomy-control 进程。再取 `preM-run-fresh.json` 核验，并比较起点几何（参考37–46：前墙2.12–2.28m、板端.48–.64m、出口宽1.64–1.66m、出口角87.9–94.0°、入射角−1.7..1.7°、侧向余量.197–.349m）。几何明显异常时停下问用户。
9. 跑一次：`$SSH 'python3 <stage>/trialM.py' > $W/trialM-stdout.json 2> $W/trialM-stderr.txt`。
10. 取回与校验：`$SSH "cd /home/bianbu/xt-stcar-console/left-cone-trial-<日期>-M && sha256sum *" > $W/trialM-remote-sha256.txt`；`scp -o ControlPath=work/vehicle-test-ssh.sock -p -r bianbu@192.168.0.156:/home/bianbu/xt-stcar-console/left-cone-trial-<日期>-M $W/`；`(cd $W/left-cone-trial-<日期>-M && shasum -a 256 -c ../trialM-remote-sha256.txt)` 要7个OK。再取 `afterM-trial-fresh.json`，确认 locked、1500/1500、未armed。
11. 分析：`python3 -I scripts/trial-timeline.py $W/left-cone-trial-<日期>-M --from 3 --objects`（逐帧决策）＋ `python3 -I scripts/trial-map.py $W/left-cone-trial-<日期>-M --png /tmp/tM-map.png --panels 5.3,5.9,6.6`（重建实际轨迹、车速、曲率，见第5节），用Read工具打开PNG看图，结合用户口述。找到失败后：①到第6节确定失败类型；②`python3 -I scripts/detector-replay.py $W/left-cone-trial-<日期>-M` 看是否是检测闪烁；③改完后用 `scripts/route-replay.py` 回放这一轮和51、52，确认新代码在失败时刻的决策变了、其它轮没变差。
12. 记录：JSON新增 `turnM_actual`；把K改为 deployed=true、physical_trial_executed=true，并加deployment块（commit、archive_sha256、backup、stage、helper sha、设置、核验文件）；改写本文件顶部入口和README首行；提交推送。

### 2. 当前路线逻辑（53候选，显式 `--continue-route`；按阶段顺序）

- **预打/起步**：中性预打到左舵、3个新反馈+1.2s后起步，电机1560。
- **左转**（TurnMotion）：原生开口/外墙几何（Rust桥算出的 `left_turn_goal`、`wall_candidates`、`corridor_candidates`）；入口端头方位上限；外墙歧义短暂保持；越过出口轴8°时：若.30s内左侧有首桶身份则保持左舵≤1s等交接（52），否则原逻辑回中coast。
- **首桶交接**：首桶跟踪器（容错检测+保持）同ID、3帧≥.25s确认、方位45–135°、距离.35–3m → 交接；交接帧保持已采用舵量。
- **首桶绕行（first_target）**：电机1550；左boost到正横，加左步长40；内侧近点限制减左；车头线到达（2帧）后减左到“实测出口通道追踪目标”（≥1500、≤当前舵）；支撑到车尾后且持续 → 首过 → right_align。单帧漏检保持舵量；漏检超过.30s → 交接后等待（自恢复：最近左侧确认目标改绑、滑行中逐步转向、只受累计预算约束）。
- **出口对齐（right_align）**：1500+180×(通道轴线角+atan(中心偏移/前视≈通道宽≤2m)+.30×轴线角速度)，1350..1720；向右须实际回中ACK；对齐成熟后 → corridor_follow。期间也会获取第二桶。
- **通道跟随（corridor_follow）**：±55追踪中心线；获取第二桶＝前方通道内唯一紧凑支撑（容错检测；证明缺口.30s内保留）；确认后 → second_orbit。
- **第二桶（second_orbit）**：绕行点=(max(.38, 支撑最近x), 支撑左缘+.40m)，舵=1500+205×追踪曲率；单帧漏检保持；丢失超过.30s或3s入口超时 → 回通道跟随（重新开始通道身份）；支撑到车尾后且右侧余量≥8cm、持续.25s → 通过 → light_turn。
- **灯区转弯（light_turn）**：按前方自由空间转入下一车道并正对（未实车验证）。红绿灯/人行横道识别由YOLO负责，不在这里。
- **全程（交接后）**：路径检查覆盖各阶段舵量（见入口第4条）；质量问题：交接后可容忍的scan_incomplete（≥324、空段≤12）和front_sparse（未桥接空段≤5、未知≤8）不停车；其它可恢复问题中性保持并持续检查，只受累计drive预算（16s）与总期限约束。
- **硬保护（仍终止）**：8cm车身近距门（交接前左侧4cm）、旧帧/时钟、桥锁/失联、期限、反馈不符。

### 3. 下一步（按优先级；53离线验证后更新）

1. **上车跑第53轮**，按第1节。重点看：首桶是否顺利交接和首过；第二桶是否在≥1m处进入second_orbit并向左绕过；路径检查（`path_evidence`/`path_override_count`）何时介入。
2. **用户要求“碰到也继续”**（8cm门改非终止）：被安全分类器拒绝，先问用户是否确认，再单独实现。
3. **第二桶曲率律**：开环回放显示1.4m处只要约1525，逐渐加到1677；闭环是否够早要看实车。不够就把路径检查的前视加长或让绕行点更早偏左。
4. **闭环仿真**（`scripts/route-sim.py`）：几何工具已与车上完全一致，但重建的世界模型（墙段、桶）还不准，不能当门槛。改进方向：只用拟合度好的帧、把同一块板两面合并、去掉板端误判的“桶”。
5. **整场自动程序**（原则5）：片段都在实车稳定后再做。
6. **舵量-曲率标定**（1550时曲率比1560大）、**右前5–17°空回波带实物检查**、**coast静止检测superseded**。

### 4. 分析时的坑

- `feedback.jsonl` 约每150ms采样一次（通常隔一帧），控制器看到的是每一帧；决策可能来自没保存的扫描。
- 要看实时字段 `state.autonomy.active.quality_issues` 和 `state.autonomy.active.turn`；`*_trigger`、`*_snapshot` 是事件快照，不是逐帧质量（44轮曾因此误判）。
- 角度约定：通道 heading_left>0 表示轴线在车头左侧，center_offset_left>0 表示中心线在车左侧；雷达bin i 表示自正前方顺时针i度（1..179在右侧）；紧凑目标 bearing_left>0 在左侧。
- 车身盒以雷达为原点：前.20m、后.18m、两侧各.14m。停后最近回波的车身间隙≈0多半是接触，以用户口述为准。
- 紧凑目标关联门：相邻两帧移动≤.30m且方位变化≤20°。保存帧相隔约.36m（1.8m/s）时，回放无法确认目标——只能回放逐帧证明，不能冒充实时确认。
- 车端时钟曾比Mac慢一天；目录名用Mac日期。
- `build-matched.py` 必须在工程根目录运行；历次deploy脚本和旧轮helper都是已用脚本，不得重跑或改写。
- git推送已配置gh凭据（`gh auth setup-git`）。
- `after.json` 不一定是静止状态：49锁定后车还以1.4→.6m/s滑了约1m。判断是否静止、是否贴墙，要看trial-map的车速列，不要只看最后一帧。
- 断电≠停车：电机1500后车只靠滚阻减速（约.5m/s²）。分析等待/停顿时，按“车还在走”来推演舵量后果。

### 5. 用雷达历史重建轨迹（scripts/trial-map.py，只读，不连车）

用途：把每轮保存的激光帧（before-lidar.json、feedback.jsonl、after.json）互相配准（2-D点到点ICP，地图由前面各帧累积），得到车在“起点车头坐标系”中的实际轨迹。这是目前唯一能实测车速、偏航率、转弯曲率与舵量关系的方法。

1. 运行（工程根目录）：
   `python3 -I scripts/trial-map.py $W/left-cone-trial-<日期>-M --png /tmp/tM-map.png --panels 5.3,5.9,6.6`
   不需要matplotlib（PNG用zlib直接写），约1秒。`--panels` 的时间用表格第一列（以首帧激光at_ms为0，比feedback的elapsed_s略早约0.05s）。
2. 读表格：每行一帧，列为 时间、帧号、雷达位置x/y（m，x=起点车头方向，y=起点左侧）、航向（度，左为正）、车速（m/s）、偏航率（度/s）、实测曲率κ（1/m，左正）、阶段、route_stage、电机、舵、配准拟合度、原因。
   - 车速单帧有噪声（扫描时间戳/旋转畸变，交替出现偏大偏小），取2–4帧平均或用位移/时间差。
   - 拟合度＜0.6或rms＞4cm的帧标“!”，且不加入地图；连续多帧“!”说明那段轨迹不可信。
   - 只有两面平行墙可见时，沿通道方向的滑动不可观测，轨迹会漂移，结论要结合墙端/桶等特征。
3. 看地图PNG（用Read工具打开）：上=起点车头方向，左=起点左侧，网格0.5m。灰点=所有帧看到的墙和桶，黑点=最后一帧（after.json最后一个样本），彩色线=雷达轨迹（蓝预打、绿drive、橙等待、紫coast、红锁定），方框=实测车身轮廓（前.20、后.18、侧.14m），约每0.45s一个带时间标签的深色框；红方块=首桶目标位置，蓝方块=第二桶。
4. 看面板PNG（`-panels.png`）：车体坐标快照，上=车头，绿框=车身，粉色射线=空回波bin，红/蓝方块=跟踪目标。用来看某一帧控制器“看到了什么”。
5. 分析套路（49示例）：①在地图上找车身框离墙最近的位置和时间；②在表格里看该时段的舵、曲率、车速——曲率是否跟上舵量，断电后车速是否真的下降；③对照trial-timeline同时间的原因字段，找出是哪条逻辑在当时给出了舵量；④把结论写成“哪个阶段、哪条规则、给了多少舵、实际走出多大半径、差多少”，再决定改哪一条。
6. 已测得的物理事实（会随电池/地面变化，新一轮要复测）：1560时U形弯1.3–1.6m/s；1550约.6–1.1m/s；断电滑行减速约.5m/s²；1655–1665舵量在1.3–1.5m/s时曲率约1.0–1.1/m，1636–1645约.75–.9/m；命令到轨迹滞后约150–200ms。


### 6. 失败类型表（每次修改前后都对照；新失败先归类，再检查同类在所有阶段是否都已覆盖）

| 类型 | 表现 | 出现轮次 | 根因 | 当前处理（53） | 用什么检查 |
| --- | --- | --- | --- | --- | --- |
| A 检测闪烁 | 桶隔帧消失、编号1→2→3、确认不了/目标丢失 | 41,47,50,51,52(第二桶),48(第二桶丢) | 桶边缘空格、桶内空格、边缘混合像素、5–17°空回波带 | 容错检测+跟踪保持，所有跟踪器和获取逻辑都用 | `detector-replay.py` 全轮次总数；`route-replay.py` |
| B 质量门停车 | scan_incomplete/front_sparse触发等待，断电滑行失控 | 41,42,44,47,49,50 | 门槛针对静态安全设计；车断电滑行约2m | 交接后可容忍规则；等待中逐步转向 | timeline看 `quality_issues` 与 `*_tolerated_count` |
| C 任务原因放弃 | 预算/越轴/身份变化后锁定或回中coast | 47,50,51 | 阶段时钟、严格改绑门、越轴回中 | 无阶段时钟、最近左侧改绑、有证据越轴保持 | timeline末尾reason；replay |
| D 转向不足/过冲 | 撞外墙、撞桶、过冲 | 45,46,48,49,52 | 步长慢、只瞄90°出口、回中太早、无避撞兜底 | 交接后步长40、出口减左瞄通道、路径检查 | trial-map曲率表；replay看PATH行 |
| E 车速 | 1560时1.3–1.6m/s，反应不及 | 49–52 | 未限速 | 交接后1550 | trial-map车速列 |
| F 硬保护终止 | 8cm门锁定 | 48,49,52 | 碰撞 | 仍终止（等用户决定） | 用户口述+REST间隙 |

### 7. 离线工具与上车前必做的验证（都不连车）

1. `scripts/trial-timeline.py <轮目录> --from 3 --objects`：逐帧决策（阶段、原因、舵、质量、通道、目标、车身间隙）。注意保存帧里state比lidar晚一帧（state里 `clearance_current.scan_seq` 是对应扫描号）。
2. `scripts/trial-map.py <轮目录> --png out.png --panels t1,t2`：ICP重建实际轨迹、车速、曲率（第5节）。
3. `scripts/detector-replay.py work/vehicle-dynamic-deploy-20261008/left-cone-trial-202610*-*`：把所有录像喂给三种跟踪器配置（51严格、52、容错），看 seen/conf/ids/first。**任何感知修改后都要跑**，TOTAL不能变差。53时的TOTAL：first/tolerant conf 187 ids 16；ahead/tolerant conf 113 ids 20。
4. `scripts/route-replay.py <轮目录> [--from 秒 --to 秒]`：开环回放——把真实录像按真实时间喂给**当前工作区**的控制器，打印每帧阶段、舵、目标编号、路径检查（PATH行）、等待原因（cause）。只在“新代码的决策与当天不同”之前有意义（之后录像不再对应新舵量）。**任何逻辑修改后都要回放51和52**：51应在开车后约1.7s交接；52应进入second_orbit并开始向左。
5. `crates/robot-core/examples/scan_geometry.rs`：在Mac上算与车上桥完全相同的几何字段（`cargo build --release -q -p xt-stcar-robot-core --example scan_geometry`；对52的57帧逐字段一致）。
6. `scripts/route-sim.py <轮目录> --seeds 3 --png out.png`：闭环仿真（真实控制代码+模拟雷达+车模型）。**实验性**：世界模型还不准，结果只作参考，不作门槛。

## 当前控制参数

以下为当前 `turn-cone / ManeuverSequence`；路线模式仅显式continue_route启用。旧直道及 `turn-left` 不自动套用。

| 项目 | 当前值与含义 |
| --- | --- |
| 电机 | 前进1560，中性1500；禁止反转刹车 |
| 初始预打 | 配置默认1670、允许1670..1720；已按当前端头限制实际预打，采用值单独记录 |
| 左舵上限 | 1720；行驶中依据新鲜几何动态增减，不固定档 |
| PWM渐变 | 中性预打20、减左20、加左/过中后向右10，每100ms最多一步、不补跳；向右前确认采用1500。路线模式首过后（出口对齐、通道跟随、第二桶）所有转向步长40（已部署a15115a）；路线模式交接后首桶阶段加左也为40（50候选，`ROUTE_ORBIT_LEFT_PWM_STEP`） |
| 容错检测与跟踪（53候选） | `TOLERANT_INTERNAL_NULLS=2`、`TOLERANT_EDGE_BINS=2`、`TOLERANT_MIN_POINTS=4`、保持租约300ms、关联门放宽1.5m/s与100°/s；仅路线跟踪器/获取/改绑使用 |
| 路径检查（53候选） | `ROUTE_PWM_PER_CURVATURE=160`、前视`ROUTE_PATH_LOOKAHEAD_M=.7`、前方余量`.04`、范围2m；候选1500..1720步20，授权时加1350/1390/1430/1470 |
| 电机（53候选） | 左转与恢复1560；交接后（含首桶绕行）及首过后1550 |
| 首桶身份延续与越轴保持（52已部署） | 首桶跟踪器对未确认身份也用有界边缘空格规则（`unconfirmed_boundary_gap=True`，不计确认、不刷新隔离）；左转越过出口轴且.30s内左侧有首桶身份时保持左舵≤1.0s（`ROUTE_LEFT_OVERSHOOT_HOLD_S`），之后保持左舵coast |
| 路线模式不放弃（51已部署） | 等待/停顿无3s绕行入口与5s等待时钟，只受累计drive预算（路线16s）与总期限；首桶等待改绑：左侧1.2m内最近的确认完整紧凑目标（`ORBIT_REBIND_MAX_RANGE_M`）；等待中减左：保持值>内侧需求+60（`ROUTE_WAIT_INSIDE_MARGIN_PWM`）；提前避让.15m（`ROUTE_AVOID_GAP_M`）右侧→≥1650（`ROUTE_AVOID_LEFT_PWM`）左侧→≤1500，步长40；交接后front_sparse豁免：未桥接空段≤5格且未知≤8格（`MANEUVER_TOLERABLE_FRONT_RUN/UNKNOWN`）；8cm车身近距门仍终止 |
| 路线模式可容忍扫描 | 50候选：交接后若scan_incomplete且有效返回≥324、循环空段≤12格（`MANEUVER_TOLERABLE_MIN_VALID/MAX_NULL_RUN`），服务端不计该问题，本阶段继续；报告 `scan_incomplete_tolerated_count` |
| 起步前采用等待 | 初始舵目标已采用、至少3个新control tick且等待1.2s；并非实测轮角反馈 |
| 预算 | 中性预打8s、累计drive10s、首目标入口3s、右对齐3s、路线模式第二入口3s、coast5s；累计预算含quality_wait，不刷新；终止coast不恢复 |
| 旧默认质量收油 | continue_route=false：接管后scan_incomplete进入coast，有限front_sparse可保已采用左舵；最多5s，不恢复动力 |
| 路线模式质量等待 | continue_route=true：接管后一次有界quality_wait；实际中性ACK后至少324有效返回，稳定达到原动力质量/同ID/双时钟门才恢复，原期限不刷新。首桶阶段（含左转）等待被拒或front_sparse（前±30°未知≤6）时，保持已ACK舵量断电终止coast（43已部署） |
| 路线模式出口与第二桶 | 已部署a15115a：首过后按实测通道中心线双向追踪（向右须中位ACK）；corridor_follow中获取前方通道内唯一支撑为第二桶，绕行点距支撑左缘.40m，通过后终止coast保持顺时针舵量；第二桶之后尚无阶段 |
| 已部署出口宽度 | 小变动更新，大跳变不单独中断drive，沿当前真实外墙及上次观测宽度继续 |
| 时间门 | new控制loop120ms；旧模式80ms；control fresh200ms、autohealth250ms、heartbeat300ms |
| 车身近距门 | 首目标pass前实际左转drive左侧点总4cm；路线模式恢复ACK等待、后续路线及其余情况总8cm；不认证未来扫掠或停车距离 |

本车此前实量轴距25cm；2026-10-07用户更新粗测雷达到车头20cm、车尾18cm、左右轮胎外沿各14cm（整体38×28cm），见 [几何记录](config/vehicle-geometry-measured-20261006.json)。后轴偏移、物理轮角/曲率、车速及滑停包络仍未标定。不要使用厂商默认305mm冒充实测。

已部署开口规则：0–90°真实连续远返回沿当前左墙延长线的投影跨度≥0.50m；null/近点/无效或相邻投影缺口>0.10m断段，保留前/右/外墙证据。0.50m为沿用的保守观测支持门；新粗测宽28cm后未下调该Rust门，不将旧34cm当当前车宽；`known_open_fraction`仅指选中支持段，不证明整条路径安全。

## 下一步与证据位置

1. 读本文件、[上传规范](上传规范.md)及[动态左转机器记录](docs/vehicle-dynamic-turn-validation-20261007.json)。先fetch核对main，再修改。历史记录只按需检索。
2. 13至18原始目录在`work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-{13,14,15,16,17,18}/`；19至29在`work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261008-{19,20,21,22,23,24,25,26,27,28,29}/`。30至39在`work/vehicle-dynamic-deploy-20261008/left-cone-trial-20261009-{30,31,32,33,34,35,36,37,38,39}/`；40–43在同目录 `left-cone-trial-20261010-{40,41,42,43}/`，44–46为 `left-cone-trial-20261011-{44,45,46}/`。原helper/摘要/完整记录均保留；车端原目录在base下。
3. 旧完整部署记录在`work/vehicle-dynamic-deploy-20261007/resume-after14/`。19至30部署及逐帧分析在`work/vehicle-dynamic-deploy-20261008/`；29为`target-evidence-deployment-result.json`，stage为base下`stage-target-evidence-20261008-212120`。
4. 18分析仍在旧目录`trial18-analysis.json`/`trial18-target-components.json`，回归材料在`web/vehicle-console/tests/fixtures/left-cone-18-boundary-null.json`。18的最终drive/orbit计数含coast；区分各自scan序号、PWM/物理轮角、计数器/动力时长。
5. 第46轮已执行（一次）。新一轮按《接手指南》第1节：`build-matched.py claude-turn46 claude-turn47 47 <记录键>` 与 `deploy-matched.py claude-turn47 47`。
6. 每次修改更新本文件的当前入口及机器记录；按上传规范提交并推送main，不强推。文档整理不改变车端代码或运动结果。

## 连接与部署

- 工程 `/Users/yuhaojin/Documents/XT-STCAR`；GitHub `https://github.com/kxkxkxa767/XT-STCAR`，直接main开发，不另建分支/PR。
- SSH `bianbu@192.168.0.156`；复用socket `/Users/yuhaojin/Documents/XT-STCAR/work/vehicle-test-ssh.sock`。失效时让用户在终端重连，不向聊天索取密码，不将旧反馈写成实时状态。
- 车端base `/home/bianbu/xt-stcar-console`；活动目录 `20260917`，用户服务 `xt-stcar-console.service`。仅执行硬件的主agent持有控制；共享驾驶台采集与唯一串口所有者，不另开相机/雷达/底盘抢设备。
- 更新须备份完整活动目录、入口、运行设置及私有配置，传齐8个Python模块、匹配Rust桥、入口、三前端和目标配置例；核对依赖、运行路径、安装哈希及锁定反馈。模型、视觉配置、雷达校准、访问码和启动命令保留。失败恢复整套。
- `work/vehicle-dynamic-deploy-20261008/` 下历次 `deploy-*.py` 及 `left-cone-trial-once-*.py` 均为已用脚本，绑定各自旧包、OLD哈希校验与证据目录，不得重跑或改写；新轮只用 `build-matched.py`/`deploy-matched.py` 生成新标签的脚本与helper。
- 当前实际手动设置1550/1450/1650/1350；与内部turn-cone参数分开。重启会重置内存设置，部署前后核对实际值。
- base下`backup-right-exit-20261009-191516`保存29用c2a6d68整套；`backup-target-evidence-20261008-212120`保存9c041b1整套；`backup-boundary-track-20261008-211326`保存acb整套；`backup-orbit-left-20261008-205509`保存26认可的966整套；`backup-entry-handoff-20261008-203943`保存6c9整套；`backup-endpoint-bearing-20261008-202942`保存d49整套；`backup-heartbeat-first-20261008-200515`保存8686整套；`backup-width-follow-20261008-194830`保存e7整套；`backup-quality-coast-20261008-193537`保存4bc整套；`backup-handover-base-20261008-191850`保存b6整套；`backup-entry-tracking-20261008-190253`保存688整套；`backup-turn-left-clearance-20261007-215109`保存f00整套；`backup-continuous-left-20261007-213347`保存用户认可的16轮5ec整套；其余8d/4e/7df备份同样保留，路径见历次部署记录，均不得删除。

## 长期约束

- 用户常驻偏好：“以后都是，只要我没明说都是直接在车上测试”。控制调参默认实车，不先跑多余Mac/目标板纯计算suite，不重复问动作许可；明确只Mac/暂停/不测时按新指令。仅在未知时确认必要现场静止/摆放事实。
- 禁止赛道固定坐标、练习长度、按时间猜拐点；用实时点云、真实端头/墙面/目标身份。软板允许形变，端点随新扫描更新；角差本身不能证明两面，缺测不当free。
- 不造pose、速度、轮角、圈数或锥桶语义；不将simulation_only/unverified改标签冒充实测。已采用PWM、物理动作、现场评价、任务完成分别记录，旧通过数字不冒充新版本验收。
- 保护门按顶部最新明确授权生效；质量门、急停/失联/旧帧/控制时限保持；缺观测不通过无限续租或自动解锁继续。1500是中性收油，不保证瞬停。新控制可作受限试验，不声称已证整车扫掠/制动。
- 直道正式停车服从传入停车点，不能绑定练习左路口。任务次序为斑马线停车→第一桶逆时针→第二桶顺时针→灯区→终点；是S形通过，不是每桶一整圈。完成/身份账本防旧元素重入。
- 两桶以雷达独立确认、跟踪和规划为主，YOLO辅助同身份；其余任务语义由YOLO。训练交队友；最终标签固定 `0:crosswalk / 1:blue_cone / 2:red_light / 3:green_light`。车上仍单类blue_cone，不冒称已四类。完整视觉任务单见历史归档。
- 新核心算法优先Rust，现有实时Python模块按现有边界维护；不为统一语言无故重写。Mac交叉构建基线GLIBC2.38，沿用项目工具链；不改车端libc/系统固件，不恢复暂停的STM32读取/刷写，不读厂商视频。
- 正常运行优先于eMMC寿命：减少无用落盘，不降感知/控制频率或因可选日志额度触发故障。必要诊断和全部失败证据保留。
- 源码/配置/必要记录进Git；密码、私钥、access文件、原始影像、模型、工具链、环境、构建产物与部署包不进Git。保留队友原有SUMMARY、ZIP、文本与比赛PDF，不暂存无关文件。

## 必须保留的直道好版本

- 用户认可的直道源码 `8efc47b`：本机 `work/vehicle-junction-coast-test-20261006/release-process/` 及完整 `run04/`；车端 `/home/bianbu/xt-stcar-console/backup-dynamic-turn-20261007-102438`。**绝不能删。**
- `backup-coast-process-20261006-161139` 是部署8ef之前的旧备份，不是run04源码。run04现场效果获认可，但软件低运动/停稳未确认，`completed=false`仍保留。
- 其余历史失败备份、原始扫描与队友原件同样保留，不用新成功覆盖旧失败。

## 代码与历史索引

| 入口 | 职责 |
| --- | --- |
| `web/vehicle-console/maneuver_sequence.py` | 左转→首目标；显式35候选通道对齐/跟随→第二目标 |
| `turn_motion.py`、`compact_target.py`（同目录） | 墙/通道关联与预打控制、真实点簇多帧身份 |
| `server.py`、`autonomy_live.py`、`coast_motion.py`（同目录） | 所有权/控制时限、车身近距、滑行/低运动 |
| `autonomy-control.py`、`stop_goal.py`（同目录） | CLI、正式停车目标契约 |
| `crates/robot-core/src/junction.rs`、`junction/turn_goal.rs` | 原生开口/墙线/端头几何 |
| `crates/robot-core`、`crates/runner` | Rust任务/导航、桥与执行；离线比赛结果不等于实车闭环 |
| `scripts/detector-replay.py` | 只读：所有录像喂给多种跟踪器配置，比较检测稳定性（第7节） |
| `scripts/route-replay.py` | 只读：真实录像开环回放当前控制器决策（第7节） |
| `scripts/route-sim.py` | 实验性闭环仿真（第7节） |
| `crates/robot-core/examples/scan_geometry.rs` | Mac上计算与车上桥相同的几何字段 |
| `scripts/trial-map.py` | 只读分析：ICP重建实际轨迹，逐帧车速/偏航率/曲率/舵量表，俯视轨迹PNG与车体坐标面板（指南第5节） |
| `scripts/trial-timeline.py` | 只读分析：起点几何、逐帧实时阶段/舵/质量/通道/目标/车身间隙，`--objects` 列紧凑物体 |
| [README](README.md)、[命令手册](Mac与车端命令手册.txt) | 代码结构、模块入口、Mac/车端命令 |
| [环境](资料/环境.md)、[资料索引](资料/资料索引.md) | 工具链、厂商文字/源码来源 |
| [动态接口](docs/动态左转与测试目标接口-20261006.md) | 实车左转及实验目标边界 |
| [完整历史归档](agent-history-20261007.md) | 精简前1614行原文、全部旧参数/失败记录/视觉任务单；仅按需查阅 |

维护方式：直接改写当前入口，不再层层追加“新窗口接手入口”；详细逐轮数据写入docs/机器记录，原始材料留work/。历史归档正文逐字节保留，归档头列原SHA256；历史中的“最新”“待部署”“暂停”等不能覆盖当前用户指令。
