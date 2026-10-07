# XT-STCAR 接手与开发约定

## 新窗口接手入口（2026-10-07，12再次撞墙；用户确认静止/复位，初1670与非对称步幅待13）

**用户最新“还是撞上了，初始1670看看，变化幅度可以大点”，已确认物理完全停住、复位斑马线后/车头朝原直道，并明确选择预打和减左20PWM、加左仍10PWM（100ms间隔）。本次13授权待消费；此前“不验证直接上车测试”继续，不再启动额外suite。root先最小改源/成套锁定上传，再执行13一次；不自动多轮。当前车端f3初1690/leftmax1720/旧10步。**

- 12 run FWRZJdQstplzM2zRu0Ay_IZA：pre134/motion33/coast0/drive1.16959s，末请求1680/目标1623，reason=probe_obstacle_close_body、min当前矩形净距2.7283cm<8cm、nearest316/r.284/x.204293/y.197283；nohandover/completed=false，用户确认撞墙，碰撞失败。maxgap84.382ms<120、max锁等1.873ms、max主机tickwall63.085ms，本次不是时序门。
- 最后6fresh locked/healthy/unowned/M=S1500/seq169；root碰撞后另stop并新6freshseq170，用户已确认物理停住和复位（不凭sensor认证）。12原始完整work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-12/，错误1690行驶cap早已撤回。
- 新mode DEFAULT/MIN1670..1720，1690初值已被最新1670覆盖；LEFT总上限1720。仅newSequence预打M1500step20，减左step20，加左仍step10，100ms不catchup；降过中心先1500，coast从中性向右保持10。pre8/drive10/entry3/coast5、采用3新tick/1.2、loop120/old80、controlage200/autohealth250/hb300、body净5+粗3=.08、质量/时钟/急停/no正aftercoast/no rearm保持。旧TurnMotion1650..1720及10步不变，不用赛道特化/假pose。
- 按用户明确要求只改必要source/断言期望，不新跑Mac/目标板suite；不能把旧347或旧47声称新1670/20步已验证。车辆root独占，新的13严格一次，真实反馈/现场结果分别记，不宣称初1670可解决碰撞。

## 新窗口接手入口（2026-10-07，用户纠正左转上限1720，并要求不再验证直接上传实测）

**最新用户“左转上限是1720”“不要验证直接上车测试”优先：root承认把1690作全left-drive cap误解。新源恢复初始预打1690、所有左转动态上限1720，每100ms最多10PWM，不给突变；其他budget/profile/heartbeat不改。按用户要求不再启动额外验证套件，改完直接成套锁定上传后执行用户本次12一次授权。本次12尚未执行。车端当前仍73e旧错误cap1690，不能当已改1720。**

- 默认initial1690/strict1690..1720，left_turn_servo_cap固定1720，drive自然目标可高于初始值但只逐步变化、误差小仍动态回中。相对目标bias仍同initial/max1720；pre8/drive10/entry3/coast5、newloop120/旧80、实际采用3tick/1.2、body净5cm+3=.08/质量/时钟/急停/no正aftercoast/no rearm保持。已启动的29序列/65服务纯Mac在新“不验证”指令前运行完通过；最后2条断言未再跑。不把旧347或旧目标板47当此新revision已完整验收。
- 用户只纠正1720最大值，初始候选1690保持。此前73e错误cap1690已按返回上传指令完整备份锁定部署，14安装哈希/47纯计算和fresh锁定已核对，但没有动力；本次必须再传匹配新server/CLI/sequence整套8Python，不能只改一个文件。root独占SSH/硬件。
- 11仍是撞左墙失败，用户确认实际已停，原raw3266近点5.795cm<8cm后中性仍继续贴近至名义矩形内，不能宣称上限改动就解决碰撞。所有原始/好直道8ef/失败备份/队友原件保持，按规范更新推main不强推。

## 新窗口接手入口（2026-10-07，1690候选已成套锁定部署；没有新动力测试）

**用户最新明确“车已经连上了，你上传吧”，已解除上节等待回实验室的部署暂停。1690候选源73e1f2f已成套完整备份/默认锁定上传，14安装哈希一致；347项Mac与车端47受影响纯计算通过。默认预打1690、左转drive动态上限1690、newmode1690..1720/loop120/pre8已独立GET/纯实例核对。车辆fresh healthy/locked/armed=false/M=S1500，未新arm/drive。用户只授权上传，本轮不启动测试。**

- 11仍为真实碰撞失败：实际完整采用1720、14tick/1.315s成熟后1560明显左转，drive约1.133s，raw3266 body5.795cm<guard8cm后锁定，但用户确认撞左墙/已完全停住/未到绕桩入口；中性后原raw有矩形内返回0。不将软件停门当瞬停/无碰撞保证，也不称新1690已实测最佳或碰撞已解决。全部原数据/人类陈述保留。
- 新mode默认initial1690，strict int1690..1720，预打和左转drive cap同selected，目标误差减小时仍动态回收至1500；只有真实compact目标确认接管后，相对反馈bias同selected、可逐步升至1720。显式1700/1720兼容，旧turn-left1650..1720/None保持；source不含赛道长度/坐标或假pose。完整两桶S尚未验收。
- servo10/100ms、实际采用3新tick/1.2s、pre8/drive10/entry3/coast5、newloop120/旧80、controlfresh200/autohealth250/heartbeat300、body净5cm+3cm=.08/质量/时钟/急停/no正aftercoast/no rearm保持。347完整Mac/28序列/65服务CLI/21独立coast与9Python编译通过；本轮目标板47专项42.34s OK，非物理运行。Rust源未变，匹配桥238a2c8f沿用，未伪称重编。
- 当前车端源码73e1f2f657298da16d0eb6a8c471a9907fdddddc，完整backup-candidate1690-20261007-190110保存683旧1720版；模型/配置/校准/凭据/启动命令保留，实际手动设置1550/1450/1650/1350原样。全部好直道8ef、失败备份、队友原件保留。
- 部署与独立各10份fresh healthy/locked/unowned/M=S1500，参数GET后最终seq0/tick119387；default1690/leftcap1690、新range1690..1720拒1650/1689、old1650兼容、new.12/old.08/pre8/old5/drive10/coast5、唯一串口owner与14安装哈希都核对。ready=true只预览，不是起点/轨迹/停稳认证，下一步重新读取。记录见[机器记录](docs/vehicle-dynamic-turn-validation-20261007.json)，忽略目录work/vehicle-dynamic-deploy-20261007/candidate1690-*存部署/独立/目标板日志。
- root独占SSH/硬件，按上传规范更新推main不强推。**本轮上传完成但没有新的动力授权**；待用户车辆摆好/完全静止并明确新测试，再重新读fresh执行单轮，保护退出不自动重复。11碰撞后的参数修正仍待实车验收，不直接宣称可跑全程。

## 新窗口接手入口（2026-10-07，11左转撞墙失败；1690候选已Mac验证，用户回来后才上传）

**最新用户明确“车放回实验室，等回来再上传，先编译+验证在Mac”。这覆盖上一条改完立即上传：本轮新1690候选仅Mac编译/验证和按规范Git源码同步，不再SSH/车端测试/部署/arm/drive，等用户回来再继续。车端最后确认仍683e024/初始1720。11授权已消费，实际明显左转、撞左侧墙，用户确认已完全停住、未到绕桩入口；必须记碰撞失败，不能记成功。**

- 11 run vteSPzBoaKT6WhQPbMBVHvrj：pre129/motion29/coast0；实际软件采用1720后14新反馈/1.315s成熟再1560，drive约1.133s、已保存1720→1707→1690→1680反馈，末软件请求1670/目标1629，未handover。硬body原scan3266保存，最近ray318/r.326、x前.242265/y左.218137到矩形.0579498m<guard.08，前帧3265为.140182；软件中性后原scan3270出现矩形内返回0，后来离开，不能将1500当瞬停或证明真实接触点/车速。碰撞来自用户确认。
- 新120ms本轮无时序退出：maxgap79.307ms、max锁等2.563ms、max主机tickwall53.982ms；terminal reason=probe_obstacle_close_body，不是120ms或8s。末6fresh locked/healthy/unowned/M=S1500/seq160；碰撞后root另stop得到新seq161及6fresh control608037→609163，新M=S1500；人类确认已停另存，最后只读pose已left_corridor_unknown，回实验室后不沿此读数发车。
- 用户要求初始1690..1720判断择值同步：选择1690作为下一候选（更小左舵，不称实测最优或碰撞已修）。旧程序只改initial会在drive继续追1720，现newmode默认1690/strict int1690..1720，预打和左转drive动态上限同步selected值；仍可回收至1500，不固定1690。真实compact目标确认后相对反馈bias同selected、可逐步加到1720。显式1700/1720兼容，旧turn-left1650..1720/None保持。servo10/100ms、实际采用3tick/1.2、pre8/drive10/entry3/coast5、loop120/旧80、质量/硬障/时钟/stop锁存保持；没有赛道坐标/长度/假pose。
- 347项完整Mac Python、28序列/65服务CLI/21独立coast通过，9Python文件内存编译通过。Rust源未变，匹配RISC-V桥238a2c8f沿用且源码账/哈希核对；不伪称重编或新目标板验收。实际11原点云纯软件目标对比1720/1720/1720/1696→1690初段、后段仍1639/1603，不当1690真实轨迹。新参数未上传车端、未新测试，Mac交付包已生成并核对15项哈希/18文件：outputs/XT-STCAR-candidate1690-Mac-73e1f2f-20261007.tar.gz，包源73e1f2f、archive SHA bb3488ee47271ab0cab44eeb8c3017d9f7777e59b121b0cd8e3baa5ca87f40ec；本地保留到用户回来，未上传。
- 11完整原始/独立复盘/候选反事实/人类碰撞陈述在work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-11/；[机器记录](docs/vehicle-dynamic-turn-validation-20261007.json)。所有直道好8ef/历次失败备份及队友原件保留。最后车端设置1550/1450/1650/1350，不把newmode1690同步理解成已经改手动设置。
- root独占SSH/硬件。验证后按上传规范推main不强推；**当前用户要求等回来，先不部署/不测试**。回来先重新读取工程Git/车端版本和fresh反馈、完整备份默认锁定传齐8Python/匹配桥/入口，再核对哈希/设置；新动力仍须车辆摆好/静止与新的当次测试指令，失败不自动重复。完整两桶S/比赛仍未验收。

## 新窗口接手入口（2026-10-07，newmode120ms与共享锁优化已验证并锁定部署；尚未再试11）

**最新用户要求“排查好了改了以后提一提这个上限”已完成：仅newmode控制循环80→120ms，owned GET不重复准入扫描/目录IO移锁外，加实际gap/锁等待/tick墙耗时。源码683e024成套完整备份/默认锁定部署，14安装哈希一致，独立GET确认new120/old80、初始1720/pre8。车辆M=S1500/armed=false，未新arm/drive。10授权已消费，只有中性预打两拍、0前进，不能记左转或绕行成功。**

- 10 run 2tjwRPqHlqFT_d-5trWRXfph，pre2/motion0/coast0；软件最后请求1510，保存底盘servo最高1500，不能称实际采用1510。最后success turnscan3554/pre_elapsed.0717986/settle0、body.310618>.08、quality空；3554 raw未保存。autonomy_control_gap在新感知更新前退出，实际gap未记录，不把observer150ms或controlticks209ms当控制间隔。
- 目标板30×16真实scan纯计算完整更新max27.048ms、wall21.816ms，不复现80ms，实际原因仍未精确归因。tick间隔含前拍运算+watch20ms+共享锁/调度；新source记录actual_gap_s/max、last/max_lock_wait_s、last/max_tick_compute_s、timing_basis（主机wall含调度等待，非CPU/物理）。halt同runid保留最后测量。owned GET明确clearance=None/admission拒绝，active.clearance_current原profile保留；files IO不再持控制lock。
- new120只由内部maneuver_sequence会话选择，客户端不能自设门；默认直道/旧turn-left80，control age200/autohealth250/heartbeat300、初始1720完整采用3tick/1.2、预打8/drive10/coast5、质量/近距/急停/no正aftercoast/no rearm保持。342项Mac Python、63服务CLI/21独立coast、目标板42受影响纯计算通过；活动GET纯代码max4.90→.47ms为临时空目录，不当实际IO/WCET/现场成功认证。
- 当前车端代码683e0243ff79a3fcb60d782e048b993a6aabcf6e，桥238a2c8f未变。完整backup-turn11-20261007-165653保存b26，所有好直道8ef/失败备份/队友原件保留；实际网页1550/1450/1650/1350与模型/配置/校准/凭据/启动命令保持。
- 部署与独立各10份fresh healthy/locked/owner与active空/M=S1500，独立末seq0/tick66679；新只读CLI初始1720/pre8/loop.12/motion_requested=false，defaultGET loop.08也核对。都是当次软件反馈，下次重读，不认证物理停稳。最新[机器记录](docs/vehicle-dynamic-turn-validation-20261007.json)，10原始/独立分析/profiler保留在work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-10/。
- root独占SSH/硬件，按上传规范推main不强推。**保护退出不自动重跑**，新的动力须用户新当次指令和已静止；已准备once11但没有执行。第一unknown紧凑目标左绕起段限3s，共用drive10，完整两桶S/比赛尚未验收，不用假pose、模拟圈数或场地特化。

## 新窗口接手入口（2026-10-07，10因80ms控制间隔退出，用户要求排查后提高上限）

**用户“开始测试吧”已消费一次10。b26车端8秒版本实际只presteer2、0动力，最后软件请求1510/目标1720；保存底盘舵反馈最高1500，不能称实际已采用1510。因autonomy_control_gap退出，不是8s、墙或车身距离门。用户最新“排查好了改了以后提一提这个上限”，正在仅newmode控制循环80→120ms并减状态查询锁内工作、加精确时延诊断；不自动发11。车端仍b26旧80ms。**

- 10 run 2tjwRPqHlqFT_d-5trWRXfph，pre2/motion0/coast0，last_success turnscan3554/pre_elapsed.0717986/settle0、body.310618m>.08/quality空；3554原ranges未保存，下一保存3555只state3554，不能倒填。终止reason在最新扫描/clearance之前的>80ms门，实际超限gap未记录，不能把observer150ms或controltick209ms当故障间隔。
- 最后6份fresh healthy/locked/owner与active空/M=S1500，controlseq4/tick355480→356444、lidar3558→3568增长；软件锁定不认证物理停稳。原始完整work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-10/，独立分析与纯profiler保留；真实compact候选15份全0/nohandover，未到1720或1560。
- 目标板30轮×16真实scan纯计算，无硬件动作：full update max27.048ms、wall match21.816ms，未复现80ms；既非WCET也不能证明实际原因。loop start间隔包含前拍运算、watch20ms等待、Console共享锁及OS调度。active GET已跳过墙preview，仍锁内360ray泛用admission+files目录IO；准备把目录移锁外，已有owner时先拒新admission而保留active真实clearance，记录gap/lock wait/tick walltime。
- 新120ms仅内部maneuver_sequence会话；default直道/旧turn-left仍80，controlfresh200ms/autohealth250/heartbeat300、初始1720/预打8、drive10/coast5、质量与车体8cm当前点门/stop锁存均保持。用户payload不能自设弱门。修复已冻结，342项Mac/63服务CLI/21独立滑行/车端42受影响专项通过；尚未部署/再arm，验证后成套备份锁定上传并按规范推main；新的动力须用户新当次要求。好直道/全部失败证据/队友原件保留，root独占SSH。

## 新窗口接手入口（2026-10-07，预打8秒版本已成套锁定部署；尚未再试10）

**最新用户“超时的阈值提高”已完成：newmode中性预打5→8s，初始1720/完整采用3tick及1.2s后1560；forward10/coast5、质量/时钟/车体距离/舵机渐变保持。源码b26e5f5已成套完整备份/锁定更新，14安装哈希一致。用户已重新登录SSH，338项Mac/38项车端纯计算确认通过；没有新的arm/drive，不自动发10。09授权已消费，只有预打到1720、0前进，5s超时；不能记左转或绕行成功。**

- 09 run sdqgPckUmFgeYAJDpfnYNeuE，presteer139/motion0/coast0/末servo1720，全程motor1500；首次保存1720的pre_elapsed3.300遇front_sparse，原3新帧/300ms恢复后约3.755重做1.2s采用等待，最后4.84090等1.08648，5.00471锁定。这是中性预算不足，未给1560。用户观察转一下后回中不动，已据软件记录解释；没有用旧09状态恢复会话。
- newSequence固定presteer8，原TurnMotion/defaultturn-left仍5；server报告/期限按实例8+drive10=18s、CLI8+drive10+coast5+1等待，外层once监督预备28s。没有改重复scan推进、质量恢复、PWM10/100ms、1720初始完整采用/3新tick/1.2s、目标确认/手动急停/收油后不正/no rearm。
- 338项Mac Python/25序列/59服务CLI通过，目标板38项受影响纯计算通过（SSH掉线时未先声称通过，重新登录后取回38/40.55s OK）。真实09时限缺额反事实只说明尚差约.114s，不虚构后续扫描/物理成功。第一目标unknown compact-object左绕起段3s/累计drive10，完整两桶S/比赛仍未验收。
- 当前车端源b26e5f5102d56a4f59ed2f96978d10ba8a160b06，桥238a2c8f未变；完整backup-turn10-20261007-162756保存41f旧5s版，全部早期失败/直道好8ef备份和队友文件保留。实际网页1550/1450/1650/1350、模型/配置/校准/凭据/启动命令保留。重连读到新boot后重新读取，不沿旧seq141。
- 部署及独立各10份fresh healthy/locked/owner与active空/M=S1500，独立末seq0/tick54427；只读newmodeCLI预览ready=true/initial1720/pre8/drive10/coast5/motion_requested=false，oldmodepre5也核对。这些是当次软件反馈，下次重读，不当物理停稳证据。记录见[机器记录](docs/vehicle-dynamic-turn-validation-20261007.json)；09原始完整目录work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-09/。
- root独占SSH/硬件，按上传规范推main不强推。**保护退出不自动重跑**；新一次动力须用户新当次测试指令及车辆已静止，重新读fresh再单轮。已准备left-cone-trial-once-10.py但没有执行，不从“重新SSH登录”推导新的动力授权。

## 新窗口接手入口（2026-10-07，09达到1720但预打5s超时，用户要求提高预打期限）

**用户确认静止并明确开始新一轮，已消费09授权；41f7680先完整预打到1720，但因front_sparse恢复与1.2s采用等待遇到总5s期限，仍0前进/0绕行，随后锁定1500。用户最新“超时的阈值提高”，本轮仅newmode中性presteer5→8s，drive10/coast5不扩。源码已冻结，338项Mac Python/25序列/59服务CLI通过；车端38专项结论因SSH掉线未取回，不能记passed。车端仍41旧5s，不自动发10。**

- 09 run sdqgPckUmFgeYAJDpfnYNeuE，presteer139/motion0/coast0、末servo1720，reason=left_turn_presteer_timeout，trigger2271；所有记录motor1500，没有给1560。第一次保存1720时pre_elapsed3.300同时front_sparse；3newscan/300ms质量恢复后约3.755重启采用等待，到最后保存4.841只等1.086s（需要1.2），5s先到。PWM仅新scan更新有真实间隔抖动，但本轮按用户指定只提高中性预算，不改重复frame步进/质量或成熟要求。
- 末6份新鲜反馈seq141/tick228326、healthy/locked/owner与active空/M=S1500。用户观察“转了一下然后回中不动了”，与中性预打→超时锁定一致，不认证物理停稳或比赛完成。09完整原始文件work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-09/。
- 新mode默认初始1720/10PWM100ms，未完整采用及3tick/1.2s前motor1500；newSequence presteer8，server deadline/duration/CLI与外层监督预算一起匹配，默认turn-left仍5，forward10与coast5/no正after收油/no自动rearm不变。真实侧向目标/unknown语义/车体.08m门不变；8s修复已完成Mac验证；SSH复用Broken pipe/连接超时，已向用户请求重新登录，车端专项需取回/核对，再完整备份默认锁定上传。尚未部署8s修复，不能记实测绕行。
- root独占SSH/硬件，验证后按规范推main并成套完整备份锁定更新/哈希/设置/新fresh复核。09本次授权已消费，故障退出不自动再解锁，新动力须用户新的当次指令。全部好直道备份/历史失败/队友文件保留。

## 新窗口接手入口（2026-10-07，默认先完整预打1720已验证并锁定部署；08未前进）

**最新用户要求“一开始舵机角度必须大点，不然后期调整来不及”。修正版41f7680已成套备份/默认锁定部署，14安装哈希一致；只读新mode预览初始目标1720，当前M=S1500、armed=false，未重新arm/发09。上一次“接上先测试”已执行08，但中性预打到1540便在真实scan637因墙歧义退出，0前进/0绕行，不能记成功。保护退出不自动重跑，新的动力试验须用户新的当次要求。**

- 仅turn-cone默认初始1720（原用户允许上限），保持10PWM100ms，实际完整采用1720/至少3新反馈/1.2s后才1560；显式1700仍兼容，默认turn-left保持旧逻辑。运行后实时墙/相对目标动态调舵，未设固定档。新sequence中性预打短时wall歧义只有此前可靠left、fresh/armed/M1500/当前servo已采用ACK时才hold该servo，清settle/count/start_ready；不续旧geometry或5s期限，恢复真实唯一墙后重新成熟。drive300ms窗、硬障/帧龄/心跳/coast不再正PWM保持。
- 332项Mac Python、22序列/56服务CLI、车端32受影响专项通过；真实637/639fixture与mock中性控制验证软件等待，不是实际重跑。实际637片段角差.3072°/rho1.50cm，ray316越两候选误差界，同面判false；保留歧义，不强并墙。下一保存639在206ms后恢复唯一，638未保存，不写固定恢复帧数。新目标身份仍unknown，禁止假pose/模拟圈数/赛道特化；第一目标起段3s/共享10s动力/最多5scoast，完整两桶S仍未验收。
- 当前代码车端41f768034203fda1031fe862fe1ba1249b6c2d2b，桥238a2c8f未变。完整backup-turn09-20261007-161045保存894，backup-turn08-20261007-155858保存d7，好直道8ef备份及全部真实失败/队友文件保留。实际网页设置1550/1450/1650/1350、模型/配置/校准/凭据/串口持有与启动命令均保留。
- 安装后10份及独立10份fresh healthy/locked/owner与active空/M=S1500。独立末seq0/tick37159，boot lJFqXCQaDu3yhU60O_G5_A；只读CLI随后default1720预览ready=true，motion_requested=false。这些是当次读数，下次重读；未凭1500反馈认证物理停稳。08原始文件在work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-08/，seq637完整保留；记录见[最新机器记录](docs/vehicle-dynamic-turn-validation-20261007.json)。
- root独占SSH/硬件，按上传规范推main不强推。新的当次要求到达后先重新核对摆放和新鲜反馈，再一次连续试验，失败不自动重复；目标成熟/交接及实际现场结果分别记录，不能直接宣称可以跑赛程。

## 新窗口接手入口（2026-10-07，左转+侧向目标已接车，08中性预打墙歧义退出，修中性等待）

**用户“接上先测试一下”授权已消费08。89470fb八Python+匹配桥成套备份锁定部署并实测；这次只预打1500→1540，电机从未给正PWM，seq637在中性presteer因outer_wall_ambiguous退出，未前进/未绕行，不记成功。当前修中性预打短时墙关联歧义；用户新要求一开始舵角必须更大，新模式默认预打1720（原上限），必须采用成熟后才1560。修好后锁定部署，不自动发09。**

- 08 run XeRgI41LFqoKX6R044MnEclk，presteer12/motion0/coast0，末servo1540，当前body距.318298m>.08，与5cm净距门无关；真实原scan637已保存在feedback.jsonl（同条state尚显示636，下一条639显示terminal637），不能误称raw缺失。末6份新鲜控制seq14/tick65163、healthy/locked/armed=false/owner与active空/M=S1500；这是软件反馈，不单独证明物理停稳。
- 八Python/启动入口/目标例/三前端/桥14安装哈希一致，完整备份backup-turn08-20261007-155858保存d7；实际设置1550/1450/1650/1350保持，匹配桥238a2c8f与模型/配置/校准/凭据保留。新mode turn-cone仅试第一真实点云紧凑目标左绕起段，语义unknown，初始1700/1560、动态gain180/趋势最多释放20%、10PWM100ms、共享10s动力/起段3s/coast5s，比赛两桶S未完成。
- 323项Mac Python通过，车端首91与最终受影响23纯计算通过；真实07动力末无隔离目标，停后才有，不能截墙/造桶/假pose或计数。新观察计时600调用max3.318ms，不是WCET或物理验收。08原始文件work/vehicle-dynamic-deploy-20261007/left-cone-trial-20261007-08/完整保存。
- 正在仅newsequence中性presteer增加等待：当前命令被新鲜反馈实际采用、电机1500、此前可靠left存在时，墙歧义hold当前servo并清settle/start_ready；新扫描继续校验、不续旧geometry或5s预算；唯一真实同墙恢复后重新完整成熟才可1560。默认TurnMotion、drive最多300ms旧目标窗、硬障/时钟/急停/收油后不再正PWM均保持。初始目标默认由1700改1720，只newmode，显式1700兼容；332项Mac Python/22序列/56服务CLI通过，车端最后32受影响专项已通过，尚未部署。
- root独占SSH/硬件，完成相应测试、更新本入口/验证记录、按规范推main、完整备份默认锁定更新并核对。**保护退出不自动重跑**，下一轮必须用户新的当次测试要求；保留好直道备份/所有真实失败证据/队友文件。

## 新窗口接手入口（2026-10-07，07初始1700已实测，正在接左转与真实侧向目标绕行）

**最新用户明确“再转一点，现在接入绕转，左转+绕桩一起测试”“你接上先测试一下”。07授权已消费，不能把下节待07当当前状态。已完成并验证一次受限的左转→真实点云侧向目标→第一目标左绕起段，尚未发08动力。不是完整两桶S比赛验收。root独占SSH与硬件。**

- d7b637c六Python/匹配桥238a2c8f已成套锁定部署，12安装哈希核对；完整新备份backup-turn07-20261007-152029保留前版57c。07 run FtBBUIKPDCAq8-xYTzswaV4u：presteer118/motion35/steer27/coast0，预打1700采用成熟后1560，drive1.18636s、动态舵末1634，scan1378因left_turn_outer_wall_ambiguous退出；body距离.120441m>.08，不是5cm净距门。无绕桩/未完成。六份新鲜锁定反馈seq155/M=S1500，物理停住与碰撞问答未单独收到，不能称无碰撞验收。
- 最新SSH复用已只读连通；重新读取6份控制/雷达递增，新鲜control seq155/tick1557620、scan15598/at1557482，healthy/locked/armed=false/owner与active空/M=S1500，网页设置1550/1450/1650/1350。当前体距.320735m、turn_ready=true；这些是当次反馈，下次重新读。
- 新模块compact_target.py观测真实邻ray隔离紧凑目标并跨新扫描确认；semantic未知，不用simulation circle参数/假body pose或圈数。maneuver_sequence.py复用先中性渐预打1700+实际采用3tick/1.2s，1560后实时墙反馈增益180、趋势最多释放自然误差20%，真实相对目标range/bearing动态左舵；侧前或侧向可靠目标可交接，不等待正对桶。服务/CLI明确turn-cone试验，累计10s动力预算、不自动rearm，车体净5cm+3cm参考余量=.08/健康/质量/时钟/coast仍有效。最终源码已冻结，323项Mac Python通过，车端首91纯计算通过，最终受影响23项亦通过；不能当已部署或实车效果。
- 按上传规范完成对应测试、更新此入口及验证记录、推main；新增模块必须完整传齐并更新启动依赖白名单，备份/默认锁定/哈希/设置/新鲜反馈核对后消费用户本次08授权一次。保护退出不自动重跑；记录实际是否交接及现场结果，不宣称跑完整赛程。好直道备份、全部真实失败证据及队友文件保留。

## 新窗口接手入口（2026-10-07，06墙歧义退出，初始1700与片段边缘修复已全验证，待上传/单轮07）

**用户最新要求“编译好了上传车辆继续测试”，已明确初始1700，并确认回到斑马线后/完全静止/车头朝原直道；07新单轮授权待消费。车端57c7f96的车身净距5cm+原3cm参考余量已成套部署并执行06，未到位，真实wall_ambiguous退出（不是距离门），用户确认停住无碰撞。当前Mac修复已完成277项Python与72模块/46服务专项；车端118专项与计时已通过，再备份默认锁定上传后执行07一次，保护退出不自动重启。SSH曾失效，用户已重新连接，已完成新目标板纯计算验证；不能把旧seq178当实时状态。**

- 06 run7lRqEFqM4pnIQmcA-JBK7kI8，presteer137/motion39/coast0、26舵变；预打1690采用13新tick/1.31286s后1560，软件drive_elapsed1.28682s，最后服务servo1643/内部terminal锁定1500，软件reason=left_turn_outer_wall_ambiguous，seq1491原始scan完整保存。body min.087138m>guard.08、stop_requested=false，旧1m/径向40/侧30门inactive；completed/entry/alignment=false，无绕桩。
- exact1491有unknown8/9，旧11边双侧PCA窗把真实已知ray10–14也割裂，候选47.123°/rho−1.5847有限成熟0组(主段13点)、52.449°/−1.6399成熟16；same_surface=None而不是证实两墙，fallback差5.326°/5.522cm超过2°/5cm，留下2群。ray10–32实际23连续点basic10–31全True，独立PCA残差max1.14cm/span.694m。新edge clamp只在每条真实basic run内选完整6点窗口、不跨unknown/真实深跳，成熟18/21、同面true/unique1；原20°局部角/4cm跳/16点/4shared/allinterval/complete-link与rho关联门不放宽，真双墙/尖折/gap/短run负例保留。
- 新可选initial_presteer_pwm=1700只用于M1500的预打舵，仍10PWM/100ms、目标actual采用+3newticks+1.2s后1560；drive恢复原实时_target，不固定1700。自然目标<=1500仍不能起步，bool/float/null/1650..1720之外拒绝。CLI只读GET preview query，无setter不改manualsettings；默认请求完全兼容。moduleb1d820e8/server17e88484/CLIedbb5388，bodyhelper092dd6eb和桥238a2c8f未变，目标板118纯模块/服务通过；旧实际7帧对unique、64候选50轮median57.704/max69.070ms，exact1491完整16原候选unique1，median18.017/max20.112ms（皆非WCET/物理验收），未扩80ms门。独立749slowPCA位掩码零差；277全Python通过。边界探索2个测试假设不适用（float5s减法尾差及1720需2.2s回中），采用明确Unknown motion/2.4s观测及exactdeadline补核1700/1720均neutral/无重启，保留原探索记录不伪称63全过。
- 用户进一步明确不必正对桶，侧前/侧面对真实桶可继续左转衔接绕桩；这是目标要求，当前实车S链仍未接。06停止后有持续隔离小物体候选，单纯原circle simulation(.003误差/minradius8cm)无法验证实际6.4–8cm截半径/噪声，不认定bluecone/假pose或圈数、不从wall歧义直接切绕桩。先本轮1700+识别修复真实验证，侧面接管需真实目标/通过与车辆曲率证据，仍不宣称比赛链完成。
- 06独立6份反馈seq178/healthy/owner=null/active=null/locked/armed=false/M=S1500，用户实际停住无碰撞确认另存；原数据work/vehicle-dynamic-deploy-20261007/left-turn-trial-20261007-06/（exact1491及before/after保留）。车端新body backup-body-margin-20261007-142046保存16c；全部历史/好直道8ef备份及队友文件保留。最新机器记录见[记录](docs/vehicle-dynamic-turn-validation-20261007.json)。root独占SSH/硬件，验证/成套上传/实际设置和新鲜反馈核对后消费一次07授权，按上传规范推main、不强推。

## 新窗口接手入口（2026-10-07，05预打后部分左转，车身净距5cm已验证，用户授权上传后直接06测试）

**用户新摆放到斑马线后并明确开始测试，已消费一次05授权。先预打成熟、再1560前进约0.957s，未对齐左通道，前1m矩形门退出；用户明确已完全停住、没有碰到挡板。之后用户要求保护距离调整，最新“改成车身四周5cm”覆盖先前雷达中心20cm选择；当前车体近距profile已验证，仅turn/将来绕桩内部使用、直道旧门保持。用户随后明确“编译完上传车上直接测试，已放到斑马线后面”，获得待消费一次06授权；成套默认锁定上传与哈希/设置/新鲜反馈核对后再执行。车端仍16c0956锁定1500，profile尚未下车。**

- 05 run nemsUbfF_3eTW097pAUtBGCT，presteer149/motion33/coast0、25次舵变，实际软件采用servo1681、14个新控制tick和1.2846s成熟后才1560；45个保存反馈/7个1560采用样本。末命令servo1632/目标1593，参考墙约70.71°仍未对齐，0 alignment/entry/completed=false。目标1640无趋势项→1593含趋势项差47PWM，不是已经回1500，也不能凭其判定实车舵角提前收回。
- 软件trigger对应4215（state.clearance证据），其原始ranges未被observer捕获；30cm半宽的generic前向1.8549m、侧距.30395m通过，turn32cm半宽失败，说明新增2cm侧带点触发前1m门，不能倒填精确ray。相邻4216近左入场板点距当前测量车体矩形最近.10189m，旧横侧距.27186m已低于30cm；不能仅叫无风险误停。最终6份新鲜反馈seq184/healthy/locked/armed=false/M=S1500（control tick421474→422388、lidar seq4217→4226），用户物理停住无碰撞另存；没有绕桩/无自动rearm。
- 新profile用实量雷达前21/后20/轮胎外沿±17cm形成当前矩形，当前点到矩形欧氏距离要求净5cm+原粗测参考3cm=.08m；正轴雷达门前29/后28/两侧25cm，四角也检查。不要误写raw20、5cm包含车头尺寸、或已证车身未来扫掠/移动停距。只turn_trial内部显式选择，straight/manual/formal旧门保持；绕桩实车链尚未接，不虚构已经可执行。
- 实现时前1m矩形/旧radial40在maneuver profile不当实际门，改当前body-distance硬退出；未知点、质量/帧龄/控制时钟/心跳/PWM/先打舵/正常coast不回正PWM/急停保持。不得由camera_required=False或客户端payload选择弱profile。256项Mac Python、车端75项helper+97项turn/service、独立1440数学边界/29集成检查通过。helper SHA092dd6eb、server16d56828、turn_motion2a1e0cab；turn candidate/coast/alignment侧余量也同步25cm，rho关联±30cm不变。05停后原扫描已有距矩形6.4–7.7cm，新门会停，M1500不能证明已即时物理停稳。验证后待成套部署前仍用16c车端；root独占SSH与硬件，helper/service代理各独占文件并独立审计。
- 见[05及新profile记录](docs/vehicle-dynamic-turn-validation-20261007.json)，真实05数据在work/vehicle-dynamic-deploy-20261007/left-turn-trial-20261007-05/，小型原扫描几何验证body-margin05-raw-fixture.json为测试数据，禁止写入此场地坐标控制常量。备份/直道好版本/队友原件保留，按上传规范验证推送main，成套默认锁定更新并核对后消费用户最新一次06授权，退出不自动重启。

## 新窗口接手入口（2026-10-07，04无舵直进撞墙失败，先打舵更正已验证并锁定部署；不发车）

**用户最新多次明确“继续修改/继续”，并纠正“转弯时应该先调整舵机，运行中决定继续往左或回中”。142dc02已成套部署并消费一次04授权；新无舵直进策略实际没有左转，用户明确“没有左转直接撞墙了”，这是碰撞失败，不能记成功。用户确认已完全停住。root补发全局stop获得ACK，6份新鲜反馈seq98/locked/armed=false/电机舵机1500。更正程序16c0956已成套备份/锁定更新并独立复核，本轮没有arm/drive，不再沿用04授权启动。下节“尚未部署/授权未消费”是历史，不能当当前入口。**

- 04：run K3Xx7ry2wg079ZPMO-B3JYmJ，presteer43/motion52/coast0、steering_changes0，motor1560且舵机始终1500；software drive_elapsed1.6679s。最后服务scan925仍stage=approach/turn_released=false/corner_current=false，保存的旧端头0.09355m仅是历史字段，完整opening已丢失；代码继续直进，最终probe_obstacle_in_straight_corridor/completed=false。用户确认实际撞墙，无左转/无绕桩；仅软件锁定反馈无法倒推碰撞前物理已停。
- 本次更正要撤除corner gate/straight-approach推进，恢复553的先渐变左预打舵：电机1500，舵机10PWM/100ms到当前真实目标，控制反馈采用目标后至少3个新tick和1.2s响应余量再1560；行驶中按新点云继续左打或渐回中，不固定最大档。正确的soft-board归并、native端头真实射线校验和中性front_sparse原3帧/300ms恢复保留；5s预打/10s前进/15s总期、原硬障/帧龄/急停/coast不回正PWM均不放宽。软件更正已完成：无corner/straight-approach控制路径，初始目标1500保持中性/start_ready=false/5s不延期，drive中新墙回中允许。237项Mac Python、目标板88项无设备模块/服务测试、独立128项turn/coast通过；原04前15份完整raw观测和实际control回放均presteer/motor1500（旧servo1500不算目标1681采用），不把回放称新物理轨迹。新module SHA28c38f31235dd46b08f00977a54fb81658c19a2ed638b175e04a9f03ebaec7d2、server5103f1d471c805c40ae9751be14c31145db5c137b831058f8636631ba564c42d，已成套锁定部署；不能当实车左转验收。
- 车端当前更正源码`16c09560b35674656050c9ff5b3c7d05bf66d948`（server5103f1d4/turn_motion28c38f31），匹配桥仍238a2c8f，Rust/前端未变。12安装哈希一致、包13项/目标板7Python静态编译/依赖/无设备桥通过。新完整备份`/home/bianbu/xt-stcar-console/backup-presteer-restore-20261007-134438`保存替换前142失败程序，用于复盘；`backup-left-entry-20261007-113948`保存553、`backup-soft-wall-20261007-110221`保存19dd、`backup-dynamic-turn-20261007-102438`保存直道8efc47b，全部保留，不把失败版本当好备份。网页1550/1450/1650/1350、模型/视觉/校准/凭据/启动命令均保持。
- 安装后10份及独立10份新鲜反馈均healthy/owner=null/active=null/locked/armed=false/M=S1500；独立末control tick95156/seq0、lidar seq951/at_ms94993，boot3QWZyaRoJ1CPo_HIuUpm2Q。设备各一匹配桥持有（当时/dev/car7361、/dev/laser7365），下次重新读取。只读CLI后续一帧preview left target1682/M=S1500，真实端头诊断current；独立state末帧turn_ready=false/left_corridor_unknown，随后CLI为true，不能记持续可用或真实转弯通过。这轮没有arm/drive、没有新动力试验。
- 碰撞后追加stop新鲜反馈control tick228470→229399、lidar seq2286→2295增长，最后seq98，healthy=true/owner=null/active=null/locked/M=S1500；这些是当时读数，下次重新读。物理停住来自用户确认，不称点云停稳/刹停模型通过。原始碰撞扫描/CLI/observer及人类陈述在work/vehicle-dynamic-deploy-20261007/left-turn-trial-20261007-04/，stop记录在collision04-stop-stdout.json。
- root独占SSH/硬件；模块和服务代理各自独占代码/测试，独立审计只读。已完成成套备份锁定更新、hash/settings/freshfeedback复核，文档按上传规范推送main；不再发正PWM，下一轮须重新摆车并明确新测试指令。好直道备份/队友原件保留。详见[最新失败与更正记录](docs/vehicle-dynamic-turn-validation-20261007.json)。

## 新窗口接手入口（2026-10-07，用户放回起点，实时端头起弯修复已验证，待成套部署及授权重试）

**用户最新明确“我放回斑马线位置，你改好后继续重试”。已获得修复、成套锁定更新以及之后一次新连续左转试验的授权；这次新授权尚未消费。当前车辆锁定，root是唯一SSH/硬件操作者。通用起弯时机、同圈端头回波证据与中性预打舵短时缺测等待已完成验证，尚未部署/消费04授权；没有继续用553代码发新动作，没有删/缩前1m或侧30cm门。下节02/03测试记录与实物停住无碰撞确认保留。**

- 已新增同圈原生incoming_left_end_support射线/点绑定和known_open_fraction。端头仅是当前候选观测，不认证物理板端/完整扫掠；不得拷此场地0.53m为参数、取外墙跟踪保存的旧incoming_left_end字段或把缺测当free。
- 候选控制先在1500舵采用反馈/原1.2s余量后1560直进接近；用当前开口/端头及车头两角沿入场轴投影减去10cm净余量+3cm粗测余量，连续新帧证明后才渐变左转。前21cm/轮胎外沿左右17cm为实量，不填未测曲率、车速或后轴外参；该前缘门只延后起弯，不声称整车尾已过/扫掠安全。接近阶段不增加出口对齐证据，累计前进最多10秒不续命，停止收尾后不再正PWM/不自动重解锁。
- 中性presteer只有fresh front_sparse及其原3帧/300ms恢复等待可hold当前舵/1500，同会话真实扫描仍继续校验，不伪造时钟或内部drive；5秒presteer/15秒总期限不延期。前进/coast缺测、真近障、失联/stale/旧帧/控制gap仍原退出。236项Python、23项Rust路口、车端87项无设备模块/服务专项、fmt及core/bridge clippy通过；71实际扫描61旧/61新goal且无丢失，端头逐点绑定原ray，Python全部接受。目标板纯关联64候选50轮中位54.99/最大68.71ms（非整个控制循环WCET），真实7帧对最大0.333ms。新桥238a2c8f交叉/LP64D/PIE/最高GLIBC2.34通过；新server SHA083eba8c、turn_motion30f317b5，Rust85e15f75。不当已部署或新实车效果。
- 验证完成需编译匹配新Rust桥，备份并传齐六Python/桥/三前端/目标示例/入口，默认锁定检查哈希/实际网页设置及新反馈。之后消费用户本次新单轮授权；途中用户新指令优先，不从故障退出自动重试。

## 新窗口接手入口（2026-10-07，软板修复后两轮测试，已前进但未到绕桩入口）

**用户新要求“开始测试吧，我已把车放到斑马线后”，随后02只打舵退出后又明确“没看清，再来一次，车没动”。已分别消费02、03两次独立单轮授权；03实际1560前进后左前挡板端头触发保护，未完成左转/未到绕桩入口。用户确认“前进了，但没有转到适合绕桩的位置”，并确认已完全停住、没有碰到挡板。不得自动再解锁或记成功；本轮未改软件/参数。下节锁定部署、软件验证及备份仍有效。**

- 02：run7G-QyRaOkk0q57SSThjPlXkX，presteer55/motion0/coast0、12次舵机指令变化，反馈只1500电机/舵机1500→1620；seq2701的正前5°–16°连续12缺测触发front_sparse/turn_perception_unavailable，2702在103ms后前向恢复。原门未知总数6/最大连续3未放宽；这不证明缺测区域为空。跟踪有效至2700，没有outer-wall歧义。6份末反馈seq57/健康锁定1500，用户说车没动。
- 03：run3P8QNDJheOJfd64hCz7HI5U2，presteer130/motion29/coast0、24次舵机指令变化，6份实际1560采用样本；舵机反馈1500→1677，前进中1677→1649动态调整。软件drive_elapsed约0.959s，点云参考墙方向约90→75.66°，这些不是实测车速/位移/轮角或完整转角。seq4180真实bin309/311/312/313回波range0.409–0.425m、X前0.257–0.290m、Y左0.309–0.318m进入原前向1m/半宽0.32m框，reason=probe_obstacle_in_straight_corridor。点群来自进场左挡板前端，距离跟踪远外墙约1.75m，下一帧仍有近点；不当孤立噪点，也不能凭其位于侧面声称转弯扫掠安全。
- 最新独立6份反馈healthy=true/owner=null/active=null/locked/armed=false/电机舵机1500、seq218，control tick417802→418778、lidar seq4183→4192递增。用户物理停住无碰撞确认另存，未标点云低运动认证。当前源码仍553b7e6/turn_motion d7129/匹配桥7f13145b，网页设置仍1550/1450/1650/1350，左转内部1560、舵机≤1720及渐变/所有原门保持；两轮均无绕桩、completed=false/entry_confirmed=false。
- 下一项是通用左转起弯时机与内侧墙端净空：用当前实测开口端头及本车曲率/扫掠证据判断，禁止删除1m/侧向门、写死此场地坐标、把模拟曲率当实测或直接固定最大舵。02还暴露中性预打舵遇瞬时front_sparse立即退出，可考虑同会话中性等待原3帧/300ms质量恢复；本轮仅记录诊断，没有改该行为或自动恢复正PWM。
- 用户随后建议往哪转就放宽哪侧距离；已说明当次首先触发前向1m/半宽32cm框，侧门是雷达横向30cm（已量得轮胎外沿17cm+净距10cm+参考测距余量3cm），不是独立固定15cm。4180横向约30.87cm、下一帧26.81cm，轮胎外侧间隙后者约9.81cm、尚未扣余量。按实测转弯扫掠调整区域有用，不能凭侧入墙就忽略真实贴近；本轮没有改门限/程序，也没有新发车。
- 见[完整两轮记录](docs/vehicle-dynamic-turn-validation-20261007.json)。原始记录在`work/vehicle-dynamic-deploy-20261007/left-turn-trial-20261007-02/`与`...-03/`，原始数据/凭据不推送。按上传规范更新交接与验证后推送main；所有好直道备份和队友原件保留。新的测试仍须明确当次指令。

## 新窗口接手入口（2026-10-07，软挡板点云关联修复已验证并锁定部署）

**用户最新要求：比赛左转侧挡板是软板、端头会形变，墙端不能固定，允许小角差，主要按当前点云是否连续平滑判同面。已实现并推送修复源码553b7e6，成套备份/部署到车端；本次修复部署没有重新arm/drive。之前一次连续左转授权已消费，保护退出后不自动重新解锁，下一轮须新的当次测试指令。下方历史参数和“待明天部署”不是当前入口。**

- 今天先完成19dd355成套部署与本机缓存清理；随后用户要求开始左转+绕桩，并确认车在斑马线停车位置、完全静止、车头朝原直道。首轮实际arm采用、1个1500预打舵拍、0前进拍/0舵机变化，在seq5413因left_turn_outer_wall_ambiguous退出，独立6份反馈seq3/locked/armed=false/电机舵机1500。未转弯、未绕桩，不记验收。捕获seq5412复现角度/rho门把不重叠的近/远有限支持同时关联；实际终止seq5413没有单独原始扫描，不能倒填。
- 修复既检查相邻帧有限支持重叠，也按**当帧真实ranges**相邻回波连续、6点局部PCA方向及法向误差、报告残差与共享回波归并软板。小角差本身不是两面证据；墙端/有限支持随每份接受的新扫描更新，不绑定原端点、固定90°或赛道坐标。缺测不插值，真实分离平行墙（含3cm）、尖折、缺口保留区分；每个组成员须相容，防宽误差拟合传递强并两墙。同seq/发布时间却改原始ranges拒绝，不续旧时钟。
- 验证216项Mac Python、车端46项无硬件专项通过；156组独立优化前后完整匹配/支持位掩码零差。目标板7组真实相邻帧都唯一关联、最大0.298ms；64候选50轮纯关联中位49.565/p95 55.433/最大56.567ms。最初未优化最大89.667ms超80ms已通过精确去重/PCA窗口及同角投影缓存解决，**没有扩大80ms门**；采样计时不是WCET或实车验收。Rust/前端未变，沿用此前匹配桥与验证。
- 最新车端源码`553b7e60dc69f27962a42b819804906a52028b8a`，turn_motion SHA `d7129d63aa78220f644ce15de80b1328675d27355a0de8ee65e5512b028880af`；匹配桥仍`7f13145b4c9aeb4d3561580442d18fd94c75707f3a4177964093076eae88c0cd`。六Python、三前端、桥、目标示例及入口脚本共12份安装哈希匹配，包13项/目标板7份Python静态编译/依赖/无设备桥检查通过。活动目录`/home/bianbu/xt-stcar-console/20260917`。
- 新完整备份`/home/bianbu/xt-stcar-console/backup-soft-wall-20261007-110221`保存替换前19dd活动目录/入口/包装脚本/视觉/私有访问配置/服务单元/运行设置。原`backup-dynamic-turn-20261007-102438`完整保存直道run04源码8efc47b；所有车端旧备份未删。模型、视觉文件、雷达校准、凭据及既有启动命令/文件哈希保留。
- 最新安装后10份及独立10份反馈healthy=true、owner/active空、locked、armed=false、电机/舵机1500，boot a5SaLtgB0384EwM6HBAT7Q，独立control tick33559/seq0、lidar seq334/at_ms33405递增。这些是当时读数，下次重新读取新鲜反馈。入口check-only私有捕获去token、turn-left仅GET ready=true，不是动作验收；/dev/car控制桥18151、/dev/laser雷达桥18155各一进程持有。
- 实际网页设置保留**1550/1450/1650/1350**，不套昨天1560内存值；左转试验内部前进1560、中性1500，舵机候选1270–1720、左转1500–1720、普通10PWM/100ms、反馈后预打舵1.2秒余量。右1270未实测；滑行只1500受限纠偏、禁止反转刹车/固定档/场地特化。雷达/底盘/心跳/近障碍和正式完成证据门不变。
- 必须保留直道好版本：本机`work/vehicle-junction-coast-test-20261006/release-process/`的8efc47b交付/展开源码/桥/复核、整个`run04/`现场证据和102438完整车端备份。旧`backup-coast-process-20261006-161139`是部署8ef前备份，不能误记run04源码。用户认可run04现场效果，但软件低运动/停稳仍未确认、completed=false。
- 已按用户要求删除199条本机缓存/垃圾路径（165缓存目录、32个Finder文件、2个逐文件匹配原ZIP的解压副本）；原ZIP及不同内容历史tar保留。工程约20.60→7.01GiB，当次可用空间增加约14.5GB；765份保留哈希一致（含117直接产物、5队友原件及直道证据）。当前包/模型/工具链/有效备份均保留，后续构建可再生成缓存。
- 实车曲率/舵角/低运动/滑停未完成物理验收，任意XY停车仍real_pose_missing，左转后连续绕桩真实执行链未串接。只有左转实验入口，不将离线Rust绕桶/模拟位姿或曲率代替实车链。本车实量轴距25cm/雷达前21后20左右17cm、单蓝桶模型和目标标签顺序0crosswalk/1blue_cone/2red_light/3green_light保持，训练由队友负责。
- 详见[首轮/软板修复/部署记录](docs/vehicle-dynamic-turn-validation-20261007.json)、[初次部署与清理](docs/动态左转车端部署与缓存清理-20261007.md)、[动态接口](docs/动态左转与测试目标接口-20261006.md)。原始证据/新旧包/只读测试与部署脚本在`work/vehicle-dynamic-deploy-20261007/`，凭据/产物/缓存不进Git。每次修改先按上传规范fetch、验证、明确暂存并推送main，保留队友文件不强推。

## 新窗口接手入口（2026-10-06晚，Mac动态左转完成，2026-10-07待部署）

**用户最新要求“先在mac上弄好，弄好后明天再部署到车上”。当前收尾只在Mac完善、验证、推送main和准备成套文件，不再连接车辆、部署或发新运动指令。新动态程序尚未下车、未执行连续转弯，绕桩实车链仍未串接。明天先取得新鲜状态并正常备份、锁定部署；不能把今天的小段/舵机静态试验当新程序实车验收，不自动沿用已经消费的动作授权。下方分段状态/范围是历史。**

- 当前本机新增一次连续动态左转实验入口`turn-left`；前进1560、中性1500，用户选定舵机候选1270–1720，左上限是1720而非1730，右1270未单独实测，不声称机械对称。普通输出最多10PWM/100ms；先在1500下预打舵、采用反馈后留1.2秒试验余量，再进入前进。预打舵最多5秒、前进最多10秒、滑行最多5秒；不是按固定时间判完成，不固定一个大舵档跑完，也不反转刹车。
- 真实雷达给前方左出口目标及任意朝向墙/通道候选，起点可以尚未进入出口。当前保存的seq14830实际目标来自墙切线93.834°及真实ray309，不用固定90°或场地坐标；数据中的(1.272373,1.571250)m只是该帧观测。跟踪同一外墙、动态重投影、通道接管及趋势释放；车后支持中点不再造成大左舵回拉。停车只报告对参考墙/通道的观测对齐，未识别真实桶入口就不标到桶，不能据此称完成绕桩。
- 新turn模式只以真实lidar/control准入，camera/YOLO不参与起步或继续门，旧模式原相机规则保持。单帧目标拟合缺失最多保留300ms最后有效目标，不续旧测量时钟、不增加对齐证据；持续失效/真近障碍/失联/急停仍锁定。原前1m、侧30cm、雷达300ms、控制/心跳/native时限保留，不用转弯后方豁免，不套直道前墙TTC/路口停车结束门。收油后始终1500，按新鲜几何1445–1555受限纠偏或渐回中，停止后不恢复正PWM/不自动解锁。
- 人工/模拟目标在显式测试源接口可登记、预览，目标不刷新传感器帧龄、不自带解锁。示例config/trial-goal-turn-exit-example.json为`simulated/turn_exit_align`，无需视觉模型即可给出出口对齐停车意图。任意XY/point_stop执行仍具体报real_pose_missing；目标来源允许不等于使用模拟车速/位姿作实际反馈。正式FinalStopGoal默认物理契约不变；所有试验仍candidate_only/physical_steering_confirmed=false/competition_navigation=false/completed=false，observed_alignment/test_sequence_finished只是几何实验结果。
- 当天已执行四个短动力段（两次启动拒绝未输出油门，随后250ms/1600一次、500ms/1600一次、500ms/1650两次）；最后动力独立seq120锁定1500。用户确认后两次左转更紧、停住无碰撞，但仍未对准左通道。后按用户要求只在电机1500下比较1650到1700步10/每档1秒、1700→1730、平滑小步到1730、单1730保持2秒；均恢复驾驶台默认锁定并保留1560/1450/1650/1350设置。用户观察1730可到位、约1秒，最后脚本还有约2.2秒主动缓升，不记成精确响应标定或机械极限。未另测右侧1270，未用新1720动态程序带动力。
- 车端最后核对四Python匹配8efc47b、旧桥515476df，正常范围1350–1650；新的stop_goal/turn_motion两模块尚未部署。未来必须备份后传齐六Python（server/autonomy_live/coast_motion/stop_goal/turn_motion/autonomy-control）、匹配新bridge、三前端、入口检查脚本和目标示例，默认锁定并核对哈希/设置/唯一串口所有者。保留队友模型、视觉配置、雷达校准、凭据及已有文件。
- 验证：204项Python、8前端通过；Rust20项junction/7项teleop通过，相关clippy、全fmt通过；新bridge RISC-V交叉与ELF通过（LP64D/PIE/最高GLIBC2.34）。194真实扫描离线回放125有一轴及反向候选、69未知，无多别名；新通道容许12°收敛与既有便携挡板规则一致，点数/内点/跨度/残差不放松。这些不是实车曲率、停距、停稳或完整比赛验收。
- 本车实量25cm轴距、雷达前21/后20/左右外沿17cm沿用，不用厂商305mm、未测后轴偏移、模拟速度/曲率表或场地特化路线。来源/边界见[动态接口](docs/动态左转与测试目标接口-20261006.md)，现场与验证见[机器记录](docs/vehicle-turn-cone-step-validation-20261006.json)。原始扫描/照片/日志在work/vehicle-turn-cone-readiness-20261006/，部署包/二进制不进Git。

## 新窗口接手入口（2026-10-06，左转绕桩改为分段响应测试）

**用户已恢复任务，放车到其称的斑马线停车位置，要求跳过直道、先左转再按赛题绕桩；随后指定绕桩PWM先1560、允许扩大舵机幅度，并明确“先一段一段的走测试”。不延续下方暂停状态。当前完成三个有界左转响应小段，尚未完成左转或自动绕桩；500ms/1600用户确认向左前行、停住无碰撞但转弯幅度不足；500ms/1650用户确认明显更紧、完全停住且没碰挡板。后续左转候选沿用1560/1650/最长500ms，不据局部效果宣称已对准通道或绕桩通过。**

- 本机与GitHub main同步08df957；车端复用SSH已恢复，六份新鲜反馈健康、locked/armed=false/motor=servo1500，桥tick/雷达seq增长。四Python哈希匹配8efc47b，stop_goal.py仍未部署；车端仅straight_probe/straight_segment/to_left_junction，competition_supported=false。左转/绕桩Rust算法仍未接真实执行链，不能把本轮受监督响应探针当正式导航。
- 用户授权减少冗余审查；没有重跑整套代码测试、扩大原有保护门或改模拟认证标志。本轮用现有受控驾驶接口作有界响应探针：电机1560，初始左舵1600、最近1650（PWM，不是实测角度）。初段250ms/单正向请求；用户明确选择只把单段上限延长到500ms，500ms段以新鲜反馈刷新原250ms指令租期、总定时器仍500ms，每段只arm一次，不自动重启。近障碍/数据龄期仍检查，不用后方豁免转向、不反转刹车，时间只是响应探针上限，不是按练习场时间标称路线完成。
- 前两次启动没有输出油门；第二次明确stale arm request。辅助脚本每请求重建HTTP客户端在车端耗时217–267ms，超过180ms解锁时钟门；第一错误正文未保留，不能倒填其精确原因。改为复用客户端后状态GET14–23ms，保留180ms原门，第三次单正向请求获反馈motor1560/servo1600（3样本），250ms停止定时器生效，最后独立反馈seq88/healthy/locked/armed=false/motor=servo1500。中性反馈不证明物理停稳或转向效果，已向用户询问前进/左转/完全停住情况。
- 第四次尝试为500ms/1560/1600，8个正向请求，8份目标PWM采用反馈，定时停止后独立seq99/健康锁定1500；用户确认向左前行、停住无碰撞，但认为转弯幅度不足、场地可用半径不大。随后在既有允许范围内左舵1650再一小段500ms（不延长时间、不提高电机），8个正向请求、6份目标PWM采用反馈，最新独立seq110/健康锁定中性；用户确认明显更紧、完全停住无碰撞。不能据PWM反馈或现场相对评价认证绝对转角/半径。
- 网页运行设置现为1560/1450/1650/1350；本轮只在锁定时更新forward/left，reverse/right保留车端当时设置，没有倒车。此前1580/1350/1650/1350是历史内存设置，新服务曾回到默认；下次重启重新核对。候选绕桩1560与左舵1600/1650不得冒充已标定速度/曲率映射。
- 赛题按图首桶逆时针、次桶顺时针形成S形通过；不是每桶一整圈，图示未规定独立的线后固定90°左转或精确半圈角度。桩序/进出口须据现场实时目标，禁止赛道特化。已有实量轴距25cm、雷达前21/后20/两侧17cm沿用；真实位姿/共同时钟/后轴外参、曲率/扫掠/滑停和认桶准入仍待实测接线。
- 证据：[分段验证](docs/vehicle-turn-cone-step-validation-20261006.json)，原始扫描/照片/脚本在work/vehicle-turn-cone-readiness-20261006/（不进Git）。最后动力小段已结束，不自动重新解锁；正式目标接口仍仅GitHub交付、未下车。下方入口和参数仅为历史。

## 新窗口接手入口（2026-10-06，通用停车目标直道接口完成，推送后暂停）

**用户最新要求“完成后推送github仓库然后暂停吧”。本轮完成通用单目标直道软件接口、离线验证与main交付后暂停；不继续部署、不连接车辆或发车，左转/绕桩未测试。四次连续直道授权均已执行，不沿用。独立进程版8efc47b此前已推送、备份部署并执行run04，用户认可现场效果，要求正式停车点由上游传入、禁止测试路口和场地坐标特化。新接口尚未部署到车，车端最后核对版本仍8efc47b；没有合格真实上游/时间/位姿/停车契约，正式模式默认拒绝启动，完整比赛实车链未完成。**

- run01：25前进/0质量等待/5滑行输出拍、2次舵机调整，前横平面丢失>350ms先1500，随后browser_timeout、CLI TimeoutError，completed=false/0路口确认。用户确认前进、无左偏、未见明显滑行、完全停住且无碰撞；独立新鲜seq321/locked/armed=false/motor=servo1500。
- run02：40前进/0等待/5滑行拍、2次舵机调整，前横平面丢失先收油、随后sensor_stale/CLI超时，completed=false/0路口确认。用户说“接近路口处停下了，车身没有左右偏”；保存现场结果，不记为软件到点成功。独立新鲜seq1120/健康锁定中性。两次末帧速度/TTC为null，首因不是2.5s接近时间，也不是识别到路口；相机/点云变化不换算精确位移/滑行。
- run03：e0fe443验证部署后一次，67前进/0等待/5滑行拍、9次舵机调整，前横平面4.8538m/相对接近估计2.0316m/s/TTC1.897s按2.5s余量1500，不是700ms丢失触发；随后autonomy_control_gap（80ms保护），CLI读取到保护结果而无HTTP TimeoutError。0路口确认/低运动pending/completed=false，独立新鲜seq74/locked/1500，用户评价“这次效果不错”。后台线程仍可能受GIL/调度影响，继续改为独立无硬件计算进程，80ms门不放大。
- run04：独立进程版8efc47b，64前进/0等待/174滑行拍、10次舵机变化，前横平面4.9903m/相对接近估计2.2168m/s/TTC1.800s按2.5s余量1500；滑行正常覆盖5s，无HTTP/反馈/80ms时序故障，低运动pending/superseded未合格，到期coast_standstill_unconfirmed/completed=false/0路口确认。新鲜独立seq240/locked/armed=false/motor=servo1500。用户评价“完成的非常好”是现场对直道效果认可，软件停稳/到点仍未认证，不能把false改成功。旧版备份backup-coast-process-20261006-161139，四Python/桥/入口哈希及子进程/串口唯一所有者已核对。
- 独立进程版本：生产后台重计算在一个close_fds私有JSONL子进程，仅CoastMotion无硬件API；父线程仅有界队列/IO，取消/换会话/过时/新扫描继续撤销旧结果。关闭仅terminate，后台回收，不等拟合/管道锁。122Python/8前端、车端21纯控制/子进程测试通过；83真实扫描协议对照分类/数值零差、0静止宣告。coast_motion新SHA4726277416335bc50e472b49cee017ef0c00a529cc8331594c854556c4dfd257；这不是WCET或实车验收。
- 车端同扫描纯计算回放，原低运动拟合284–364ms（扩大样本旧最高467ms）且持有共享控制锁，能阻塞HTTP/心跳/桥反馈；浮点mean→fmean保留83份扫描结构/分类，数值差<=8.88e-16，车端仍75–152ms。因此新增独立后台拟合，最多一个计算中+一个最新待算，控制锁不等待；新会话/取消/新扫描/过时结果必须失效，不能用旧stationary宣告当前静止。118Python、8前端、目标板临时目录17项无硬件控制测试通过；底盘Rust未改，沿用515476df匹配桥。
- 用户已确认：已知前横平面拟合丢失350→700ms，底盘反馈接收150→200ms、自动健康200→250ms；completed仍需真实完成证据，不改保护退出为成功。300ms雷达质量、前1m/侧30cm、80ms自动循环、200ms客户端心跳、250ms旧指令/300ms原生桥、急停等保持；仅后方起步最多1秒、舵机1500/仅一次。尚未实车验证新增窗口的停车余量，不保证滑停距离，不反转刹车，不用场地坐标特化。
- 本机正式接口已接入：Rust OnlineMissionReport新增独立FinalStopGoal，最终姿态/边界不受滚动Target.point裁剪，按源Track/revision/时间/误差导出，保持sim-only/physical_ready=false/clock_epoch=null。Python stop_goal.py和服务/CLI消费配置好的本机目标、契约与车辆profile；goal-status及未execute的goal-straight只GET，execute才登记自己的源并启动。正式模式不读left_junction，PWM按最终目标与实测起步/加速/中性滑停包络降档，进入coast不恢复正PWM。完成须中性采用ACK后新鲜独立位姿、整车边界/位置/朝向/角点速度和新源时间保持合格；未知或保护停车不改成功，斑马线要求真实停稳后3秒。
- 同一停车请求goal_id/task_revision/track须稳定到执行反馈或撤销；重复读取不续租，旧/乱序/换身份停止，几何修订清保持计时。终点completion_permissions也须匹配当前goal_id/task_revision/track_id、同epoch及新鲜source_at，其他任务许可不能沿用。Rust模拟任务会自行推进阶段revision，不能靠改标签直接作实车feed。前向/侧向/既有后方起步窗一致，有限侧界反射后核全车四角。当前完整比赛、左转与绕桩的实车输出未验证，详细入口/准入见[通用接口](docs/上游停车目标与实车直道接口-20261006.md)。未来部署须备份并传齐五Python（含stop_goal.py）、匹配Rust桥与入口脚本，默认锁定；本轮未部署。
- 最终离线验证：149项Python（含27项目标契约/服务/CLI）、8项前端、650项Rust通过/0失败/3项历史忽略；Rust fmt、workspace全target clippy警告即错误、五Python与入口静态编译通过。匹配桥及robot runner均重新RISC-V交叉构建/ELF通过：桥SHA515476dfb396fa063775bcc2b2951ce72e0b20f90f888b1525b2eb229f671d24未变，runner SHAbc62fedd2ad1e5df3201da985a8a2944351bf3c0afa31df9ec02d29580c7536d；构建不等于真实源/目标板验收。这些不认证低运动、停距、转向扫掠或比赛完赛。
- 用户实量本车轴距25cm、雷达旋转中心至车头21cm/车尾20cm，结合此前左右轮外沿17cm，存config/vehicle-geometry-measured-20261006.json；不可再用厂商默认305mm充当本车轴距。测量误差、后轴偏移、打角曲率与转向扫掠/滑停预算仍缺；这都是车辆参数，不记录练习赛道长度、墙坐标、拐点和桩位。
- 通用接口已显式处理坐标符号：Python X前/Y右/右转正，Rust X前/Y左/左转正，同原点(x,y,yaw)反射一次为(x,-y,-yaw)，有限侧界也反射；真实外参/原点仍须验证，原生heading_left_rad已是Rust符号不能再反。lidar at_ms、control tick、Python monotonic不是同纪元；目标来源/身份/revision/最终边界/误差/期限、共同时钟和滑停预算缺失时拒绝真实启动/完成。
- 详情：[本轮实车与修正](docs/直道停车与滑行修正-20261006.md)、[机器记录](docs/vehicle-junction-coast-validation-20261006.json)。原始证据在work/vehicle-junction-coast-test-20261006/；最终需按上传规范验证、提交推送main，保留队友文件。下方旧参数/未测试状态为历史，本节后续以同节最终更新为准。

## 新窗口接手入口（2026-10-06，滑行版本已备份部署，保持锁定未测试）

**用户重新连接车辆并要求“你看看有没有需要上传到车上的”，本轮完成新鲜反馈/旧版核对、私有备份、成套部署和独立只读复核，覆盖下方“先不连接车”阶段。没有发送arm/drive/自主启动请求，没有启动电机或进行带动力测试。后续仍须用户确认摆放、静止并明确要求当次测试；部署授权不扩大为运动授权，不沿用历史启动授权。**

- 修改前本机main/origin/main为`8aca022fbee4771de39d9d4bcb245c12fc1180b7`，包含88b7436滑行源码。用户建立的work/vehicle-test-ssh.sock复用正常；车端Bianbu2.2/riscv64/Python3.12.3/GLIBC2.39。旧三Python/桥哈希与昨晚部署一致、coast_motion.py不存在，确实需要更新；首次5份及部署前5份新鲜反馈均健康锁定中性，旧指令seq12，不再依赖历史seq84或无ACK夜间stop。
- 成套更新四Python(server.py/autonomy_live.py/coast_motion.py/autonomy-control.py)及匹配新Rust vehicle-bridge到`/home/bianbu/xt-stcar-console/20260917`；入口检查脚本同步至base/start-console.py。新桥SHA256 `515476dfb396fa063775bcc2b2951ce72e0b20f90f888b1525b2eb229f671d24`，六文件哈希全部匹配本机。车上五Python静态编译/ldd通过，桥dry control执行physical_output=false且locked/1500；没有用--execute做动力校验。
- 旧版备份`/home/bianbu/xt-stcar-console/backup-coast-20261006-144947`，临时目录coast-stage-20261006-88b7436-144947；先正常停服务并确认inactive/MainPID0和串口释放，再替换全套。保留模型/视觉配置/视觉模块、雷达校准、页面、unit/启动器和记录，保留访问token；旧coast不存在，回退要恢复整套并删除新增模块。私有凭据备份只留车端，不进Git。
- 重启默认锁定，新boot后已取健康中性反馈；部署脚本后又独立读取10份，同boot、控制tick和扫描seq/at_ms增长，全部healthy=true、owner=null、active=null、mode=locked、armed=false、motor/servo1500、coast_max_s5.0。部署独立复核最后指令seq0，为重启后正常序号。实际进程路径和/dev/car、/dev/laser各一个新桥所有者已核对；这是软件/串口中性状态，不是物理停稳证明。
- 连接时网页手动设置已回源码默认1550/1450/1650/1350；部署后仅在锁定态恢复并GET确认1580/1350/1650/1350。内存设置不持久化，下次重启重新核对。自主1580起步/1570巡航/1550最低前进、1500中性、直道舵机1445–1555保持；已有手动--allow-reverse保留，不新增或使用反转刹车。
- 部署复核时最小已知回波约0.324m、后方180°约0.339m，通用短探针拒绝probe_obstacle_close，保护未改变。用户询问启动影响后另读5帧，已无40cm内回波，侧距约0.444–0.447m/前向通道约3.49–7.38m均通过；其中1帧前方连续未知4°超过3°门、front_sparse需中性质量等待，其余4帧质量合格。状态页probe_ready不代表straight必然拒绝；既有直道后方115–245°起步豁免最多1秒、舵机1500且仅一次，不能放行前/侧障碍与质量异常。最新只读桥反馈seq289/operator_stop，仍armed=false/1500，助手未发送运动请求，不据序号推断物理移动。视觉仍单blue_cone、unverified/simulation_only、部署查询结果龄约1267ms；它影响比赛能力而不参与当前短直道起步门，真实相机帧龄仍检查。四标签顺序与队友训练职责不变。
- 用户提出“启动的时候只要注意两侧就行”后，经范围澄清明确选择“仅放行后方，保留前方1m通道检查”。沿用现有一次性后方起步窗口，前向1m/侧向30cm、近障碍和质量/故障保护均保留，不新增忽略前方的策略，不需修改控制代码；该确认不是启动电机授权。
- coast最多5秒、电机仅1500、雷达受限纠偏，低运动合格再回中锁定；未知超时失败，急停/近障碍/失控立即锁定，不自动重新解锁。run08仍左偏撞挡板停车失败；71低运动样本全unknown、at_ms整圈发布时间、停稳未标定和完整比赛未接链等限制不变。本轮部署/目标运行通过不能当实车纠偏、滑行或提前停车验收，下一步按新现场授权优先有界短段检查，不直接跑全程。
- 证据：[本次部署](docs/滑行版本车端部署-20261006.md)、[部署机器记录](docs/vehicle-coast-deployment-validation-20261006.json)、[准备记录](docs/滑行版本部署准备-20261006.md)。原始证据在work/vehicle-coast-deploy-20261006/，本轮更新agent/README/验证记录按上传规范推送main，队友已有文件保留。下方“不连接/未部署”仅是此前阶段历史。

## 新窗口接手入口（2026-10-06，本机版本核对与滑行部署准备，先不连接车）

**用户最新要求“先不连接车”。本轮后续仅做本机与GitHub核对、交叉编译和部署材料准备；没有部署、重启车端服务、解锁或发送电机/舵机控制指令。后续接手先保持不连车，待用户允许连接后才读取实时状态；车辆摆放、静止和明确当次测试授权齐全前不得启动电机。下方入口全部为历史，以本节及2026-10-05晚最新滑行逻辑为准，不采用旧PWM或旧运动授权。**

- 核对基线：本机main、origin/main和GitHub main均为`88b743697642b721a7ee9a576306e285afe7130e`，fetch后差异0/0。完整读取本文件和上传规范，保留队友已有SUMMARY、ZIP、文本和规则PDF，未纳入本次提交。最新验证JSON的8份源码哈希全部一致；本轮仅新增部署准备记录，核心代码未修改。
- 用户收紧范围前，旧SSH套接字已不存在，直接SSH及22/8081探测超时，没有任何新鲜车端反馈；随后已停止连接检查。**当前车端状态未知，seq84和夜间无ACK的stop均不能当本次锁定或停稳确认。** run08仍为左偏撞前挡板、停车失败，completed=false，未到路口、未左转。
- 已从88b7436源码本机离线交叉编译新Rust vehicle-bridge，替代本机旧的10月5日18:35产物；SHA256为`515476dfb396fa063775bcc2b2951ce72e0b20f90f888b1525b2eb229f671d24`，573336字节。ELF检查通过RISC-V/RVC/LP64D/PIE，libm+libc，最高GLIBC引用2.34、基线2.38；尚未目标板执行。链接器有一条deprecated optimization setting提示，构建成功。
- 本机`work/vehicle-coast-prep-20261006/bundle/`已备齐server.py、autonomy_live.py、coast_motion.py、autonomy-control.py、新桥，以及更新后的start-console.py入口检查脚本、manifest/SHA256/ELF证据。包是现有驾驶台更新文件集，不含模型、凭据或原始影像；产物不进Git。不得复用漏模块的旧部署脚本；旧straight-preflight.py会写旧设置，也不是只读入口。
- 本轮109项Python、8项前端、13项Rust路口/前横平面、4份Python静态编译、Rust fmt、单桥交叉编译/ELF通过。645项Rust和clippy为上一轮已有证据，本轮未重复整套；这些都不是实车纠偏/滑停验收。Python3.14.7有HTTPError清理ResourceWarning但0失败，目标板运行仍待核对。
- 部署恢复连接后先取得不同新样本：同boot、control.tick增长、lidar.seq/at_ms增长，健康/帧龄合格且owner/active空、mode=locked、armed=false、motor/servo1500。锁定时指令seq不必增长或仍为84；软件中性不证明物理静止。先备份全套与配置，再成套部署、核对哈希/依赖/实际运行路径，启动默认锁定；失败恢复整套备份。保留现有单blue_cone模型与校准。
- server源码网页手动设置默认仍1550/1450/1650/1350，重启只内存设置会回默认；部署后在锁定态核对最新1580/1350/1650/1350。自主1580起步、1570巡航、1550最低前进、1500中性及直道舵机1445–1555不变。现有--allow-reverse是手动倒车能力，不代表反转刹车，本轮未新增或使用反转刹车。
- coast最多5秒、电机仅1500、雷达继续受限纠偏；低运动合格再回中锁定，未知超时失败，急停/近障碍/失控立即锁定，不自动重新解锁。71样本全部unknown的限制不变，at_ms仍为整圈发布时间；完整比赛任务未接实车执行链。下一次测试优先真实纠偏/回中、1500滑行、低运动与提前停车，不直接宣称可跑全程。
- 详情：[部署准备](docs/滑行版本部署准备-20261006.md)、[机器记录](docs/vehicle-coast-deployment-preparation-20261006.json)、[停车规划](docs/停车点与PWM曲线规划-20261005.md)。本轮按上传规范提交并推送main，车端仍只保留上次已知版本的历史记录，不称已部署新滑行版。

## 新窗口接手入口（2026-10-05晚，碰撞记录、滑行纠偏与GitHub交付）

**用户最新要求“你先改，改完后先上传github,明天再在车上测试”。本轮只完成本机代码、离线验证与GitHub上传，新增滑行版本尚未部署到车辆；今晚不再测试。下方入口是历史，以本节为准。不得沿用今晚已经消费的启动授权；明天重新核对连接、静止反馈、代码版本和现场授权后再测。**

- run08为一次连续直道尝试：82前进指令拍、0质量恢复拍、4次修正、1580→1575→1570→1565→1560→1555→1550，0路口确认，`front_boundary_stop/completed=false`。用户明确“左偏然后撞到前方挡板了”“已完全停住，碰到挡板”，随后放回起点；这是停车失败，未跑完整道、未左转。收油时已知横平面3.439m、相对接近估计2.347m/s、到前向1m边界约1.040s；1500后扫描仍变化约2.2s，最终正前回波约0.217m。没有精确位移、车速/停车距离标定，不能相减单束回波冒充滑行距离。
- 最后独立新鲜反馈`healthy=true、owner=null、active=null、mode=locked、armed=false、motor/servo=1500、seq=84`；这是放回起点前的反馈，后续静止/摆放来自用户确认。额外夜间stop因SSH失联没有ACK，不能当最新成功停车确认。明天重连必须重新读取状态，不能把本记录当实时车端状态。
- 当前车端最后部署为感知自恢复版，备份`~/xt-stcar-console/backup-straight-20261005-205619`，运行1580/1350/1650/1350；精确源码哈希在新验证JSON。感知不足先1500等待，同会话无质量问题、>=3个不同新扫描且稳定>=0.3s才恢复，2s仅诊断；急停、真实近障碍、失联/底层故障、旧指令与非法PWM仍锁定，不自动重新解锁。本次0恢复拍，不能称自恢复已实车长时验收。
- 本机新增正常停止状态`drive→coast→locked`：正常直道到期、确认入口或前横平面请求收油后，电机只保持1500，舵机继续按新鲜可靠墙线受限纠偏。保留原所有者/心跳最多5s，不重新解锁或恢复正PWM；低运动合格后回中锁定，未知到期`coast_standstill_unconfirmed/completed=false`。急停/近障碍/失控仍立即回中锁定，优先于滑行。CLI保持心跳覆盖滑行，正常/异常结束都等待新鲜中性锁定反馈；网络失败不能凭stop请求或进程返回说已停车。
- 低运动参考须等真实1500采用反馈、扫描超过滑行开始的序号；唯一关联的非平行延伸墙面支持刚体运动拟合，>=5对、发布与接收两个窗口均>=0.5s，累计两钟差<=0.15s。平行长墙前后方向不可观测、旧/重复/压缩缓冲帧、迟帧或面关联歧义都返回未知。`standstill_confirmed`仅表示**未标定点云低运动收尾证据**，诊断`evidence_type=uncalibrated_lidar_low_motion`，不是物理完全静止、实车里程计或比赛3s停稳认证。真实run08的71个样本全为未知、0静止宣告：避免了过早声称静止，**还未在该实车场景验收停稳估计**。
- 纠偏仍1445–1555，扩大修正保持普通15PWM/250ms、紧迫20PWM/150ms；回中按最多130PWM/s、单次<=13/至少50ms的新帧时间预算，避免99ms扫描错过整阶梯后多等一帧。最近3份连续墙线的方向趋势只削弱旧修正，不制造反向；反向先经1500且由不同新帧确认，序号/发布时钟保留高水位拒绝旧帧；雷达接收入口仅递增序号/发布时钟刷新帧龄，重复JSON不能维持感知新鲜度。相同真实轨迹开环抽样回放中，第578帧旧1529→新1514，第580帧旧1514→新1501；抽样省略原生帧，不能推断改变控制后的真实轨迹或宣称已修好碰撞。
- 原生前横平面观察上限由4m截断改为当前扫描声明且验证的有效量程，保留真实8点/35cm跨度、拟合一致性、双侧墙内部等门。真实run08首次检测5.540m/seq571，旧3.909m/seq578，提前约0.740s；部分帧/8m窄道360bin仍不足真实支持，保持未知、不插值。Python同步接收至现有12m量程的有效横平面。**`at_ms`是整圈发布时间，不是逐束采集时间**；下方历史“采集时刻”表述由本节纠正。2.5s仍是未标定测试余量，不保证停车距离。
- 1580起步、1570巡航、1550最低前进及正常每100ms最多降5保留，实际停车/质量等待1500；未启用反转刹车。场地位置和跑道长度不写进控制律。斑马线按近沿+车头外参/完整车身+误差余量规划目标、按已标定停距预算降PWM、1500滑行纠偏、可靠停稳后再计3s；已有Rust任务接口和阶段防重入复用，完整比赛规划仍未接实车。当前模型仍单类blue_cone，目标四标签顺序0:crosswalk、1:blue_cone、2:red_light、3:green_light；训练仍由队友负责。
- 验证109项Python/PTY/故障、8项前端、645项Rust通过（0失败、3项历史忽略），fmt及workspace全target clippy警告即错误通过。新模块部署必须包含`coast_motion.py`及其它3份Python，并编译/部署匹配新Rust桥，启动默认锁定；**今晚没有部署或启动新一轮测试**。真实转向/回中、1500落地滑行包络、电池/地面差异、点云低运动阈值和目标板时限仍需明天实测，不将软件测试当实车验收。
- 证据：[本轮碰撞与滑行验证](docs/vehicle-coast-stop-validation-20261005.json)、[停车点与PWM规划](docs/停车点与PWM曲线规划-20261005.md)、[保护清单](docs/实车控制保护清单-20261005.md)。完整点云/图像/测试日志在被忽略的`work/vehicle-upload-20261005/`；凭据/原始影像不进Git。队友原有SUMMARY、zip和规则PDF不改动、不上传进本次源码提交。


## 新窗口接手入口（2026-10-05，1580起步/1570巡航的1秒启动复验）

**用户新授权“你启动电机看看”。重新读取确认已回起点、侧向净距通过，但静止两墙拟合不稳定；只执行一次1秒straight，没有连续跑至路口、左转或自动重试。结束后独立读取新鲜反馈确认healthy=true、owner=null、active=null、mode=locked、armed=false、motor/servo=1500、seq=35。该次运动授权已用于此测试，不自动续跑。**

- 代码/部署仍为bc37236，1580起步保持0.6s后1580→1575→1570，前进下限1550、舵机1445–1555。桥实际观察1580、已解锁；33前进指令拍/0质量等待/0转向修正，1秒正常到期probe_complete/completed=true。这里只表示短段完成，不表示全直道或路口到点成功。
- 20份只读净距检查通过；起点两墙拟合None，发车后恢复拟合，结束方向约−1.68°、中心偏差约2.69cm。前后图像/点云变化；没有编码器位移、速度或滑行/停距标定，不把正前回波7.98→5.03m当精确行驶距离。
- 用户现场反馈“这一次倒是没有左偏”，记为本次1秒启动观察。最新正前回波约5.03m，90°右约0.456m、270°左约0.473m；未触发新增较大舵机，因此**此前偏左/过度纠偏问题和1445–1555实际响应仍未验收解决**。下方最终参数“没有再次落地”由本节更新为“仅起步/巡航短段通过”。
- 只读核查发现：CLI正常完成等待中性，异常/保护退出不自行等待桥中性反馈；本次监督助手有4秒停后观察，随后又独立读取新鲜state确认。后续异常退出仍须单独确认armed=false/motor=servo=1500，不能只凭CLI返回或stop写入声称已停车；此缺口尚未改代码。
- 证据：[1秒启动复验](docs/vehicle-straight-start-validation-20261005.json)，原始记录/扫描/图像在work/vehicle-upload-20261005/straight-start-07。没有重新部署、改变保护、训练、倒车刹车或全程比赛执行链。

## 新窗口接手入口（2026-10-05，直道可迁移限制、1580/1570/1550与纠偏幅度）

**用户要求不能针对练习赛道特化，放回起点后执行第五轮；确认低PWM不动并再次放回后执行第六轮。第六轮仍靠左，未到左转入口。随后用户要求1580起步、1570巡航、最低1550并稍微增大直道舵机；已修改、检查、上传，但该最终版本没有再次落地行驶。当前healthy=true、owner=null、active=null、armed=false、motor/servo=1500、seq=0，保持锁定，不沿用已消费的两轮启动授权。**

- 长期约束：任何正式直道/路口识别使用实时点云几何和相对运动；不记这条练习赛道的长度、起点、墙坐标、停车位置或按时间猜拐点。删除上一版前横平面固定<=2.8m停车条件。车体尺寸、PWM与控制时限是车辆参数，仍须标定，不当作场地参数。原生横平面观察窗口仍约4m，不是已知赛道坐标。
- 纠偏按实时两侧墙线与可用半宽（width/2−30cm）归一化。侧距仍为用户实测雷达至外轮17cm+最小净距10cm+3cm测距参考余量=30cm；前向1m、横向±0.32m、急停/失控/旧指令等原保护保留。容忍带随通道宽度缩小，不持续小幅抖动；方向和前方1m预测偏差共同触发，可靠墙线丢失回中。
- 最新直道舵机1445–1555（原中位±50增至±55）；普通每次最多15 PWM/250ms，预测侧净距不足时最多20 PWM/150ms，回中/方向反转也受相同限幅限率。此前右低PWM/左高PWM约定不变。**幅度/回中速度已软件验证，过度纠偏尚未实车解决，不声称提高速度或放大转向已修复偏左，不盲目反转方向。**
- 直道默认1580起步，首次有效前进后保持0.6s，随后最多每100ms降5 PWM到1570巡航。正常前进只允许1550–1580；纠偏及接近前横平面目标最低1550，停车或质量等待仍1500。纠偏结束且尚未见前横平面时，每200ms最多加2恢复巡航；见过横平面后上限只降不升。急停/已知近障碍直接1500，不等待1570→1550降速曲线；不把前进下限套到中性停车。CLI默认1580，较低显式PWM仍不超过自身初值。
- 前横平面按扫描采集时刻估计连续一致的相对接近速度，至少两份速度证据才计算距前向1m边界的剩余时间；<=2.5s或已知平面丢失>0.35s请求1500锁定，front_boundary_stop/completed=false。没有可靠速度时渐降到1550。2.5s是暂时保守测试余量，不是已测停距/制动包络，不能保证任何电池/地面/大学赛道通用。不能差分偏转后的任意单束正前回波当车速。
- 第五轮旧临时1560/1540/1535参数：30s找路口超时，959前进指令拍/7质量恢复拍、20次修正，1535后扫描长期静止；用户明确1540不动。指令拍数不等于车轮持续转动，也不能写成跑完直道。
- 第六轮1570/1560/1550：100前进指令拍/0恢复拍、6次转向修正、4次PWM变化（1570→1565→1560→1555→1550），0路口确认。墙线方向从约−3°越过中心至+6°，侧前挡板进入1m通道后probe_obstacle_in_straight_corridor/completed=false。停车后270°左回波约0.216m、侧净距不足；不能从该位置自动续跑。原生前横平面最低已知约3.985m，尚未建立一致接近速度；本轮没有实车验收时间余量停车或完整左转到点。用户现场反馈“停下来的时候还是靠左”。
- 61项Python/PTY/故障检查通过，包括36个宽度0.8/1/1.5/2m、长度3/7/12m、方向−8/0/+8°组合、左右对称、前进下限/新起步巡航、采集时钟/跳帧、限幅/反转回中、2s恢复、急停和不自动重启。36组是墙线及控制输出检查，**不是完整车辆动力学、全程比赛或异地实车验收**。Rust/桥未改，不重跑历史630项。
- 最终部署备份backup-straight-20261005-203322，3份Python及app.js哈希与本机一致；运行设置1580/1350/1650/1350，重启后仍默认锁定。单蓝桶模型、单串口所有者、四标签crosswalk/blue_cone/red_light/green_light不变；没有训练、反转刹车、左转或完整比赛执行链。
- 下一步优先核对实时转向响应/回中滞后及滑行，基于动态墙线趋势抑制越过中心后的持续修正，不用针对练习场的固定坐标掩盖问题。新参数复验须用户重新摆放并明确授权；保护停止后不能循环解锁。终端stop仍全局锁定；退出0也须检查completed，保护停止不算到点。
- 证据：[可迁移直道与两轮实测](docs/vehicle-generic-straight-validation-20261005.json)。完整扫描/影像在work/vehicle-upload-20261005/straight-run-05、straight-run-06；最终静止只读状态在straight-final-locked，凭据/原始影像不进Git。下方2.8m、1535/1540、旧纠偏幅度条目仅为历史记录，以本节覆盖。

## 新窗口接手入口（2026-10-05 19:46，恢复直道测试、横挡板停车余量与再次偏左）

**用户吃饭后重新放回起点授权一轮直道；报告差点撞前横挡板后，又放回起点明确授权一轮复验。两轮均已执行，均未到点成功，当前保持锁定，不自动再解锁。最终healthy=true、owner=null、active=null、armed=false、motor/servo=1500、seq=76。**

- 本轮先核对main/origin一致、车端重启后锁定、旧部署哈希，再补提前纠偏与纠偏降PWM。现有8cm/4°触发之外，墙线方向偏差>1.5°且前方1m预测中心偏差>5cm也触发；退出须同时偏差<4cm、方向<2°、预测偏差<3.5cm。仍有容忍带，舵机1450–1550、最多10PWM/250ms；可靠墙线丢失回1500。
- 第三轮（本次恢复首轮）166运动拍/0恢复拍、9次舵机修正，1560逐降到1537。雷达原生曾有3份左转候选、控制最高2票，检测断续，未满足连续三帧；最终触发前向通道保护。收油后正前回波最终约22cm，用户明确“差点撞到前方蓝色挡板”。**旧每米降10PWM、最低1535策略的停车余量不足；不能记为成功到路口。**
- 据此加强本次受限`to-left-junction`的提前停车：原生横向平面进入4m后，目标PWM按`1500+(初值-1500)*clamp((最近已知平面距离-2.8)/1.2,0,1)`渐降；每次最多降5、间隔>=100ms、不同及时新扫描才更新、不在会话内重新加速。纠偏目标上限1550，同一扫描先算纠偏再限PWM。
- **原生已知前横平面<=2.8m即请求1500并锁定，原因`front_boundary_stop`、completed=false，不冒充左转路口完成。** 这是根据本轮近碰反馈增加的受限测试停车余量，不是速度/停距标定或全程比赛参数；可能提前停在入口之前。旧1m通道、侧向30cm、急停/失控等保护仍保留，无反转刹车。CLI现显式打印completed，进程退出0也不能当到点成功。
- 第四轮（用户再次放回起点后的复验）74运动拍/0恢复拍、3次修正、1560→1550；再次向左偏，侧前挡板进入前向通道，`probe_obstacle_in_straight_corridor`、completed=false。没进入4m原生前横平面范围，**新增2.8m停车余量仅通过实际旧数据开环回放/软件故障检查，尚未实车验收**。最终正前约4.1m，左侧270°回波约24cm，低于本次要求的侧净距；不能继续前进或反复从保护停止处自动解锁。
- 用户反馈“停下来后车整体向左偏，但前轮是正的”。程序停车时同时回舵机1500，停车后回正属于预期，不能据此推断行驶中1480的实际纠偏方向。待核对动态转向响应、收油滑行和更早降速，不凭符号或单束回波推定实际车速/刹车距离，不盲目反转舵机方向。此前方向约定右1350、左1650保留。下次测试须重新摆放并明确授权；不沿用本轮两次已使用的启动授权。
- 53项Python/PTY/故障检查通过；Rust与原生桥未改，沿用此前630通过及交叉ELF。新备份`backup-straight-20261005-194305`，3份Python与app.js本机/车端哈希一致、原生桥哈希不变，/dev/car及/dev/laser各单所有者；参数1560/1350/1650/1350，四标签及单蓝桶模型不变。
- 证据：[本轮恢复与两次实测记录](docs/vehicle-straight-resume-validation-20261005.json)。完整扫描、影像、部署摘要在`work/vehicle-upload-20261005/straight-run-03/`与`straight-run-04/`，不上传凭据/原始影像。**没有左转、没有全程完成。**

## 新窗口接手入口（2026-10-05 18:40，用户吃饭暂停、直道纠偏与渐降PWM）

**用户明确“先暂停，我吃个饭回来继续弄”。本轮已请求stop并确认healthy=true、owner=null、active=null、armed=false、motor/servo=1500、seq=86；不得自动续跑，回来后按新的明确恢复要求继续。**

- 已接通 `to-left-junction` 单个连续会话，默认最长30s；原起点用户确认已用于两次实际测试，不把这次暂停后的静止确认当续跑授权。CLI：`python3 ~/xt-stcar-console/20260917/autonomy-control.py to-left-junction --pwm 1560 --max-seconds 30 --execute`；急停为同脚本 `stop`。
- 原直道固定1560仅调舵机、停车直接1500，已向用户明确承认缺少正常降速。用户取消反转刹车验证，要求接近路口逐步减PWM；**没有部署/实测反转刹车**，最终停止仍1500。
- Rust原生桥新增左转入口几何：延伸入场墙线、前横墙、已知左开口和保留右前墙，不把null/单桶/T口当左转。Python每会话重新计三帧不同及时一致证据，近左墙端头雷达坐标<=40cm请求停车，不执行左转；`turn_path_certified=false / navigation_validated=false`。
- 原生 `front_boundary_m` 独立用于提前减速，不代表左转身份：4m内横向已知平面须>=8点/35cm延伸且处于拟合两墙内部。新会话从1560起，内4m每接近1m目标降低10PWM，最低min(1535,初值)，每次最多降2、至少150ms，新帧才更新，丢失平面不重新加速。确认路口后回1500。距离/PWM关系仍未实车速度或停距标定。
- 用户测得雷达至两侧轮外沿17cm，最小净距10cm；側阈值30cm（含3cm测距参考余量）、前向1m和±0.32m居中检查保留。发车後向115–245°仅直道会话首次前进最多放行1s，期间舵机1500；後方清空或到时恢复、不重新放行，前/侧及未知质量/急停/底层故障仍保护，短probe不放行後方。
- 墙线拟合允许两侧12°夹角、>=0.25m跨度，保留40点、80%内点/残差/宽度条件；实测贴左扫描成为回归样本。舵机已扩大1450–1550、最多10PWM/250ms、增益200；8cm/4°启动、4cm/2°退出保持。用户认可第二轮改善，但启动纠偏时机/随纠偏降速仍待进一步研究，**尚未修改成预测偏离触发**。
- 首轮连续窗口52运动拍、0恢复拍、0修正，因偏左触发前向通道停车，后来左回波18.2cm。修正後第二轮83运动拍、0恢复拍、2次舵机修正（最後1520），仍`probe_obstacle_in_straight_corridor`、completed=false、0路口确认。未进入原生前墙检测范围，仍1560、0降速拍；**不能写成到左转入口成功或减速实车验收通过**。用户回复“这次效果不错”。未左转、未完成比赛全程。
- 用户在架空测试确认1560/300ms起转、回1500很快停转。此前“正前6.1→1.6m=滑行4.5m”已在会话撤回：车头偏转后可命中不同墙，不能直接相减。不能据此认定电调失效，也不能把软件1500/中性收油当主动制动或实测停车距离证书。
- 最新源码检查630项Rust通过、0失败、3项历史忽略，fmt/clippy警告即错误、RISC-V ELF通过；48项Python/PTY/故障通过。真实旧路口回放、36个宽度/出口/±8°组合、前墙减速证据和限速/重复帧/人工停止/後向放行恢复都有验证。
- 最新部署备份`backup-junction-20261005-183651`，Rust桥及3份Python哈希与本机一致；1560/1350/1650/1350。单蓝桶模型、共享单串口所有者不变；未训练、未接通完整比赛各元素目标速度/物理标定。
- 证据：[本轮直道和路口停车记录](docs/vehicle-left-junction-stop-validation-20261005.json)。完整扫描/影像在work，凭据不入Git。恢复后先研究纠偏提前量与纠偏降速，再按现场授权复验；保护停止后不自行重新解锁。

## 新窗口接手入口（2026-10-05，1秒直道居中与迟帧门限）

用户继续授权沿当前直道分段测试，前进1560，每段1s；跳过斑马线/灯不表示完整比赛规划已接通。

- 新增 `straight` 单段执行，默认1s、明确时限1–30s；单段只解锁一次，人工/故障停止后不重新解锁。
  原 `probe` 仍100–500ms。终端：`python3 ~/xt-stcar-console/20260917/autonomy-control.py straight --pwm 1560 --max-seconds 1 --execute`；
  独立终端 `.../autonomy-control.py stop` 急停。分段续测用 `--expected-run-id=上一正常段ID` 防止停止/重启后的自动恢复。
- 直道两侧长平行墙拟合；横向偏差>8cm或方向偏差>4°才修正，<4cm且<2°结束修正。
  舵机1485–1515，每次最多5 PWM、调整间隔>=350ms；缺少可靠墙线回中，不把两桶当平行墙。
  居中模式检查前方1m/横向±0.32m，原纯直行±0.30m；这不是已标定的曲率/制动或全车扫掠证明。
- 用户本轮尺量雷达中心到左右轮胎外沿均17cm，最小允许侧向净距由15cm进一步放宽到10cm。
  侧向阈值=17+10+3cm测距参考余量=30cm，不是雷达距离15/10cm；前后径向40cm、前向1m保留。
- 相机迟帧门限500ms、雷达300ms；相机>=1s或雷达>=300ms先中性等待，连续质量异常2s锁定。
  质量类型切换不重置计时；自动模式不再被通用相机/雷达1s健康检查提前锁定。
  底盘状态接收门限80→150ms：真实复验曾出现82ms且0条drive的误停；底层300ms指令超时、
  自动循环80ms、心跳200ms、旧指令/非法PWM/人工急停/近障碍仍立即生效。不能保证断流时继续开。
- 36项Python/PTY/故障测试通过（含真实输出路径的容忍带、缺帧2s、单次解锁、停止不重启和截止心跳竞态）。
  一段1s曾正常回中但18运动/15恢复拍；放宽相机/雷达后30运动/2恢复拍，前后墙距和图像变化。
  又一段在接收帧龄82ms时0 drive提前停；已据此修改150ms门限，后续实测见新验证记录。
- 换电池并重新连接后，已补部署stdout缓冲读取和跨分段持续质量计时（不缓冲电机stdin）。
  备份`backup-straight-20261005-173354`，36项测试通过；两段1s分别35/34运动拍、0恢复拍、
  各一次舵机微调，结束seq=73、armed=false、motor/servo=1500。不是精确车速/停车距离验收。
  用户随后现场报车头距横挡板4.2m，撤回仅凭画面猜测“低挡板盲区”：没有证明雷达漏看。
  再完成两段1s，各34运动/0恢复拍、舵机不调，现seq=145、armed=false、motor/servo=1500。
  8帧只读扫描确认左转路口候选：前墙约1.85m，左前60°约2.71m、右前60°约0.78m；
  左侧近墙0.522m，原近墙端头在前方约0.238m。尚未将路口识别/转弯轨迹接入活动控制器，
  未左转、未完成比赛全程；不能把开口候选当完整车身可通行证书。换电池前SSH断开，额外stop未获确认；
  重连已核对启动锁定，再按用户新授权测试，不能沿用断开期间停止请求的成功假设。
- 四类标签仍crosswalk/blue_cone/red_light/green_light，车上单蓝桶模型不变；未训练、未改Rust离线比赛配置。
  本轮源文件/部署与最终实车状态以 [直道验证](docs/vehicle-straight-validation-20261005.json) 为准。

## 新窗口接手入口（2026-10-05，感知恢复与1560短动成功）

用户确认感知质量抖动尝试恢复、连续异常2s才锁定，并要求放宽A8/A9、修改后再次前进。

- A8允许前方±30°最多6个未知bin且连续缺口<=3°；保留null，不插值自由空间。A9改查
  雷达原点前方1m、横向±0.30m直行通道，侧挡板在通道外不误触发。已知近障碍及全周<0.4m仍立即停。
- 相机迟帧/雷达迟帧、整体覆盖不足/前方缺口过多进入连续质量计时；恢复清零，换异常类型不重置。
  观测不足时先回1500但不锁定，尝试恢复；持续2s锁定后不能自动恢复。完全断流、人工急停、
  非法/旧指令、串口/底层时限与控制版本屏障未取消、未延迟。
- 仍为**最长500ms的直行探针**，完整比赛定位、循迹和绕桶规划未接入；2s边界用注入时钟验证，
  这次实车没有跑2秒或全程。参数不是米制速度/制动标定，也不是验证过的完整车体碰撞包络。
- 26项Python模拟/PTY/故障检查通过；前端未改，此前8项通过。车端12份只读样本全可移动，
  随后1560/400ms窗口成功，13次前进指令、桥状态观察1560，结束armed=false、seq=15、
  motor/servo=1500、healthy=true。前后画面改变，**车轮位移/停车距离未测量**，不编造行驶米数。
- 已补部署server.py/autonomy_live.py，四份活动源码哈希与本机一致；备份
  `~/xt-stcar-console/backup-quality-20261005-01`。运行参数1560/1350/1650/1350保留。
  原Rust桥、模型和完整离线导航未改；本次运动授权已用于一个窗口，不循环重试、不自动续跑。
- SSH复用为`work/vehicle-test-ssh.sock`；接手重新检查连接和锁定状态。
  保护清单见[实车控制保护清单](docs/实车控制保护清单-20261005.md)，机器证据见
  [本次恢复与短动记录](docs/vehicle-quality-recovery-validation-20261005.json)；原始影像/样本在work目录，不进Git。

## 新窗口接手入口（2026-10-05，受限实车入口与保护逐项审核）

用户要求补实车链路并低速验证，随后要求先列全保护、逐项决定。**仅按换电池后的用户明确要求将A1上限及CLI默认改1560，其余保护未取消或放宽。**

- 已安装共享驾驶台采集/独占原串口桥的有界直行探针，CLI为车端
  `python3 ~/xt-stcar-console/20260917/autonomy-control.py probe --pwm 1560 --duration-ms 400 --execute`。
  status/省略execute只读，stop全局急停；自动最多500ms，不会自行重启或绕桶。
- boot/epoch/run_id屏障、独立客户端/循环时限、人工停止与先锁定再解锁的接管已加入。
  这只是执行接入与短动探针，**完整比赛定位/规划/循迹仍未接到实车输出**。
- Python模拟/PTY/故障18项、前端8项通过；车端尝试因`probe_front_unknown`预检拒绝，
  **arm/drive请求均0，电机未启动，未测车轮运动或停车距离**。最后读取healthy=true、
  armed=false、seq=0、motor/servo=1500；运行参数1530/1350/1650/1350恢复。
- 雷达前方存在窄角度null；0°约7.1m，−30°约0.792m侧前方回波。当前全前方bin完整/扇区
  距离条件偏保守，会误阻止正常赛道；不据此推断正前方有人，也不填null假定自由空间。
- 保护清单及所有编号见[实车控制保护清单](docs/实车控制保护清单-20261005.md)。
  用户已明确选择“先列出全部保护，我逐项决定”，后续按具体编号确认，不自行取消基础停车。
- 活动目录`~/xt-stcar-console/20260917`，备份`backup-live-control-20261005-01`；正常重启已执行，
  原Rust桥/模型未替换。换电池后SSH会话已恢复为`work/vehicle-test-ssh.sock`，
  最后报告/接管修复及1560上限已补部署并核对四份哈希；健康、锁定1500，前进1560，
  其它1350/1650/1350保留。没有重新发送运动请求。
- 完整证据在`work/vehicle-upload-20261005/`；源码/清单提交main，访问码、图像与构建产物不进Git。

## 新窗口接手入口（2026-10-05，车端算法上传与控制权约定）

用户要求上传到已连接车辆。本轮以 `faaee1f12d80dab3edb83291abda46db83292752`
源码重新交叉构建并上传到 `/home/bianbu/xt-stcar-deploy/algorithms-20261005-faaee1f`。
**上传完成不等于实车自主链路完成；本轮没有解锁、启动电机或全程行驶。**

- Mac 完整 Rust 检查 623 通过、0 失败、3 项按设计忽略；fmt、全目标 clippy
  警告即错误、RISC-V/GLIBC 2.38 构建与 ELF 检查通过。车端 34 个文件 SHA256
  全部通过，动态依赖齐全，两个程序 `--help` 及 `online-compile` 执行通过。
- 本次上传最新 `xt-stcar`、`xt-stcar-robot`、离线配置、源码快照及构建证据到独立目录；
  原驾驶台、串口所有者、单蓝桶模型及自启动服务未替换、未重启。
  `xt-stcar-robot` 仍只有离线仿真/回放输出，没有实车自主启动命令。
- 用户指定前进 PWM **1530**，已通过运行中的驾驶台设置接口应用；其它参数保留
  reverse=1350、left=1650、right=1350。该设置为本次运行状态，不保证服务重启后保留，
  不是车速标定。上传后 healthy=true、armed=false、motor/servo=1500。
- 当日连续 10 次只读采样均 healthy，10 个不同雷达序号；相机/雷达最大接收帧龄
  154.0/113.5 ms。雷达仍 `navigation_validated=false`，视觉仍单类 blue_cone、
  `simulation_only=true / calibration_status=unverified`。这不构成定位、避障或制动验收。
- 用户确认后续控制权顺序：**急停/断连/传感器故障最高，人工接管次之，自主驾驶正常运行**。
  默认选择自动模式不等于开机自动解锁。采集共享、串口只保留一个执行所有者；人工停止后
  锁存，自动不能恢复，需用户明确恢复。该实车双来源仲裁及自主执行链尚未实现，不能说已部署。
- 用户本次希望跳过人行横道/红绿灯测全程，这是后续受限测试需求；当前程序没有对应实车
  启动/跳过开关。本轮没有改任务状态机、伪造观测、删质量门或将模拟参数标成实测。
- 先前 Mac 长批次已经依用户要求停止，不因本次构建上传恢复范围扫描。
  训练仍由队友负责，最新有序标签保持 `crosswalk / blue_cone / red_light / green_light`。
- 详细上传路径、哈希和后续缺口见 [本次上传记录](docs/车端算法程序上传-20261005.md)
  与 [机器记录](docs/vehicle-algorithm-upload-20261005.json)。本机完整证据在
  `work/vehicle-upload-20261005/`，构建产物、访问码和模型不进 Git。

## 新窗口接手入口（2026-10-03，Mac 本地 CPU 范围扫描）

用户取消租用 4090 执行，授权在 Mac 本地跑离线仿真。**未连接车辆、训练或发送电机命令；历史运动授权不延续。**

- 修改前 fetch，main 与 origin/main 均为 `bd629910a39db66739b07f02f22ae44d0a9eccdd`；队友未跟踪文件保留。
- 原 32 场冻结旧版已复跑：27 整场完成、29 完成两桶，5 个失败原样保留。
- 新 `lidar_sweep` 支持任意规则范围真值。两段直道各自 3–6 m，分别变化；内挡板明确作为可选假设，同时进入原始雷达和完整车身/中间运动碰撞检查。
- 当前批次 `work/local-sweep-20261003/round-02/` 已启动两进程调度，62,026 个候选作业，0.5 m 六轴网格、桶位独立网格和部分噪声/可见性样本。不兼容输入单列；**尚未跑完，不是连续全范围证明，也不是九轴全部联合穷举。**
- 限制语义可见范围的标准场地试验已找到新失败：两桶顺序和半圈通过，进入灯区前停滞，111.7 s 普通停车超时。优先研究缺少下一元素观测时的安全接近/搜索，不能把旧理想视觉通过率当实际 YOLO 识别通过。
- 冻结二进制、源码快照、SHA 清单、SQLite 队列、完整压缩证据和进度保存在上述目录；之后修改必须用新批次。调度器只跑/收集，不自行修改源码。
- 两进程当前 worker PID `82670`，防闲置休眠 PID `82675`；PID 仅是启动记录，接手先核对是否仍存活。低于 10 GiB 磁盘余量暂停；合盖/关机仍可能中断，可续跑。
- 新版原 32 场回归在 `work/local-sweep-20261003/regression/`，以实际 `results.json` 为准，不把启动等同于通过。
- 详细范围、限制、查看/续跑方法见 [Mac 本地范围仿真](docs/Mac本地范围仿真-20261003.md)。之前 Ubuntu 交付 ZIP 保留作为参考，当前不上传服务器。

## 新窗口接手入口（2026-10-02，雷达保底与桶位偏移复验）

**本节为最新源码状态，覆盖下方 9 月 28 日的原生仿真结果。用户要求检查雷达保底认桶、
桶偏离标准位置时按题目顺序绕行，并修复仿真中发现的问题。仅做离线源码与仿真，未连接车端、
训练、部署、重启车端服务或发送电机命令；不能沿用历史运动授权启动小车。**

- 修改前已 fetch，main 与 origin/main 一致，基线 `ffdfc244a2d6a2f12663ee74b13688be1504877c`。
  用户原有 SUMMARY、两个 ZIP、规则 PDF 和其他未跟踪文件保留；最终按上传规范提交 main。
- 保留雷达独立确认/同身份关联，YOLO 锥桶是辅助。连续唯一关联的雷达位置增加平滑，
  同时扩大误差覆盖当前测量，不伪造视觉证据、不延长几何租期。长间隔重新确认。
- 雷达模式的入口和区域远距离接近保留目标切线；RollingLocal 的带朝向局部目标可使用
  原有预算内的短单圆弧。绕行半径同时筛查已观察灯前边界的整个半圈。
- 出口先在半圈切线处满足原位置/朝向条件并停稳，再清出桶区。实际半圈、外侧、出口投影、
  已处理身份、第一桶逆时针/第二桶顺时针与 task_revision 防旧路线重入继续保留。
  未提高速度、容差、净空许可、节点/终端预算或 10 秒普通停车时限。
- 最终矩阵新增前后/横向 30 cm 及两桶独立偏移、±2 mm 有界量程噪声；全程零视觉锥桶。
  控制器配置在桶位变化中完全相同，真值仅进扫描渲染和独立裁判；非桶语义和位姿为理想模拟输入。
  标准 7×5 m 的 18 次两桶全部通过、17 次整场完成；含延迟扰动和其他场地共 32 次，
  29 次两桶通过、27 次整场完成。结果以 [偏移仿真报告](docs/雷达绕桶偏移仿真-20261002.md) 和
  [机器记录](docs/lidar-offset-validation.json) 为准；必须区分“两桶裁判通过”和“整场完成”。
  小场地出口姿态停滞、部分场景绕远仍未解决，不能据此宣布任意场地/任意偏移可用。
- 最终完整工作区回归 618 通过、3 项历史忽略；fmt、全目标 clippy 警告即错误通过。
  全矩阵若有未通过场景会输出完整 JSON 后以非零状态退出，不把安全停车算通过。
  本机原始输入、源扫描窗口、每控制拍/每新计划的实际轨迹在 `work/lidar-offset-20261002/`；
  只有 `verified*` 是最终冻结源码复验，其他目录是调试版本，不得混算。
- 下一步优先检查窄场与绕远场景的入口/出口最后短距离可达性及实际采用时基，不能靠加大预算、
  容差或超时掩盖问题。雷达圆截面参数仍未经实车标注标定，需验证方底座、墙端、遮挡分裂簇、
  相似异物、去畸变、定位误差和车端耗时；单层几何不能区分回波完全相同的桶与柱体。
- 未重新读取车端状态；最后记录仍是 9 月 23 日单蓝桶模型/只读 vision-shadow，不能写成当前实时状态。
  队友继续负责四类 YOLO 训练：`0:crosswalk / 1:blue_cone / 2:red_light / 3:green_light`。
  StopLine/FinishMarker 的真实输入仍待确定，所有自主配置仍是 simulation_only/unverified。

## 新窗口接手入口（2026-09-28，雷达独立绕桶源码与原生验证）

**本节覆盖下方 9 月 23 日“本轮只改文档”的历史实施状态。用户已要求按修改意见改源码并上传；
本轮没有训练、部署、连接实车控制或发送电机命令。车端状态不能由本机源码变化推断。**

- 开工已 fetch，源码基线 `532f8fcf772edc70d6f49d2bf6177c47a4607c50` 与 origin/main 一致。
  用户的 `SUMMARY.json`、两个原始审阅 ZIP、规则 PDF 和其它既有未跟踪文件原样保留。
  完整新包名为 `XT-STCAR_lidar_first_sim_532f8fc (1).zip`，已校验包内 66 项 SHA256，
  解压参考位于被忽略的 `work/lidar-first-review-20260928/`。
- 新增 Rust 原始雷达聚类、圆截面候选、独立多帧确认与身份关联。雷达证据单列，纯雷达
  `last_visual_at=null / observations=0`，不再需要先视觉确认或依赖视觉 20 秒租期。
  YOLO 锥桶可补充同身份的真实视觉证据，不能建第二份身份或更新雷达几何租期。
- 两桶候选按任务轴/侧别/剩余任务数/误差消歧，放开旧 ±45° 条件到侧前方；多义目标停止选择。
  锁定身份、已处理空间排除、第一桶逆时针/第二桶顺时针、实际半圈和出口判据继续保留。
- `RoadFrame.camera_captured_at` 与 `observation_pose` 分开相机健康和 YOLO 语义时间/位姿。
  同步控制、后台输入、命令源时间和执行证书已接入；只有雷达绕桶阶段可不等新 YOLO。
  仍必须有真实新鲜相机采集、雷达和定位，不能把循环时间写成相机采集时间。
  下一阶段旧绿灯不放行，既见灯前负约束不删除，超过车辆运动/误差界的定位跳变停止。
- `task_revision` 覆盖阶段/目标变化和同属 Cones 的第一桶→第二桶。后台撤销旧任务命令后
  先发布新任务 Stop，再基于真实采用历史重新规划，晚到旧路线不能回到上一元素。
- 新建配置 `OnlineControlConfig::from_limits` 默认雷达优先；历史 JSON 缺少
  `online.world.lidar_cones` 时仍走旧通路。原 `online_matrix` 显式保留视觉优先作为对照，
  新 `lidar_matrix` 才是零视觉锥桶原生探针。所有自主配置仍为 simulation_only/unverified。
- 原生验证和后续任务见 [雷达优先绕桶接入与原生验证](docs/雷达优先绕桶接入与原生验证.md)
  及 [机器记录](docs/lidar-first-validation.json)。矩阵采用真实 Rust Navigator、KnownFree、
  制动包络、后台执行检查和独立轨迹裁判；非桶使用模拟专用理想语义/米制观测。
  **整场仍未跑通，不能把 Python 的 5/8 或本轮测试通过写成比赛完赛、实车可驾驶。**
  最终八场同步/异步加三组延迟扰动为 **0/19 整场完成、4/19 完成两桶**，其余在入口、
  绕行后段或灯前停滞；全部最终速度/曲率为零。中间版本 nominal 异步两桶通过不能替代最终结果。
  最后 128 次决策的完整扫描窗口已对 5 组复采，实际窗口摘录随 Git 保存并用于重放测试。
- 雷达圆截面阈值尚未实测标定，方底座、去畸变、真实遮挡/相似异物与车端实时性未验收。
  后续先根据最后停车窗口分析入口、绕行后段及灯前求解停滞，再做真实扫描标注和只读跟踪；
  不增大容差/预算凑通过率。新增功能不自动安装到车辆。
- 队友仍负责四类 YOLO 训练及视觉接入：`0:crosswalk / 1:blue_cone / 2:red_light / 3:green_light`。
  本轮未改 `vision/road.rs`、灯角色映射或模型权重；下方视觉任务单继续有效。
  StopLine/FinishMarker 的真实输入方案仍未确定。历史运动授权不作为今后随意试车授权。

## 新窗口接手入口（2026-09-23，最新标签与实车状态）

**本节优先于下方旧接手入口及历史四类模板。用户本轮要求更新交接文档后新开窗口；此轮只改文档，不改模型、不部署、不启动电机。**

### 最新感知职责决定（2026-09-23，覆盖下方旧方案）

用户最新明确：**除两个锥桶外，任务物体全部由YOLO视觉模型识别；两个锥桶以激光雷达识别、定位和持续跟踪为主，YOLO也可识别锥桶并提供辅助语义/关联；雷达独立确认有效时即可按比赛规定的顺序和绕行方向规划，不再等待或依赖视觉确认锥桶。** 这替代此前“优先视觉确认后雷达维护、纯雷达仅作可选候选”的实施方向，也替代独立经典RGB/HSV识别斑马线和灯色的比赛主通路。以下为队友待实现需求，不是当前部署能力。

- YOLO负责斑马线、红灯、绿灯等已训练任务类别的语义识别；模型未训练的类别不能声称能识别。几何、标定、边缘/条纹分析可用于YOLO已检出目标的轮廓细化、测距和质量检查，不独立产生任务类别，不因经典形状不满足就抹掉已存在的模型检测显示；应分别报告语义检测与米制几何有效性。HSV不另行决定灯色，缺少YOLO证据时输出Unknown，不回退成HSV自主放行。
- 雷达负责两个锥桶的建立身份、几何确认和跟踪，也继续提供墙体/障碍距离与空闲空间约束。“识别出来”是通过多帧点簇、形状尺寸、墙体排除、唯一关联及定位质量检查的雷达锥桶目标，不是任意两个回波。未见锥头、整桶出画、YOLO蓝桶漏检都不得单独阻断已有效的雷达绕桶；不得要求先视觉观察，也不得用视觉20秒有效期让独立雷达目标过期。
- 锥桶不依赖颜色决定身份；按赛道拓扑、起始朝向、实时位置和任务进度确定第一/第二桶及通过侧，保留真实半圈/出口完成条件和已处理排除。若实际规则必须借颜色区分而几何不能唯一消歧，应明确记录规则与输入缺口，不能伪造颜色。路径随实时几何更新，不按固定时间油门或照抄规则图坐标行驶。
- 四类有序标签仍保留 `["crosswalk", "blue_cone", "red_light", "green_light"]`，此次没有授权删除blue_cone或重排训练标签。YOLO blue_cone可用于辅助识别、显示和经时间/外参/唯一关联校验后的同目标佐证；它不是两桶任务创建、续期或放行的必要条件，不另建与已有关联雷达目标重复的任务身份，也不能仅凭类别框覆盖雷达定位。StopLine/FinishMarker不擅自新增训练类别或从灯框推算；其实际观测方案仍待确定。
- 需要改的是感知来源和任务准入，不能删除雷达几何有效期、姿态/时间/误差校验、碰撞/制动/最终执行门。当前车端仍单蓝桶模型与只读vision-shadow，尚未实现此分工；本轮只更新agent.md，不改模型/代码、不部署、不训练、不发电机命令。

### 队友视觉识别接手任务单（2026-09-23，本轮最新）

用户明确要求把后续视觉识别工作写入本文件，交由队友完成。以下是待办与验收要求，不表示已经实现。训练由队友负责；本次助手只更新文档，不改代码、不训练、不部署、不操作车辆。用户曾请求自动前进到斑马线前停车，但本轮未执行任何运动；之后用户手动到白线处，又明确说已回发车区。之后又取得面向蓝挡板的相机/雷达样本（见下方追加现场核对）；最新具体位置不能仅由先前“已回发车区”推断，也不能把旧线前照片当当前画面。 后续接手重新读取状态；本轮已有运动请求不作为队友任意试车的持续授权，电机动作仍需用户明确指定当次范围。

#### 1. 已知事实、素材与目标

- 最终模型有序类别必须精确为 `0:crosswalk / 1:blue_cone / 2:red_light / 3:green_light`；无红锥桶类。车端实际模型仍是单类 `0:blue_cone`，映射到内部 `cone_blue`。不能把单类模型的类别0改名为crosswalk冒充更新。
- 本轮分别读取近景、发车区和用户手动到线后的640×480相机JPG。前一次近景人工标注4条完整白条及右侧1条截断白条；这只是当帧可见数量，不是完整赛场条数。到线后的另一张图可见一排白条；发车区白条在远端，仅占很少像素。
- 对应独立读取的视觉状态 `crosswalk=null`、`observations=[]`，模型检测为空；图像和视觉状态不是同步同帧，不用于精确逐帧根因分析。不能因此声称场地一直没有检测结果。
- 当前白条实物297×105mm，条边净距108mm；沿短边排列时节距213mm。不是规则模板的105mm净距；方向、总条数和完整外参仍须确认。详见 `docs/现场人行横道尺寸.md`。
- 视觉仍 `simulation_only=true / calibration_status=unverified`。两次静态采样处理约0.82–0.84秒、结果年龄约1.11–1.46秒；是单帧读数，不是吞吐或最大延迟结论。这样的旧视觉不能未经时延预算直接驱动线前停车。
- 本轮雷达8次静态读取中，左右侧扇区投影中位距离约0.54–0.55m/0.46m，原点为雷达中心。不是车身净空、精确墙线拟合或未来无碰撞证明；水平扫描没有提供地面白线距离。粗校正−0.020m/顺时针5°保留。
- 目标分两层：先在实际视角、俯仰/侧看/远近变化下稳定识别斑马线并显示像素轮廓；再在实测投影、时间同步与停车能力成立后，为“车头停在近沿前、不压白条”提供有误差界的观测。视觉任务完成不等于自动停车完成。
- 原始照片与人工标注在本机工程 `work/crosswalk-static-20260923/`：`crosswalk-camera.jpg`、`crosswalk-annotation.json`、`start-area-camera.jpg`、`white-line-arrival.jpg`，另有 `start-state.json`、`arrival-state.json`。该目录被Git忽略，队友另一台电脑不会随git pull得到；需另行传递原始文件或重新静态采集，不能拿生成图片替代现场证据。人工轮廓只作近似参考，不作自动检测成绩或直接认定训练标注质量。

#### 2. 先复现现状，诊断YOLO检测与几何后处理

主要入口：`crates/vision/src/road.rs`（`detect_crosswalk`、`proposal_covers_blob`、地面mask/投影）、`crates/vision/src/ground_markers.rs`（`detect_crosswalk_element`及完整性/元素输出）、`crates/vision/src/semantics.rs`、`crates/runner/src/perception.rs`、`crates/runner/src/bin/vision-shadow.rs`、`web/vehicle-console/vision_shadow.py`。

- 固定原图、源码版本、模型/配置/provenance哈希，离线重放真正送入识别器的同一帧，同时保存采集序号与各阶段耗时。单图入口 `road-detect` 和部署检查脚本的参数见现有视觉文档；不要另外打开真实相机与驾驶台争用。
- 先单独输出YOLO原始检测、类别/置信度/框及阈值拒绝，再对框内几何处理增加诊断，避免把“模型检出但不能测距”记成“模型没检出”。给框内白色mask、连通域和最终分组增加可显式启用的有界诊断：候选数量、像素轮廓、面积、填充率、长短轴、方向、间距，以及首个拒绝原因。区分ROI/灯区域排除、白条与反光地面连通、候选框覆盖、面积/长宽比、组内方向/重叠/宽长比/间距、条数、投影/物理尺寸、边界裁切、最终元素确认等原因。
- `stripe_axis=either` 已允许任意主轴，不能把“加一个允许旋转开关”当作修复。现算法的像素方向/长宽比/间距假设可能受透视影响，但本轮尚未做逐阶段复现，具体漏检原因未证实。
- 现代码 `proposal_covers_blob` 在模型未声明Crosswalk角色时允许经典RGB分支继续工作，这是旧实现事实。最新比赛通路须改为显式YOLO语义准入：缺模型候选不由经典白条分支生成crosswalk任务；历史经典分支仅保留为明确隔离的回归/诊断模式。单蓝桶模型无法满足新方案的斑马线/灯色语义要求，须取得并验证对应新权重。
- 保存基线失败后再做最小改动；候选数量、内存、计算预算和过期拒绝保持有界，诊断默认不逐帧落盘。

#### 3. 图像层识别与角度适应

- 分离“图像中识别到斑马线”和“地面米制观测有效”两个状态。输出可含像素多边形/白条轮廓、近沿像素线、有效/截断白条数、置信度、方法、源帧号/源时间、拒绝原因；未标定时可显示图像识别，但米制距离必须缺失或明确无效，不能用0或模拟值填充给控制器。
- 保留原图，仅叠加实际算法计算的轮廓；人工参考、模型框、几何确认三者显示上明确区分。页面和保存图片必须使用同一源帧，不能把旧结果画到新实时图上。
- 在YOLO斑马线框内评估局部亮度对比/边缘与颜色联合分割，用于轮廓和近沿估计，处理灰白反光地面、曝光变化、阴影、白墙和蓝挡板。只下调白色阈值容易把地板连成一片，需用真实负例验证。
- 在YOLO已检出的斑马线区域中，从白条四边形/边缘建立条纹组合，允许透视下逐渐变化的条宽、间距与长度，使用共同投影关系和排列一致性，而非要求图像里近似等宽等距。局部图案透视归一化可辅助识别；没有尺度/标定依据时不能据此生成绝对距离。
- 区分两种变化：车辆朝向/距离改变而相机刚性安装不变，与相机安装俯仰/滚转本身改变。前者应由视角鲁棒识别覆盖；后者会使旧地面投影失效，必须重新标定或使用已验证的动态外参估计。IMU只有在相机与IMU刚性关系、时钟与估计有效时才可能补偿车体姿态，不能感知任意手动掰动相机。
- 支持部分遮挡/出画时的图像候选，但不补造不可见白条或近沿。只有远端几像素、仅一条白线或没有足够几何证据时输出候选/Unknown，不硬凑确认。近沿截断时尤其禁止虚构停车距离。
- 多帧确认必须用不同采集帧和真实源时间；重复结果不增加次数，丢失/过期撤销有效状态。记录确认带来的延迟，不能靠延长TTL掩盖迟缓。

#### 4. 队友训练与新四类模型接入

- 队友收集本车真实相机视角：发车区远端、中距离、线前近景、左右偏移、俯仰/滚转变化、不同曝光/反光/阴影、部分遮挡/裁切；另含空过道、普通单白线、瓷砖缝、亮斑、白墙/门牌等负例。
- 明确crosswalk按整个可见条纹组标框的标注规范，截断/遮挡/难例处理一致；辅助白条多边形可另存，不擅自把单条白条加成第五类。数据按采集场次/位置序列拆分训练、验证、测试，避免相邻视频帧泄漏。合成透视增强只作补充，不能替代真实变角测试。
- 交付可信 `best.pt`、精确有序类别表、数据集版本与拆分、训练配置、每类/各视角与距离分组的验证指标、失败例和权重哈希。当前单蓝桶权重另留作回归和回退。
- 修改 `config/yolo26-race4.json` 及旧视觉文档中的历史四类示例，保持标签与实际ONNX metadata、provenance完全一致；不得只换显示名。`blue_cone -> cone_blue`、`crosswalk -> crosswalk` 可沿用内部角色。
- **红绿灯需要代码语义适配**：当前 `RoadClass` 仅有 `ConeRed/ConeBlue/TrafficLight/Crosswalk`，`validate` 要求角色唯一，因此不能直接把red_light和green_light同时映射到TrafficLight，也不能在JSON填写尚不存在的角色。应显式设计颜色语义或带颜色的灯候选，更新序列化、验证、`lamps_only`、全部灯候选/排除ROI/HSV与输出分支，并保持旧官方80类/单蓝桶兼容。
- 由YOLO red_light/green_light类别判断灯色，保留暗灯、过曝、同帧红绿冲突、低置信/过期时Unknown或停车语义；明确冲突规则并测试。RGB质量检查可以拒绝不可用图像，但不得独立改判灯色或绕过YOLO放行。绿灯标签不提供灯前停车区几何，也不能直接授权通行；原独立帧确认与绿灯等待门不放宽。
- 沿用 `scripts/export_yolo26.py`、`scripts/validate_yolo26.py` 和可信训练清单。保持已验证的YOLO26n Detect one-to-one、FP32、静态320、batch1、opset17、nms=False、输出 `[1,300,6]`；验证坐标、类别、预处理/letterbox和哈希。若另评估分辨率/量化/EP，作为独立方案验证兼容、精度与时延，不能偷偷替换现有契约。

#### 5. 地面标定、距离和性能

- 静态测相机内参/畸变、刚性安装姿态、相机到车体/车头的外参；用实际白条尺寸与独立尺量地面点拟合并验证投影，留独立验证位置、误差和适用角度范围。显示车头到近沿的距离时必须扣除已测车头外参，而不是报告相机/雷达原点距离。
- `RoadDetector` 当前只接受模拟/未验证标定配置；真实标定支持需设计实际数据与验证路径，不能仅将两个标志改成已验证。修改API时明确保持旧模拟回归与拒绝行为。
- 真正融合雷达时核对相机/扫描/位姿采集时间及外参、误差和关联质量，关联成功才标 `VisualLidar`；单视觉仍为 `GroundProjection`。两侧墙距不等于白线距离，雷达扫描覆盖好也不等于完整车体未来轨迹已安全。
- 在目标板用真实连续帧记录预热后样本数、处理吞吐、采集到可用结果的帧龄分布及P50/P95/最大值，分列采集缓冲、JPEG、预处理、ORT、几何、IPC/绘制；区分丢帧与重复结果。共享相机、最新帧策略保持，控制线程不等待推理。
- 测量新四类YOLO和框内几何处理的真实瓶颈，优化最新帧调度与预算；不能为追求帧率回退到无YOLO的白条/HSV任务识别。雷达锥桶更新独立于YOLO推理节奏，保留其自身源时间与有效期，不挤占原心跳/传感器读取。性能是否够用由停车反应预算决定，不能只报平均FPS。

#### 雷达独立识别两锥桶并按比赛规则绕行（最新实施任务）

用户已明确选择雷达独立锥桶通路，不再只是研究可选后备。视觉完全未见过锥桶、转弯后锥头或整桶出画时，只要雷达目标仍确认有效，均可进入/继续两桶任务；当前阶段真正所需的其它传感器、几何和控制条件仍须有效。

**已有代码与必须调整的位置：**

- `crates/robot-core/src/local_world.rs::maintain_confirmed_cones` 是旧视觉确认轨迹的雷达维护接口，受视觉确认次数、`last_visual_at` 与默认20秒语义有效期约束，不能直接冒充雷达独立建轨。复用其唯一关联、误差更新和点束冲突检查，新增明确的雷达来源与独立确认状态；雷达自己的确认次数/几何时间不写成视觉证据。保留旧视觉维护模式作兼容回归，但新两桶任务选择雷达模式。
- `crates/runner/src/online.rs` 当前先做 `associate_visual_cones` 再维护，且输入耦合road/elements/pose同源时刻；需增加按雷达与匹配位姿更新的通路，在没有视觉锥桶甚至没有新YOLO结果时仍更新两桶轨迹。不能伪造空视觉帧时间来满足旧接口。相机负责的斑马线/红绿灯按各自真实源时间、阶段需求和有效期进入任务。
- `crates/robot-core/src/online_mission.rs::select_track/cone_step` 需接受已雷达确认的锥桶身份，不要求颜色或先有YOLO蓝桶框；`required_cone_colors=None` 本身只取消颜色过滤，不能替代建轨/确认接口改造。任务锁定活动TrackId，避免跟随最近点簇换桶；已处理目标不能换ID后再次计数。

**队友实施与验收：**

1. 建立雷达“候选→确认→跟踪→丢失/歧义”状态。采用多帧稳定点簇、有效点数、尺寸/形状/残差与墙体分离证据确认锥桶，排除墙角、挡板脚和长条边界。实测扫描高度截到锥体还是方底座，不强制所有截面都按圆拟合。阈值依据真实标注数据确定，不为了凑两个目标强行拆墙。
2. 已确认雷达目标可以直接用于第一/第二桶任务，不再等待视觉背书。使用实时几何、起始参考方向、赛事拓扑和已完成进度消歧；允许两桶先后出现，不要求每一帧同时看见两桶。目标顺序或关联仍歧义时停止/有界重观察，不猜颜色、不任意选择。
3. 绕行方向、通过侧、实际半圈、出口与两桶先后关系以比赛规则图及当前任务实现核对；用观测位置生成持续更新的短目标。障碍/未知区域、完整车体、制动包络及最终执行门继续检查，不把“雷达识别了两个桶”当成全路径自动安全证明。
4. 视觉遮挡、blue_cone漏检或旧视觉语义20秒到期不应让有效雷达轨迹失效；同时保留雷达自身的几何过期、定位质量、观测时间/误差和唯一关联约束。雷达丢失/歧义不得无期限沿旧桶位置走。各来源生命周期分开，不全局删除TTL或延长传感器超时。
5. YOLO blue_cone作为辅助识别输入，可在时间、外参、几何与唯一关联均通过时补充同一锥桶的视觉语义和证据，不因视觉缺失阻断有效雷达目标。关联时复用稳定身份，不能产生一视觉一雷达两个任务桶；证据冲突明确标记并按可信度/歧义规则处理，不能瞬间换桶或改任务顺序。纯雷达轨迹使用真实雷达来源；只有实际融合成功才使用VisualLidar，不伪造颜色、视觉次数和时间。保留现有视觉维护接口测试，另测新独立及辅助融合通路。
6. 必测：从始至终无视觉锥桶也能完成合成两桶顺序/绕行；转弯出画且扫描连续；两桶同色；仅一桶/逐个出现/多余障碍；墙角和挡板端点；两簇合并分裂/遮挡重现；同簇争抢两个ID；定位跳变；旧帧/重复帧；雷达有效而视觉已过20秒；雷达自身几何到期；已处理桶重检。另验证斑马线/灯色仍需YOLO，不能因雷达模式跳过它们的任务条件。
7. 先静态真实数据验证识别与身份，再离线连续序列/模拟任务、车端只读实时跟踪，最后另行明确授权的受限运动。当前只读雷达输出 `navigation_validated=false`，真实外参/时钟/定位/执行链尚未验收；本次文档修改不表示已经可自动行驶。

#### 元素顺序与已完成元素防重入（用户最新强调，必须实现）

比赛任务严格按 **斑马线识别与线前停足规定时间→第一锥桶→第二锥桶→红绿灯区停车/绿灯放行→终点** 推进；具体方向、通过侧与完成条件按现有规则资料核对。感知可以同时看到多个阶段的物体，但观测结果不能自行切换比赛步骤，也不能因为重新看见旧元素就返回上一段路线。

- 任务状态机决定当前可消费的元素类型与实例；类别ID不是任务序号。两桶可能同色同类，必须用稳定身份、几何位置、关联历史和已完成进度区分第一/第二桶，不能“检测到blue_cone就回第一桶”。未来阶段的观测可缓存，但不能跳过当前完成条件。
- 只有真实完成事件才能推进并标processed：斑马线需满足线前停车/停稳计时；每个桶需满足正确通过侧、实际绕行和出口门；灯区需满足停车区域及有效绿灯条件；终点需满足整车入区/停稳等规则门。不得在第一次看见、短暂出画或路径规划成功时提前当作已经走过。
- 每次比赛运行维护已完成元素账本（run/session、阶段/实例ID、完成事件、位置/几何及误差）。已完成元素再次出现仍可参与障碍约束和定位，但不得重新产生该步骤的任务目标；不能为忽略任务而从碰撞地图删除锥桶/墙体。
- 跟踪ID重建、遮挡重现、雷达簇分裂合并、YOLO/雷达重复检测时，结合空间误差、历史关联与拓扑检查是否是已处理实体，不能只查新TrackId的processed标志。保留已处理空间排除与身份记录，同时避免误把附近真正的第二桶排除；歧义需显式处理，不能随便给新ID绕过账本。
- 当前阶段不得因旧元素重检、感知类别短暂跳变、关联失败或目标丢失而回退；失去必要观测时停车/有界重观察当前目标，仍保留任务进度。解锁、普通Stop、传感器重连不能自动清空已完成账本；整场重置只能通过明确的新一轮开始操作。程序重启若不能可靠恢复进度，应保持未就绪并要求明确初始化，不能静默当第一阶段继续行驶。
- 阶段切换时清理/失效旧阶段的活动目标、搜索状态、局部路线和未采用的异步规划结果；证书/采用侧核对run、阶段或任务revision及目标身份。保持最后已采用动作的真实执行历史供停车/安全证明，不把“清旧路线”实现成清空真实速度/曲率历史。
- 专测晚到结果：上一阶段的YOLO帧、雷达关联和后台规划在下一阶段到达时，不得复活旧目标或采用旧路线。保留源时间/序号，并按阶段完成时刻与revision处理；不要只看结果发布时刻认为它新鲜。
- 必须新增全链路回归：已停过斑马线又看见白条；绕完第一桶后在第二桶阶段又看见第一桶；进入灯区后又检测到旧桶；完成灯区后看到旧红/绿灯；终点与旧元素同帧；旧目标换ID/跨源重复；阶段切换时旧worker结果晚到；停止/解锁/重连不清进度；新比赛显式reset才重新计数。检查实际任务阶段、选中目标、采用命令和行驶轨迹，而非只断言processed数量。独立裁判核对元素顺序、通过侧和无重复完成。
- 重点检查 `online_mission.rs` 的阶段转换、`select_track/cone_step`、processed排除，`local_world.rs` 的身份/回收/关联，以及 `runner/src/online.rs`、`control_runtime.rs`、`control_execution.rs` 中规划发布与最终采用的版本边界。现有部分防重入机制应复用；缺失处补齐，不把本任务单写成已实现。

**本次追加只读现场核对：**

- 用户要求读取当前相机和雷达，助手取得640×480相机图及6个不同seq扫描（21773/21774/21776/21777/21778/21780），扫描帧龄约23–82ms，有效回波约98.6%–99.2%；`navigation_validated=false`，仍仅粗修正的雷达原点坐标。
- 本次画面朝向蓝色挡板，图像中未见锥桶；独立读取的视觉结果单蓝桶模型 detections=[]、cones_body_m=[]。视觉结果与相机照片/雷达逐项读取、并非硬同步，不把它们当同刻融合输入。
- 六帧叠加可见左前方约左1.2m、前0.4m处稳定小簇，是待分类候选而非已确认锥桶；其它回波与挡板轮廓相连，尚未独立确认第二桶。用户称雷达看到两桶的观察保留，但不能记作算法已经确认两桶。需要用户指出目标或现场真值对应，再复核点簇与遮挡，不为了符合“两桶”预期拆墙。
- 软件状态 healthy=true、armed=false、motor/servo=1500、operator_stop。助手只读采样，无控制指令。最新画面以本次样本为准，不据前次“回发车区”推断相机仍面向原发车方向。
- 数据及照片/点云图副本在本机 `work/cone-visibility-20260923/`，被Git忽略，需单独交付队友；仅画已测回波，不补造锥桶位置。本轮未修改感知/导航代码或配置。

#### 6. 验收顺序与交付清单

1. **离线基线与定位**：本轮原图复现漏检并给出首因；保存原配置和失败，不将人工标注算检测通过。
2. **功能回归**：重点覆盖 `crates/vision/tests/road.rs`、`crates/vision/tests/ground_markers.rs`、semantics/model契约测试、`tests/test_yolo26_tools.py`、`tests/test_vision_shadow.py`；涉及页面再测 `web/vehicle-console/tests/`。覆盖旋转/透视/尺度/曝光、反光与普通白线负例、少条/截断/过曝、空候选、重复/乱序/过期帧、错误类别顺序、红绿冲突、未标定/无效投影和资源上限。原官方80类与单蓝桶配置必须保持兼容，但旧识别策略不能隐式进入新的比赛模式。增加YOLO缺失不由RGB/HSV独立生成任务类别、雷达锥桶不依赖视觉的回归。
3. **独立真实素材评估**：测试集与阈值选择集分开；按距离和角度记录漏检/误报、近沿像素误差、可测距时的米制误差、多帧确认延迟。先定义用途所需的通过门槛再评估；本轮没有足够数据指定通用精度或阈值，禁止只凭一张线前图宣称全视角通过。
4. **代码验证**：按改动运行相关测试及工程要求的fmt/workspace测试/clippy。部署前完成Mac交叉构建和RISC-V ELF/动态库兼容核验；代码测试、目标构建、车端静态连续观察分别记录。
5. **车端只读验证**：用户确认停稳且暂停遥控后才做涉及服务的部署/重启；先备份当前程序、模型、配置和哈希，准备回退。初期只运行vision-shadow，不打开第二个相机、不写串口、不解锁。实际核对多距离/多角度、异常/掉流/过期显示、处理负载和原控制状态。部署检查可用 `scripts/check-vision-deployment.py`，具体参数先读脚本/文档。
6. **明确单列运动前置条件**：视觉通过后仍需可靠起步/低速、制动距离与延迟、车体足迹/外参、侧向碰撞检查、独立急停和最终控制门。未来移动必须有距离/时长上限、当前任务必需观测丢失/过期时停止、用户现场可急停；独立雷达绕桶不因缺少视觉锥桶单独停车，斑马线/灯阶段仍遵循YOLO观测门；不能执行没有终止上限的“直到识别才停”，也不能由聊天轮询截图代替本机实时控制循环。当前不声称这些条件已满足，不凭本任务单启动电机。
7. **交付**：提交源码/配置/必要测试和非敏感验证记录，更新本文件、README、`docs/视觉模型接入与只读测试.md`、`docs/视觉驾驶台部署交接.md` 与相关命令说明。交付匹配模型/规格/provenance的独立文件清单、哈希、实测指标、已知失败和回退步骤。模型/训练数据/照片/环境/产物/凭据不进Git；按上传规范main先fetch再改、验证后提交推送，不强推。最终明确区分“实现完成、离线通过、车端只读通过、尚未运动验收”。

### 本次接手追加：用户手动到白线处（2026-09-23）

用户从发车区改为自行操控到白线处，已回复“好了”。助手只读采样确认 healthy=true、armed=false、motor/servo=1500；模型仍单类 blue_cone，所查视觉 crosswalk=null。当前设置1550/1350/1650/1350，与旧记录前进1570不同，本轮未修改。已保存发车区/到位实拍及之前人工白条轮廓；角度适应只提出方案，尚未改算法、部署或标定，自动线前停车未验证。未发送任何控制命令、未启动训练。详情见 [静态观察与手动到位](docs/人行横道静态观察-20260923.md)。后续先做只读识别验证，不把用户手动到位当自动验收；新的运动须按用户当时明确指令处理。

### 用户最终确认的视觉标签顺序

| 类别 ID | 标签（精确拼写） | 含义 |
| --- | --- | --- |
| 0 | `crosswalk` | 人行横道／斑马线 |
| 1 | `blue_cone` | 蓝锥桶 |
| 2 | `red_light` | 红灯 |
| 3 | `green_light` | 绿灯 |

有序类别表为 `["crosswalk", "blue_cone", "red_light", "green_light"]`。这是用户最新指定的训练/接入标签，覆盖旧示例 `cone_red / cone_blue / traffic_light / crosswalk`；新四类中没有红锥桶，红绿灯分成两个类别。训练仍由队友负责，接入时必须与实际权重的类别顺序一致，不能只改显示名冒充模型更新。

### 当前实际部署与未完成部分

- 2026-09-23刚通过SSH及`/api/vision`只读核实：车上仍加载**单类模型**，实际 `0: blue_cone`，内部角色映射 `blue_cone -> cone_blue`；并非上述新四类模型。车端 `~/xt-stcar-console/vision.json` 启用了独立只读 `vision-shadow`。
- 当前模型 `/home/bianbu/xt-stcar-deploy/blue-cone-20260922/models/blue_cone.onnx`；规格文件同目录 `blue_cone-diagnostic-010.json`，320×320、置信度阈值0.1；道路配置同目录 `road-perception-sim.json`。运行时显示 `calibration_status=unverified`、`simulation_only=true`；所查一帧检测为空、人行横道为null，不能据此断言场地一直未检出。
- `config/yolo26-race4.json` 与旧视觉文档**尚未按新四类修改**，本轮只在交接中明确新要求。后续接到模型接入任务时，先核对队友的新权重/有序类别表/模型规格，再适配 `class_names`、`road_classes` 和红绿灯分开类别的语义处理及回归测试；不要把旧 `traffic_light` 角色直接替换成两个未验证的角色名就部署。
- 当前实物人行横道单条297×105毫米、条边净距108毫米；详见 [现场尺寸](docs/现场人行横道尺寸.md)。条数及相对车头方向未确认。规则模板间隙105毫米与本次实物不同，尚未修改模板或相机标定。
- 驾驶台频繁锁定修复已提交并部署（`2f4db29`）：新指令时标、心跳排队、首次停因保留；7项前端及11项模拟/PTY测试通过，部署后24次只读检查健康、锁定1500。实际键盘驾驶效果尚待用户复核；不得写成实车操作验收通过。原PWM1570/1350/1650/1350和雷达粗修正−0.020米/顺时针5度保留。
- 工程 `/Users/yuhaojin/Documents/XT-STCAR`，`main`；本次文档修改前最新已推送 `f40ac3a`（现场人行横道尺寸）。按[上传规范](上传规范.md)先fetch，以实际HEAD为准。车端驾驶台 `/home/bianbu/xt-stcar-console/20260917`，用户服务 `xt-stcar-console.service`；网页 `http://192.168.0.156:8081/`。SSH复用socket为工程下 `work/vehicle-test-ssh.sock`，接手时重新检查是否有效，不把普通SSH当作可复用连接。
- 保留未跟踪用户原件 `XT-STCAR_a623638_review.zip`、比赛规则PDF和`新建 文本文档.txt`，不删除、不纳入本次提交。访问码只留车端私有文件，不写入Git。

2026-09-23现场人行横道：用户提供单条长为A4长边、宽为A4短边一半，条边净距10.8厘米，换算297×105毫米/间隙108毫米。沿短边并排时节距213毫米；条数和相对车头方向未确认。已记录docs/现场人行横道尺寸.md；规则场地模板间隙105毫米保留原来源，不误当本次实测值；未改视觉阈值/外参或操作车辆。

2026-09-23驾驶台频繁锁定修复：实车旧事件记录含13次bridge_stale_request、4次bridge_heartbeat_timeout，晚到指令反复覆盖首因。前端心跳到期即排队最新按键、单在途请求；新drive时标由最近桥时标加收样后的单调经过时间生成（样本超过250ms仍停止），不改写已发送指令，解锁先取新状态。后端保留每次解锁后的首次停因，启用TCP_NODELAY、雷达HTTP写出移至锁外。300ms心跳/250ms旧指令/Rust控制与传感器保护未变，不自动重新解锁。已部署app.js/server.py，备份~/xt-stcar-console/backup-relock-20260923-154647；保留PWM1570/1350/1650/1350及粗雷达校准。7项前端、11项模拟/PTY测试通过；本轮未发实车arm/drive，驾驶体验仍待用户刷新后复核。见docs/console-relock-validation-20260923.json。

2026-09-23前向追加：用户报前轮距前墙1米、雷达中心1.190米。只读44帧原始墙距1.2088米，现有全周粗修正后1.1908米（相对尺量约+0.8毫米）、前向角残差0.02度；44帧逐bin修正映射通过。保留现有参数，无新增偏移、重启或控制命令；不将尺量残差当毫米级精度，不将19厘米当精确轴/保险杠外参。见docs/vehicle-lidar-front-wall-reference.json。

2026-09-23后向追加：用户明确当前是后墙参照，后轮外侧距墙1米、雷达中心1.155米，不覆盖之前右墙1.150米。只读41帧原始后墙拟合1.1757米，已有全周−2厘米/顺时针5度修正后1.1576米（偏大约2.6毫米）、后向角残差0.17度；保留参数，无新增偏移、重启或控制命令。不能把0.155米直接当后轴/车尾外参。见docs/vehicle-lidar-rear-wall-reference.json。

2026-09-23最新：用户追加右墙后轮外侧1米/雷达中心1.15米并要求按粗测修正。右侧46帧拟合1.1730米、墙线角5.32度，与左侧两组共同选取量程−0.020米、顺时针5度。Rust vehicle-bridge雷达分支新增可选同目录lidar-calibration.json；已安装车端并验证360个bin、实时健康及锁定1500，原始raw_ranges保留，网页/保存使用修正ranges。原点仍雷达中心，不是完整后轴外参；导航配置与质量门不变，不恢复自动驾驶。备份~/xt-stcar-console/backup-lidar-20260923-151730，PWM设置保留，未发送arm/drive。详见车辆尺寸文档追加节及vehicle-lidar-left-wall-reference.json。

2026-09-23：用户转入雷达静止标定，确认左后轮外侧到平行左墙垂直净距1米、扫描高度无遮挡。只读驾驶台取得42份显示扫描，墙线拟合垂距1.1621米、相对雷达前向约+4.50度；未发送任何控制命令，采样前后锁定且1500。用户随后补量雷达中心距墙1.150米、左前轮外侧将近1米并重申平行，首组拟合偏大约12.1毫米；用户手动移车后报中心距墙1.670米，第二组46帧拟合1.6894米（偏大19.4毫米）、墙线角5.80度；两组角差1.31度，未固定应用安装角或两点比例/零偏修正。两组距离对照完成，用户接受粗对照、指出手动摆放偏差并要求到此为止；不追加采样，保留现有读数/外参，精确标定未完成。不能把16.2厘米差值或单组4.5度直接写入外参。生产参数未改，见docs/vehicle-lidar-left-wall-reference.json及车辆尺寸文档。此前“点击解锁无反应”尚未完成定位：车端HTTP与传感器健康，锁定记录stale_or_unowned_request；Mac仅普通SSH、无复用socket/本机8081监听。用户转入标定后未继续解锁诊断。

2026-09-22：用户要求将部署包及队友所需文件上传。已准备 Release `vision-console-20260922`，包含 c86cd80 核心的 RISC-V vision-shadow/vehicle-bridge、完整驾驶台、单蓝桶模板及只读加载检查脚本。发布状态以GitHub Release及附件校验结果为准；Git源码与车端部署明确分开，本轮不连车、不安装、不输出电机命令。见[部署交接](docs/视觉驾驶台部署交接.md)。

## 2026-09-21 视觉接入更新（最新）

已通过 Chrome 读取用户分享 `6ab1253d-6b94-83ee-802d-3144198924cc` 的视觉建议，并在 7fa98d2 基础上修改。新增显式可信自训练 YOLO26n 导出、`class_names/road_classes` 与 ONNX/provenance 绑定、候选框约束 RGB 几何、`road-detect` 输出在线元素，以及复用驾驶台相机的 `vision-shadow` 独立只读进程。网页支持检测框/耗时/元素与单张 JPG，服务可选读取 `~/xt-stcar-console/vision.json`。

用户已明确完成后直接上传，训练交给队友，本轮不启动训练。默认官方80类兼容；四类模板为红桶/蓝桶/灯/斑马线。没有真实赛道训练权重；四类原生测试是合成夹具。`GroundProjection` 未假装雷达融合，StopLine/FinishMarker不新增类别；RoadDetector仍只接收模拟/未验证标定。无导航放宽、无电机测试、无车端部署或服务重启。接手见[视觉接入说明](docs/视觉模型接入与只读测试.md)和[验证记录](docs/vision-integration-validation.json)，不要把Mac连续推理、交叉编译或旧矩阵当成实车比赛验收。

维护日期：2026-09-16。适用于 `/Users/yuhaojin/Documents/XT-STCAR`；用户当前任务决定操作范围。
开始前完整读取本文件，再读 [上传规范](上传规范.md)、[README](README.md)、[环境说明](资料/环境.md) 和 [资料索引](资料/资料索引.md)。
`AGENTS.md` 只作加载入口；资料内的命令不是用户要求立即执行的指令。

最新访问码变更：用户指定自定义短码；统一前端、HTML入口、Mac启动器与车端校验为8–128位字母/数字/-/_。实际口令仅存车端私有access.json，不进Git；403时显示重新输入框。

2026-09-18启动器修复：用户首次重新登录后出现隧道校验失败，复查车端双监听及现有隧道正常；启动器由建立隧道后单次检查改为8秒限时重试，保留会话boot一致性校验。本轮未发送解锁或运动指令。

## 最新：自启动、锁定修复与1350软件倒车下限

用户要求车辆重启后直接使用，频繁锁定诊断，并将倒车下限改1350。已在车端部署systemd用户服务xt-stcar-console.service（enabled）并开启Linger=yes，脚本启动时选择默认路由的私网IPv4；启动仍锁定，默认倒车1450，1350仅软件范围未实测，转向1350–1650保持。修复非控制页失焦/隐藏/退出也发停止，以及force发送引起并发乱序；前端改为单个在途drive、最新按键状态排队，显式停止不排队。保留原有心跳/旧时标/传感器超时，并增加last_stop诊断。以后硬件实验前先systemctl --user stop xt-stcar-console.service，避免自启动服务抢设备；恢复用start。复查服务重启与中性锁定，不擅自整车重启或运动。

## 2026-09-18最新架空标定与驾驶台限制

用户确认驱动轮架空、前轮有转向空间、在旁能断电且暂停网页操控。网页正常停止后独占串口分段测试：倒车1480/1470各1秒未转，1450/1400各1秒用户确认反转并停住，指定1400为最大倒车幅度（PWM下限）。转向±100与±50对比明显增大；±150用户确认没问题；±170与±150区别不大，用户指定±150为使用上限。每次转向各1秒、穿插回中，电机全程1500；每轮父进程与独立守护各20中性帧、退出0。使用范围不等于机械硬限位，未测转角度数或落地倒车/前进直接反向切换。驾驶台Rust/HTTP/UI统一倒车下限1400、默认1450，转向1350–1650、默认左右端点，前进1550默认/1620上限不变；启动器启用倒车，服务启动仍锁定。原始日志在work/vehicle-reverse-steering，公开记录docs/vehicle-reverse-steering-validation.json。覆盖历史“倒车尚未验证/禁用”的当前状态，不改历史测试事实。

## 最新网页交接（2026-09-17）

新增根目录 `打开车辆驾驶台.html` 通用浏览器入口；车端驾驶台增加 `--lan-bind 192.168.0.156` 与原127.0.0.1并行监听，安卓/Windows/Mac同网可直连8081并输入访问码。令牌不进Git，Mac启动器也会启用SSH目标地址的局域网监听。修复非安全HTTP上下文的randomUUID兼容问题，以及底盘锁定后owner残留导致再次解锁受阻；前端抑制重复解锁并取消失焦中的解锁。9项模拟/PTY测试通过，真实双入口相机雷达在线、1500锁定、owner为空；浏览器LAN页面已验证，未在实体安卓/Windows验收。本轮仅发送停止、正常重启服务，未发实车arm/drive；重启PWM恢复默认1550。所有历史实车限制继续保持。

## 车辆到场与当前优先任务（2026-09-16，最新用户指令）

用户已明确：车已到，SSH 已连接，车轮已架空，要求先记录进展并测试各模块。
这条新指令覆盖下方历史“暂不接车／不执行目标程序”的限制：当前授权车端模块检查、必要的测试部署与受控架空底盘测试。
不读厂商视频、不改车端 libc、不恢复 MCU 固件读取/刷写仍保持；架空测试不等于落地闭环或比赛验收。
先核对系统/依赖、设备身份与占用，再检查传感器、真实图像推理、车端程序及规划回放；底盘需有明确停机路径、限时低幅输出及测试后停止确认。
当前任务替代上一节后续算法优化的优先级，已完成的首拒诊断保留在提交 `3f7b809`。
用户已建立复用SSH会话，当前接入成功。实测Bianbu2.2 / RISC-V / GLIBC2.39；传感器和相机已实际采集，官方独立ORT候选库已完成Rust原生真实图像推理。完整固定场目标端模拟在105.64秒内完成（首次90秒超时保留）；这是合成闭环，不是落地行驶。底盘架空小幅转向、1520/1540/1560µs各0.3秒油门脉冲已执行；用户确认舵机动作、电机明显转动且最终停住，已补中性帧收尾。后续用户确认转向1450向右、1550向左、1500回中；架空0.5秒油门脉冲1510/1519未起转，1520向前转并在1500后停住。1515仅确认结束静止，脉冲期间是否起转未确认，不补推结论。用户随后要求10秒保持：1519间歇转动两次；1520起转并最终停住，车端记录指令阶段10.0195秒，是否连续转满10秒未确认。因此不能把短脉冲1519/1520差异写成固定死区。用户估计回复至转动约20秒未与车端同步，不能当作电机响应延迟。未写入生产标定，速度/转角映射和响应/制动时延未标定。
雷达/相机追加检查完成：10秒2815包，生产Rust组帧在Mac回放99圈、覆盖99.17%–100%、拒绝0；相机MJPG传输约29.8fps，OpenCV BGR读循环640×480约14.91fps、1080p约9.93fps，读取失败0，不能作为推理帧率。见 `docs/vehicle-sensor-followup-validation.json`。
新增 `scripts/camera-preview.py`，使用车上已有OpenCV及标准库提供8080网页/MJPEG/单帧，已部署到车端独立测试目录。实际640×480单帧与连续MJPEG验证通过，SIGTERM退出0；测试后服务已停止，用户尚未要求常驻运行。启动方式见模块检查文档。
用户随后明确要求更新车端ONNX Runtime：已将官方SpacemiT 2.0.6（ORT1.24.2+spacemit.a1）安装到 `~/.local/opt/xt-stcar/onnxruntime/spacemit-2.0.6`，`current`指向它；`~/.local/bin/xt-stcar`为工程默认启动入口，配置 `~/.config/xt-stcar/runtime.env`。已实测API22、安装目录及启动入口两次真实图片推理、新登录PATH可见。系统原生1.18.1/Python1.18.0保留，当前sudo需密码，未修改系统包/libc、未启用EP。详情 `docs/车端ONNXRuntime升级.md`；不要把用户级安装写成系统/Python全局升级。
用户继续要求车上能更新的都更新：已在用户审计目录重新获取并验证当前Bianbu2.2配套源全部索引（29.1MB，apt update退出0）。普通upgrade模拟为0升级/0新增/0删除/0保留；dist-upgrade模拟却删除当前linux-image-6.6.63并降级bianbu-esos0.0.10→0.0.9，未执行任何系统包事务。不得建议直接full-upgrade/dist-upgrade。系统sudo仍需密码，无需为零更新额外授权；不改源/优先级/系统索引/libc/内核。发现realtek-bt.service自9月15日开机即bring up hci failed，当前无HCI，软硬阻断均否，尚未修复。详见 `docs/车端系统更新检查.md`。
用户随后要求验证蓝牙：BlueZ主服务运行，但无HCI控制器，实际限时扫描报No default controller available（命令退出0不代表通过）。厂商8852bs分支需要rtk_hciattach，标准程序路径和dpkg文件记录均缺失；这是明确初始化阻碍。内核HCI UART/H4/H5已注册、rfkill未阻断，不判硬件损坏。只验证，未安装工具/重启服务/配对，见 `docs/vehicle-bluetooth-validation.json`。
用户授权补齐蓝牙，并要求先确认可以再装系统：已取得官方rtk_hciattach固定源码及RTL8852BS固件/配置，Mac交叉编译PIE/GLIBC2.38、车端-l/ldd通过。已部署 `~/xt-stcar-tests/20260916-bluetooth-repair/repair.py`，等待用户在SSH执行sudo；脚本临时mount namespace内验证控制器及收到扫描事件才安装三文件，安装后复测失败则回退。第一轮临时测试因Zig串口头文件B115200常量与车端GLIBC2.39不匹配失败，未安装系统；已用车端termios头文件-I覆盖重编译，无设备C探针对照通过，修正版hash为1a807e8408bd27212c7414b91072591c2168ddcd55cebf338f6afbbf150f65d8，等待用户sudo重试。尚未实际扫描成功或安装系统。见 `docs/车端蓝牙修复.md`，不能把准备完成写成修复成功。
蓝牙后续：第二轮临时初始化已成功识别RTL8852BS、加载固件并创建hci0，但脚本未等BlueZ就绪就退出；第三轮因缺少蓝牙上电复位，芯片保持上次1.5M链路状态而H5_SYNC超时。已补BlueZ实际就绪等待及与厂商一致、校验type/name的蓝牙rfkill关闭/开启各1秒，模拟等待/中断恢复测试通过；修正版上传，等待用户第四轮sudo执行。前三轮均未安装系统，记录详见蓝牙修复文档。
最新蓝牙第四轮：临时扫描已收到8设备；三文件系统安装后，原厂服务FIFO收0/1导致控制器停止重启，第二次扫描撞上切换而失败，脚本已回退系统文件。已把系统阶段扫描改为最多3次真实事件验证并保存错误，模拟回归通过；等待sudo重试。用户明确全权继续、要外出，不需重复确认授权，但实查sudo -n仍需密码，不能绕过认证。当前没有进行任何无人看护底盘测试。
最新第五轮：临时扫描收到9设备，厂商服务随后收到FIFO关闭指令，3次系统验证失败并回退。已定位车端Blueman KillSwitch插件调用realtek_bt.sh hci_start/hci_stop；插件子进程阻塞FIFO并拖住applet。用户级org.blueman.general plugin-list已由[]改为["!KillSwitch"]，原值备份在车端蓝牙修复目录blueman-plugin-backup.json；结束阻塞的用户子进程、重启用户applet后，D-Bus QueryPlugins正常且不含KillSwitch，PowerManager保留。未修改厂商系统脚本，仍待sudo安装复测；此插件调整尚不能替代实际系统扫描验收。第五轮原始报告归档到本机work/vehicle-bluetooth-repair/attempt5-result.json，附近设备地址不上传。
最终第六轮蓝牙修复完成（覆盖上面各轮待安装状态）：临时扫描收到8设备，系统安装后扫描收到7设备，repair-result.json记录system_install_completed=true。独立复核厂商realtek-bt.service为active/enabled，控制器Powered=yes，三份系统文件SHA256与已验证候选完全一致。用户级KillSwitch禁用保留、原配置已备份。未验证重启后恢复、配对或音频/数据协议，不重复执行拒绝覆盖已有文件的安装脚本；公开结果见docs/vehicle-bluetooth-repair-validation.json。
2026-09-17固件复查完成：刷新用户级APT索引成功，发现u-boot-spacemit/opensbi-spacemit 2.2.7、spacemit-flash-dtbs 1.2.3低优先级候选。仅下载解包和依赖模拟，未安装；postinst会直接写eMMC启动分区及环境，未验证定制镜像兼容和恢复方案。VPU/GPU源内无新版，fwupd现有24小时元数据无匹配更新；未宣称所有设备最新。蓝牙仍active/Powered=yes。详见docs/车端固件更新检查.md；不能自行强制覆盖APT pin刷启动链。
2026-09-17最新状态：用户要求暂不更新非车辆配套固件。落地1米请求下先按用户指定试1530/1540各0.3秒，两次均反馈未移动，随后明确“先暂停”。停止帧各由父进程和守护补发20次，正常退出，/dev/car与/dev/laser无占用。当前暂停所有运动测试；1米行驶、雷达位移及相机精度对比未完成，不能自动恢复。详见模块检查文档末节。
随后用户恢复单次1550、1秒落地试探：正常执行并回1500，用户确认慢速起步、无明显偏转、最终停住。离线雷达ICP估计原点净位移约0.212m，五种初始平移一致；无独立距离真值，不当作准确里程或稳态速度。1米行驶/相机对比仍未完成，当前保持停车，未自动追加动作。结果docs/vehicle-ground-probe-validation.json。
用户要求尺寸归档已完成：产品PDF第1页外形388×221×290mm，原文未标三轴名称；出厂代码轴距L=0.305为默认参考，非实测；导航足迹640×360/400×250mm不等于实际外形。轮距/轮径/外参仍缺可靠值，未修改生产标定。集中来源docs/车辆尺寸与几何来源.md。
箱面单点参照已记录：用户确认后轴到上/下齐平近侧箱面1m、箱面正对车头；升高目标后静态19圈0度bin中位0.912m，范围0.897–0.912m。条件式推算雷达前置纵向偏移约0.088m，含测距/摆放误差，不是完整外参标定；没有写入生产参数、没有运动。见docs/vehicle-lidar-box-reference.json及车辆尺寸文档。
最新运动状态：用户确认清空路线后尝试1米。1550/1秒本轮未检出明显位移；用户指定1600后执行0.3秒，停止帧父进程/守护各20次正常，用户反馈约1.25m、无偏移并明确“先到这吧，记得上传”。一米目标超距约25cm，不能写成闭环成功；窄ICP约0.083m已因匹配失败弃用，宽搜索约1.224m也仅作低置信离线核对。当前停止所有运动，/dev/car和/dev/laser无占用，后续先解决起步重复性、停车距离及实时停止链路，不按时间比例自动追加油门。详见docs/vehicle-one-metre-attempt-validation.json及模块检查文档末节。
最新1秒起步PWM测试完成：1510–1545每5一档未检出净位移，1550首轮用户观察前倾后回位；加入动作全程8秒雷达后，1552约0.392m、1551约0.337m、1550复测约0.180m，用户确认1552/1550复测前进。最低已观察起步1550，但同值不稳定，不给固定最低PWM或稳态速度；最高车速未测（当前室内通道不足）。均回1500且守护停止正常，最后雷达稳定、设备无占用，未自动继续。原“未位移”仅前后净位移，不能漏记短暂动作；无硬同步制动时间结论，详见docs/vehicle-startup-pwm-validation.json。
2026-09-17最新软件进展：按用户操场起步需求新增受限PWM起步辅助、真实扫描里程计结果适配接口和startup-replay离线命令。连续可靠静止才按配置步进增加；微动/回位禁止继续加档，确认前进交还正常速度控制，超时/上限/定位失效等停车锁存。默认关闭，演示参数仅合成数据；未接通实车自动油门，仍需响应/停车包络标定、最终执行门和看门狗接线。两包回归539通过/2忽略，最终起步专项17通过、CLI专项3通过（与回归重叠）；fmt/clippy及runner RISC-V交叉编译/ELF检查通过。车端SSH先断开后Host is down，离线目标回放和部署未验证；本轮未发送运动命令。详见docs/自适应起步辅助.md及docs/startup-assist-validation.json。
2026-09-17晚间测速尝试：SSH恢复。用户确认十几米无人路线、有独立急停；未执行满油门。1550/2.009秒与1600/0.302秒用户均确认未动，随后按用户要求执行1600/2.010秒并回1500，父进程/守护各20次停止帧成功，串口无占用；1600/2秒用户估计前进4–5米且随后确认完全停住。用户确认剩余十几米并要求1620，0.302秒试探前进不到1米、完全停住且放回起点；再按用户要求执行1620/2.008秒，已回中且主进程/守护各20停止帧成功、串口无占用，用户观察1620/2秒向左偏、可能初始摆放偏左，明确“就到这吧”；现已停止所有运动测试，末次复核无串口占用或测试进程。偏转原因及距离未确定，用户已明确确认最终完全停住，不自行恢复测试或修改舵机中位。五轮雷达全部整圈被当前质量门拒绝，未输出可信测速，最高速度仍未测；详见docs/vehicle-speed-probe-validation.json。
2026-09-17网页驾驶台已部署：web/vehicle-console + Rust teleop状态机/vehicle-bridge。用户明确按住多久输出多久，已取消固定2秒限制，松键回中、失焦/断网/300ms心跳超时锁定；最高前进PWM1620，转向1450–1550。倒车输出接口已实现，因电调行为未实测，当前服务不带--allow-reverse。车端/home/bianbu/xt-stcar-console/20260917，127.0.0.1:8081经现有SSH master转发到Mac8081；入口令牌在车端access.json及本机work/vehicle-console/access.json，不上传。服务当前后台运行、保持锁定，只发1500，独占底盘/雷达/相机，其他模块测试前需正常关闭。最新用户授权测试网页但明确不要启动电机，本轮未发送动力命令；只验证真实双画面、相机JPG/雷达PNG/单张合成PNG保存及帧序列/点云录制，真实键盘行驶/倒车未验收。两包Rust545通过/2忽略，Python接口/PTY/重启8通过，fmt/clippy/RISC-V桥编译通过；使用、正常停服务及限制见web/vehicle-console/README.md，凭据/图像/录制不进Git。用户纠正保存应为单张图片，已将三种图片保存作为主入口、ZIP收进可选区域；正常重启复用私密访问令牌，每次启动另换控制会话标识，旧指令不可跨启动复用；失联时明确标记旧画面停止。
2026-09-17追加交付：用户确认网页控制现在可以，要求保存到工程且点击即开。根目录新增可执行“打开车辆驾驶台.command”，调用scripts/open-vehicle-console.py，复用/交互建立SSH、检查或启动既有车端服务、核对本机隧道对应的控制会话，再用Mac默认浏览器打开令牌入口；不内嵌凭据、不自动解锁或发送运动。实查--check-only通过；--no-open --port 18082新建转发并验证通过，测试转发已取消。用户原有8081入口保留。
实际结果逐项记录到 `docs/车辆到场模块检查.md`，不得将准备或连接尝试写成验证通过。

## 新窗口接手入口（2026-09-16，上传完成后）

**本节优先于下方历史轮次和 `work/` 内旧暂停快照。请完整读完本文件再继续，不要重新从交叉编译环境安装开始。**

- 工程根目录 `/Users/yuhaojin/Documents/XT-STCAR`，分支 `main`。上一轮实现与交付提交为
  **`ab323a277ca1aa1bc195c221333a1948a24b8e35`**（`新增观测驱动任务与局部导航并归档验证`），
  已成功推送；上传后本地 HEAD 与 GitHub `refs/heads/main` 完全一致。之后的交接文档提交不改变这个实现基线，
  不要把上述实现提交号硬当作最新 HEAD。新窗口修改前仍先 fetch、核对并按上传规范同步。
- 当前没有待合入的代理补丁或待上传的源码；本机 core/含模型两包也已生成并独立核验。
  用户原件 `XT-STCAR_a623638_review.zip` 与根目录比赛规则 PDF 故意保持未跟踪，不删除、不暂存。
  新窗口不能依赖旧窗口的子代理、变量或命令会话；以磁盘文件和 Git 为准。
- 当前目标仍是**任务顺序固定、元素位置在线估计、持续更新短目标**。默认 PP，LQR 为实验选项；
  保留旧 Fixed 模式作回归，不能退回用场景真值生成在线必经坐标。最新方案正文已保存在
  `work/online-mission/new-plan.txt`，无需仅因换窗口重新获取历次分享或重读所有旧复核包。
- **构建交付完成，在线比赛功能尚未验收。** 最终 Rust 565 通过、3 项按设计忽略；example 2/5/2、Python 37 通过，
  fmt/clippy、双 RISC-V GLIBC 2.38 交叉链接与 ELF 检查通过，150 份编译输入已核对。旧固定目标八场 8/8 通过原性能门。
  在线最终 19 场仅 4 个预声明缺失场景符合安全停止检查，正常整场完成 **0/15**；14 场证明两桶后缺灯区几何，
  另三个小场在第一桶尚未通过。不能把“测试通过、上传完成”写成“比赛已经跑通”，也不能说只剩实车验证。

### 下一步从哪里继续

0. **最新诊断进展（2026-09-16，本次接手）**：已补齐下述 33180/33260/33380 ms 的真实原始输入、
   Navigator 边界更新前/plan 前后全部缓存、terminal seed、障碍与共享账本；夹具为
   `crates/robot-core/src/navigation/fixtures/online33260_first_rejection.json`。
   详见 [小场异步首拒快照](docs/小场异步首拒快照.md) 和 [本次验证](docs/online-first-rejection-validation.json)。
   两套独立源码/target 的原版与观测版均保留小场失败，439 条计划及全部功能 JSON 逐值一致（仅主机耗时不同）；
   固定提交复采脚本又生成了逐字节相同的夹具。生产控制算法没有修改，原在线矩阵与部署包未替换。
   33260 ms 的17次运动段拒绝均定位到有限停车线侧带：15个搜索段端点车体余量小于40mm净空，
   两次终端尝试约42.802mm，低于原净空加起步制动预留44.333mm；即使将扫掠管半径降至0，这些候选也不能通过。
   本轮完整 Rust 测试 **567通过、3忽略**，fmt/clippy通过；与上次交叉构建证据分开记录。
   用户追加的7组14份实验案例已读，并直接核对出厂ZIP；见 [实验案例对照](docs/实验案例对照.md)。
   RPP固定前视和Smac/MPPI运动学候选可供后续参考，但教程存在关闭碰撞检测、配置版本不同和默认配置文件缺失，不能照搬参数。
   **下一步检查同约束下的局部目标/路线形状、终端初猜及候选覆盖，不再重复补抓，也不继续仅靠收紧扫掠管证明。**
   这不是物理无解证明，不删除原净空、起步预留、source/history/lease 门。同步33600及灯区正几何缺口仍另行处理。
1. 先读 [在线任务说明](docs/在线元素驱动任务.md)、[验证报告](docs/online-mission-validation.json)、
   [最终矩阵](docs/online-mission-matrix.json) 和 [交付报告](docs/online-mission-delivery.json)。
   [开发实验](docs/online-mission-experiments.json) 保留 31 条历史/最终记录，不覆盖旧失败以追成绩。
2. 原接手待办（现已由第0项完成捕获）：**小场异步 planned=33260 ms、source=33200 ms、delivery=33269 ms** 的完整首拒快照。
   已知该帧因 `boundary_changed` 重建，恢复搜索 `open_empty`，6 节点/5 展开/25 次 primitive 尝试，
   terminal 2 solver/2 iteration/48 samples，预算未耗尽且 rollout=0；尚未进入 Drive 的 `permits_command`。
   提示 cap≈0.23443 高于可执行下界≈0.19341，不是 `empty_speed_interval`。
   当时缺少完整 Navigator before/cache、terminal seed 和障碍快照；本次已真实补采并建立回放 fixture，
   仍不能证明物理无解。不要从最终位姿重造缓存，也不要拿之后 Stop 回舵的次生失败继续堆连接候选。
3. 小场同步的新首个来源包络拒绝在 **33600 ms**：前段已连续降速，新观测线几何下假设 cap≈0.10781 可证，
   但真实 source 速度≈0.13711，最终门仍必须使用真实/历史速度而拒绝。不能把较低目标速度当作已经实现的车速。
   上述同步/异步定点捕获来自末轮工作副本，完整最终场结果另见正式矩阵；不得混用不同运行的时刻或源码证据。
4. 灯区是另一问题：灯色不提供停车区域距离，官方真实停车/终点可见图案仍未确认；当前白条协议只是显式实验协议。
   保留正几何期限与负约束，不能靠延长 TTL、猜距离、场景真值或删除停车门让整场通过。
   小场仍需算法/几何诊断，不能把它归入灯区信息缺口。

### 本机证据与复跑入口

- `work/online-mission/source-speed-hint-independent-review.md`：当前来源降速提示的独立审查与首拒证据边界。
- `work/online-mission/sixth-small-review/source-hint-sync-capture/gate-33600.json`、
  `work/online-mission/sixth-small-review/source-hint-async-capture/hint-33260.json`：真实来源门与异步提示快照。
- `work/online-mission/sixth-small-review/source-hint-sync-trace.jsonl`、`source-hint-async-trace.json`（同目录）：对应定点捕获的运行记录。
- `work/online-mission/final-full-matrix.json`、`work/final-matrix-run.json`：最终原始结果及运行前后源码/二进制/输出凭证；
  `work/online-mission/final-independent-classification.json`：各场最后非 fault 行的真实原因。
- `work/online-mission/final-independent-review.md` 保存各次独立复核；`INTEGRATION-HANDOFF.md`（同目录）顶部已标交付完成，
  下方的暂停、未构建、未上传内容均为历史。`work/` 被 Git 忽略，只在这台 Mac 上可用；换电脑应以仓库报告/fixture为准，缺材料须重新真实捕获。
- 小场现状复跑：在工程根目录执行 `cargo run --release --locked --offline -p xt-stcar-robot-runner --example online_matrix -- both small_5x4`。
  目前返回非零是如实保留失败，不能改预期将其转成通过。正式复跑/构建步骤见下方说明与命令手册。

上一次交接仅改文档；本次补充主机捕获脚本、真实fixture及测试，未修改生产算法或替换上一轮交付包。继续开发时按实际改动重验；
暂不接车、不读视频、不动 MCU/车端 libc、不执行目标程序或模拟器的边界继续生效。

## 当前交接快照（2026-09-16）

### 用户目标与授权

- Rust 为主、Mac 交叉编译、Muse Pi Pro / RISC-V Linux、YOLO26n Detect；各模块程序写好。
- 最新要求：根据官方整车资料继续完善，能用 Rust 的地方用 Rust；写完更新本文件和 README，
  README 必须说明代码结构、每个模块位置和职责。之后要求总体审查架构与实现，并阅读根目录赛项三规则初稿。
  最新追加：比赛任务状态机、斑马线与灯色、导航绕障闭环先写完，实车到后再验证；已增加 Rust 模块与合成闭环。
  根目录 `Mac与车端命令手册.txt` 记录 Mac 和车端的命令/用途/参数/执行位置。不能把模拟完成写成已实车参赛。
- 用户说明车上使用 eMMC 5.1：以正常运行和比赛为优先，仅减少不必要落盘；不要降感知/控制频率，
  不因可选日志额度触发控制故障，不改系统挂载/swap/journald。必要诊断照常保存，硬件寿命不压过比赛性能。
- **暂不接车**。本轮未 SSH/部署/打开真实设备/控制电机/刷系统或固件，也未在 RISC-V 目标或模拟器执行。
  新增串口实际读写测试只使用 Mac 创建的 PTY，不要将它写成实车验证。
- 用户指定 GLIBC **2.38**。只修改本地交叉构建的目标基线，不安装或覆盖车端 libc。
- 官方资料目录 `/Users/yuhaojin/Documents/进迭时空无人车（2026）学习资料`，用户明确 **不要看视频**。
  只读取文档和源码；视频只枚举名称，不播放、不提取内容、不转录。
- 用户已授权将源码、脚本、配置和文档提交/上传到 `https://github.com/kxkxkxa767/XT-STCAR.git`，分支 `main`。
  编译链、环境、模型、构建产物、大型厂商包、镜像和缓存不进 Git。
  最新上传约定见 [上传规范.md](上传规范.md)：日常直接在 `main` 修改、提交和推送，不另建功能分支或 PR；
  每次修改前检查云端更新，有更新先拉取；`git commit -m "..."` 简要说明本次具体改动。
- STM32F103 固件读取由用户明确暂停；不恢复解保护、读取或刷写任务。不要回旧 XT-NetRC。

### 当前整合状态（2026-09-16，最新）

- 用户已继续统一整合与复核；本轮源码、最终矩阵与本地交叉构建已完成整合，实现提交 `ab323a2` 已推送 main。
  上传后已核对远端 HEAD；后续文档提交和最新同步状态仍以 Git 实查为准，不替用户修改模型设置。
- 冻结源码后的最终完整矩阵：**4/19** 符合预声明检查；通过的只是 missing_markers 与 missing_cones 各同步/异步，
  正常场 **0/15 完成**。独立裁判共14场证明两桶（12个正常场、2个缺标记场），随后都在 ApproachLight
  因看见灯但缺确认的停车区几何而停止。三个小场均未通过第一桶：同步46.200s、异步43.863s、扰动异步45.045s结束。
  全部19场最终都未完成比赛，故障为普通停车超过10s。**在线比赛闭环尚未验收，不能把所有失败都归为灯信息不足**。
  11个预声明输入与运行前后源码账、真实命令、主机可执行文件、stdout/stderr哈希均已核对；各轮开发失败保留。
- 生产新增：仅 OrbitCone 的实际姿态+5个观测短弧姿态/8步停车包络选速提示；原 source/history/负线/lease 门保持。
  StopLine/FinishMarker 槽内身份在真实唯一重观察时可续接，过TTL首新帧重新计1、第二不同采集帧才恢复 confirmed；
  observations表示当前确认周期的独立视觉帧数；invalid/歧义不能复活，processed保持，原TTL不延长。
  runner显式pin至多2个当前引用的StopLine/FinishMarker槽，避免其身份被过期槽回收；pin不续期、不伪造fresh或确认。
  未被保留且已回收的槽不能找回旧ID。锥桶规则不变。
- RollingLocal 的 <0.35m 定向目标按原严格双弧→原严格单弧→原半容差区域→lattice，在新boundary下同账重证，实际18100/18800大圈反例已修；
  Fixed 不改。新增 fixture `online18100_boundary_short_arrival.json`。Finish 完成显式要求原全车清灯线，重叠区域未清尾则 Fault。
  最终workspace为565通过、0失败、3项按设计忽略；早期定向计数不再作为当前总数。
- 新增仅RollingLocal的完整停车后备证明：原disk失败后，用StoppingEnvelope覆盖原2控制周期+全制动、历史速度上界、
  Kmax下任意变向和完整车体；不是固定转向名义轨迹，原rollout/误差/采用门保留，Fixed不变。
  零sweep仍按实际切向证明整车体KnownFree，不直接判Unknown或自动放行；真实出桶完成门不变。
  第四轮出口交接反例及后续修正均保留；当前结果以最终矩阵为准，不直接套用前轮trace推断当前故障。
- 三处追加生产修正已纳入最终构建，开发与最终矩阵分别记录：Rolling原region及zero候选全失败后可同账认证
  “实际κ保持短段→原rate回中→直行”，三段连续传递误差、真实终点满足原half位置/航向容差。38300原成功及Fixed三帧保持，
  38400/38500短接定向回归通过，真实越线/障碍/误差/预算/非法输入负例拒绝；不据此宣称整场完成。
  原strict continuation动作候选全部失败后，仅Rolling从该候选完整周期预测姿态/曲率/error调用旧region证明，
  同账保留原运动/碰撞/制动/采用门，不调用新三段候选，也不直接报告任务到达或adopt。
  仅Rolling+recovery+有限侧带边界新增整车扫掠二证：端点车体凸包加L/2×(1+Kmax×body_radius)、原净空/重启和累计误差，
  所有点须由同一允许半空间分离；world bounds/obstacles仍原胶囊，Fixed和point-only参考检查保持。
- 后续增加仅该同一范围内的第三份时间域证明：原胶囊与一阶扫掠仍失败时，以实际MotionTransition给出
  M=A+V²K+r(AK+VS+V²K²)，端点完整车体凸包加M×dt²/8覆盖全段。V/K取initial/target上界，A取原accel/decel较大者、
  S取原slew；饱和处v/κ及车体点一阶导数连续，恒速/恒曲率也不削减A/S上界。
  primitive的dt=同次ds/max_speed，rollout用与predict一致的实际缩尾duration.min(dt)，不除以实测低速；
  保留endpoint位置/heading累计误差、clearance/restart、roundoff及共同分离侧，world/obstacles仍原胶囊。
  Fixed在新界计算前退出，无额外samples/solver/node预算。真实五帧fixture为online32000_body_chord_tube.json，
  不能把32100历史停舵状态的短route当Drive，也不宣称31700..31900原region路径与现在优先strict完全相同。
  独立数学审查已追加final-independent-review。小场32500新鲜StopLine负向门已实查并追加当前源选速提示，
  生产已冻结、11场真实CLI编译完成且控制器逐值相同；最终19矩阵与完整build均已完成。
- 最新旧8证据为 `work/online-mission/legacy-integration-timing-final.json` 与 `legacy-integration-comparison.json`：
  8/8完成且原性能门通过；九项关键指标、序列化输出计数及八项单因素比较与v9精确一致，未声称逐命令或完整轨迹逐点相等。
- 最终Rust工作区565通过、0失败、3项按设计忽略；实际日志另记录tracking/timing/online三个example为2/5/2通过，Python交付37通过。
  fmt、all-targets clippy -D warnings、双RISC-V交叉链接与静态ELF检查通过；150份编译输入与当前源码核对一致。
  目标基线riscv64gc-unknown-linux-gnu.2.38，实际最高引用GLIBC2.34，RISC-V64/LP64D/RVC/PIE，加载器/lib/ld-linux-riscv64-lp64d.so.1。
  xt-stcar为1,217,056B，SHA256 b83012928876ff87f322c9b4bac8e51c9a338b4c56c495c1a3598e8bc32b11d5，仅需libc；
  xt-stcar-robot为2,376,448B，SHA256 259b419d6f9061ccb6d56b746d5c23d80a2d0a23d51bcb337e97514b0b793042，需libm+libc。
  正式证据为docs/online-mission-{matrix,build,validation,experiments}.json；本地两包路径、哈希及核验见包外online-mission-delivery.json。
  未运行RISC-V程序、未接车。测试与链接通过不代表在线比赛完成；开发版489测试/旧二进制保留为历史。

### 在线元素驱动（2026-09-16，尚未通过整场验收）

- 用户最新授权按分享 `https://chatgpt.com/share/6aa93bcc-f29c-83ee-996b-865058fcd348` 直接改；
  修改前 fetch 确认 main/origin=`4dfd201db977ffa5c18c1eca9f896dade4ad7875`、0/0。
  分享正文已完整读取，本地开发记录 `work/online-mission/new-plan.txt`：任务顺序固定、位置在线估计、
  持续更新短目标，只执行一段再观测。不是再调 LQR 或恢复整场固定坐标。
- 新增 Rust `local_world.rs`（有界元素身份/视觉雷达关联/空闲障碍未知）和 `online_mission.rs`（观测驱动任务）。
  `runner/online.rs` 接入旧 Autonomy、PP、完整车体制动包络和异步最终证书；旧任务几何不进入在线目标。
  `online_simulation.rs` 隔离场景真值，只供渲染与独立裁判；改变场景时控制器配置逐值相同。
  CLI `online-example/online-compile`，内存感知 `RoadPipeline::new_online`；旧入口保持兼容。
- 已确认锥桶的视觉语义与当前几何分开：`last_visual_at` 最长20s、`last_geometry_at` 无更新满1.8s失效。
  360°雷达只维护已视觉确认的唯一圆簇身份，不新建桶、不猜颜色、不增加确认次数；同刻位姿、外参、半径及误差门保持。
  独立拟合误差保留视觉下限，不逐帧重复累加；仅纯roundoff保留旧浮点表示，位置上限1nm，真实毫米变化仍更新。
- `KnownFree` 已改为完整面积的自适应证明：原外接圆成功路径保留，否则用固定栈按长边二分定向外包矩形，
  每个接受叶矩形完整包含于已证空闲圆，最多64次原空间查询；未知、障碍或额度耗尽不能放过未覆盖区域。
  `Obstacle` 独立检查同源真实有限回波到最多32点凸包的欧氏距离，包含原量程/位姿/年龄误差；
  覆盖圆多出的面积不制造碰撞。未来短弧Unknown只能作规划假设，当前目标和整个执行停车包络仍须KnownFree。
- 绕桶R按实际车体半宽、桶几何误差与原曲率能力取下界，前后悬保留在分段扫掠证明；仍可行的R保持。
  入口必要时用原Stop的位置/航向/停稳条件对齐，无新增hold；稳定带航向目标可用两个horizon（默认1.6m），
  更远是0.8m无航向临时引导，搜索仍单horizon。未来短弧未知时先用原approach限速.18，完整空闲才允许cruise .3。
  出口目标沿切线延伸“桶位置误差+2×原目标位置容差”，实际出口投影须越过位置误差，同时满足原半圈/外侧/near条件。
  圆顶尚差5–6mm过中线就提前processed的缺陷已修，圆弧进度仍封顶π，未虚增角度或放宽容差。
  剩余转角为零仍按该绕行方向的切向做完整车体和净空KnownFree证明，不能直接当Unknown，也不免去出口完成门。
- runner的OrbitSpeedHint仅在OrbitCone用实际pose+5个当前观测短弧参考姿态检查完整StoppingEnvelope，
  采用原Kmax及感知年龄+控制周期horizon；原upper已证则保持，否则8次二分只选已证正下界，target/continuation同步限速。
  这不是连续采样执行证书，无证明不伪造零速目标、不改写真正较高速度；source/history/negative/lease与实际执行门全保留。
- 仅绕桶且负线未获绿灯释放时，在Nav前对current-source姿态的完整StoppingEnvelope加pending/confirmed负线证明，
  线误差沿原source/checked时钟增长，最多8次二分已证正速度上限，target/continuation同步限速。
  下界用同源测量或planning投影速度按原max_decel与实际tick间隔（不超原period）计算，再与有效adoption速度区间下界取max；
  source/planned时间须一致。几何cap低于可执行下界则不改提示，不抬高未证cap、不伪造当前低速，原actual-source/history门继续拒绝不安全Drive。
  新fixture/tests为online_source_stop_speed_hint；无新车体/控制/TTL参数，不刷新线身份或阶段，不解除原负约束。
  开发async33260的boundary_changed后OpenEmpty只有有限小搜索，未耗预算且rollout=0，尚未进permits；cap高于采用下界。
  缺完整缓存只能称原约束下有界规划未找到路径，不能称几何无解/节点耗尽/source gate拒绝；同步局部改善同样未完成整场。
- 导航三个真实快照问题已有修正和专项回归：26580ms同目标短缓存漂移在原账内优先重证，
  49580ms滚动无航向点在[.35,min(2×lookahead,2m))内先试短单弧，严格失败后可认证半位置容差内真实端点；
  21380ms定向斑马线接近在原单/双弧后，另认证实际初始曲率按原模型向零渐变的短前进候选，包含完整累计误差及半位置/航向容差。
  `TargetPolicy::Fixed` 为默认，在线统一`RollingLocal`；不按场地开关，不重置预算，不伪造端点或已采用命令。
  <0.35m定向RollingLocal目标按strict2→strict1→region→lattice同账重证新boundary，18100/18800真实大圈修正，邻帧strict成功保持。
  仅RollingLocal原停车disk失败时可追加完整StoppingEnvelope，涵盖2周期反应+全制动、历史速度、Kmax任意变向；Fixed不改。
  最新 `work/online-mission/legacy-integration-timing-final.json` 旧八场全部完成、原性能门通过；九项关键指标、序列化输出计数及八项单因素比较与v9精确一致。
  初版通用点连接造成的旧八场退化保留在 `legacy-point-timing-final`，不能删除中间失败。
- 灯色不提供地距。`vision/ground_markers.rs` 仅实现显式实验白条协议，库默认禁用，演示场景显式启用；
  不宣称官方现场有这些标记。官方资料只给光电到灯约1m及终点80cm，未确认可见线图案。
  规则中的随机布置没有提供左桶出口到灯的正距离下界；不能用旧 FieldSpec 布局检查冒充规则先验。
- 两个独立观测缺口仍在：固定灯色ROI可先看到灯而无停车区距离；已确认的地面标记又可能在清尾前退出前向ROI。
  默认直行的2.44s/计既有误差约4.89s仅为特定配置估算，不是全场不可行证明。缺几何有灯色仍Stop，不增加stale正向授权。
  已processed的线有真实观测可刷新但不能重选；过期StopLine负约束按来源时钟继续扩张误差，完整包络仍可在线前或侧带外通过，
  不因TTL瞬间封死整个侧带。过期证据不能支持新任务、绿灯或清尾；Crosswalk目标也减含航向项的完整region_error。
  区域过TTL后真实唯一重观察首帧重计1、默认第二独立采集帧恢复确认；ID/first_seen/processed保留，invalid/歧义不复活。
  任务显式pin≤2个当前引用区域只防槽回收，不延长TTL或跳过确认。终点完成显式要求light_cleared；合法区域重叠而未清尾则Fault且不mark_processed。
- 首轮2/19、第二轮3/19、第三至第五轮各4/19仅为历史阶段矩阵。冻结源码后的最终矩阵4/19，正常完成0/15；
  14场通过两桶但灯前缺几何，三个小场尚未通过第一桶，另两场缺桶有限搜索后安全停止。通过项仅四个预声明缺失场景。
  独立`online_referee`检查真实轨迹/顺序/半圈，任务自报processed不能替代裁判。在线比赛闭环尚未验收，不能用旧35/36或旧八场代替。
  82.8s同步两桶后缺几何Stop、84.383s异步第二桶回环耗尽语义期限、早期搜索/覆盖与移动carrot附heading失败均保留。
- 停车包络速度提示已接入最终源码；565 Rust/3按设计忽略及本轮双ELF是当前构建证据，早期`development-build-1.*`的489/2仅为历史。
  三份online配置已生成，11种真实CLI编译记录在`work/online-mission/cli-compilation-final.json`，配置、控制器一致性、源码与报告已核对。
- 预声明11输入（含3异步时序变体）、共19同步/异步 PP 场次；包含物体移位、5×4、上下道不等宽、
  斑马线移位、短遮挡、缺桶、缺标记。输入已在首跑前冻结，不能删除失败或改场地追成绩。
  原车体、限速、净空、3s/300ms、普通停车10s、节点40000、终端256/1024/65536保持。
  几何TTL1800ms仍保留；新增雷达维护不能新建颜色身份或增加视觉确认，并有独立有限语义年龄。
- README、`docs/在线元素驱动任务.md`、命令手册已同步最终入口与结果；正式 `online-mission-*` 矩阵/build/validation已归档。
  两包及核验以包外delivery报告为准，实现提交 `ab323a2` 已推送并核对远端；后续状态以Git实查为准。
  本轮继续直接main、GLIBC2.38本地交叉基线、暂不接车、不读视频、不执行RISC-V/模拟器。

### 赛场规格自适应（2026-09-15，历史）

- 用户已授权“你来补齐自适应赛场规格的吧”；开工前 fetch 确认 main/origin 为 `98368200f6c0eb8189c3e1d79941469c821a10b9`、0/0。
- Rust `robot-core/field.rs` 读取显式米制 FieldSpec，按规则图5外场5–8×4–6m、上下直道1–2m等检查范围与组合，生成官方右桶→左桶→上方直道拓扑。
  0.8m发车/终点区、纸条0.105×0.297m及间距0.105m、0.28m方底锥桶（外接圆半径0.14√2）保持，不缩放车辆/控制限值。
- `FieldPlanning` 从原导航参数取得footprint/clearance/曲率/网格尺寸；首选绕行半径按侧墙间隙和经原Grid膨胀的灯禁带角点限制。
  输出右/左实际半径、每桶5个45°间隔切向通过点和上直道朝向0入口；参考几何不是动态可行性证明。
- `runner/field.rs` 在启动前编译并冻结配置；CLI `field-example/field-layout/field-compile`，模板 `config/field-example.json`。
  校验派生几何与规格一致；仅浮点叶允许8EPS×max(1,abs)的JSON读回舍入，整数/结构严格，厘米篡改拒绝。
  measured只标来源，仍simulation_only/unverified；没有自动测全场、定位灯/终点或重分左右桶身份的实现。
- Mission可选 `cone_waypoint_headings_rad` 默认空，保持旧配置序列化；新通过点要满足原位置+朝向容差。
  定向PassThrough复用原Stop的有界短连接保护，仍有真实任务验收/后段认证，不强制停车或假造到达。
- `light_boundary_region` 限定上直道禁带；整个车体/凸包/胶囊须位于同一允许半空间，不能只验各角/端点。
  原Grid失败后可用同账连续车体恢复，旧Grid成功路径保持；切域重建真实误差路径，不沿用未认证缓存。
- 九种输入在首次完整矩阵前冻结，不删除失败/不改尺寸追成绩；新场统一180s时限，原普通停车10s/停稳3s/绿灯300ms及40000/256/1024/65536预算保持。
  最终36场35通过：默认PP18/18，实验LQR17/18；tall_5x6同步LQR在左桶阶段节点耗尽/普通停车超过10s，失败完整保留。
  22/36与34/36等开发失败见 `docs/field-adaptation-experiments.json`，输入未删改。大场异步PP原灯前卡在目标6.7cm外，最终99.787s完成。
- 旧八场时序回归已复跑，八场完成且固定性能门通过，时间/路程逐值保持v9；最终源码的受影响检查和完整构建均已通过。
  原预算fixture字节保持。封墙测试仍Stop、验新node耗尽原因与总节点上限；定向直线through现在有1次短连接检查，原连续交接断言保留。
- 原精确中心连接及普通/连续搜索失败后，可用同一终端账尝试真实积分端点进入原位置/朝向容差一半；包括完整累计误差/车体/边界认证。
  不重置预算、不替换adopt、不假造到中心。节点用满后该后备不新增搜索节点，耗尽诊断保留；终端额度耗尽仍拒绝。
  `arrival_region_attempts/accepted`仅诊断，零值旧JSON省略；大场灯前最终尝试3/接受1。独立只读复核无阻塞，范围见验证报告。
- 本轮技术说明 `docs/赛场规格自适应.md`，正式证据前缀 `field-adaptation-*`；README/TXT已接入新模块/命令，最终409 Rust/2跟踪example/5矩阵example/36 Python交付（39子测试）通过，2项按设计忽略；fmt/clippy/双交叉链接与ELF通过。
  112份编译输入，GLIBC2.38基线/实际最高2.34；app SHA ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9 /1,217,056B，
  robot SHA ee6de9fc7f6337304e103c45fef9f0d937b94a0fbe5170d7b8ef8e9303011a8f /2,090,544B；分别需libc和libm/libc。
  三种真实CLI生成/读回/整场完成：5×4 74.9s、7×5 109.2s、8×4 95.0s；与内存矩阵独立记录，不宣称全轨迹逐值相同。
  持续主机钟23源/119转弯Drive，poll最大21ms，末源2200、2460到期Stop，终态v/k0无碰撞越界；非目标WCET。
  本地包见 `docs/field-adaptation-delivery.json`，Git实际提交/同步状态以HEAD与origin为准。
  暂不接车、不执行RISC-V/模拟器、不读视频，PP默认/LQR实验，GLIBC2.38本地交叉基线保持。

### 第九轮路径连续性修正（2026-09-15，v9历史）

- 用户直接授权第九份分享 `https://chatgpt.com/share/6aa89fa3-3894-83e8-afe8-45451cbbc42f` 和
  `XT-STCAR_a623638_review.zip` 修正；修改前fetch确认main/origin=`a6236380a68431342ab9e76f468c587c9dd775f3`、0/0。
  本轮按上传规范直接main；实际提交/远端状态通过HEAD/origin核对，不把构建基线当最终提交号。
- 4630字符正文完整读取；ZIP SHA256 `5ac964c02693734ebe2dfdd8931c61b0631a62a405054c9ca6ea4cc54ae1c718`，
  Python算术结果与提供文件一致。外部材料不包含Rust/实车执行；原ZIP和规则PDF保留、不暂存。
- 三版本Rust离线消融：7d原版和检查顺序-only均54.447s/10.317616m，285条实际命令和状态逐值相同，前瞻预扣区间减少2.62%；
  v8坏LQR88.647s/16.515558m，实际Stop都4513ms。独立Cargo target避免历史mtime复用旧产物，首次无效共享target试验已剔除记录。
  复现脚本 `scripts/review-motion-ablation.py` 固定commit/源hash/补丁，只在work隔离主机运行，不恢复旧时基到生产。
- 原45360ms灯前同目标、同当前姿态旧剩余0.502864576→新4.910317092m，44候选接受，无节点/终端耗尽、无连续恢复。
  原始四帧、传感源、任务、每帧128真实history、完整缓存和176候选首因在
  `crates/robot-core/src/navigation/fixtures/lqr45360_terminal_seed_origin.json`。
  原45040候选认证+100ms状态，而下一规划45160为+120ms，旧初值未推进导致冷/旧warm各8迭代失败。
- Rust `navigation/continuity.rs` 缓存候选预测下一状态，原方法失败且额度剩余时按正向位移推进数值初猜，再从当前pose/k/Grid/误差完整认证。
  提前/负向不虚构行程，无效/耗尽提示弃用；预测anchor不是实际采用时钟。恢复成功才延续该连接族，普通成功路径保持原顺序。
  **恢复族的候选优先级有意改变**：完整原硬门通过后，先认证接续，再接近已认证首段曲率，最后原评分。
  used_seed来自实际求解分支，不用诊断计数做控制。目标/朝向、到达策略、边界和任务停驻变化清理提示，规划不替代真实adopt。
- 新可选 `route_length_change` 仅用于同目标、旧缓存可用且footprint drift重建成功的同姿态剩余路线比较；不据缺失判断没有绕路。
  所有原共享时基/最终poll/lease/完整车体/边界/误差包络/Q/R/任务门保持；节点40000、终端256/1024/65536、单次8迭代保持。
- 性能预算在试改前固定于 `crates/runner/examples/fixtures/motion-v9-performance-budget.json`，最终逐字节相同；控制器不读取预算。
  v7/v8同工况较快成功场为参照：整场时间/距离110%，阶段+max(1000ms,10%)，实际Stop+1000ms；缺失/重复/无效指标也失败。
  `runner/examples/support/performance_gate.rs` 仅在8场完成后验收，超限保留JSON并非零退出；不调门追成绩。
- 最终固定顺序input[60,80]×adopt[3,7,9]/[5,9,11]、input[40,60]×adopt[3,7,9]/[5,9,11]：
  PP75.467/54.985/56.443/54.449秒，LQR54.663/55.265/54.249/53.445秒，8/8完成且原安全与固定性能门通过；
  PP四组时间/路程逐值保持v8，坏LQR少34.398s/6.224552m，灯前59.400→24.994s、实际Stop4513ms保持。
  全场证书/曲率slew拒绝和终端总额度耗尽均0；原3000ms停稳/300ms绿灯/整车终点/终态v/k0保持。
  同步PP58.8s/11.508841m、LQR54.3s/10.249675m保持；默认PP仍有174恢复活动tick，不能称任意工况收敛。
- 最终371 Rust通过/0失败/2忽略，跟踪example2、矩阵example5、Python交付35通过；fmt/clippy/双RISC-V静态链接通过，108编译输入。
  新7项continuity测试检查原始history一致性并重放记录投影状态的Navigator，不是独立重跑全部worker历史重建；全链路由真实worker矩阵另覆盖。
  第二份45180 fixture是标注的数值试验，不能称a623原始数据。独立只读复核无阻塞问题，范围见route-regressions。
- 默认异步开关计时功能字段逐值相同，导航平均/最大PP6.216/59.705ms、LQR4.346/11.099ms，功能钟等待worker时冻结，非目标WCET。
  持续主机时钟23源/119转弯Drive，poll最大21ms；末源2200、原期限2450、2460 Stop、2686完全停稳回中，无碰撞越界。
- 该轮app SHA `ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9` /1,217,056B、需libc；
  robot SHA `365e34dcba4d7dee93610cc97f96b4edc094370af1fe7c2570c6552fa2344c93` /2,029,552B、需libm/libc。
  GLIBC2.38基线、实际最高2.34；该轮构建/验收/本地包见 `docs/motion-v9-build.json`、`motion-v9-validation.json`、`motion-v9-delivery.json`。
  本轮技术说明 [路径连续性与性能回归](docs/路径连续性与性能回归.md)，README/TXT已同步；模型/ORT未改、不重跑原生推理。
  PP默认/LQR实验、暂不接车、不执行RISC-V、不读视频、正常运行优先于eMMC寿命约束保持。

### 前轮采用时基与交叉时序修正（2026-09-14，v8历史）

- 用户直接授权按第八轮复核修改；修改前 fetch 成功，`7d28761c40f7d7e69a8ac77977f5b6710dd5a0c6`
  与 origin/main 一致 0/0。最终提交/推送状态以实际 HEAD 与远端为准，继续直接 main。
- 分享 `https://chatgpt.com/share/6aa7e186-f7a4-83e8-8cad-c7d07e02d3e1` 已从服务器 hydration 提取并完整读取4792字符正文。
  原 ZIP `XT-STCAR_7d28761_review_reproduction.zip` SHA256
  `d130cc0096adc5cee61d89e9a8fbde74dcf1a275acc8f4901739cc2cdb5e7d04`；两份Python脚本在work隔离副本执行，生成结果与提供文件一致。
  它们是算术/仓库数据复核，不能写成Rust或实车验收；原ZIP与规则PDF不修改、不提交。
- `robot-core/admission/slew.rs` 共享曲率目标限幅时基：从同一seqlock历史取得真实命令变化时刻及revision，
  导航/当前采用检查/最终证书均以规划窗起点减去该时刻计算；poll仍按实际now、revision、原lease独立复核。
  相同整条命令重复输出不刷新，只有速度变化也算新命令；Stop目标归零、物理模型继续回中。
  revision0且无真实变化才用原启动周期，伪造/缺失/未来时刻拒绝。后备前瞻保留相对秒和半毫秒，速度变化同步推进虚拟采用时钟。
  原真实源历史V/K不清除、不延长lease、不在poll临时裁剪动作。手工processor越界候选现在被最终证书提前Stop，
  正常Navigator已在生成时避免该候选；不能把该测试中的行为变化说成旧Drive保持。
- 多源后备前瞻移到原基础rollout成功后、终端认证/选择之前；全部原门仍保留，rollout不重算。
  原PP19380ms完整361障碍/285点含续段缓存已从隔离基线取得；44个Grid拒绝、accepted0保持，
  forecast 132场景/3850投影/167398区间/1650源检查降为0。基线未记录该帧真实last_change，
  排序专项显式用一周期合成时基保持旧候选区间；真实采用时基另用独立Rust/worker回归，不能混为原始完整历史。
- 新可选route_rebuild_reason记录首次缓存失效原因，失败搜索续留，成功后清除，复用缓存不重复输出；
  原候选评分、完整车体/净空、连续恢复/累计误差界、共享节点和终端预算、Q/R均保留。
- `runner::async_simulation::simulate_async_observed` 只给离线模拟提供输出后观察回调，不进入worker/poll或替换命令。
  `runner/examples/async_timing_matrix.rs` 固定2×2输入/采用时序×PP/LQR，内存限额16384命令变化/2048规划/每规划2048路径点，
  超额显式标记；只在该例子启用，默认无逐tick落盘。真实输出按时刻比较、规划按同source比较；
  返回路径含当前投影位置及剩余参考，坐标或点数首次差异不证明拓扑分叉，需结合revision/重建原因/任务阶段。
  实际Stop时长/段数、导航Stop、被拒绝时保持Drive、采用拒绝分开，Stop时长含启动/终态制动且不等于物理静止。
- 隔离原版完整8场7完成1失败；新交叉input[60,80]×adopt[5,9,11]的LQR在91.669秒普通停车超时，终态v/k0。
  baseline矩阵加入只读observer前后全部summary除wall_time_ms外一致；原4场与v7仓库结果一致。
  本轮技术说明 [采用时基与时序交叉验证](docs/采用时基与时序交叉验证.md)，证据统一 `docs/motion-v8-*`。
- 最终固定顺序input[60,80]×adopt[3,7,9]/[5,9,11]、input[40,60]×adopt[3,7,9]/[5,9,11]：
  PP75.467/54.985/56.443/54.449秒，LQR85.863/86.269/88.647/53.445秒，8/8完成且cert/slew拒绝全0；终端预算耗尽全0。
  全部原3000ms停稳/300ms绿灯/整车终点/终態v/k0保持。同步PP58.8秒、LQR54.3秒保持。
  原失败LQR完成86.269秒；但早input早adopt LQR54.447→88.647秒、10.317616→16.515558m退化，默认PP73.863→75.467秒。
  不宣称普遍鲁棒或性能全面改善，不进一步改Q/R/限值来追成绩。PP默认恢复4次/174活动tick；LQR四场均无恢复，路线仍时序敏感。
- 最终364 Rust通过/0失败/2忽略，跟踪和矩阵example各2通过，Python交付35通过；fmt/clippy/双RISC-V静态链接通过，103份编译输入。
  节点/终端旧回归与nav42项保持；独立审查修复临时不可达目标切回原缓存时的pending重建原因残留，控制/缓存行为不改。
  真实controller旧测试原本期待输出拒绝，已改验证10ms=.04区间、source100于350ms到期Stop及500ms模型归零，6项期限测试通过。
- 主机同步profile预热1/测3：PP1619.924→1632.190ms(+.76%)、LQR1491.721→1513.593ms(+1.47%)；外部负载未控制，非普遍加速证据。
  默认异步导航均值/峰值PP6.395/59.369ms、LQR4.573/13.166ms，含非Drive计划；开启计时后功能字段一致。
  本轮PP有恢复，不能把局部省下预测推为整场加速；功能时钟等待worker会冻结。持续Instant短场23源/119转弯Drive、poll最大21ms，
  source2200原期限2450、2460ms Stop、2687ms停稳回中，无碰撞越界；都不是目标WCET。
- 该轮app SHA ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9 /1,217,056B、需libc；
  robot SHA ed420aa4615948ced31bccfa61bb2f8081b4c8005224646edad101ba0fbd8bd6 /2,026,384B、需libm/libc，最高实际GLIBC2.34。
  最终测试/构建/包分别见motion-v8-validation/build/delivery，包只在本地dist、未上传车端。
- 暂不接车、不读视频、不执行RISC-V目标，PP默认、LQR实验、GLIBC2.38本地交叉基线/模型/工具链保持；STM32任务仍暂停。

### 前轮采用约束与前向恢复（2026-09-14，v7历史）

- 用户已授权“你修改吧”，按第七轮意见实施；修改前 `git fetch origin` 成功，
  `c440abf26eb159e8ec742dfcd3663fb9d2db701b` 与 origin/main 一致 0/0。最终提交状态以实际 HEAD/远端为准。
- 最新分享 `https://chatgpt.com/share/6aa7b111-c61c-83ee-9d3d-437a65ddce42` 已读取完整正文。
  用户原 ZIP `XT-STCAR_c440abf_review_reproduction.zip` 的三个文件已在 work 隔离复算；
  SHA256 `4d22c46cdf693590cb2739ac9718aeb214d00831d0a50cc92370d614a47f3010`。原 ZIP 与规则 PDF 不修改、不提交。
  分享把 246 组称为合法目标组合不够精确：它们只满足绝对限值，不全满足该拍加减速/转向变化限制，
  也不是完整 Rust 认证。本轮 Rust fixture 单独检查正常运动合法子集，当前真实历史 V/K 不清零。
- `control_execution::certify` 改为结构化 `Result`，保留原早退顺序和独立最终证书；首因包含障碍索引、符号余量、
  源/规划/lease/采用窗、历史与最终 V/K 和停车矩形。Worker 与 observed_plan 用 Arc 分享诊断，异步报告保留有界窗口。
  最终仍只能执行 `ControlPoll.command`，不把被拒绝 report 当执行输出；默认无逐拍落盘。
  新增诊断 JSON 全默认/None 省略，缺失=对应 Rust 默认值，有值完整保留；默认PP事件输出9行/99310字节，17帧首失败窗口保持。
- `robot-core/admission.rs` 共享原源车体系停车矩形，`runner/control_admission.rs` 从真实历史与 runtime lease 每源准备一次约束。
  当前候选前置检查保留全部历史峰值；下一阶段用 3 个固定采用延迟、实际采集相位和本次源年龄预测正常减速后备。
  预测最多 64 个规划时刻、2 秒，额度不足或未完整停稳直接拒绝；不改原源期限、不缩小当前停车覆盖。
  这只是有限模型前瞻，不是任意采用延迟序列或递归可行性证明；最终连续采用窗证书和新观测不可省略。
- `navigation/recovery.rs` 在普通前向 OpenEmpty 后使用剩余节点预算做连续胶囊恢复；完整外接圆、原净空、地图和半平面保持。
  连续域额外预留 .5×min(vmax,aT) 最小起步候选所需停车距离，修复中间版本的原双锥回归；原实际停车门不变。
  `primitive_envelope.rs` 按 v/k 饱和时刻分段，解析朝向/路程、固定至多 12 个位置积分子段与导数余项。
  胶囊使用真实弧长的 K L²/8 与累计位置界；误差经过节点、终端两段、PassThrough、候选下一拍与缓存续段重验。
  恢复末端以真实积分姿态连同误差在原到达容差内验收，不传送到目标。普通模式数值路径、v6 终端预留与 Q/R 保持。
  普通/恢复/后续路段共享原节点总账，节点/终端预算分别诊断；这些计数不是 CPU 时间。
- 本轮入口 [采用约束与前向恢复](docs/采用约束与前向恢复.md)，构建和验证证据统一 `docs/motion-v7-*`。
  同步 PP58.8秒/11.508841m、LQR54.3秒/10.249675m 保持；默认异步 PP73.863秒/14.173789m、LQR89.069秒/15.807183m，
  扰动异步 PP56.849秒/11.124416m、LQR53.849秒/10.217582m 均完成。4场最终证书拒绝0；默认/扰动每tracker曲率slew拒绝仍5/2，
  与实际Stop计数分开。原3000ms斑马线、300ms绿灯、整车终点和终态v/k0门保持；默认时序PP较v6慢16.2秒，不宣称全面改善。
  原停稳源与新增中间灯前源的8项恢复专项通过，265/12节点，42.344494/1.876981µm位置界。
  中间尝试的首次恢复停稳gate会使LQR79.263秒失败，已撤去；起步预留自身通过原双锥回归。
- PP 继续默认，LQR 仍实验。GLIBC 2.38、工具链、模型均不变；暂不接车、不读视频、不执行 RISC-V 目标。

### 前轮恢复预算与完整异步验证（2026-09-13，v6历史）

- 用户在第六轮复核后明确“你改吧”，已实施。修改前fetch成功，`b9f75c7427411c12285411efa4af793fa167b8c7`
  与origin/main一致0/0；提交/推送状态始终以当前HEAD与远端核对，不把构建基线当本轮提交号。
- 第六份 `https://chatgpt.com/share/6aa6b2fa-86bc-83ee-8424-04afe06f7d3c` 正文已完整读取；
  通过服务器hydration同时取得上一轮正文。v5“未取得正文”仅描述当时情况，当前不再阻塞。
  根目录 `XT-STCAR_b9f75c7_review_reproduction.zip` SHA256为
  `4cd3aa49b07b455c7841576cb827a35f786d4c20e35e199c6fb7654e86e569dd`，仅含README/Python几何及预算复算/JSON，
  不当成Rust补丁或完整异步验收。只在隔离work副本执行；原ZIP与规则PDF不修改、不提交。旧f83898a ZIP现已不在根目录，不恢复。
- `navigation/terminal.rs`在既有256求解/1024迭代/65536预扣样本内，为已认证短连接接续保留一次求解机会；
  普通候选触及暂时上限记cold deferral，不锁死全局预算。恢复前解除预留，普通已认证最佳仍优先，原rollout不重跑。
  前置route/lattice开销已计入；不足完整预留时只保留剩余额度，仍允许完整认证成功后执行，不把不足最坏估算当物理不可达。
  原42.2秒21/41/61/81档×镜像均Drive；21档51541样本保持，41档以上63799样本，新障碍仍拒绝旧初值。
  42.3秒真实lattice的194求解/604迭代/18087样本保持；该帧未同时触发短连接恢复，不能混写证据。
- 同步完整比赛PP58.8秒/11.508841米、LQR54.3秒/10.249675米，与v5共有报告字段逐值一致。
  PP保留2次可恢复Stop；LQR权重、默认跟踪器、原期限/碰撞/容差均未调整。
- `async_simulation.rs`新增完整RGB/雷达/同源位姿→真实AutonomyWorker→certify→poll.command→渐变转弯plant。
  默认100ms采样、60/80ms源延迟、20ms输出、3/7/9ms采用偏移；PP57.663秒完成，LQR29.463秒失败，终态v/k均0。
  LQR首次16.863秒证书拒绝后曾恢复，19.263秒lattice预算耗尽，19.469秒起最后停车；随后无前向路线，
  29.463秒“ordinary stop exceeded 10 seconds”锁存Fault。不能把第一张拒绝直接当最终唯一根因。
  此绕桶过程未启用新增短连接预留。保留失败且LQR仍实验；不放宽证书/延长10秒期限/加大全局预算来制造完成。
  下一轮若继续此问题，先细分证书拒绝原因，核对规划与可采用停车空间，单独复现停住后的前向搜索；不直接调Q/R。
- `control_diagnostics.rs`显式开启主机计时；排队、投影、processor、纯导航、终端求解、证书、发布/观察分开记录。
  原逻辑source/planned/lease不受Instant影响。默认不读钟、不逐tick写盘；共享Arc保留最新发布尝试（包括未采用/故障）。
  `observed_plan.report`与`latest.command`都只是诊断，车辆只能使用最终`ControlPoll.command`。
  调度钩子仅离线故障注入，在worker侧/锁外执行；默认关闭，不在poll里回调。
- 完整异步功能测试等待后台时冻结逻辑钟，不能用于deadline结论；`async_deadlines.rs`在后台阶段挂起时持续推进
  poll和车辆，覆盖过窗、过期、latest替换、revision/曲率变化率与完整停车。`async_host_clock`另跑一次持续Instant时钟短转弯/断流，
  使用独立短场景（斑马线尚未可见），记录真实主机poll间隔与各段耗时，不声称完整比赛实时性或目标WCET。
- 本轮入口 [恢复预算与完整异步验证](docs/恢复预算与完整异步验证.md)，机器证据统一`docs/motion-v6-*`；
  当前测试/构建/源哈希/包状态分别以validation/build/delivery报告为准，旧轮报告保留历史。
  正式主机同步顺序profile：PP平均1606.468→1612.415ms，LQR1480.943→1480.708ms，未见明显总耗时变化。
  持续Instant短场景23源/119次转弯Drive、poll最大间隔21ms，末源2200ms于2461ms轮询Stop，2687ms完全停稳回中。
  暂不接车；未改变工具链/模型/GLIBC基线，未执行RISC-V目标。

### 前轮终端接续、计算额度与异步停车空间修正（2026-09-13，v5历史）

- 用户要求根据第五份运动意见和工程根目录复现ZIP继续修正，拟定计划后直接实施。修改前网络恢复后fetch成功，
  `f83898a050649e03f25ad51b9c18c0bb12c743b2` 与origin/main一致0/0；最终提交/推送状态以HEAD和远端核对。
- 分享链接 `https://chatgpt.com/share/6aa6a3ce-6304-83ee-b875-2298fc1e846` 网络恢复后仍由服务器返回
  “Can't load shared conversation”，未读到第五份完整正文。已请求新的分享/文字，本轮依据已核实本地ZIP实施，不能声称逐条读完分享。
  `XT-STCAR_f83898a_review_reproduction.zip`只有README、Python几何探针和两个JSON，无Rust补丁/原点云/整场验证。
  原ZIP和比赛PDF不修改、不暂存；ZIP哈希与来源见 `docs/motion-v5-before.json`。
- 原提交隔离重放额外仅加三条内存记录，取得42.1/42.2/42.3秒完整输入与每帧360世界障碍，
  整场和三帧导航诊断与v4逐值一致，证据 `docs/motion-v5-original-input-window.json`。
  原42.2秒44个冷候选未被八次迭代等长双段求解认证；几何失败不能证明不可达。
  下一拍新路线较旧剩余路线长3.523601m。ZIP35:65几何见证须经Rust完整Grid/车体/运动/灯前边界重新验证。
- 新 `robot-core/src/navigation/terminal.rs` 提取原单/双段渐变曲率求解，缓存已认证双段的剩余段比与数值初值，
  同目标/朝向时每次重求解并检查当前网格。原冷候选全部失败时，恢复轮按接近已认证首段曲率顺序，采用首个完整认证动作；
  原冷候选有解时保留原评分选择。缓存不是执行凭证，不改变已采用命令状态，不跳过任何安全门。
- 整次导航共享256次域内求解、1024次迭代、65536份预扣采样额度，单次最多8轮；lattice原节点上限另保留，
  终端额度已耗尽则结束不能产生结果的剩余lattice搜索。已有完整认证候选可用，未认证不得因为预算放行。
  `terminal_work.solver_iteration_exhaustions`区分单次迭代用尽；`budget_exhausted`为整拍共享额度不足。
  `primitive_samples`为primitive前预扣额度，可能大于障碍早退时实际执行样本，非CPU操作计数。
  `terminal_budget`表示受整拍额度限制而未继续认证的候选，不推断加额度即可行；`terminal_unreachable`是有限认证失败分类，非数学无解。
- 中间“所有恢复候选仍按旧评分选最好”消除了42.2秒Stop却使LQR退化79.7秒/15.057722m，已撤销。
  最终首认证接续方案：PP58.8秒/11.508841m保持；LQR54.3秒/10.249675m，比v4快16.3秒、少3.190640m。
  LQR灯前26.2秒/4.390629m、5次路线生成、全场无非预期Stop；PP原两次短Stop保留。
  二者终速0，斑马线3000ms、绿灯300ms、最小锥桶间隙PP.242509m/LQR.273528m；无共享终端额度耗尽。
  中间失败与保留策略见 `motion-v5-terminal-experiments.json`，完整结果 `motion-v5-competition-comparison.json`。
- `runner/src/control_execution.rs`异步采用证书由全向圆换为源车体系有向矩形：V/K保留源到规划的实际采用历史上界，
  总弧长S=V*T+V²/(2b)，覆盖原lease再加一拍和完整刹停；横移min(S,KS²/2)、角点旋转min(RKS,2R)，
  KS>=π/2时额外覆盖后向S。地图和灯前半平面查四角，激光TF和障碍圆盘半径保留，相切拒绝。
  采用窗/命令序号/原源期限、正常速度和slew/侧向过渡检查保持；同步导航原紧急停车检查不缩小。
- 0.9m合成直道真实默认Worker：60/80ms源延迟、3/7/9ms采用抖动、23帧连续Drive，默认任务限速.18m/s，
  超过.17m/s并前进>.25m；最后源2200ms按2450ms原期限Stop并制动至0。另直接覆盖.30m/s直道证书和前墙拒绝。
  该宽度不是官方赛道测量。独立.5ms积分覆盖101种采用时刻×4轨迹、历史高曲率/Stop回中、大角度后向运动及TF/旋转边界。
  仍依赖静态障碍、无侧滑/超调和制动能力下界；完整异步比赛/实车/目标板时限未验。
- `runner/src/phase_statistics.rs`追加终端工作总数/每拍最大值与额度拒绝，固定内存/饱和计数，不影响控制和日志策略。
  新 `runner/examples/motion_profile.rs` 显式预热一次+重复1..20次整场simulate主机墙钟，含合成传感器、控制、车辆和sink遥测，
  报告序列化不计入；不是单solver时间、每tick延迟或板卡WCET。基线正式测量恢复原simulation.rs字节，仅加入同一profile示例。
- 本轮说明 [终端连接延续与异步停车包络](docs/终端连接延续与异步停车包络.md)；README模块结构/运动控制和命令TXT同步。
  当前构建、测试、包哈希以 `motion-v5-validation.json`、`motion-v5-build.json`、`motion-v5-delivery.json` 为准；
  v4及之前证据仅保留历史，不能沿用旧二进制哈希。GLIBC2.38仅本地交叉目标，默认PP、LQR实验，Q/R和比赛门限保持。
  暂不接车、不读视频、不执行RISC-V目标，STM32固件任务暂停。工具链/环境/模型/产物不进Git。

### 前轮任务交接、阶段统计与异步运动修正（2026-09-11，v4历史）

- 修改前 fetch 确认 `aac7421` 与 origin/main 一致（0/0）。用户已授权修正第四份运动控制意见，
  继续直接 main 验证、提交、推送；最终同步状态以 Git HEAD/origin/main 核对。
  本轮说明为[任务交接与异步运动修正](docs/任务交接与异步运动修正.md)，证据统一使用 `docs/motion-v4-*`。
- `PassThrough.admission_radius_m` 由 Mission 传真实 `goal_tolerance_m`，修复 .065 m 任务验收与
  .045 m 导航停车容差混用导致的提前减速过晚。新增半径/距入圈余量诊断；从较早帧连续减速，
  已来不及满足正常减速约束仍 Stop，不能通过放宽 epsilon 修复。原 LQR 22.5 s 空速度区间已核验为真实遗漏。
- 缓存参考偏离采用完整车体对应四角的最大位移，同时纳入位置和航向；不只查中心横向偏差。
  保留原目标容差一半的重规划数值门限和原 Q/R。对必停定向目标，若已有单弧求解器未认证而双弧可认证，
  新候选在原安全门通过后还需验证完整下一周期仍有短连接/严格入圈；这是有界连接族检查，不是全局可达证明。
  每拍最多165次状态查询，每次最多单/双弧各8迭代，有短Vec分配，不重跑lattice；目标板时限未验。
  既有变动点云测试的拍末值Euler模型改为独立1ms连续积分；关闭新guard仍复现原失败，原场景/停车/碰撞门限保留。
- 最终完整合成比赛 PP58.8秒/11.508841m、LQR70.6秒/13.440314m均完成，终速0；
  斑马线3000ms、绿灯确认300ms。对比aac7421，PP慢3.2秒、多走.38345m，LQR快13.3秒、少走2.34996m。
  PP最小锥桶间隙.24251m、LQR.27353m；变化不全是改善。PP两次短Stop保留；LQR原交接空区间已消除，
  另在42.2s有一次可恢复终端门控Stop。原始阶段/候选数据见本轮competition-comparison，不能把候选拒绝数写成停车次数。
- `motion_transition::project_motion` 提供无堆分配、有时域/数值预算的分段车辆模型投影，
  区分速度/转向渐变与饱和点，解析积分航向和距离；独立精细积分回归不冒充真实反馈。
- 导航新增当前、选中候选和下一完整周期的停车余量/来源诊断。保留原硬停车包络与候选评分。
  曾尝试按未来余量优先排序，但 PP 在 19 s 出现所有候选都切同一占用格角的回归，已撤销该排序。
  不是新扫描让当前格变为占用；旧网格独立复算同样拒绝。详见 `docs/motion-v4-stopping-experiment.json`。
- 新 `runner/src/phase_statistics.rs` 将阶段实际时长/距离、路径生成/长度、非预期Stop次数/命令时长和
  固定导航/候选原因累计进 `SimulationSummary.statistics`；每阶段仅留最近16次路线事件与完整计数。
  最后制动单列 `final_braking` 并与总时长/距离对账。无新增逐tick落盘，不降控制/感知频率。
- 新 `runner/src/control_execution.rs` 固定128条已采用命令变化历史；同命令poll不耗槽位，
  源时刻早于最近poll仍可查历史，历史不足返回 `HistoryUnavailable`。分段推算位姿、速度及曲率到规划时刻。
  `PlanningContext`/`tick_with_projection` 保留源测量用于任务、Safety和世界障碍投影，导航另用模型规划状态。
  后台证书约束采用时间窗、真实采用前提、原源期限、正常运动限幅、侧向峰值和静态停车空间；
  `poll` 不重做搜索/不等待锁。同源不续租、原期限先验、晚到Drive不能清Fault等约定保持。
  poll还按真实命令变化时间检查曲率slew，不能借用更长规划间隔。原始源期限受Mission/Nav/Safety最短限制。
  投影/证书共享原导航±1e-6测量边界，归一化仅用于模型副本；真实越界仍InvalidInput，命令上限仍严格。
  默认spawn已有空旷直路异步集成：10Hz采样、60/80ms延迟、50Hz输出，13份源快照Drive后按原期限Stop并减速到0；
  另有固定60ms源延迟+3~9ms采用抖动、窗口过期/前提变化等专项，不把冻结模拟时钟的线程测试写成目标耗时测量。
  证书有可能保守拒绝狭窄通道；未验证完整异步比赛，真实多传感器时钟、动态障碍与执行模型仍待车到后验证。
- `README.md` 与 `Mac与车端命令手册.txt` 同步模块和验证入口；新构建/包以
  `motion-v4-validation.json`、`motion-v4-build.json`、`motion-v4-delivery.json` 为准。
  v3/v2和更早数据保留历史，不能沿用其二进制哈希或测试数量当本轮实测。
- 当前仍暂不接车、不读视频、不执行RISC-V目标，GLIBC2.38仅本地基线。PP默认，LQR实验。
  工具链/环境/模型/产物不入Git；原比赛PDF不修改或暂存，STM32读取任务继续暂停。

### 前轮运动执行状态、过渡峰值、通过点与模拟积分修正（2026-09-10，v3历史）

- 修改前 fetch 已确认 `9b59125` 与 origin/main 一致；继续按用户授权直接 main 开发/验证/提交/推送。
  本轮已完成下述主机验证与交叉链接，提交/推送状态以 Git HEAD/origin/main 核对；不沿用 v2 的数字或哈希。
  该轮说明为[运动执行与通过点修正](docs/运动执行与通过点修正.md)，历史证据使用 `docs/motion-v3-*`。
- `robot-core/src/navigation.rs` 新增 `SteeringEstimate { at, commanded_curvature_per_m, applied_curvature_per_m }`。
  估计只按已采用命令推进，`advance_to` 沿旧目标推进，`adopt` 先推进再换目标；Stop 归零目标但估计逐步回中。
  即使字段各自有限，其差值溢出也会原子拒绝，失败不修改原估计。
  `plan_with_arrival` / `plan_stop` 不采用规划结果；旧同步 step/stop 包装采用其返回命令。
  `runner/src/autonomy.rs` 同步 `tick` 在最终 Safety 后采用命令，异步 `tick_with_execution_state` 只规划、不自行采用。
- `runner/src/control_runtime.rs` 的真实 Worker 使用上述异步接口。只有输出线程 `poll` 最终的 `ControlPoll.command`
  推进执行估计，固定大小原子槽回送给提交方；未采用、被替换或晚到的计划不会更新执行历史。
  `try_submit` 把提交时已知状态绑定到快照，只有估计时刻不晚于源时刻才能向前推进；否则返回 `ExecutionAhead` 等待新源。
  不倒推未来状态、不阻塞 poll、不续租旧命令；同源 Duplicate 和故障锁存保持。
  同时刻先提交后采用的新命令不会追溯更新排队副本，持续滞后实车源仍需共同时间轴/有界历史关联；这不是测得的转向反馈。
  专项 `control_runtime` 14 项及对应 clippy 已通过，包含在下述本轮完整验证范围中。
- 新 Rust `robot-core/src/motion_transition.rs` 的 `MotionTransition` / `lateral_acceleration_peak` 解析检查
  速度和曲率独立渐变时 `v²|曲率|` 的端点、分界及内部峰值，固定计算量、不分配堆内存。
  导航检查至少覆盖完整控制周期和配置预瞄时域，不因接近目标缩短几何预测而漏检；无效数值/超限拒绝候选。
- `ArrivalBehavior::Stop/PassThrough` 区分必停目标与锥桶通过点。续段从第一段实际末端位置/朝向/曲率连接并检查，
  只为速度制动上限提供经过验证的余量；第一段仍单独跟踪、独立按原容差验收，前视和进度不能跳到续段跳过必经点。
  `next_max_speed_mps` 提供下一阶段上限，进入其验收半径前提前限速，避免阶段切换后的速度上限突变。
  它约束候选目标，不承诺入区瞬间测量速度必定达标；外部超速/测量偏差仍可触发空速度区间 Stop。
  续段不可行则按停车处理。锥桶阶段就约束灯前续段的禁越线，停稳/绿灯/普通停车期限等规则门限未放宽。
- `runner/src/simulation.rs` 使用不超过 1 ms 的子步、速度/曲率渐变分界、包含交叉项的航向积分与位置数值积分，
  每个子步检查车体/障碍/边界，故障后制动沿用同一逻辑。控制/感知周期不降低，也不新增逐子步磁盘日志。
- 新积分暴露 PP 灯前回归，额外修正两个已复现问题：带目标朝向时，缓存参考横向偏离超过 `goal_tolerance_m × 0.5`
  便提前清路重规划，不能只因缓存无碰撞就假定末端仍可达；oriented rollout 整段评分改为对应四个车身角世界位置差的
  最大值，使用纯米制车体位姿误差，避免提前回正后仍残留横向误差。这改变了位置与朝向的相对影响；
  未调整的是 LQR q/R 参数及比赛门限，不能声称评分权重完全未变。恒候选曲率预览不代表所有未来策略。
- 本轮最终复验 PP 55.6 秒、LQR 83.9 秒均完成，终态速度均为 0；最小锥桶间隙分别为
  0.3032708854919852 m / 0.2735281037908029 m，两者斑马线停稳 3000 ms、绿灯确认 300 ms。
  首次可恢复阻塞分别为 18600 ms / 22500 ms，均保存 17 帧。完整结果见 `docs/motion-v3-competition-comparison.json`，
  中途 PP 失败保存在 `docs/motion-v3-before.json`，不宣称实现期间始终通过；控制周期、容差及时限未放宽。
  测试/构建见 `docs/motion-v3-validation.json`、`motion-v3-build.json`；本地包状态和哈希以 `motion-v3-delivery.json` 为准。
  **PP 默认不变、LQR 仍为实验项。** 暂不接车、不读视频、不执行 RISC-V 目标或真实设备，GLIBC 2.38 仅本地交叉基线。
  本地包、模型、编译链及环境不进 Git；用户规则 PDF 不修改/不暂存，STM32 任务继续暂停。

### 前轮导航预测、灯前约束与诊断修正（2026-09-10，v2 历史）

- 已读取用户[第二份修改意见](https://chatgpt.com/share/6aa27143-7204-83ee-ad9a-7067ef49a1bc)，按源码和独立复现核验。
  修改前 fetch 确认 `8aceeaf` 与 origin/main 一致（0/0）；用户已授权本轮直接 main 提交/上传。
  本轮按既有规范完成检查后直接提交 main；后续接手以 Git HEAD/origin/main 核对同步状态。
  规则 PDF 保持原件，不修改、不暂存；公开分享及完整原始 trace 仅留 `work/`。
- `robot-core/src/navigation.rs` 将正常候选预测与紧急停车范围分开：正常速度按加减速率、曲率按变化率渐变，
  从当前测量速度和上次指令曲率推进；停车可达圆盘仍用 `max(当前速度, 目标速度)`，覆盖命令周期、反应及制动。
  较低目标速度或较短剩余路程不能缩小当前速度所需的保守停车范围，真实制动能力仍待标定。
- 终端连接从父节点曲率实际积分，先尝试单段渐变曲率，再用有界双段 S 形连接补足带朝向的短距离到达。
  两段交界继承实际末端曲率，按实际端点/朝向/碰撞验收；不再把末点直接替换成目标，不允许原地旋转补朝向。
- `robot-core/src/reference.rs` 在每次导航调用中一次验证并准备局部参考窗口，候选复用；弧长 cursor 顺序推进，
  不为每个预测步重扫整个路径。带显式目标朝向时，在原 rollout 的每一步累计参考位置/航向误差，
  曲率偏好按 `0.5 × (v × dt)^2` 缩放，避免固定偏好压过末端误差；无目标朝向时保留追踪点距离评分，
  曲率偏好按剩余距离与前视距离之比的平方淡出。**PP 仍为默认，LQR 权重未改且仍属实验项。**
- `robot-core/src/autonomy.rs` 新增 `HalfPlane`，与 `mission.rs` 共用灯前停止线几何。
  `runner/src/autonomy.rs` 在进入 `ApproachLight` 的同一拍向导航设置边界，全局路径、局部预测、停车可达圆盘及
  `Reached` 都受约束；保持原绿灯新鲜帧/停稳/去抖门限，任务进入 `Finish` 才解除。比赛容差、时限和安全门限未放宽。
- `TrackingDiagnostics` / `NavigationDiagnostics` 提供实际跟踪模式、LQR 已算误差、路径版本/进度、请求/限速/选中曲率、
  预测误差及候选拒绝计数。`runner/src/navigation_diagnostics.rs` 在 simulation 记录首次非预期阻塞/故障前 8 帧、
  触发帧和后 8 帧，最多 17 帧，只保留紧凑内存上下文，结束进入摘要 `first_navigation_failure`。
  正常任务停车/目标制动不触发，首次异常可能恢复，不能自动当成最终故障根因。无每 tick 落盘，不降比赛控制或感知频率。
- 该轮实际结果：导航集成 11 项通过；同一完整合成比赛 PP 在 55,900 ms、LQR 在 53,300 ms 完成，二者终态速度均为 0。
  这不代表实车验收或 LQR 普遍优于 PP。修改前 LQR 在 17,400 ms 首次阻塞、27,900 ms 停车超时的复现保存在
  `docs/motion-v2-before.json`；前轮 `motion-control-*` 的通过项和失败项继续保留为历史。
- 历史说明见[导航预测与失败首因修正](docs/导航预测与失败首因修正.md)。该轮全量测试、fmt/clippy 与两程序 RISC-V 交叉链接已通过，
  最终测试数量、ELF/源码哈希、包及验收结果分别以 `docs/motion-v2-validation.json`、`docs/motion-v2-build.json`、
  `docs/motion-v2-delivery.json` 为准；这些 v2 产物不是本轮 v3 二进制。
- 当前仍暂不接车，不读视频，不执行真实设备或 RISC-V 目标；GLIBC 2.38 仅为本地交叉链接基线。
  低速切换滞回、规划器原生参考曲率采样、共同时间轴/扫描去畸变、真实速度/转向反馈及物理 MotionSink 仍待后续。
  本轮新增折线投影与弧长 cursor 不等于已经保留规划器的原始连续曲率。

### 前轮运动控制评估与实现（2026-09-09，历史）

- 用户要求读取分享链接、评估运动控制并同步README。已成功获取并提取[《算法比较建议》正文](https://chatgpt.com/share/6aa0bf59-c8f8-83ee-b001-13dd5945ce14)，
  先前网络超时不再是阻塞。公开页面原始数据仅留`work/`，没有发布整个分享HTML。建议按当前源码独立核验。
- 修改前fetch确认`7fb06c8`与origin/main一致。继续直接main提交/推送，规则PDF不修改、不暂存。
- `robot-core/src/tracking.rs`新增`PathTracker`、`PurePursuitTracker`及实验`LqrTracker`。
  `NavigationConfig.tracking`省略时仍PP；serde拒绝未知算法和多余字段。PP前视链/公式保持，导航既有1e-6m/s测量容差传入跟踪前统一归零/夹限。
  LQR使用原缓存路径局部投影、三点曲率前馈及横向/航向反馈，闭式连续直线附近模型，无新依赖/轴距猜测/设备I/O。
  低速PP回退，线性化误差域或输入不合法则导航Stop；后续候选、碰撞、停车与Safety通路共用。
- `robot-core/examples/tracking_comparison.rs`：4条路径×PP/LQR×0/200ms假定延迟，共16组进入终点容差；
  当前LQR权重下横向RMS反而更高，不宣称优于PP，模型未含定位噪声/制动，模拟时间不代表CPU时延。
- `runner/examples/motion_comparison.rs`：同一整场比赛PP仍55.3s完成；LQR在绕锥桶阶段停住，27.9s普通停车超时，最终模拟速度0。
  **该历史版本 LQR 未通过全场验收。** 2026-09-10 的修正与新结果见上节；保留该失败证据，默认算法仍不替换。
- README新增运动控制模块/单位/通路/实测边界，TXT增加两个Rust A/B命令；完整说明见[运动控制设计与对比](docs/运动控制设计与对比.md)。
  当前没有ESKF/EKF、速度PI、真实转向反馈或电机MotionSink；先解决时间同步、去畸变、标定及反馈质量，再扩展这些模块。
- 额外审查未找到证据充分的新制动/看门狗/PWM缺陷。侧向加速度检查约束候选目标，Stop记录的是归零目标；
  实际过渡/回中待测，保守停车圆盘考虑中间转向但依赖实际制动能力满足配置。

### 前轮文档补充（2026-09-08）

- README新增“整车与硬件详细参数”：产品介绍第1页全部12项、规则初稿第10页及出厂源码通信/相机配置已对照。
  包含尺寸、CPU/GPU/内存/eMMC/供电、MCU、电调/BEC、电池、舵机、雷达/相机/IMU和零碎接口参数。
- 型号证据分层：出厂驱动选择LSlidar N10，实物待核对；相机/IMU/舵机完整型号未提供，不把平台CMP10A等参考外设认作本车。
  电池5400mAh（产品）/5300mAh（规则）、底盘阿克曼/四轮差速表述、MCU32KB/4KB/40MHz及“最大转速40km/h”均单列来源/疑点。
- `car_controller_new.cpp`的L=0.305、lfw=0.1675、controller_freq=30只记录为源码默认参考，不代替测量。
  该轮仅改README和交接文档，未改变代码/配置。旧0f3ab64、motion-control及motion-v2构建证据保留历史，该轮证据使用motion-v3前缀。

### 当前代码：5 个 crate、2 个程序

| 模块 | 已实现 |
|---|---|
| `crates/vision` | 纯 Rust 模型契约、RGB/letterbox/NCHW、阈值与坐标解码；road.rs 斑马线/灯色/带色锥桶及地面投影，ground_markers.rs 显式实验地面标记 |
| `crates/app` → `xt-stcar` | self-check/preprocess/replay/infer；原生 ORT C API 动态加载、模型来源及元数据验证、常驻 Session；Python 参考后端须显式选择；file_io 提供两套 CLI 共用文件边界 |
| `crates/robot-core` | 强类型传感器语义、frame/时间/单位校验、急停/deadman/超时/限值状态机、仅记录的 MotionSink；底盘/WIT IMU/N10 协议与标定表；固定及在线任务、LocalWorld有界身份/几何/面积证明、Stop/PassThrough、灯前半平面、整圈扫描、ICP、执行曲率估计、解析过渡峰值、车模型导航/局部参考/PP与实验LQR |
| `crates/device-io` | 安全 rustix 串口配置、独占、8N1、关闭软硬件流控、读回检查、nonblocking poll 与整体包截止时间、故障锁存、Drop 尝试恢复 |
| `crates/runner` → `xt-stcar-robot` | 严格 JSONL 回放、真实图像推理、原始传感器解析、可选标定 PWM 预览及传感器采集；自主模拟/快照回放、背景感知/规划/独立看门狗及采用状态回传、连续渐变子步车辆模型、有界内存日志/首次异常窗口与结束原子提交 |

#### 前轮比赛扩展（2026-09-08）

本次修改前 fetch 确认 `d245174` 与 origin/main 一致；用户规则 PDF 未修改/未暂存。
完整说明：[Rust比赛自主闭环](docs/Rust比赛自主闭环.md)，日常命令：[Mac与车端命令手册.txt](Mac与车端命令手册.txt)。

- `robot-core/autonomy.rs`（均在 `src/` 下）提供米制坐标、足迹、位姿质量、道路观察共享结构；
  `mission.rs` 实现 Idle→ApproachCrosswalk→CrosswalkStop→Cones→ApproachLight→WaitGreen→Finish→Completed/Fault。
  连续新鲜反馈确认停稳才累计3秒；灯前全部足迹/朝向/停止与绿灯新帧去抖；终点全车进区；普通停>10秒失败。
  正常比赛 Stop 与锁存安全 Fault 分开，移动/未知灯色/旧帧不能偷偷完成任务。
- `vision/src/road.rs`：Rust HSV/连通域、白条几何、红蓝锥桶脚点、ROI 灯色和地面投影。
  当前80类 YOLO不含斑马线/锥桶/灯色，`zebra`仍为动物；不更换原权重，YOLO9类只辅助圈定交通灯。
- `robot-core/src/scan.rs` 整圈组帧与覆盖/连续盲区检查；`localization.rs` 有界 ICP，无编码器/全局SLAM声明。
  `runner/src/laser_pose.rs` 检查整圈后按显式TF生成车体点云，输出保留扫描源时戳；掉圈超时需显式已知位姿reset。
- `robot-core/src/navigation.rs` 用膨胀网格连通检查、带转弯和转向速率约束的搜索、跟踪及保守停车可达圆盘检查，覆盖转向回中期间的中间曲率。
  修复全局车模型允许穿网格角而局部正确拒绝导致反复停住的问题；全局/局部均拒绝切障碍角。
  灯前和终点带目标朝向，不允许靠原地旋转伪装阿克曼调头。搜索预算有界但不保证目标CPU最坏时延。
- `runner/src/autonomy.rs` 串联同步Pose/Scan/Road→Mission→Navigation→Safety。
  扫描与用于其世界投影的位姿必须同采集时刻，斑马线锁定/视觉锥桶投影同样要求图像与位姿采集时戳一致；
  不能用freshness/skew代替关联。重复帧不能改内容；故障锁存，输出仍只有检查后的值/记录。
- `runner/src/perception.rs` 常驻原生ORT和同图RGB复用，PerceptionWorker只保留最新待处理图像；
  `control_runtime.rs` AutonomyWorker将规划放后台，独立周期调用poll获取命令。
  lease由源快照/最旧传感器时刻决定，先检查旧期限后接纳结果；晚到Drive不能恢复Fault。
  必须使用ControlPoll.command，latest仅诊断，不能旁路；Drop不无限join卡住的原生线程，也不能无限重建线程。
- `runner/src/simulation.rs` 真实执行合成RGB识别、360束测距、任务/导航与有加减速运动学反馈；
  不是手写Motion回放。模拟Pose明确为理想反馈，不冒充ICP实测。运行中视觉错误进入Fault并保存终态；
  Fault/超时后继续模拟制动直至速度0并检查碰撞，不从中途Err直接绕过停车分支；Stop目标速度/曲率均0，按速率回中。
- `runner/src/autonomy_replay.rs` 单调SensorSnapshot{at,pose,scan,road}诊断输入，拒绝Motion与旧event格式；EOF强制Stop。
  `telemetry.rs` 默认transitions，结束才落盘；trace默认1MiB、上限8MiB、重要事件独立保留，满额只丢调试记录，
  logging_errors/dropped_trace_records可查，不让可选日志影响控制。默认不存逐帧图像/点云/张量。
- 新CLI：autonomy-example、autonomy-sim、autonomy-replay、road-detect。
  新配置：competition-sim、competition-controller-sim、road-perception-sim、scan-assembly-sim、laser-localization-sim（均config/*.json）。
  最后两份供库接口，不能当成已有设备CLI参数。全部为合成值，禁止把simulation_only/unverified改标签就当实车已标定。

#### 已完成的 Rust 模块（前轮扩展）

1. **N10 Rust 解析**：`robot-core/src/protocol/n10.rs`。依据出厂 `.cc`，固定 58 字节、16 槽、
   大端角度/距离、uint8 强度、前 57 字节累加模 256。支持分包、粘包、坏帧重同步、有限缓存与过期保护。
   保留无效槽位，修正厂商按有效点数计算插值分母的角度偏移。
   `packet_sample()` 只生成 **16 束局部样本**，全未知或零跨度返回 None。
   局部包新鲜不代表全圈覆盖或前方无障碍。当前新 `scan.rs` 另行整圈拼接，旧replay仍为局部包；没有 ROS 发布。
2. `replay --n10-config` 接入 `n10_bytes`，禁止与直接 lidar 输入混用；frame 必须一致。
   没有有效样本只推进 Tick；保存首字节接收时间，坏帧不刷新传感器时间。
   新合成样例实测 t80 触发 lidar 超时 Fault/Stop，11 运动记录、3 Drive/8 Stop、1 包/1 样本。
3. **物理单位标定预览**：`protocol/calibrated_chassis.rs`，显式速度/曲率分段表，严格单调、
   零锚点、PWM 包络、显式倒车策略、禁止外推。当前 schema 只接受 `simulation_only=true` 和 `unverified`。
   `replay --chassis-calibration` 先映射再记录运动；映射失败记录锁存急停、`chassis_error` 和 Stop，CLI 非零退出。
   示例 `.1 m/s、.2 m⁻¹ → 1530/1540 µs` 是合成表结果，不是实车参数。没有电机串口发送入口。
4. **串口采集**：`serial-capture --config ... --output ...` 默认只输出计划，不打开设备、不写文件。
   显式 `--execute` 才按配置读取 IMU/N10，不调用写方法。时长≤60s、字节≤512KiB、读取记录≤10000，
   统一 Instant 纪元给 raw/idle/末尾 Tick 打戳；达到上限正常结束并在摘要写 `ended_by`。
   捕获失败保留旧日志。不同采集文件不是同一时钟，不能直接拼成同步传感器流。
5. 串口显式清 `IXON/IXOFF/IXANY`（仅 make_raw 在 Linux 不足），检查 VMIN/VTIME，PTY 验证原始配置恢复。
   `device-io::write_packet_until` 为整个包共用截止时间并锁存错误；通用 `PacketWriter<Write>` 自身没有截止时间。
6. 修正 IMU 多样本批次日志重复：完整 `imu_decode` 每块只写一次，后续 step 用 `imu_sample_index`。
   N10 同样只写一次 `n10_decode`，后续 step 用 `n10_packet_index`。

7. 总调度静态文件非阻塞打开后 fstat 校验普通文件，拒绝 FIFO 阻塞；JSON 配置实际读取≤1MiB、
   事件≤64MiB。图像尺寸检查与解码复用同一已验证句柄，保留解码内存/像素限制。

使用说明：[N10 协议依据](docs/N10协议依据.md)、[底盘标定](docs/底盘标定映射.md)、
[Rust 串口采集](docs/Rust串口采集.md)、[原有底盘/IMU 协议](docs/厂商协议Rust适配.md)。

#### 前轮总体审查修复

审查开始前 fetch 确认 `6c7de79` 与 origin/main 一致、工作区干净；用户随后放入规则 PDF，保留原件但不暂存。
完整结论、回归证据与当前产物见 [总体架构审查](docs/总体架构审查-2026-09-08.md)。

- `app/src/file_io.rs` 统一静态输入非阻塞打开/fstat/实际读取量；runner 重新导出旧 API，删除重复实现。
  app 图像尺寸与解码复用同一文件句柄，配置/输出张量/provenance ≤1MiB、ONNX ≤64MiB；Python 独立工具同样有界。
- 视觉输出和 Python worker 拒绝覆盖 FIFO 等非普通目标；预处理两个输出先共同预检。
  显式/PATH 中的 Python 解释器纳入输入保护，保留虚拟环境符号链接；PATH 未设置时裸命令须改用显式路径。
- 雷达派生角跨度/末束角度也须有限；溢出进入锁存 Fault/Stop，不被后续正常样本清除。
- ELF 版本核验绑定动态加载映射，检查所有 verneed 要求，新增 `version_requirements`；不能用 section 表伪装低 GLIBC。
  ELF 输入非阻塞且实际 ≤64MiB；报告拒绝输入别名和非普通目标，通过后原子保存，失败保留原报告。
- 交付测试取消对未跟踪 `tmp/` 目录的依赖，新克隆未构建时正确 skip。

crate 分层无环；最新扩展增加 vision→robot-core 的纯类型依赖。历史审查不覆盖全部新模块，新模块证据单列。

### 构建、模型与验证证据

- 历史v9：371 Rust/2跟踪example/5矩阵example/35 Python交付通过，2项按设计忽略；fmt/clippy/双交叉链接与ELF通过。
  108份编译输入、GLIBC2.38基线/最高实际2.34。该轮结果与本地包为 `motion-v9-validation/build/delivery`；当前见开头在线元素驱动快照。
  8组时序性能门全部通过；原生模型推理和目标执行未验证，以下旧轮记录仅为历史。

- 历史 v7：Rust常规350通过/0失败/2忽略，跟踪example2项、Python交付35项通过；fmt、all-targets clippy -D warnings、双RISC-V交叉链接通过。
  98份源码/清单/嵌入fixture记录于 `docs/motion-v7-build.json`；基线GLIBC2.38、最高实际引用2.34。
  app SHA256 `ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9`，1,217,056字节，需libc；
  robot SHA256 `3b30c966de36d4a28c8054794dbf1119cc92bda7930e3c7de5ae805033a5f673`，2,020,984字节，需libm/libc。
  本轮精确功能/性能范围与本地包分别见 `motion-v7-validation.json`、`motion-v7-delivery.json`；原生ORT和模型执行未重跑。
  正式顺序同步profile：PP平均1609.081→1621.487ms（+0.77%），LQR1494.640→1484.827ms（−0.66%），各预热1次/测3次；不作目标WCET结论。
  默认完整异步导航阶段均值/峰值 PP4.557/11.875ms、LQR4.809/20.414ms；含非Drive计划，开启计时后功能字段逐值一致。
  持续Instant短场23源/119转弯Drive、poll最大26ms；末源2201、原期限2451、2460 Stop、2687完全停稳回中，无碰撞越界。

- 历史v6：Rust常规322通过/0失败/2忽略（原生ORT与独立证书性能样例未重跑），跟踪example2项、交付34项通过；
  fmt、全目标clippy -D warnings、两个RISC-V交叉链接通过，89份Rust源码/清单哈希核对。
  构建、完整同步/异步成功与失败及主机时钟观察见 `docs/motion-v6-validation.json`；
  新增预算预留和可选内存诊断，默认PP保持。实际ELF、全部Rust源码哈希与包核验分别见v6 build/elf/delivery，
  完整异步LQR尚未通过，不能将同步或构建通过写成实车可参赛。

- 历史v5：Rust常规305通过、0失败，默认忽略原生ORT与显式性能基准各1项，性能基准另跑1项通过；
  跟踪example 2项、Python交付34项通过，fmt/all-targets clippy -D warnings/双RISC-V交叉链接通过，82份源码哈希核对。
  app哈希 `ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9`；
  robot哈希 `f286246369cd068271ba6bd4c987b12a40aecade7c458ebda9c62f965c2f8b9f`，1,975,968字节。
  GLIBC基线2.38、实际最高引用2.34，动态依赖仍为app/libc，robot/libm+libc。
  主机release整场预热1次再各3次：PP平均1.621→1.614s；LQR平均1.921→1.496s（任务变短，非每tick加速证明）。
  证书360/2048点各2000样本，平均8.862/19.234µs、峰59.791/56.875µs；默认Worker23帧平均2.626ms、峰3.790ms。
  都是本机采样，不是目标板WCET；资料入口为motion-v5-validation/host-profile/async-validation/build/delivery。

- 历史 v4：Rust 常规 294 通过、0 失败，原生 ORT opt-in 1 项忽略；跟踪 example 2 项、Python 交付 34 项通过。
  fmt、all-targets clippy -D warnings 和两个 RISC-V 交叉链接通过。该历史阶段入口为 `docs/motion-v4-validation.json`；本轮见开头在线元素驱动快照。
  两者实际最高 GLIBC 引用 2.34，构建基线 2.38；robot 需要 libm 与 libc，xt-stcar 需要 libc。
  构建源码哈希已核对，该轮二进制哈希见 `motion-v4-build.json` 和两份 ELF 报告。
  模型测试套件和原生 ORT 推理未重跑；打包另做模型格式、来源与包内文件校验，包状态见 `motion-v4-delivery.json`。
  本地 core 包 54 个文件、含模型包 58 个文件均已核验；包内文档、两个 ELF 及 80 份源码哈希与最终文件一致。
  两包仅在本地 dist，未上传或执行车辆；Git 仅同步源码、脚本、文档和验证记录。

- 前轮 v3 运动执行修正（历史）：Rust 常规 266 通过、0 失败、原生 ORT opt-in 1 项忽略；跟踪 example 2 项、Python 交付 34 项通过。
  fmt、all-targets clippy -D warnings、两个 RISC-V 交叉链接及 PP/LQR 完整场景复验通过。
  该轮汇总入口为 `docs/motion-v3-validation.json`，构建与交付为 `docs/motion-v3-build.json`、`docs/motion-v3-delivery.json`。
  交叉链接通过不代表本地包已核验；包的状态、路径和哈希以 delivery 报告为准。不把以下历史证据算入本轮。
- 前轮 v2 运动修正（2026-09-10，历史）：Rust常规237通过、跟踪example测试2通过、交付测试34通过；
  fmt、all-targets clippy -D warnings、两个RISC-V交叉链接通过；导航原11项及PP/LQR全场通过。
  历史汇总入口为 `docs/motion-v2-validation.json`，构建与交付分别见 `docs/motion-v2-build.json`、
  `docs/motion-v2-delivery.json`；不把该轮数字计入本轮。
  历史robot大小1,918,960字节，SHA256 `f5652cb05757ea135eaff714be5f5af4e2770151118c50197534356942bd6863`；
  历史xt-stcar SHA为`ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9`，当时二者最高GLIBC引用2.34。
  该轮PP首个可恢复阻塞在14300ms、窗口17帧，最终仍完成；LQR没有触发首因窗口。模型/原生推理未重跑。
- 前轮运动扩展（2026-09-09，历史）：Rust常规209通过、跟踪example测试2通过、Python交付34通过，fmt/all-targets clippy及两个RISC-V交叉链接通过。
  `docs/motion-control-validation.json`汇总，配套build/ELF/log/两份A/B JSON均在`docs/motion-control-*`。
  该轮模型和原生推理代码未改，未重跑模型/ORT执行测试；这些通过项不能计入本轮实测。
  历史robot大小1,884,168字节，SHA256 `902b350111e26b1767d343c5ec43a450fb16fe8f5c86e2603a53c5016feb6704`；
  历史xt-stcar SHA为`ec385c9e19f1e09d297cb9e86d33bb1df387081b31397a2d0ae13535d7cfdcd9`，当时二者最高GLIBC引用2.34。
  历史core/模型包大小/哈希/路径与核验见`docs/motion-control-delivery.json`；不把它们当成本轮产物，编译链/环境不进包或Git。
- 前轮比赛扩展（历史）：Rust常规191通过、原生ORT opt-in 1通过、Python模型48/交付34通过；fmt/all-targets clippy通过。
  两程序RISC-V交叉链接通过。最终合成场景55.3s/554tick、最小锥桶间隙0.301m、斑马线3000ms/绿灯300ms、终点速度0。
  默认日志24,375字节、无丢弃/错误；这些是合成结果，不是实车性能。
  详见 `docs/competition-validation.json`、`competition-simulation-summary.json` 及对应日志。

- 本机 Rust/Cargo **1.97.1**，rustfmt/clippy/RISC-V std 已安装；`rust-toolchain.toml` 和 `Cargo.lock` 锁定，
  不改全局默认。Zig **0.15.2**、cargo-zigbuild **0.23.4** 在项目 `toolchains/`，无需重复安装。
- 新串口依赖 rustix **1.1.4**；两个目标程序由 `scripts/build-riscv.sh --offline` 检查并构建。
  目标 `riscv64gc-unknown-linux-gnu.2.38`，ELF64 LE RISC-V / RVC / LP64D / PIE，
  加载器 `/lib/ld-linux-riscv64-lp64d.so.1`。v2最高 GLIBC 引用 2.34；当前实际引用和依赖以本轮 online-mission 最终构建/ELF报告为准，不复用历史哈希。
  前轮 robot 程序额外需要 `libm.so.6`，不能声称两个程序都只依赖 libc。
- **当前构建证据见本轮快照；`docs/motion-v9-*`、`docs/motion-v8-*`、`docs/motion-v7-*`、`docs/motion-v6-*`、`docs/motion-v5-*`、`docs/motion-v4-*`、`docs/motion-v3-*`、`docs/motion-v2-*`、`docs/motion-control-*`、[Rust比赛自主闭环](docs/Rust比赛自主闭环.md) 与 `docs/competition-*`保留历史证据。**
  `docs/architecture-*` 及 [总体架构审查](docs/总体架构审查-2026-09-08.md) 保存前轮扩展前的构建、测试和包，不能当当前版本。
  `rust-expansion-*`、旧 2026-09-07、`factory-*`、`glibc-*` 记录保留为历史，不是当前二进制。
- 主程序在 `target/riscv64gc-unknown-linux-gnu/release/`，新 core/模型包在 `dist/`；
  打包白名单已包含 N10、标定、串口、5份比赛配置、比赛说明及TXT命令手册。Git 不提交这些产物。
  上传脚本默认 dry-run，只允许明确的新车账号/IP/独立 release 目录；本阶段不执行车端上传。
- 模型栈位于独立 `.venv-model/`，基础 `.venv/` 保留。Ultralytics **8.4.142**、Torch 2.14.0、
  torchvision 0.29.0、ONNX 1.22.0、ORT 1.29.0、OpenCV 4.14.0.94、NumPy 2.5.3 已在先前验证。
  50 项依赖锁在 `requirements-model-macos.lock.txt`，不要混装两个环境。
- 官方 `models/yolo26n.pt` SHA256：`9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef`。
  `models/yolo26n.onnx` SHA256：`c52d204571c6df9f1132dedd7aab3e87336434589b055b9fa4026d117f1d4045`。
  同 stem provenance 必须匹配；445 节点、0 NMS，静态 FP32 `[1,3,320,320] → [1,300,6]`。
- Mac 原生 ORT 在 `toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib`，
  SHA `8ab8982e8fc0a3d5121bf95404dba7a15d70b7df49a8bab1ba481ff961d93dc8`，仅 Mac 使用，不进目标包。
  `ort=2.0.0-rc.13` 选择 std/load-dynamic/api-22，不在交叉构建时下载目标原生库。
- 此前真实 Mac 模型推理已通过：bus.jpg 5 框（4 person/1 bus）、Rust/Python 同张量结果一致；
  26 个预处理对照最大通道差 1 灰度级，机器人 19 事件/2 图像帧的混合回放通过。
  这些历史证据在 [2026-09-07 验证](docs/验证记录-2026-09-07.md)，不能当作当前 RISC-V 实测。

### 官方资料与协议依据

- 根目录用户提供的《赛项三-RISC-V_轻量无人车赛比赛规则（初稿）》已完整读取 10 页并逐页查看渲染。
  封面日期 2026 年 8 月，SHA256 `24dfa87d60142be349eb4e7e862e09b1ec0c4e3771010e9471d2c850144eb7d6`。
  [规则与工程差距](docs/赛项三规则与工程差距.md) 按页列要求与歧义；PDF 原件留本机，未读视频/网盘/提交材料。
  Rust 未禁、ROS2 未明文强制；ROS1 与 bag 工具明确禁用，范围需最终规则澄清，不引入它们作为比赛依赖。
  斑马线前停足3秒、按图绕两锥桶、绿灯才放行且四轮在灯前指定区域停车、终点四轮入区；技术案例占20分。
- 已完整读取 Bianbu 案例 1–15 的文字正文及关键源码，见 [案例核对](docs/Bianbu案例1-15核对与Rust兼容性.md)。
  它们是 Muse Pi Pro 平台参考；TOF/云台/GPIO 舵机/EtherCAT 案例不是本车底盘协议。
- 整车资料关键来源 `4.出厂源码/racecar.zip`，4990 成员，SHA256
  `9bde1aef4721ffbc11e2fc904c118bb3c9d9272871cb71c75775d46700d0169f`。
  外部已解压目录不完整，应查 ZIP；选择的文本在 `work/factory-2026/`，N10 `.cc` 在 `work/n10-protocol/`。
  34 份 Word 已提取文字，10 个视频只计名称，不读取内容。源码没有提供可核验的 MCU 工程/hex。
- 出厂底盘 `/dev/car` 38400 8N1，7 字节 `AA motorLE servoLE sum55`；普通/one 转向系数1300/1200，
  遥控话题另用 PWM/角度语义，不混为物理速度/曲率。IMU `/dev/imu` 115200，N10 `/dev/laser` 230400。
- 教程明确没有轮编码器，里程计参考 RF2O/Cartographer，不能假造轮速反馈。
  相机格式/分辨率与 video 节点在不同示例中不一致；不要直接指定真实相机设备。
  原厂关闭某些碰撞/回环检查的导航配置不应原样作为新车安全配置。
- 镜像名标 Bianbu 2.2，只枚举目录，未展开 rootfs/安装；不因此改变 2.38 本地基线。
  ROS 脚本优先 Humble、可退 Foxy，必须核实实际系统，不能按 Ubuntu 版本直接装 Jazzy。

### 尚未完成与后续入口

- 新任务/视觉/扫描/定位/导航模块和独立线程接口已有实现，真实相机采集、共同单调时钟、扫描去畸变、
  位姿插值/融合、实际地面矩阵/灯ROI/场地图和真实制动反馈仍待验证。模拟配置不允许直接实车启用。
  新闭环中的Motion来自传感器计算；旧replay仍按输入事件诊断，不能混为比赛实时程序。
- **官方 RISC-V ORT/EP 2.0.6 仅静态核验**：资料在 `work/spacemit-runtime-followup/`，见
  [原生库核验](docs/SpacemiT原生运行库核验.md)。头文件 API24、导出 OrtGetApiBase；tag 代码支持 API22，
  但发布 manifest 提交不同，尚未执行发布库 GetApi(22)，未测试模型或 EP 初始化。
  ORT/EP 需 GLIBC 2.38，EP 另需 GLIBCXX 3.4.32 / CXXABI 1.3.15；未安装/打包厂商运行库。
- N10整圈、ICP和局部导航已完成离线模块；真实多传感器连接、ROS 2 接口及 MCU 反馈/watchdog 尚未实现或验收。
  接收健康与安全状态机不代替避障；静态标定表不表达 ESC 动态制动、死区、迟滞或多步骤倒车。
- 现有串口传输具备可执行实现，但没有真实设备测试，物理实时闭环和 MotionSink 尚未接入。
  用户准备接车后先核实设备/系统和厂商服务，再在独立目录做只读诊断及协议对照。
- Windows/WSL 仍为指南，未在队友设备验证；不把 Mac PTY、单图、回放或 ELF 检查当成车端帧率/停车验证。

### YOLO26n 固定接口

- 导出 nms=False 选 one-to-one；`nms=None` 默认不是同样接口。FP32 quantize=32、static320、batch1、
  max_det300、opset17、simplify=False、CPU。导出前确认 one-to-one 分支存在。
- `[1,300,6]` 每行 `[x1,y1,x2,y2,score,class_id]`，严格 `score > threshold`，不重复 sigmoid/objectness/NMS。
  必须核对 metadata 和精确哈希，形状本身不足以确认接口。低置信反向框先被阈值过滤，保留候选才检查几何。
- letterbox auto=False/scale_fill=False/scaleup=True/center=True、RGB、填充114、ties-to-even round；
  保存整数 left/top 和名义 r，以 `(coord-pad)/r` 还原后裁剪。
- 原文与许可证在 `资料/官方参考/Ultralytics_8.4.142/`；详见 [YOLO26 接口](docs/YOLO26接口.md)。

## 工程与平台

- 工程根目录为 `/Users/yuhaojin/Documents/XT-STCAR`，队友电脑使用其实际工作目录。
- Muse Pi Pro 主控是 64 位 RISC-V Linux；PDF 标注 Ubuntu 24.04、Python 3.12。
  Mac 本机 arm64、旧车 aarch64 Linux 与新车 riscv64 不可混用二进制和依赖库。
- ROS 发行版待车端核实。进迭时空 K1 官方文档在 noble-ros 源提供 Humble；
  不根据 Ubuntu 版本自动装 Jazzy，不把其他架构的 ROS 软件源或包直接用于新车。
- Mac 基础环境已安装：公共工具在 `/opt/homebrew`，项目 Python 库在 `.venv/`。
  实际版本和激活命令见 `资料/环境.md`，锁定文件为 `requirements-dev-macos.lock.txt`。
  独立 `.venv-model/` 已完成模型导出与主机推理；未进行训练或车端 ROS/推理验收。
- 用户明确要求 Mac 交叉编译后传给车端；当前已验证 Zig 路线的两个 Rust 主工程 RISC-V Linux 程序交叉链接。
  直接链接 ROS/OpenCV/厂商库时，仍需匹配车端 ISA/ABI、glibc 和依赖的工具链及 sysroot。
  不将厂商仅支持 Linux 主机的 SDK 当作 macOS 原生工具链安装。
- Windows 可先用 PowerShell/SSH 开发；需要本地 ROS、Linux 测试或 Linux SDK 时再使用 WSL2。
  WSL 的 Ubuntu/ROS 组合按厂家版本选择，不把 x86_64/ARM64 主机编译产物当成 riscv64 车端程序。
  两端同步源码与资源，禁止同步 `.venv`、build/install/log 或编译器缓存。
- 用户已明确首选 Rust：新写的核心算法、控制逻辑、状态机和协议处理优先 Rust。
  已有厂商 C++/Python 驱动与 YOLO 推理节点可复用，通过 ROS 2 接入，避免仅为统一语言重写。
  Rust ROS 接口候选为 rclrs，先在厂商系统验证消息生成、发布订阅和依赖，再锁定版本。
  高层算法、ROS 接口与底盘协议分层；依据已取得的厂商源码做互操作，不凭产品简介重写 MCU 固件。
- 当前主控编译目标选用 `riscv64gc-unknown-linux-gnu`，实机兼容性仍待验证。
  `rustup target add` 只提供目标标准库，完整交叉链接还需匹配 linker、sysroot 和外部库。
  STM32 裸机目标另行核对，不把 RISC-V 主控支持推导为下位机固件已支持。

## 实物与证据

- 主控系统镜像、ROS 版本、摄像头/雷达/IMU 型号、MCU 完整型号与通信协议均需实物核对。
- PDF 同时写“阿克曼结构”和“四轮差速驱动”；不要据此直接选择差速控制器。
  先确认转向机构、转角单位、轴距、轮距、轮径、反馈和电机结构。
- 有里程计教程不等于有编码器；有 IMU 不等于有 GPS。确认反馈来源、刷新率、时间戳和单位。
- 新车重新标定摄像头内外参、传感器 TF、转向零点和驱动映射。
  不复制旧 XT-NetRC 的相机身份、端口、640×480 假设、地面矩阵、GPIO、PWM 或 GPS 参数。
- YOLO 模型允许评估复用，依据 `资料/官方参考/进迭时空_YOLO部署_原文.md` 核对版本、
  ONNX 导出、类别、预处理/后处理和车端运行库。保留原权重，在实测精度和延迟后确定部署配置。
- 设备优先按实际 by-id/by-path 绑定。HTTP 端口、video/ttyUSB 编号和 IP 都不是永久身份。
- 标注“PDF 声称”“官方平台参考”“现场实测”“待确认”，不把缺失信息写成确定结论。
  标定和实验保留原始数据、参数、软件版本和结果；自动生成的图片不能替代测量证据。

## 工作方式

- 开始前核对目录、Git 状态、远端和文档；远端为用户指定的 `kxkxkxa767/XT-STCAR`。
  按 [上传规范](上传规范.md) 直接使用 `main`；修改前 fetch 检查，有远端更新先拉取后修改。
  完成检查后以简短、具体的 `git commit -m` 提交说明提交并推送；推送前再次检查远端，保留队友改动，禁止强推。
- 用户已授权的工作继续完成；资料/文档不另设重复审批。上传代码、部署与实车动作按实际授权区分。
- 旧工程已从文稿移到废纸篓，见 [清理记录](docs/旧工程清理记录.md)。
  不把新项目自动绑定到旧 `stupid_car` 仓库；不迁入旧工具链、虚拟环境或大备份。
- 不提交密码、私钥、工具链、系统镜像、构建产物、模型大文件或 rosbag。
  发布厂商代码/资料前核对许可和用户授权；已归档用户产品 PDF 与官方平台/Windows 参考。
- 找到的产品、平台和环境资料归档到 `资料/`，在 `资料/资料索引.md` 写来源、取得日期和适用范围。
  官方参考保留原文和许可证；原文相对图片链接可能无法离线显示，查图时打开原始网页。
  环境变化同步 `资料/环境.md`，区分 Mac、Windows/WSL 和车端命令，不复制旧车安装说明。
- 新功能优先使用厂商已验证接口，依赖版本写入环境记录和 package.xml；按需增加包。
  不执行来源不明的一键脚本，不混装多套 ROS/Python/OpenCV 来掩盖依赖冲突。
- 只因换平台，不卸载 Mac 通用开发工具或 STM32 工具；它们可能用于新车或其他项目。

## 首次接入与运行

- 先记录 uname、系统版本、ROS/Python、USB/串口与已有服务；只读命令见环境文档。
  未收到新车连接信息前，不尝试沿用旧车账号、IP 或密码。
- 保留出厂系统、驱动和 MCU 固件，在独立工作空间开发；刷系统/固件需有明确任务与可用恢复资料。
- 先检查传感器、TF 和里程计，再开展控制。首次带动力测试须有现场人员、明确测试范围和停车手段。
  节点启动前检查 launch 文件及自启动服务是否会输出运动命令。
- 不猜测 `/cmd_vel` 的消息类型、转向语义或底盘串口格式。
  断连、旧帧、失控和超时处理须覆盖真实底盘接口；不能用持续发油门掩盖通信问题。
- 激光雷达/IMU/相机数据融合前核实坐标系、时间同步和标定，不能承诺仅凭产品精度实现距离控制。

## 验证与交付

- 文档变更核对链接和事实；代码变更执行相关单元测试、构建及目标架构检查。
  ROS 工作空间创建后使用 colcon build/test 和 test-result；当前两个 Rust 主工程程序已编译/链接通过；ROS 尚未集成，不能据此声称 ROS 验收通过。
- Rust 工程创建后维护 Cargo.lock 和明确工具链版本，执行 cargo fmt --check、cargo test、
  cargo clippy --all-targets -- -D warnings；目标编译、链接和车端运行分别记录。
  `target/`、Cargo 缓存与 rustup 工具链不提交。尽量使用安全 Rust，FFI/unsafe 边界需说明依据。
- 主机测试通过不等于车端兼容；车端构建通过不等于实车控制验证。
  按实际完成情况记录命令、输出、失败原因及待确认项目。
- 交付时说明修改、测试、部署/上传状态和必要的下一步，使用简洁中文。
- 本文件保存长期约定；环境信息写入 `资料/环境.md`，实测状态写入 `docs/`。
  `AGENTS.md` 仅作此文件的加载入口，维护时完整核对本文件，不留旧车型规则。

2026-09-18：通用HTML已加入跨平台SSH启动步骤及复制按钮。共享启动脚本scripts/start-vehicle-console.py部署为车端~/xt-stcar-console/start-console.py，--url显示入口；复用现有服务，不自动解锁/运动。网页访问码与Linux SSH密码是两回事，均不写进HTML。

本轮检查：Rust release workspace 623 通过、3 忽略；Python 调度断点/退出码/证据保留测试 1 通过；fmt、全 target clippy 无警告通过。范围队列仍在运行，静态验证不等于比赛通过。机器记录见 [local-sweep-validation.json](docs/local-sweep-validation.json)。
