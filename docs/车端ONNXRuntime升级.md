# 车端 ONNX Runtime 更新记录

2026-09-16，用户明确要求更新车上的ONNX Runtime。完成用户级原生运行库安装，并接入工程默认启动命令；系统包与Python包未作全局升级。

## 安装结果

| 项目 | 结果 |
| --- | --- |
| 官方发布包 | SpacemiT 2.0.6，内含ORT `1.24.2+spacemit.a1` |
| 固定目录 | `~/.local/opt/xt-stcar/onnxruntime/spacemit-2.0.6` |
| 当前版本链接 | `~/.local/opt/xt-stcar/onnxruntime/current` → `spacemit-2.0.6` |
| 工程入口 | `~/.local/bin/xt-stcar`，源码 `scripts/vehicle-vision.sh` |
| 配置文件 | `~/.config/xt-stcar/runtime.env` |
| 原系统环境 | 原生ORT1.18.1、Python onnxruntime1.18.0保留；dpkg包版本1.2.2是厂商包号，不能当ORT版本 |

固定版本来自此前已核验的[官方发布包](https://github.com/spacemit-com/onnxruntime/releases/tag/2.0.6)，不宣称它是所有平台的最新版本。重新检查车端压缩包SHA256，逐个对照lib文件与归档内容，复制库和许可证/版本/manifest/README。库符号链接只指向同目录文件。

当前APT缓存中的候选仍为厂商包1.2.2，未执行apt更新或更换软件源。SSH账户sudo需要密码，因此采用无需管理员权限的用户级安装；没有改写系统库、系统Python、libc或全局搜索路径。EP库随包保存但未初始化，推理仍使用标准CPU路径。

## 使用

车端SSH终端执行，图片路径换成实际PNG/JPEG：

```bash
~/.local/bin/xt-stcar infer --image /绝对路径/图片.png
```

新登录会话已实测 `command -v xt-stcar` 指向上述入口。已有终端若PATH未含 `~/.local/bin`，继续用绝对入口即可，无需修改全局shell配置。

入口只为infer补齐未提供的模型、配置与原生运行库参数；显式参数优先。其他子命令原样传递给Rust程序。Python仅用于原先的相机诊断，不参与这个启动入口的原生推理。

原Rust可执行文件仍要求显式参数。需要手动调用或供robot视觉参数使用时，在车端执行：

```bash
. ~/.config/xt-stcar/runtime.env
"$XT_STCAR_RELEASE/bin/xt-stcar" infer \
  --image /绝对路径/图片.png \
  --model "$XT_STCAR_RELEASE/models/yolo26n.onnx" \
  --config "$XT_STCAR_RELEASE/config/yolo26n.json" \
  --runtime-lib "$XT_STCAR_RUNTIME_LIB"
```

`XT_STCAR_RELEASE`当前指向 `/home/bianbu/xt-stcar-tests/20260916-bringup/release`，升级应用部署目录时需同步该配置。原含模型发布包与Rust二进制保持不变；它们的帮助和JSON内通用历史提示未重新编译。

## 验证与回退

- 新库动态依赖齐全，`GetVersionString`返回1.24.2+spacemit.a1，`GetApi(22)`非空。
- 从固定安装目录实际运行YOLO26n，再经新入口从 `/tmp` 运行一次，均退出0。真实照片检出人0.86995363、杯子0.35725343，与之前同图观测一致。
- 入口那次CLI报告1763.37ms，包括程序内部处理，不能换算成持续推理帧率。安装脚本的较长墙钟还包含后续Python导入检查，单独标注，不当成推理耗时。
- 原系统原生库安装前后SHA256一致，系统Python仍可导入且返回1.18.0。未发送底盘命令，未操作相机设备。

机器记录见[升级验证](vehicle-runtime-upgrade-validation.json)。原始安装过程保存在本机 `work/vehicle-runtime-update/` 和车端测试目录，现场照片不上传。

回退有两层：厂商程序始终使用原系统环境，无需回退；若需撤销工程新默认入口，可将 `~/.local/bin/xt-stcar` 重命名留存，再直接使用原部署包的 `bin/xt-stcar`。当前工程需要API22，系统1.18.1不满足，不能把改回系统库写成可用回退。可改用之前已实测的独立测试库路径：

```bash
--runtime-lib /home/bianbu/xt-stcar-tests/20260916-bringup/spacemit-ort.riscv64.2.0.6/lib/libonnxruntime.so
```

这是回退安装路径而非降版本；以后若有另一个通过验证的版本，可再切换 `current`。不删除旧库或修改厂商工作区。
