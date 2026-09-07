# YOLO26n Detect 接口与 Mac 参考推理

接口依据为 Ultralytics **8.4.142**，Git 提交
`2c5d376eee77b3d2217db609ce7b85837c1b191e`。该版本 `nms=False` 启用有 one-to-one
分支的模型；`nms=None` 不等价。依据源码已归档到
[Ultralytics_8.4.142](../资料/官方参考/Ultralytics_8.4.142/SOURCES.json)，同时保留上游 LICENSE。

## 默认 Rust 推理入口

默认 `xt-stcar infer` 使用 `NativeOrtBackend`，Rust 通过 `ort=2.0.0-rc.13`
（`std/load-dynamic/api-22`，无默认特性）调用标准 ONNX Runtime C API。
实际使用 Mac ORT 1.29.0 C 库验证，不启动 Python。库在项目内稳定路径
`toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib`，仅适用于 Mac ARM64。
模型旁 `.provenance.json` 绑定 ONNX SHA256、静态图校验和 metadata；实际 session 再核对
输入/输出名称、dtype、shape 和 metadata。原生加载/推理有协作取消，不能当成硬实时隔离。

```bash
cargo run --locked --bin xt-stcar -- infer --image path/to/image.png \
  --model models/yolo26n.onnx \
  --runtime-lib toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib
```

`crates/app` 同时提供可复用 `NativeOrtBackend::new(model, runtime_lib, spec)`；
机器人 runner 使用常驻 Session 处理多帧。RISC-V 只交叉编译 Rust 包装与程序，
车端 ORT 动态库需另外核实准备，不能用 Mac 库替代，也不假定厂商 C ABI 兼容。
只有显式选择 `--backend python-reference` 才启用本文后面的 Python worker。

实际模型已导出：445 个节点、0 个 NMS，ONNX SHA256
`c52d204571c6df9f1132dedd7aab3e87336434589b055b9fa4026d117f1d4045`。
公交车样例中 Rust 原生和 Python 参考的 5 个检测结果（4 person / 1 bus）逐值一致；
同输入 PyTorch 与 ORT 的高置信框最大差约 `4.58e-5` 像素、置信度最大差约 `1.55e-6`。
这些是单样例接口对照，不是数据集精度验收。详见 [实测记录](验证记录-2026-09-07.md)。

## 导出与模型身份

首版导出脚本只接受官方 [yolo26n.pt 权重](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt)，
加载 PyTorch 权重前核对 SHA256：
`9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef`。
该值已与 GitHub Release 资产 digest 核对；仅凭文件名不能证明是 nano 模型。
自训练权重、其他尺寸或类别数需另行建立明确的配置和可信来源后扩展，本脚本会拒绝它们。

```python
model.export(format="onnx", imgsz=320, batch=1, dynamic=False,
             nms=False, quantize=32, max_det=300, device="cpu",
             simplify=False, opset=17)
```

| 接口项 | 锁定值 |
|---|---|
| 输入 | `images`，FLOAT32 `[1,3,320,320]` |
| 输入语义 | RGB、连续 NCHW、除以 255，数值 `[0,1]` |
| 输出 | `output0`，FLOAT32 `[1,300,6]` |
| 每行 | `[x1,y1,x2,y2,score,class_id]`，输入画布像素坐标 |
| 类别 | 80 个；class_id 为浮点存储的整数 |
| 后处理 | 图内 TopK；Rust 消费端严格 `score > threshold` |
| 不需要的运算 | 额外 sigmoid、objectness 相乘、第二次 IoU NMS |

`validate_yolo26.py` 同时检查 metadata `task=detect`、`head=Detect`、`version=8.4.142`、
`end2end=True`、80 个 `names` 整数键、单输入/输出的名称/类型/静态形状、opset 17，
以及主图、控制流子图和本地函数中没有 `NonMaxSuppression`，并执行 ONNX full checker。
缺少元数据、FP16、动态尺寸、raw `[1,84,2100]` 等输出会被明确拒绝。
元数据与图结构验证证明的是接口符合性；权重与 ONNX 的可信来源由 SHA256 和 provenance 文件另行记录。

## 独立 Mac 模型环境

模型栈位于 `.venv-model/`，原 `.venv/` 基础开发环境保留。两者不混用 OpenCV 包，
`.venv-model/` 使用 `opencv-python` 4.x。重建与验证命令：

```bash
/opt/homebrew/bin/python3.12 -m venv .venv-model
.venv-model/bin/python -m pip install -r requirements-model-macos.lock.txt
.venv-model/bin/python -m pip check
.venv-model/bin/python scripts/export_yolo26.py --weights models/yolo26n.pt --spec config/yolo26n.json
.venv-model/bin/python scripts/validate_yolo26.py --model models/yolo26n.onnx --spec config/yolo26n.json
.venv-model/bin/python -m pytest -q tests/test_yolo26_tools.py
```

导出成功产生 `models/yolo26n.onnx` 与 `models/yolo26n.provenance.json`。
provenance 包含输入/输出契约、导出参数、依赖版本、模型/权重 SHA256 和官方来源；其中绝对路径仅作本机审计。
权重、ONNX、大型依赖环境均为本地验证资源，不提交或默认发布。
锁文件是 Mac ARM64 / Python 3.12 的已解析依赖清单，不代表 RISC-V wheel 可用。

## Rust 默认运行与显式 Python 对照

应用默认 `infer` 使用 Rust 原生 ONNX Runtime C 动态库接口。
Mac 的标准 C ABI 库已从 ONNX Runtime 1.29.0 官方 PyPI wheel 中提取到
`toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib`；已核对 arm64 架构及 `OrtGetApiBase` 导出。
它仅用于 Mac 本地验证。车端需要匹配 RISC-V 架构且实际验证可用的 C ABI 库，不能复制此 dylib。

```bash
target/debug/xt-stcar infer --image IMAGE.png --model models/yolo26n.onnx \
  --runtime-lib toolchains/onnxruntime-macos-arm64-1.29.0/lib/libonnxruntime.1.29.0.dylib
```

原生后端读取同名 `yolo26n.provenance.json`（也可显式 `--provenance FILE`），
将模型 SHA256、固定接口与会话元数据核对后推理。
provenance 中 `metadata` 保留 ONNX 元数据全部原始字符串，供 Rust 比较 `names` 等字段。
默认运行不启动 Python；Python 保留为离线导出、校验和显式参考对照工具：

```bash
target/debug/xt-stcar infer --backend python-reference \
  --python .venv-model/bin/python --worker scripts/onnx_worker.py \
  --image IMAGE.png --model models/yolo26n.onnx
```

以下说明参考 worker 的独立协议。

## 一次性参考 worker 协议

```bash
.venv-model/bin/python scripts/onnx_worker.py \
  --model models/yolo26n.onnx --spec config/yolo26n.json \
  --input tmp/input.f32le --output tmp/output.json
```

`--input` 必须恰好为 1,228,800 字节：连续 little-endian float32 NCHW，**不是**图像文件。
worker 先校验 ONNX 与输入，再使用标准 `onnxruntime.InferenceSession`，明确选择 `CPUExecutionProvider`。
输出仅通过 `--output` 文件传递，以原子替换写入：

```json
{"shape":[1,300,6],"values":[10,20,30,40,0.75,2]}
```

以上仅展示首行格式；真实 `values` 是展平的 **1,800** 个有限 float32 数值。
worker 不做阈值过滤；stdout/stderr 均可输出诊断，不属于协议。成功退出后调用方才读取 JSON；
失败退出码非零，调用方负责超时、中止和清理临时目录。
worker 同时拒绝非有限值和非法概率/类别。Rust 解码器先按阈值过滤，再拒绝保留检测中的反向/空框；
低置信度回归行的几何形状不作为 worker 失败条件。

此后端是 **Mac 上验证的标准 ONNX Runtime CPU 参考实现**。
它没有调用 `python3-spacemit-ort`，不能据此推断厂商 Python API、YOLO26n 算子覆盖、NPU 加速或车端帧率。
RISC-V 端运行库必须在取得厂商示例与实际系统后另行适配。

## 预处理与坐标还原

采用 letterbox `auto=False, scale_fill=False, scaleup=True, center=True`，填充 114、INTER_LINEAR。
名义缩放比例 `r=min(320/W,320/H)`，缩放尺寸按 ties-to-even round，记录整数 left/top。
还原按 `(x-left)/r`、`(y-top)/r` 并裁剪到原图。纯 Rust 与 OpenCV uint8 插值的差异须以数值对照记录，
不能仅因算法名相同就称逐像素一致。JPEG 解码库差异也可能影响输入。

## 2026-09-07 本机实测

独立 `.venv-model/` 的 `pip check` 通过，50 个依赖锁定在
`requirements-model-macos.lock.txt`。核心实际版本为 Ultralytics 8.4.142、Torch 2.14.0、
torchvision 0.29.0、ONNX 1.22.0、ONNX Runtime 1.29.0、OpenCV 4.14.0.94 和 NumPy 2.5.3。
安装包的 exporter/head 文件 SHA256 与归档锁定源码一致。

真实导出的 `models/yolo26n.onnx` 为 445 节点、9.3 MB，完整契约及 ONNX full checker 通过。
本次 ONNX SHA256 为 `c52d204571c6df9f1132dedd7aab3e87336434589b055b9fa4026d117f1d4045`；
导出元数据含时间，重新导出后的文件哈希可能不同，必须使用新生成的 provenance 配对。

对 Ultralytics 包内真实 `assets/bus.jpg`（810×1080）执行 OpenCV letterbox、PyTorch one-to-one
与标准 ORT CPU 同输入对比：`score > 0.25` 均得 **5 个框：4 person、1 bus**。
保留检测的类别和数量完全一致，框坐标最大绝对差 `4.5776e-5` 像素、分数最大差 `1.5497e-6`；
全部 300 行类别一致，全张量最大绝对差 `0.00036621`。这验证模型转换一致性，不代表实车场景精度或速度。

- 原始 JPEG SHA256：`c02019c4979c191eb739ddd944445ef408dad5679acab6fd520ef9d434bfbc63`。
- 实际 f32le 输入 SHA256：`aaa0f5aa8cf60c6bc219a83f0a836d85e4cf8426946a4d0a34ffe2c56bb8b4c2`。
- 原始输入和输出：`tmp/yolo26-reference/bus-cv.f32le`、`bus-ort.json`。
- 报告：`models/yolo26n.reference-validation.json`；重跑命令如下。

```bash
.venv-model/bin/python tests/measure_yolo26_reference.py
```

在同一 Rust 预处理输入下，默认 Rust 原生 ORT 与显式 Python 参考 CLI 的 5 个检测结果
逐项相同（类别、坐标、分数误差均为 0），报告为 `models/yolo26n.native-reference-comparison.json`。
Rust 原生输出与参考输出分别保存在 `work/native-ort-bus.json`、`work/python-reference-bus.json`。

Python 工具与双程序交付边界完整测试为 **56 passed、15 subtests passed**。
其中合成 ONNX 夹具仅验证协议和拒绝路径；真实模型证据是上面的权重导出与图片对照。

## 上游来源

- [exporter.py](https://github.com/ultralytics/ultralytics/blob/v8.4.142/ultralytics/engine/exporter.py)：one-to-one 开关、导出参数、元数据。
- [head.py](https://github.com/ultralytics/ultralytics/blob/v8.4.142/ultralytics/nn/modules/head.py)：Detect、TopK 与输出布局。
- [nms.py](https://github.com/ultralytics/ultralytics/blob/v8.4.142/ultralytics/utils/nms.py)：end-to-end 分支严格置信度阈值。
- [augment.py](https://github.com/ultralytics/ultralytics/blob/v8.4.142/ultralytics/data/augment.py)、
  [ops.py](https://github.com/ultralytics/ultralytics/blob/v8.4.142/ultralytics/utils/ops.py)：LetterBox 与坐标还原。
- [标准 ONNX Runtime Python API](https://onnxruntime.ai/docs/api/python/api_summary.html)：
  `InferenceSession` 接受 ONNX bytes、显式 provider 与 NumPy 输入；
  [2026-09-07 页面归档](../资料/官方参考/ONNXRuntime_PythonAPI_原文.html)。这是上游 API，不能代替厂商接口调查。
