# SpacemiT K1 原生运行库只读核验

2026-09-07。仅下载与静态检查，未安装、未执行库/示例、未连接开发板、未修改主工程。完整证据保存在工程 `work/spacemit-runtime-followup/`，包括 `runtime-validation.json`、`sources.json`、`release-inventory.json`、官方发布包和 `lib*.objdump.txt`。

已取得可落实的官方候选运行库。此前“尚未取得目标原生库”的资料缺口已关闭；目标系统兼容与实际推理尚未验证。

## 原生库与 ABI

- [官方 Release 2.0.6](https://github.com/spacemit-com/onnxruntime/releases/tag/2.0.6)，2026-07-27 发布；[RISC-V Linux 包](https://github.com/spacemit-com/onnxruntime/releases/download/2.0.6/spacemit-ort.riscv64.2.0.6.tar.gz) 为 15,002,263 bytes，SHA256 `bebcdfb7df6b49eefa3863afcd85a3da2aa83c3ae9252d7d856188c38a70b0e6`，与 GitHub 发布 digest 完全一致。已保留包内 MIT LICENSE。
- `libonnxruntime.so.1.24.2+spacemit.a1` 导出 `OrtGetApiBase@VERS_1.24.2`；包内 `onnxruntime_c_api.h` 定义 `ORT_API_VERSION=24`。两主库均为 ELF64 little-endian RISC-V、RVC、LP64D。
- ORT 最大未定义符号版本为 `GLIBC_2.38` / `GLIBCXX_3.4.30` / `CXXABI_1.3.15`；EP 为 `GLIBC_2.38` / `GLIBCXX_3.4.32` / `CXXABI_1.3.15`。两者 NEEDED 均为 `libatomic.so.1`、`libstdc++.so.6`、`libm.so.6`、`libgcc_s.so.1`、`libc.so.6`、`ld-linux-riscv64-lp64d.so.1`。因此不能凭本工程此前可执行文件的 GLIBC 2.27 基线断言此库可用。
- 发布 tag 是 `61e7fc2319cd16aa5487fd1155dc15c5390c8a90`，包内 manifest 的 `git_commit_id` 是 `5a5b59149c701931aeb32c36bc0181b4edf34dcc`，两者不同，已分别记录。tag 的 [GetApi 实现](https://github.com/spacemit-com/onnxruntime/blob/61e7fc2319cd16aa5487fd1155dc15c5390c8a90/onnxruntime/core/session/onnxruntime_c_api.cc#L4853) 接受版本 1–24。这是 API22 的源码与发布头/符号证据；尚未执行发布库的 `GetApi(22)`。

## 已找到可用于 Rust FFI 的 C 接口

包内 `include/spacemit_ort_env_c_api.h`，由 `libspacemit_ep.so.2.0.6` 导出未修饰符号：

```c
OrtStatus* ORT_API_CALL OrtSessionOptionsSpaceMITEnvInit(
    OrtSessionOptions* options,
    const char* const* provider_options_keys,
    const char* const* provider_options_values,
    size_t num_keys);
```

Linux 下 `ORT_API_CALL` 为空，使用平台 C ABI。options 应来自同一已加载 ORT；keys/values 是等长、以 NUL 结尾的 UTF-8 字符串指针数组，长度为 num_keys。调用期间维持缓冲区有效。头文件未规定调用后是否保留指针及无效参数行为，不能自行假定。

返回值遵循 ORT 状态约定：NULL 表示成功；非 NULL 时通过同一 OrtApi 的 `GetErrorCode` / `GetErrorMessage` 读取，再 `ReleaseStatus` 恰好释放一次。包内 C++ `Status` 明确接管 C API 返回状态并在析构释放，可交叉核对；不要将错误当成已启用 EP。

## 线程、输入与量化

完整阅读了 [docs-ai ONNXRuntime 章节](https://github.com/spacemit-com/docs-ai/blob/e250c5fe3649241be2675e66dcf328fd5a467f93/zh/compute_stack/ai_compute_stack/onnxruntime.md)、EP FAQ、算子表、XSlim 章节，以及教程第 15 篇。

- ORT 与 EP 线程池独立。官方说明只设置 ORT intra=N 时会同时令 EP intra=N，产生约 2N 计算线程；初始可测 ORT intra=1、EP intra=4，不能照抄 K3 绑核编号。`SPACEMIT_EP_USE_GLOBAL_INTRA_THREAD=1` 要求共享池的 session 串行执行。
- EP 通过 `SPACEMIT_EP_INTRA_THREAD_NUM`、`SPACEMIT_EP_INTER_THREAD_NUM` 等字符串选项配置。可用 DUMP_SUBGRAPHS / DEBUG_PROFILE 检查实际分图与执行；不支持的算子回落 CPU。DUMP_TENSORS 虽出现在在线文档，发布 README 明确说 release 版本不提供，须以实际包为准。
- 示例公开 float32 NCHW 输入；INT8 模型不等于外部输入必须是 int8。每个模型仍需读取输入名称、类型、shape。当前模型的 RGB、letterbox、114 padding、/255 契约应保持。
- [XSlim 精确源码](https://github.com/spacemit-com/xslim/tree/9a33f2f770d00fd02ff8bc0f1907135e9bf47f8c) 的 VERSION_NUMBER 是 2.1.2（README 徽章仍写 2.1.1）。已全文读 README、使用示例，读配置/构建元数据。通用 YOLO 示例实际是 YOLOv8：建议 precision_level=1、finetune_level=2，使用校准数据，可接自定义 NCHW 预处理。
- 没找到 YOLO26n 320 FP32 的明确导出量化配方。XSlim 可能把 YOLO decode 改写为 `spacemit_functions.YoloDecode` 并保留 FunctionProto；现有锁定模型的 provenance 不能直接用于这种新图。量化应另生成模型、重新检查接口与精度，不截掉我们需要的六列输出头。

## 下一步

1. 先匹配开发板用户态的 GLIBC、GLIBCXX、CXXABI 符号及上述依赖；不为新运行库擅自升级系统。
2. 获准实机验证后，先动态加载 ORT，调用 `GetVersionString` / `GetApi(22)`，再以 CPU 后端运行已有 FP32 bus 基准。
3. 再按已发现的 C 头接入显式 EP 初始化、错误释放、线程选项与 provider 验证，比较精度和耗时。
4. 最后单独开展 XSlim 校准量化。官方已报告 K1 YOLO26n 640 INT8 成绩，但不能代替本工程 320 模型的验收。

附：安全解压只写普通文件，符号链接仅记录未创建。Apple LLVM 可读该 ELF 元数据及符号，但不支持 RISC-V 反汇编；本报告没有反汇编结论。docs-ai 根目录 LICENSE 请求返回 404，未伪造许可；XSlim 和发布包各自 LICENSE 已保存。

2026-09-07 后续按用户要求，将本地应用构建目标升级至 GLIBC 2.38；见 [升级验证记录](GLIBC-2.38升级记录.md)。未安装或执行上述厂商库，Rust EP 初始化尚未集成。
