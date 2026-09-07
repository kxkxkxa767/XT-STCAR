# GLIBC 2.38 升级记录

2026-09-07，按用户明确要求完成本地 RISC-V 交叉编译目标升级。
目标为 `riscv64gc-unknown-linux-gnu.2.38`；Mac 系统使用 libSystem，本次没有安装或替换车端 libc，车辆仍未连接。

## 实现和验证

- 构建命令、构建证据及部署 manifest 统一为 `.2.38`；ELF 检查上限 2.38，报告分别记录 `glibc_baseline` 与实际 `maximum_glibc`。
- 打包拒绝旧 `.2.27` 构建记录；归档校验检查 build 与 manifest 的 target 一致。
- 执行 `cargo clean --release --target riscv64gc-unknown-linux-gnu --package xt-stcar --package xt-stcar-robot-runner` 清理两个程序的目标产物，再运行 `bash scripts/build-riscv.sh --offline`。编译日志确认重新链接两个程序，未复用旧程序冒充新构建。
- fmt、42 项 Rust 常规测试、clippy `-D warnings`、两个 RISC-V release 构建均通过。1 项 native opt-in 测试本次 ignored；此前真实模型验证仍见旧记录。
- `python3 -m unittest discover -s tests -p test_delivery.py -v`：18 项通过，覆盖 2.38 允许、2.39 拒绝、版本表而非任意字符串检查、旧目标拒绝、重算哈希后包内目标不一致拒绝，以及 upload dry-run 不启动网络子进程。
- 三个构建/交付 shell 脚本 `bash -n` 通过；file / LLVM 独立检查架构和加载器。LLD 仍有既有非致命 deprecated optimization setting 告警。

## 新产物

两个程序均为 ELF64 LE RISC-V / RVC / LP64D / PIE，加载器 `/lib/ld-linux-riscv64-lp64d.so.1`，DT_NEEDED 仅 `libc.so.6`。
实际最高 GLIBC 引用为 **2.34**，符合 **2.38** 构建上限；不要求强行引用 2.38 新函数。

| 程序 | 字节数 | SHA256 |
|---|---:|---|
| xt-stcar | 1,213,880 | `0592ea68f687d8cd3a9609ace273e7722670e9d32d7074cd145e600cb4d303ff` |
| xt-stcar-robot | 1,284,872 | `49d1f6ebc0d5e15398e59aabad097a1e058bbc81c136d0b3b3a736e8a7336205` |

构建、ELF 与测试证据：`riscv-build-glibc-2.38.json`、`riscv-elf-glibc-2.38.json`、`riscv-robot-elf-glibc-2.38.json`、`glibc-2.38-build.log`、`glibc-2.38-delivery-tests.log`。
完整包校验为 `delivery-validation-glibc-2.38.json`，core 和含模型包均重新生成并通过逐文件/压缩包 SHA256 检查。

- `XT-STCAR-Rust-riscv64-core-glibc-2.38.tar.gz`：1,349,568 字节，SHA256 `dd6005403e583e10bcff5e81a5c1fbe50496362d127d1214c2e4874f2575656f`。
- `XT-STCAR-Rust-YOLO26n-riscv64-glibc-2.38.tar.gz`：9,994,218 字节，SHA256 `57a956e57aed4b2bb8c2fcdc3f2e0bf0dee3b836ac906ea42befe0aa27a4f0cd`。

## 外部运行库边界

已取得的官方 SpacemiT ORT/EP 2.0.6 要求 GLIBC 2.38，EP 还需要 GLIBCXX 3.4.32 / CXXABI 1.3.15。
详情见 [原生库静态核验](SpacemiT原生运行库核验.md)。它们未安装、未放入部署包，实际 GetApi(22)、厂商 EP 和目标模型推理尚未验收。
本次没有更改 Rust 算法或 YOLO26n 模型，不把编译成功当成车端运行成功。
