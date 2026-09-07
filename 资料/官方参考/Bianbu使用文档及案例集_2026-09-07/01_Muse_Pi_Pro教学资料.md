---
sidebar_position: 1
---

# K1 MUSE Pi Pro 教学资料

## 1. 快速入门

- [Muse Pi Pro 开发板简介](https://www.spacemit.com/community/development-kit/k1-muse-pi-pro)
- [Muse Pi Pro 官方使用手册](https://www.spacemit.com/community/document/info?lang=zh&nodepath=hardware/eco/k1_muse_pi_pro/pi_pro_user_guide.md)

## 2. 硬件资源

- [原理图](https://cdn-resource.spacemit.com/file/product/K1/MUSEPi%20Pro_schematic-V1.1-20250619.pdf)
- [物料清单 (BOM)](https://cdn-resource.spacemit.com/file/product/K1/MUSEPi%20Pro_BOM-V1.1-20250506.xls)
- [PCB 位号图](https://cdn-resource.spacemit.com/file/product/K1/MUSEPi%20Pro_PCB-V1.1-20250630.pdf)

## 3. 开发工具套件

| 工具名称      | 下载链接                                                     | 功能说明                           |
| :------------ | :----------------------------------------------------------- | :--------------------------------- |
| Titan Flasher | [官方下载](https://www.spacemit.com/community/document/info?lang=zh&nodepath=tools/user_guide/flasher_user_guide.md) | 固件烧录与调试                     |
| MobaXterm     | [官网下载](https://mobaxterm.mobatek.net/download-home-edition.html) | 多功能终端/SSH工具                 |
| VSCodium      | [开源版本](https://vscodium.com/)                            | 轻量级代码开发环境串口通信必备驱动 |

## 4. 新手入门指南

**视频教程、示例代码及 PPT 下载：**  
👉 [Gitee 仓库](https://gitee.com/chen_zhao_qi/learn-riscv-01.git)

### 4.1 SpacemiT RISC-V 入门

#### 4.1.1 SpacemiT RISC-V 简介

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665)

教程内容：

- RISC-V简介
- Spacemit RISC-V 技术栈和生态简介

#### 4.1.2 RISC-V 架构基础及快速入门

[参考官方文档](https://www.spacemit.com/community/document/info?lang=zh&nodepath=courses/RISC-V/01_RISC-V%E6%9E%B6%E6%9E%84%E5%9F%BA%E7%A1%80%E5%8F%8A%E5%BF%AB%E9%80%9F%E5%85%A5%E9%97%A8)

#### 4.1.3 MUSE Pi Pro 系统准备与刷机教程

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&p=2&spm_id_from=333.788.videopod.sections)

**教程内容：**
- MUSE Pi Pro 开发板介绍
- Bianbu LXQt 镜像下载与 Titan 工具安装
- 使用 Titan 工具刷写系统
- 初始化系统设置

#### 4.1.4 官网文档介绍

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=3)

### 4.2 Bianbu LXQt 系统基础使用

#### 4.2.1 串口调试

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=4)

**教程内容：**
- 硬件接线示意图
- MobaXterm 配置教程
- 串口通信使用说明

#### 4.2.2 SSH 远程连接

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=5)

**教程内容：**
- 网络配置指南
- MobaXterm SSH 登录
- Windows PowerShell SSH 登录
- `screen` 命令使用
- 配置 SSH 免密登录

#### 4.2.3 常用命令和 Git 使用

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=6)

**教程内容：**
- Linux 常用命令介绍
- Git 基础操作
- 代码管理最佳实践

### 4.3 内核编译与配置

[参考官方文档](https://www.spacemit.com/community/document/info?lang=zh&nodepath=software/SDK/buildroot/k1_buildroot/intro.md)
[Linux 内核编译指南](https://www.spacemit.com/community/document/info?lang=zh&nodepath=software/SDK/buildroot/k1_buildroot/source.md)

### 4.4 代码开发环境配置

#### 4.4.1 VSCodium 简介与安装

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=7)

#### 4.4.2 VSCodium 远程连接

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=8)

### 4.5 Python 代码开发方法

#### 4.5.1 虚拟环境与 pip 使用

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=9)

教程内容：

- 主环境与虚拟环境的区别和联系
- 如何创建虚拟环境
- 使用 pip、设置 Spacemit Pip 源
- 如何查看已提供whl包支持 python 包

#### 4.5.2 相机调用示例

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=10)

#### 4.5.3 使用 Jupyter Notebook

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=11)

**教程内容：**
- 主环境中使用 Jupyter Notebook
- 虚拟环境中使用 Jupyter Lab
- VSCodium 中集成 Notebook

#### 4.5.4 Python 实战案例

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=12)

**教程内容：**
- ONNX Runtime 使用
- Matplotlib 数据可视化
- SciPy 科学计算
- Scikit-learn 机器学习

### 4.6 C++ 代码开发方法

[视频教程](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&spm_id_from=333.788.videopod.sections&p=13)

**教程内容：**
- OpenCV 安装
- CMake 构建配置
- 项目编译与运行
- OpenCV 相机应用实战

## 5. 实战案例库

- [外设硬件调试](./04_外设模块参考说明.md)
- [AI 基础模块参考说明](./06_AI基础模块参考说明.md)
- [综合应用案例](./07_综合应用案例目录.md)
- [应用案例课程](https://www.spacemit.com/community/course)

## 6. 参考资料

- [官方文档中心](https://www.spacemit.com/community/document?lang=zh)
- [视频教程频道](https://www.bilibili.com/video/BV1Dh5Lz7EQD?buvid=YC4F7CC82F8DD56B4FD6AC6F6FF7187C0A99&from_spmid=search.search-result.0.0&is_story_h5=false&mid=rnn1kaOkAxuNW2GQpPH%2BIw%3D%3D&plat_id=116&share_from=ugc&share_medium=iphone&share_plat=ios&share_session_id=B8E3DF94-1B59-4760-9141-DC092B2F8A58&share_source=COPY&share_tag=s_i&spmid=united.player-video-detail.0.0&timestamp=1747018853&unique_k=f7XIcVl&up_id=3537125114906665&vd_source=d100e556fb0a91c2658ca2841b30a8b5)
- [示例代码仓库](https://gitee.com/chen_zhao_qi/learn-riscv-01.git)

## 7. 关于我们

🌐 **官方网站**: [https://www.spacemit.com/](https://www.spacemit.com/)  
📚 **开发者社区**: [https://www.spacemit.com/community/](https://www.spacemit.com/community/)  
🛠️ **技术论坛**: [https://forum.spacemit.com/](https://forum.spacemit.com/)  
💼 **加入我们**: [招聘页面](https://app.mokahr.com/social-recruitment/space-t1/67915)
