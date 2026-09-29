# 第三方代码与公开整理说明

- `24_H.c` 开头保留了 Texas Instruments 的 BSD 风格许可声明。该文件由 TI DriverLib 空白工程模板继续开发；使用或再发布时应保留文件中的声明。
- MSPM0 SDK 的 DriverLib、CMSIS、启动文件以及 SysConfig 生成的 `ti_msp_dl_config.c/.h` 不随仓库提交。编译时从本机 TI SDK 和 SysConfig 取得，并遵守其各自许可。
- 原工程的 `Drivers/MPU6050/` 含 InvenSense DMP 文件，但未参与当前构建，因此没有纳入公开仓库。
- 原工程使用的 OLED 字库和显示驱动没有可核对的再发布许可信息，因此公开版本移除了 OLED 显示依赖；小车导航流程仍使用原工程的灰度传感器、HWT101 与电机控制代码。

仓库根目录的 MIT 许可适用于本仓库其余应用代码、文档和构建脚本；上述带有独立声明的组件按各自条款使用。
