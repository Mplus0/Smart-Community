# Seekware SDK 3.4

来源：厂家提供的 `Seekware_SDK`（2020-05-18）。只保留采集需要的文件：

- `lib/{x86_64,aarch64,armv7l}/libseekware.so.3.4`：Linux glibc 运行库，分别来自 SDK 的 `x86_64-none-linux-gnu`、`aarch64-none-linux-gnu`、`arm-none-linux-gnueabihf`。
- `include/seekware/seekware.h`：厂家接口和结构体定义，供核对 Python ctypes ABI。
- `udev/10-seekthermal.rules`：原厂 USB 权限及禁止自动休眠规则。
- `LICENSE.txt`、`CREDITS.txt`：原厂许可与第三方声明，使用和分发须遵守原文。

节点按 CPU 架构使用绝对路径加载运行库，不需要设置 `LD_LIBRARY_PATH`。
依赖系统 `libusb-1.0-0`、glibc、libstdc++；不需要 SDL2。
不包含固件升级工具，不修改相机固件。
