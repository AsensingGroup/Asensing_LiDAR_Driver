# Asensing_LiDAR_Driver


## 概述

**Asensing_LiDAR_Driver** 是 Asensing 激光雷达的驱动库，目前支持 A0、A2 型激光雷达。


## 如何构建

**Asensing_LiDAR_Driver** 支持的操作系统及编译器如下。注意编译器需支持 `C++14` 标准。

- Ubuntu (16.04, 18.04, 20.04)
  - gcc (4.8+)

- Windows
  - MSVC  (Win10 / VS2019 已测试)

### 安装依赖

**Asensing_LiDAR_Driver** 依赖的第三方库如下。

- `libpcap` (可选。如不需要解析PCAP文件，可忽略)
- `eigen3` (可选。如不需要内置坐标变换，可忽略)
- `PCL` (可选。如不需要可视化工具，可忽略)
- `Boost` (可选。如不需要可视化工具，可忽略)


### Ubuntu下的编译及安装

**安装第三方库**

```bash
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
```

**编译**

```bash
mkdir build && cd build
cmake .. && make -j4
```

默认开启 demo 和 tool 模块，您可以在构建目录中找到 demo_pcap、demo_online、simple_viewer 等可执行文件，并使用它们快速进行测试。

注意：本工程默认编译为 Release 版本，如果需要编译为 Debug 版本，可以在 `cmake` 命令指定，如下所示。

```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

**安装**

```bash
sudo make install
```

### 作为第三方库使用

配置您的 `CMakeLists.txt` 文件，使用 find_package() 指令找到 **ag_driver** 库，并链接。

```cmake
find_package(ag_driver REQUIRED)
include_directories(${ag_driver_INCLUDE_DIRS})
target_link_libraries(your_project ${ag_driver_LIBRARIES})
```

### Windows下的编译及安装

首先需要安装第三方库。

**安装libpcap**

安装 [libpcap 运行库](https://www.winpcap.org/install/bin/WinPcap_4_1_3.exe)。

解压 [libpcap 开发者包](https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip)到任意位置，并将 `WpdPack_4_1_2/WpdPack` 的路径添加到环境变量 `PATH`。

**安装PCL**

如果使用 MSVC 编译器，可使用 PCL 官方提供的 [PCL 安装包](https://github.com/PointCloudLibrary/pcl/releases) 安装。

安装过程中选择 `Add PCL to the system PATH for xxx`。

**打开VS工程文件**

工程目录 `demo/win` 和 `tool/win` 下，有编译实例程序和工具的 MSVS 工程文件，在 Visual Studio 中打开工程即可编译。

## 示例程序和工具

在 `demo` 目录中，提供了两个示例程序。

- `demo_online.cpp`：解析在线雷达的数据，输出点云。
- `demo_pcap.cpp`：解析 PCAP 文件，输出点云。编译 `demo_pcap` 需要安装 `libpcap` 库。

要编译这两个程序，需使能 `COMPILE_DEMOS` 选项。

```bash
cmake -DCOMPILE_DEMOS=ON ..
```

在 `tool` 目录中，提供了一个点云可视化工具 `simple_viewer`。

要编译这个工具，需使能 `COMPILE_TOOS` 选项。编译它需要安装 PCL 库和 Boost 库。

```bash
cmake -DCOMPILE_TOOLS=ON ..
```

## 接口文件

**Asensing_LiDAR_Driver** 的主要接口文件如下：

- 点云消息定义: `src/msg/point_cloud_msg.hpp`, `src/msg/pcl_point_cloud_msg.hpp`
- 接口定义: `src/api/lidar_driver.hpp`
- 参数定义: `src/driver/driver_param.hpp`
- 错误码定义: `src/common/error_code.hpp`

