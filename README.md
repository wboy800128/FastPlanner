# FastPlanner: 2D Trajectory Optimization (Pure C++)

[![C++](https://img.shields.io/badge/Language-C++-blue.svg)](https://isocpp.org/)
[![Build](https://img.shields.io/badge/Build-CMake-brightgreen.svg)](https://cmake.org/)

##  项目简介

本项目将 **FastPlanner** 后端基于 B 样条（B-Spline）的轨迹优化核心算法进行了剥离与封装，移除了 ROS 依赖，提供了一个可在本地编译运行的 C++ 版本，便于做 Windows（MSVC）或 Linux 下的验证与二次开发。

---

##  Windows (MSVC)  依赖与快速上手

下面给出在 Windows + Visual Studio 2022（MSVC, x64）下的推荐流程，包含 cpkg、Eigen、NLopt 与 Python 可视化的安装与构建步骤。

- 前提：已安装 Visual Studio 2022（含 C++ 桌面开发工作负载）和 CMake。
- 建议安装 cpkg 用于管理 C++ 库（Eigen 等）。如果已在系统其他位置准备好依赖（如 	hird_party/nlopt），也可跳过 vcpkg 部分。

1) 获取并初始化 cpkg（若已安装请跳过）：

`powershell
git clone https://github.com/microsoft/vcpkg.git D:\GitHub\vcpkg
cd D:\GitHub\vcpkg
.\bootstrap-vcpkg.bat
`

2) 使用 cpkg 安装常用依赖（Eigen 必要，Boost 可选）：

`powershell
cd D:\GitHub\vcpkg
.\vcpkg.exe install eigen3 --triplet x64-windows
# 可选：如果需要完整 Boost 支持（生成消息、序列化等），安装 boost
# .\vcpkg.exe install boost --triplet x64-windows
`

3) NLopt：
- 如果仓库已有已构建的 	hird_party/nlopt（例如你之前已用相同生成器构建并安装到该目录），在 CMake 配置时通过 -DNLopt_DIR= 指向其 lib/cmake/nlopt 即可。
- 如果没有，请在仓库内构建：

`powershell
cd D:\GitHub\FastPlanner\files\nlopt-master
mkdir build && cd build
cmake -G "Visual Studio 17 2022" -A x64 -DBUILD_SHARED_LIBS=ON ..
cmake --build . --config Release -- /m
# 可选：安装到指定前缀
# cmake --install . --config Release --prefix D:/GitHub/FastPlanner/third_party/nlopt
`

---

##  在 Windows（VS2022）上配置、构建与运行

下面示例使用 cpkg toolchain（如果你没有使用 vcpkg，请省略 -DCMAKE_TOOLCHAIN_FILE 与 -DVCPKG_TARGET_TRIPLET，并通过 -DNLopt_DIR= 指定 NLopt 路径）。

`powershell
cd D:\GitHub\FastPlanner
mkdir build
cd build

# 使用 vcpkg toolchain 并指定 NLopt（如果可用）
cmake -S .. -B . -G "Visual Studio 17 2022" -A x64 \
  -DCMAKE_TOOLCHAIN_FILE=D:/GitHub/vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_TARGET_TRIPLET=x64-windows \
  -DNLopt_DIR=D:/GitHub/FastPlanner/third_party/nlopt/lib/cmake/nlopt

# 并行构建 Release
cmake --build . --config Release -- /m

# 运行 main.exe（生成位置：build/Release/main.exe）
.\Release\main.exe
`

构建成功后，可执行文件位于 uild\Release\main.exe（或 IDE 中对应的输出目录）。

---

##  可视化轨迹（Python）

本项目不绑定特定 Python 可视化库。推荐使用系统 Python（3.8+）并安装 matplotlib 与 
umpy：

`powershell
python -m pip install --user matplotlib numpy
`

若 main.exe 或程序导出为 	rajectory.csv（CSV 格式，首行为列名或纯数据），可用下面的快速脚本绘制：

示例脚本 plot_trajectory.py（将此脚本复制到仓库根目录并运行）：

`python
import numpy as np
import matplotlib.pyplot as plt
import sys

path = sys.argv[1] if len(sys.argv) > 1 else 'trajectory.csv'
data = np.loadtxt(path, delimiter=',', skiprows=1)
# 假设列 0/1 为 x/y
plt.figure(figsize=(6,6))
plt.plot(data[:,0], data[:,1], '-o')
plt.axis('equal')
plt.grid(True)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Trajectory')
plt.savefig('trajectory.png')
print('Saved trajectory.png')
`

运行示例：

`powershell
python plot_trajectory.py build\Release\trajectory.csv
`

如果没有 	rajectory.csv，也可以直接在程序中将轨迹输出为 CSV（建议在 main 中添加保存代码），或使用仓库中已有的绘图脚本（若存在）。

---

## 常见问题与排错

- 若 CMake 找不到 Eigen，请确认已使用 cpkg 并通过 -DCMAKE_TOOLCHAIN_FILE= 指定 vcpkg 的 scripts/buildsystems/vcpkg.cmake。
- 若链接时报找不到 
lopt，请确保 NLopt 用相同的 VS 生成器与架构（x64）构建，并在 CMake 命令中给出 -DNLopt_DIR= 指向 
lopt 的 CMake 包目录。
- 若出现大量生成消息依赖 Boost，建议通过 cpkg 安装 oost：

`powershell
.\vcpkg.exe install boost --triplet x64-windows
`

---

## 结语

如果你希望我为仓库添加：

- 一个自动化 PowerShell 脚本（scripts\\run_windows.ps1）来一键安装 vcpkg 端口、CMake 配置、构建与可视化，或
- 一个简单的 plot_trajectory.py 文件并把它加入到仓库（我可以直接创建并验证），

告诉我你优先希望我做哪项，我会接着实现并运行验证。

*Maintained by JackJu-HIT*

**一键自动化脚本**

- **用途**: 自动重建 NLopt（启用 Luksan）、安装到 third_party、重建项目、复制运行时 DLL、运行可执行并生成可视化图片。
- **前提**: 已安装 Visual Studio 2022（含 C++）、CMake、Python（含 matplotlib/numpy）。可选：vcpkg（如果使用 vcpkg 管理依赖）。
- **使用方法**: 在仓库根目录的 PowerShell 中运行：

  ```powershell
  .\scripts\rebuild_nlopt_and_run.ps1
  ```

- **输出**: 脚本会把生成的 `trajectory.csv`（若程序产生）与 `trajectory.png` 保存在仓库根目录，并在控制台输出运行日志。

