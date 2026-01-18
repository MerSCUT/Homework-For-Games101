# Homework For Games101
Assignment 使用 VisualStudio2022 自建环境完成.

 `Eigen3` 库 和 `opencv` 库使用 `vcpkg` 下载并管理.

 `Assignment_i/` 下的 `CMakeSettings.json` 由 VisualStudio2022 自动生成, 该文件中 "Cmake工具链文件" 一项中填入了本地路径中的`vcpkg/scripts/buildsystems/vcpkg.cmake` 

原有的 `CMakeLists.txt` 环境配置默认在 Linux 下, 需要主动写明对 `Eigen3` 库的链接. 

再将代码中对 `Eigen3`的引用修正 即可正常编译运行.
