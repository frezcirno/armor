# 分支说明

| 分支名        | 说明                     |
|---------------|------------------------|
| master        | 步兵代码with曼德卫视相机 |
| master-dahua  | 步兵代码with大华相机     |
| shaobing      | 下哨兵代码               |
| shaobingshang | 上哨兵代码               |
| compen        | 已废弃                   |
| curve         | 已废弃                   |

1. 哨兵代码与步兵代码为什么分开？

以前是不分开的，但是比赛时发现哨兵由于位置较高，视野中目标装甲板的形状与步兵有所不同，导致在装甲板识别和判定部分有所区别，故开了两个分支

2. MindVision相机和大华相机的区别？

帧率不一样，曝光不一样

3. 上下哨兵代码为什么分开？

以前上下哨兵相机不一样╮(╯▽╰)╭

# 目录结构说明

| 文件           | 说明                     | 备注                               |
|----------------|--------------------------|------------------------------------|
| daemon/        | 守护进程                 | 详见内部的README文件               |
| data/          | 程序运行时数据           | 详见内部的README文件               |
| doc/           | 开发文档                 | 详见内部的README文件               |
| Model/         | 数字分类器模型           |                                    |
| info/          | 风车代码使用             |                                    |
| pics/          | 风车代码使用             |                                    |
| tools/         | 开发小工具存放地         |                                    |
| video/         | 测试视频存放地           |                                    |
| include/       | C++头文件                | h/hpp格式                          |
| source/        | C++源代码文件            | hpp格式                            |
| tensorflow/    | tensorflow头文件和库文件 | 文件很大，*仓库中没有*，需要手动拷贝 |
| main.cpp       | C++源代码文件            | cpp格式，主程序入口                 |
| .clang-format  | 格式化配置               |                                    |
| .gitignore     |                          |                                    |
| config.toml    | 程序运行时配置文件       |                                    |
| env.sh         | 环境配置脚本             |                                    |
| CMakeLists.txt | 编译脚本                 |                                    |
| README.md      | 说明文档                 |                                    |

# 环境配置方法

1. 配置科学上网环境，保证能访问GitHub
2. 修改Ubuntu软件源，改成速度快的源
3. 执行env.sh脚本安装需要的软件包：`sudo ./env.sh`
4. 配置相机SDK，附带的`sdk.zip`中有
    1. 安装Dahua SDK：提供了可执行文件，直接执行即可 PS:Dahua SDK可能会有更新，经常上网搜一下
    2. 安装MindVision SDK：将mindvision库全部文件拷贝到/opt/mindvision/下面
5. 拷贝tensorflow头文件和库文件

---

History:

# 7.15 交接

1. 添加说明文档

# 5.25 分区赛

## 比赛录像

链接：https://pan.baidu.com/s/1v0JMzAExDN5rjNb27RgtRg 
提取码：ult8 
复制这段内容后打开百度网盘手机App，操作更方便哦

# 4.20任务：

1. 对接电控调零飘、底盘pid、自瞄pid，负责人：

- 新步兵：tzx

- 英雄：lxc

- 哨兵：wyx

2. 装甲板图像数据集：tzx

3. Socket远程调试：wyx

4. 研究反小陀螺：wqt

5. 研究数字分类器：lxc

6. 风车负责人：zzh，ttx

## 待分配

1. 装甲板

- 机器学习调参：tzx

2. 击打策略：优先打大装甲板

3. 基地自瞄

4. 血条识别

# 近期工作

- 目标匹配算法 - DEEPSORT模型
- 风车代码测试

