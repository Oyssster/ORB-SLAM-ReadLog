# ORB-SLAM-ReadLog
The ORB-SLAM learning Notes

## 1.ORB-SLAM概述

### 1.1 ORB-SLAM贡献及系统框架

![System](.\MarkdownPhoto\System.png)

ORB-SLAM主要分为Tracking, LocalMapping和LoopClosure三个线程进行，三个线程分别放在三个文件中:Tracking.cpp, LocalMapping.cpp和LoopClosing.cpp。论文框架如下图所示：

![Architecture](.\MarkdownPhoto\Architecture.png)

### 1.2 Tracking线程

主要工作：从图像中提取ORB特征，与上一帧匹配进行位姿估计，或者通过全局重定位初始化位姿，然后跟踪已有局部地图，进行Local BA优化位姿，然后在根据一些规则确定新的关键帧。

### 1.3 LocalMapping线程

主要工作：完成局部地图构建，包括：关键帧插入，验证新生成的地图点并进行筛选，然后确定地图点并进行Local BA，最后对插入的关键帧进行筛选，去除多余的关键帧。

### 1.4 LoopClosing线程

主要工作：1.使用BOW进行回环检测，人后通过Sim3算法计算相似变换。2.回环校正，融合闭环和Essential Graph进行图优化。