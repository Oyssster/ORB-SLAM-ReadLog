# ORB-SLAM-ReadLog
The ORB-SLAM learning Notes

## 1.ORB-SLAM概述

### 1.1 ORB-SLAM贡献及系统框架

![System](https://github.com/Oyssster/ORB-SLAM-ReadLog/blob/main/MarkdownPhoto/System.png)

ORB-SLAM主要分为Tracking, LocalMapping和LoopClosure三个线程进行，三个线程分别放在三个文件中:Tracking.cpp, LocalMapping.cpp和LoopClosing.cpp。论文框架如下图所示：

![Architecture](https://github.com/Oyssster/ORB-SLAM-ReadLog/blob/main/MarkdownPhoto/Architecture.png)

ORB-SLAM的主要贡献：

~~~markdown
1. 所有环节使用统一的特征：ORB特征，具有良好的一致性；
2. 在大场景下，仍保持了实时性，同时引入了Covisibility Graph这一先进概念（关键帧为Node，共视点数量为边权）；
3. 回环检测使用了Pose Graph来实现实时全局优化，只优化位姿，不优化3D点，所以效率更高；
4. 因为特征点和描述子的优势，在视角变化过大和光照变化剧烈的情况下，具备良好的鲁棒性；
5. 创新性的开辟了基于模型选择的自动化初始方法，利用一定计算方式在平面模型和非平面模型之间自动选择：计算位姿是使用F矩阵还是H矩阵；
6. 支持单目、双目和RGB-D相机；
~~~

### 1.2 Tracking线程

主要工作：从图像中提取ORB特征，与上一帧匹配进行位姿估计，或者通过全局重定位初始化位姿，然后跟踪已有局部地图，进行Local BA优化位姿，然后在根据一些规则确定新的关键帧。

### 1.3 LocalMapping线程

主要工作：完成局部地图构建，包括：关键帧插入，验证新生成的地图点并进行筛选，然后确定地图点并进行Local BA，最后对插入的关键帧进行筛选，去除多余的关键帧。

### 1.4 LoopClosing线程

主要工作：1.使用BOW进行回环检测，人后通过Sim3算法计算相似变换。2.回环校正，融合闭环和Essential Graph进行图优化。

## 2. 代码模块详解

变量命名规则：

~~~markdown
m: 类成员变量(member)
t: 线程类型变量(thread)
l: list类型变量
n: int类型变量
p: 指针类型变量
b: bool类型变量
v: vector类型变量
s: set类型变量
~~~

<font color =red> Tracking线程位于主线程(main函数)之中，Local Mapping和Loop Closing 线程位于主线程中的系统初始化中。</font>

系统流程图如下所示：

![Procedure](https://github.com/Oyssster/ORB-SLAM-ReadLog/blob/main/MarkdownPhoto/Procedure.png)

### 2.1 Tracking线程

#### 2.1.1 New Keyframe Decision

1. 距离上一次全局重定位超过20帧：保证当前帧足够稳定；
2. Local Mapping线程空闲或者距离上一次关键帧插入超过20帧：<font color = red>其实Local Mapping线程空不空闲不重要，因为新来一帧后会中断当前的Local BA；</font>
3. 当前帧至少跟踪到不少于50个点：当前跟踪状态稳定,<font color = red>防止插入运动过快或者抖动的帧；</font>
4. 当前帧中新点的数量大于一定值：必须要有一定的视角变化；

### 2.2 LocalMapping线程

#### 2.2.1 KeyFrame Insertion

1. 更新共视图：①新增节点（关键帧）②更新边的权重（观测次数可能发生变化）；
2. 更新生成树：与$K_i$相连的相同共视点最多的一个关键帧；
3. 计算新关键帧的BoW，用于加速匹配，快速进行三角化。

#### 2.2.2 Map Points Culling

新增一个地图点后，需要在该点被创建后的连续三帧进行校验，以剔除不满足条件的地图点：

1. 能够在至少25%的预测帧中看到该点；
2. 如果该点创建后，已经过了一个关键帧，则必须从至少在三个关键帧中看到该点；

- [ ] 预测关键帧是啥?
- [ ] 为什么要过了一个关键帧之后才需要在三个关键帧中看到？

### 2.3 LoopClosing

### 2.4 Viewer

