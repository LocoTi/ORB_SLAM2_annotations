# ORB-SLAM2代码解析

参考：[SLAM_-CSDN博客](https://blog.csdn.net/moyu123456789/category_8945611.html)

## 整体框架

![img](%E5%9B%BE%E7%89%87%E5%BA%93/20190515203756572.png)



## 系统初始化

mono_kitti.cc中main函数里有创建System对象的语句：

```
ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
```



System构造函数：

```

/**
 * strVocFile 为词典文件
 * strSettingsFile 为设置配置文件
 * sensor sensor类型，单目、双目和RGBD
 * bUseViewer 是否进行相机位姿和图片帧显示
*/
System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer)
```

1）读取ORB字典，为后期的回环检测做准备；

2）创建关键帧数据库KeyFrameDatabase，用于存放关键帧相关数据；

3）初始化Tracking线程。其实Tracking线程是在main线程中运行的，可以简单的认为main线程就是Tracking线程；

4）初始化并启动LocalMapping线程；

5）初始化并启动LoopClosing线程；

6）初始化并启动窗口显示线程mptViewer；

7）在各个线程之间分配资源，方便线程彼此之间的数据交互。


## Tracking线程

Tracking线程的输入为图片帧，然后根据输入的Frame来进行四个步骤：

1）ORB特征提取；

2）从上一帧进行初始化位姿估计或者进行重定位；

3）跟踪局部地图；

4）新的关键帧生成。

Tracking线程的输出为新生成的关键帧，并将新生成的关键帧传给LocalMapping线程继续后边的工作。

### orb特征点提取及匹配

[ORB-SLAM2特征点匹配代码分析_heshaofeng2ly的博客-CSDN博客](https://blog.csdn.net/heshaofeng2ly/article/details/107992564)




### 单目初始化
单应矩阵，基础矩阵，RANSAC原理：

单应矩阵，基础矩阵，RANSAC代码：

[ORBSLAM2单应矩阵计算及代码分析_heshaofeng2ly的博客-CSDN博客](https://blog.csdn.net/heshaofeng2ly/article/details/107999895)



单应矩阵/基础矩阵,求解R,t ：
https://blog.csdn.net/qq_37708045/article/details/103038192



### 重定位



### PnP

RANSAC算法

![img](%E5%9B%BE%E8%A1%A8%E5%BA%93/20191112170136340.png)

### 跟踪局部地图




### 关键帧生成





## LocalMapping线程

1）将关键帧插入地图中（对应KeyFrame Insertioin）

2）剔除地图中不符合要求的MapPoint（对应Recent MapPoints Culling）

3）创建MapPoint（对应New Points Creation）

4）局部BA优化（对应Local BA）

5）剔除关键帧（对应Local KeyFrames Culling）

6）冗余地图点的融合



### 1）将关键帧插入地图中（对应KeyFrame Insertioin）

当有新的关键帧传入LocalMapping线程时，要将该关键帧插入到地图当中，所有的关键帧连接起来，我们就可以清晰的看到相机的运动轨迹



### 2）剔除地图中不符合要求的MapPoint（对应Recent MapPoints Culling）



### 3）创建MapPoint（对应New Points Creation）



### 4）局部BA优化（对应Local BA）



### 5）剔除关键帧（对应Local KeyFrames Culling）



### 6）冗余地图点的融合





## LoopClosing线程

1）闭环检测

2）计算sim3

3）闭环校正



![img](%E5%9B%BE%E8%A1%A8%E5%BA%93/2019061216255872.png)



## 优化

### 位姿优化PoseOptimization

只对当前帧的位姿进行优化，而把地图点的位姿当成固定值，所以在该函数内定义的顶点只有一种：

```
g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));//初始迭代值x0
vSE3->setId(0);
vSE3->setFixed(false);//是要优化的变量，不是固定值
optimizer.addVertex(vSE3);//添加顶点,相机位姿用李代数表示
```

上面函数设置的初始迭代值如下：

在TrackReferenceKeyFrame函数中，初始位姿设置为上一帧的位姿：

```cpp
mCurrentFrame.SetPose(mLastFrame.mTcw);
```

在TrackWithMotionModel函数中，初始位姿设置为上一帧的位姿*匀速运动模型：

```cpp
 mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
```

### 闭环处的Sim3位姿优化

当检测到闭环时，闭环连接的两个关键帧的位姿需要通过Sim3优化（以使得其尺度一致）。优化求解两帧之间的相似变换矩阵，使得二维对应点（feature）的投影误差最小。

如下图所示，Pos6和Pos2为一个可能的闭环。通过$(u_{4,2},v_{4,2})$和$(u_{4,6},v_{4,6})$之间的投影误差来优化$S_{6,2}$。

<img src="%E5%9B%BE%E8%A1%A8%E5%BA%93/879417-20160529224957272-216970690.png" alt="img" style="zoom:50%;" />

### 局部优化

https://www.cnblogs.com/luyb/p/5447497.html

https://zhuanlan.zhihu.com/p/37843131

局部BA优化，优化的是局部关键帧的位姿和这些局部关键帧可以观测到的地图点的3D地图点。LocalMapping线程中优化的就是当前关键帧和其共视关键帧、以及这些关键帧能够观测到的地图点之间的连接关系。局部关键帧有两种帧，1、当前帧及当前帧有共视关系的关键帧(一级相连)，也称作相连关键帧或一级共视帧，他们观测到的地图点，称为局部地图点；2、能被局部地图点观测到，但不属于局部关键帧的关键帧(二级相连)，这些二级相连关键帧在局部BA优化时不优化，那些能观测到这些地图点的关键帧，二级共视帧。

其中1参与优化，2不参与优化只给定约束

[路游侠-ORB-SLAM（五）优化](https://link.zhihu.com/?target=http%3A//www.cnblogs.com/luyb/p/5447497.html)中的图片能够更清晰的理解：当前关键帧是Pos3，参与优化的是Pos3,Pos2,Pos1,X1,X2，其中Pos1就属于保持不变的定点。

<img src="%E5%9B%BE%E8%A1%A8%E5%BA%93/879417-20160529210611194-1582716555.png" alt="img" style="zoom:50%;" />

```
//Pos2,Pos3关键帧
g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
vSE3->setId(pKFi->mnId);
vSE3->setFixed(pKFi->mnId==0);//第0帧是固定值，也就是初始化的那一帧
optimizer.addVertex(vSE3);

//Pos1关键帧
g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
vSE3->setId(pKFi->mnId);
vSE3->setFixed(true);//固定值
optimizer.addVertex(vSE3);

//X1,X2地图点
g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
Point->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));//在RGBD中，是直接根据深度值计算的
int id = pMP->mnId+maxKFid+1;//跟着关键帧的点序号继续往下排
vPoint->setId(id);
vPoint->setMarginalized(true);//把特征点设置为marginalize，详解见下文
optimizer.addVertex(vPoint);
```

上面程序中由于同时要优化求解关键帧位姿和地图点位姿，所以对地图点setMarginalized，称为边缘化，参考十四讲9.2



### 全局优化

在全局优化中，所有的关键帧（除了第一帧）和三维点都参与优化。

<img src="%E5%9B%BE%E8%A1%A8%E5%BA%93/879417-20160529213326803-2127938983.png" alt="img" style="zoom:50%;" />