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

