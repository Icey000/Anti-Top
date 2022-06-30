# Anti-Top
关于反小陀螺的一个方法  
位于Anti-Top/zero/include/ArmorDetector/AntiTop.hpp

```
int unidentify_factor = 0;          //未识别到系数
int anti_top = 0;                   //陀螺系数
int continue_identify = 0;          //持续识别系数
```
反小陀螺流程图如下：
![image](https://user-images.githubusercontent.com/89527420/176666222-f54db0af-0f4b-4dbc-acfa-2216933cf904.png)

参考了博客：https://blog.csdn.net/weixin_42754478/article/details/108159529

# 前哨站

前哨站的击打旋转装甲板原理和Anti-Top类似，不过由于前哨站是三个装甲板因此做了小的修改，

位于Anti-Top/zero/include/ArmorDetector/Outpostn.hpp

