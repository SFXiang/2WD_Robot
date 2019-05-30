# 概述

本功能包用于saturnbot机器人的校正以及性能测试。其中校正主要包括线速度校正、角速度校正、摄像头的校正等等。

# 线速度校正

首先，在机器人的上位机上启动下面的文件：

```
roslaunch base_controller base_calibration.launch
```

接着在你的主机上启动线速度校正脚本：

```
rosrun saturnbot_calibration calibrate_linear.py
```


然后打开rqt_reconfigure:

```
rosrun rqt_reconfigure rqt_reconfigure
```

在rqt_reconfigure窗口中选择calibrate_linear节点（如果发现calibrate_linear不在列表上，点击GUI界面左下角的蓝色刷新按钮）。点击在start_test旁边的勾选框来开始测试（如果机器人没有开始运动，取消选择并重新勾选）。这时saturnbot应该会向前移动大概1米。要得到修正系数，执行以下几步：

- 用卷尺测量并记录下机器人的实际移动距离
- 用实际移动距离除以目标距离，并记录下这个比值
- 回到rqt_reconfigure的GUI界面，用参数odom_linear_scale_correction乘以上一步得到的比值所得的乘积更新这个参数
- 把saturnbot放回卷尺的起始端，在rqt_reconfigure窗口中勾选start_test选框，重复测试
- 不断重复测试，直到你得到满意的结果。在1米的距离中精确到1厘米大概就是足够好的结果了

当你得到最终的修正系数之后 ，你需要用该修正系数更新base_controller功能包下参数文件（包括complex_odom_params.yaml和simple_odom_params.yaml）中的`odom_linear_scale_correction`参数。

# 角速度校正

我们同样需要在机器人的上位机上启动base_calibration.launch文件：

```
roslaunch base_controller base_calibration.launch
```

接着在你的主机上启动角速度校正脚本：

```
roslaunch saturnbot_calibration calibrate_angular.py
```

然后，同样是启动rqt_reconfigure:

```
rosrun rqt_reconfigure rqt_reconfigure
```

同样，我们需要点击start_test复选框来开始测试。该校正节点会使机器人旋转360度，但是角度测量比距离测量麻烦的多，因此建议可以找一个参照物，比如墙、或者桌子。你应该让机器人正朝墙或者桌子，然后大概估计机器人偏离的角度。执行以下步骤即可获得校正系数：

- 如果实际旋转不足360度，目测机器人旋转的角度，并以这个结果来填写rqt_reconfigure窗口中的odom_angular_scale_correction的值。如果机器人看起来旋转了一圈的85%，就输入0.85。如果机器人旋转了一圈又5%，就输入1.05。

- 让机器人正面中点对齐参照物，点击rqt_reconfigure窗口中的start_test复选框来重复测试。

- 在一次测试中，机器人旋转越接近一圈越好。如果不足一圈，就稍微减少参数odom_angular_scale_correction的值后重试。如果超过一圈，就稍微增加这个值后重试。

- 重复上述步骤直到得到满意的结果。

同样，当你得到最终的修正系数之后 ，你需要用该修正系数更新base_controller功能包下参数文件（包括complex_odom_params.yaml和simple_odom_params.yaml）中的`odom_angular_scale_correction`参数。

# 许可证

该功能包使用GPL v3.0协议。
