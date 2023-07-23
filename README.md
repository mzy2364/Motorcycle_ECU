# Motorcycle_ECU

单缸摩托车发送机控制单元

<img src='Document\Image\ECU01.png'></img>

<img src='Document\Image\PCBA01.png'></img>



### 支持功能

曲轴信号识别

点火线圈控制

喷油器控制

怠速电机控制

CAN 通讯

转速信号输出 / PWM输出

IgnON、Engine Stop 等开关识别

节气门位置传感器、进气歧管压力传感器、空气温度传感器、缸体温度传感器、氧传感器等

氧传感器加热

MIL 灯控制、继电器输出、燃油泵控制、怠速阀控制



### 硬件结构

<img src='Document\Image\硬件框图01.png'> </img>



### 软件功能

仅提供支持发动机运转的基本功能，包括：

曲轴位置识别

喷油角度和脉宽计算

点火角度和蓄能时间计算

根据曲轴位置控制点火和喷油

根据进气歧管压力传感器识别冲程

CAN通讯，关键信息发送，基于 CAN 的简单标定

转速脉冲信号输出

燃油泵驱动





### 参考资料

https://www.nxp.com.cn/applications/automotive/electrification-and-powertrain/motorcycle-engine-control-unit-ecu-and-small-engine-control:MOTORCYCLE-ECU-SMALL-ENGINE-CONTROL

https://www.st.com/zh/applications/powertrain-for-ice/single-cylinder-motorcycle-engine.html

https://www.infineon.com/cms/cn/applications/automotive/powertrain-systems/small-1-cylinder-combustion-engine-solution/

