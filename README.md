# View
SCNU 2024电控组转正考核任务
## 串口的使用
基本要求：使用STM32F系列单片机，搭配USB转TTL（USB转串口）模块、上位机的串口调试助手等应用，实现单片机与上位机的通信。

<details><summary>要求细则<em><b> </b> </em></summary>

 - 单片机端不限制展示信息的形式，甚至可以在Debug界面展示
- 实现不定长数据接收，并显示本次接收的信息的长度
- 采用非轮询等占用单片机资源的方法
</details>
<!--调整图片大小并居中显示-->
<center class ='img'>
<img src="./gif/串口发送不定长数据.gif"  width = 80%>
</center>


## 陀螺仪的使用
<details><summary>要求细则<em><b> </b> </em></summary>

- 读取到全部六项信息（Pitch, yaw, roll角度， x, y, z加速度）
- 采用任何形式的滤波算法且效果非负面，计10分
- 在机器人一般工况下每分钟x轴或y轴角度偏移不超过10度

## CAN总线电机的使用

