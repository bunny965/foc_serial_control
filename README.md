#Simple_FOC

此包包含5个项目文件夹及一个串口测试脚本，这些项目是基于simplefoc库及参考官方文档进行开发，主要是为外界通过串口控制提供api,根据指令自由控制机,您也可以自定义控制命令，下面描述了如何根据需求添加命令，在上层，您无需理会如何实现，串口测试脚本为上层自由控制提供了一个dmo

---
##before start
在主控上搭载simplefoc时使用此库，在开发过程中主要参考官方文档：

https://docs.simplefoc.com/docs_chinese

---
github上有许多例程，使用**arduino ide**时可直接使用，注意按照硬件修改一些初始化参数：
以esp32lolin32lite为主控时引脚配置：
```c++
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26,27,14,12);
```
foc控制模式：svpwm和正弦pwm控制
 //FOC模型选择
``` c++
motor.foc_modulation = FOCModulationType::SinePWM;//正弦pwm
motor1.foc_modulation = FOCModulationType::SpaceVectorPWM; //svpwm
```
其他foc配置同理
其他的按示例的设置可以

具体操作步骤：
开环测试：
[(154条消息) Makerbase SimpleFOC 第一课 入门配置_Makerbase_mks的博客-CSDN博客](https://blog.csdn.net/gjy_skyblue/article/details/115353908)
传感器测试：
[(154条消息) Makerbase SimpleFOC 第二课 基本测试_Makerbase_mks的博客-CSDN博客_simplefoc 电流检测b相反转](https://blog.csdn.net/gjy_skyblue/article/details/115390824?spm=1001.2101.3001.6650.10&utm_medium=distribute.pc_relevant.none-task-blog-2~default~BlogCommendFromBaidu~Rate-10-115390824-blog-115353908.pc_relevant_3mothn_strategy_and_data_recovery&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2~default~BlogCommendFromBaidu~Rate-10-115390824-blog-115353908.pc_relevant_3mothn_strategy_and_data_recovery&utm_relevant_index=11)（含闭环控制）

固件提供了监控显示电机的状态以及实时了解到电机每时每刻的状态参数的 api，更多细节：https://www.cnblogs.com/guohaomeng/p/15872176.html

*tips*：
*   I2c模式下，date线连接不稳定会导致串口输出固定位置和0速度）
*   Simplefoc使用2.1.1版本时不会出现不支持esp32的情况，
Simplefoc改成2.2.1版本时出现不支持esp32某些库的情况:若用最新版foc库，下载esp32 6.0版本则不会报错
Eg:lolin32.......mcu.cpp not such file 之类的报错是由于esp6.0以下版本的package缺乏此包，请更新esp32软件包
更新arduino和platformio的esp软件包：[安装 — Arduino-ESP32 2.0.6 文档 (espressif.com)](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)

---
##develop

###电机控制：

<details>
<summary><b>电机控制代码说明</b></summary>
（*bldcmotor.c*内）


  //运动控制模式设置
```c++
motor.controller = MotionControlType::angle;
motor1.controller = MotionControlType::angle;
```
如果要双电机一起控制，可直接全局修改target值（指通过串口),add命令里的回调也只需合并成一次:```command.scalar(&target_velocity, cmd);```

Eg：
```c++
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
command.add('T', doTarget, "target velocity");
.....
......
......
motor.move(target_angle);
command.run();
```
如果要分开控制：
```c++
void doMotor1(char* cmd){ command.motor(&motor, cmd); }
void doMotor2(char* cmd){ command.motor(&motor1, cmd); }
command.add('A', doMotor1, "motor 1");
command.add('B', doMotor2, "motor 2");
.........
motor.move();
motor1.move();
command.run();
```

</details>
---
###串口通信：
#####project introduction

<details>
<summary><b>project*3.0*测试代码说明</b></summary>

<br/>

串口发送：
S：
>？

(询问有什么指令可用,串口将列出可用指令，如下面的M即为可用指令（请忽略aaa及into等:joy:，仅做测试用））
R：
>aaa 
intorun
M:motion control

S:
>M
R:
>aaa
intorun
intomotion
intotarget
intomotion
intotarget

S:
>ME0 

（停用电机）

R:
>aaa
intorun
intomotion
Status: 0
intomotion
Status: 0

S:
>ME1

（启用电机）

R:
>aaa
intorun
intomotion
Status: 1
intomotion
Status: 1

S:
>MC1

R:
>aaa
intorun
intomotion
Motion:vel
intomotion
Motion:vel

S:
>M40 

（控制速度）

R:
>aaa
intorun
intomotion
intotarget
set the target
Target: 40.000
intomotion
intotarget
set the target
Target: 40.000
</details>






#####project introduction

<details>
<summary><b>闭环速度控制 project说明</b></summary>
在给出的project：*闭环速度控制*中，使用以下命令进行串口控制

S:
>?

(列出可选命令）
R:
>A:motor 1
B:motor 2

S:
>A3 

（控制电机1速度）

R:
>aaa
intorun
intomoto
intotarget
set the target
Target: 3.000

S:
>B4 

（控制电机2速度）
R:
>aaa
intorun
intomoto
intotarget
set the target
Target: 4.000

*tips*：
+ （通过串口控制位置同理，其实foc控制的目标值可从串口数据解析得，```commander.scalar(&variable,cmd)```函数即把串口接受的解析后传给variable（详细见*cummunication*里的*command*文件），改变
+ 每次重启当前position会变回0
+ 互控时，5的数值是比较合适的
    ```c++
    motor.move( 5*(motor1.shaft_angle - motor.shaft_angle));
    motor1.move( 5*(motor.shaft_angle - motor1.shaft_angle));
    ```

</details>
#####project introduction
<details>

<summary><b>串口控制1.0/2.0 project说明</b></summary>

在project：*串口控制1.0*中，使用此项目提供的接口用于串口控制电机，通讯协议：(*2.0*添加了monitor命令，在某种控制模式下按C进入)

**Macro**
```c++
Spacing //maix postion change value您可通过修改此参数更改限位（相对位置）
```
*tip:*
```c++
    if((target - shaft_angle) >= spacing ) target = shaft_angle +  spacing; //limit motor_angle change value限制位置改变值(spacing为相对值)
    shaft_angle_sp = target;
```
可在此处(*BLCDmotor.cpp*内)更改限位值 注：***spacing*** 为相对值

**command：**

S:
>s/p/t/o

mode control command
s:come into control velocity mode 
p:come into control position mode
t:monitor
*tip*：in this progress,parameter p is a absolute value,remember to change relative value into absolute value 

S:
>A[v/p]

v:motor1 target velocity
P:motor1 target position

S:
>B[v/p]

v:motor2 target velocity
P:motor2 target position

S:
>1

break to change mode
brief:if you use this command,mode control comand is expected to input again

S:
>s

R:
>control velocity

S:
>C

monitoring,根据设定频率输出监控数据

S:

>A3

首次控制电机请重复8次，首次退出命令也需输入10次后正常
*tip:*首次在监控模式下进行串口控制有个bug，前8次命令会无效，直到第九次命令有效后才能正常控制
R:
>

S:
>1

R:
>end monitor

//不再输出监控信息,退至模式选择

*tip：*在文件夹内包含一个用户测试的脚本*serialtest.py*，可自行更改用于测试）
在速度控制时请不要随意执行模式切换指令，先按1退出操作后再切换mode，


在2.0版本之前有一个问题：退出*move*循环后切换为位置模式，位置若不处于预期值会一直旋转直到回到预期值，**3.0**更新了此操作，再次进入*move*循环时
重置**shaft_angle**的值，此功能位于函数```reset_target(FOCMotor* motor);```中,但```move()```函数内的
```c++
shaft_angle = shaftAngle(); // read value even if motor is disabled to keep the monitoring updated but not in openloop mode
```
 会导致**shaft_angle**被置为读取到的位置值,其值由**sensor_direction*LPF_angle(sensor->getAngle()) - sensor_offset**得到，（**sensor_offset**为零漂）
 如果要其退出后再进入由于**shaft_angle**远离**shaft_angle_sp**导致进入**move**后会一直往**shaft_angle_sp**靠近，可将**shaft_angle_sp**设为目前的**shaft_angle**，
 若要第n次进入时重新使初始位置即起始位置，可保存切换模式前(第n-1次)的位置**shaft_angle_bef=shaft_angle**,再在串口命令解析函数后的```target()```加上**shaft_angle_bef**
 ```c++
 void Commander::target(FOCMotor* motor,  char* user_cmd, char* separator){
  ...
  switch(motor->controller){
    ...
    case MotionControlType::angle:
      pos= atof(strtok (user_cmd, separator));
      pos += shaft_angle_bef;
      motor->target = pos;
  }
 }
 ```
 但不建议这么做，因为不清楚除了在**move()**中用到**target**，是否在其他地方需要，则执行其他功能函数时可能受影响

采用置0式使再次进入位置控制时当前位置即0位置：或可在**BLDCMotor.cpp**内的**move()**中，**shaft_angle = shaftAngle();** 前添加标志判断是否为切换模式的第一次进入**move()**，如果是，不获取底层sensor得到的位置，直接将**shaft_angle**、**target**、**shaft_angle_sp**设为0，但直接操作shaftAngle可能会使其丢失从开机到目前的旋转角度（或许可以从sensor再次读取）
最粗暴的方法：在每次退出后输入**p**指令时直接init电机可直接全员置0
推荐仅执行```reset_target()```函数，仍然使用绝对角度来position控制
</details>

#####project introduction
<details>
<summary><b>add_command project说明</b></summary>
project：addcommand给出了添加可用命令的demo
*command.cpp*文件里引出了很多控制接口，但需通过回调函数引出到用户串口控制
如:pid函数：
```c++
void onPid(char* cmd){ commander.pid(&pid,cmd); }
commander.add('C',onPid,"my pid");
```
(添加此回调函数在main)
记得初始化相应的操作对象
如:电机初始化：
```c++
BLDCMotor motor1 = BLDCMotor(7);
```
串口通讯:
S：
>CP 

获取 P 增益

R:
> P:1.0

S：
>CD0.05

设置 D 增益

R:
>D: 0.05

*tip：*
```Serial.print(sensor.getAngle());```以及```motor.monitor();```都能用于获取相关数据，但会影响控制的主循环，为了在主循环函数中顺着```move()```函数调用，```motor.monitor() ```,如果要使用```monitor()```为尽量减少对主循环的影响，可降低采样率```motor.monitor_downsample = 1000;```(默认值为10),此值表示每100次执行monitor函数才有一次输出。
默认情况下，监控的变量为**target**,**voltage.q**,**velocity**,**angle**，(即对应下面的宏 **_MON_TARGET**）可在初始化时设置
```c++
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
  motor.monitor_downsample = 1200;
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
  motor1.monitor_downsample = 1200;
```
</details>