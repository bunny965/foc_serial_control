#Simple_FOC
##before start
在主控上搭载simplefoc时使用此库，在开发过程中主要参考官方文档：

https://docs.simplefoc.com/docs_chinese

---
github上有许多例程，使用**arduino ide**时可直接使用，注意按照硬件修改一些初始化参数：
以esp32lolin32lite为主控时引脚配置：
```
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26,27,14,12);
```
foc控制模式：svpwm和正弦pwm控制
 //FOC模型选择
``` 
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
###串口通信：
**project3.0**测试代码说明
串口发送：
S：
>？

(询问有什么指令可用,串口将列出可用指令，如下面的M（请忽略aaa及into等，仅做测试用））
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

R:
>aaa
intorun
intomotion
Status: 1
intomotion
Status: 1

S:MC1
R:aaa
intorun
intomotion
Motion:vel
intomotion
Motion:vel

S:M40 （控制速度）
R:aaa
intorun
intomotion
intotarget
set the target
Target: 40.000
intomotion
intotarget
set the target
Target: 40.000

aaa
intorun
intotscalar
10.000


在loop里添加该函数用于监视，但会影响性能
// 监视函数向串行终端输出电机变量的监控
  motor.monitor();

电机控制：（*bldcmotor.c*内）


  //运动控制模式设置
  motor.controller = MotionControlType::angle;
  motor1.controller = MotionControlType::angle;
如果要一起控制，可直接全局修改target值（指通过串口)
command.scalar(&target_velocity, cmd);
add命令里的回调也只需合并成一次
Eg：
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
command.add('T', doTarget, "target velocity");
.....
......
......
motor.move(target_angle);
command.run();

如果要分开控制：
void doMotor1(char* cmd){ command.motor(&motor, cmd); }
void doMotor2(char* cmd){ command.motor(&motor1, cmd); }
command.add('A', doMotor1, "motor 1");
command.add('B', doMotor2, "motor 2");
.........
motor.move();
motor1.move();
command.run();


S:?(列出可选命令）
R:A:motor 1
B:motor 2
S:A3 （控制速度）
R:aaa
intorun
intomoto
intotarget
set the target
Target: 3.000
S:B4 （控制速度）
R:aaa
intorun
intomoto
intotarget
set the target
Target: 4.000
（通过串口控制位置同理，其实foc控制的目标值可从串口数据解析得，```commander.scalar(&variable,cmd)```函数即把串口接受的解析后传给variable（详细见*cummunication*里的*command*文件），改变
每次重启当前position会变回0
如果每次loop内的时间太短，move函数还未执行到target值又被重置了target值，请保障move实现traget有足够的时间
互控时，5的数值是比较合适的
  	motor.move( 5*(motor1.shaft_angle - motor.shaft_angle));
  	motor1.move( 5*(motor.shaft_angle - motor1.shaft_angle));




在闭环控制project中，使用此项目提供的接口用于串口控制电机，通讯协议：

Macro
Spacing //maix postion change value你可通过修改此参数更改限位（相对位置）

command：
S:s/m//i/o
///mode control command
//s:come into control velocity mode 
//m:come into control position mode
//2:come into control 
//3:monitor

//in this progress,parameter p is a absolute value,remember to change relative value into absolute value 
S:A[v/p]
///v:motor1 target velocity
//P:motor1 target position

S:B[v/p]
///v:motor2 target velocity
//P:motor2 target position

S:1
///break to change mode
//brief:if you use this command,mode control comand is expected to input again

S:s
R:control velocity

S:M
//monitoring,根据设定频率输出监控数据

S:A3

S:1
R:end monitor
此项目内对于*力矩*/*角度*/*速度*控制电机只能全局设置，所以建议
//不再输出监控信息



（在文件夹内包含一个用户测试的脚本*serialtest.py*，可自行更改用于测试）
在速度控制时请不要随意执行模式切换指令，先按1退出操作后再切换mode，



可在此处更改限位值 注：**spacing** 为相对值

注：*command*文件里引出了很多控制接口，但需通过回调函数引出到用户串口控制
如：pid函数：
void onPid(char* cmd){ commander.pid(&pid,cmd); }
commander.add('C',onPid,"my pid");

(添加此回调函数在main)
记得初始化相应的操作对象
如
电机初始化：
BLDCMotor motor1 = BLDCMotor(7);

串口通讯:
***S***：CP          # 获取 P 增益
***R***: P:1.0
***S***：CD0.05      # 设置 D 增益
***R***: D: 0.05
注：```Serial.print(sensor.getAngle());```以及```motor.monitor();```都能用于获取相关数据，但会影响控制的主循环，为了在主循环函数中顺着```move()```函数调用，```motor.monitor() ```
默认情况下，监控的变量为**target**,**voltage.q**,**velocity**,**angle**。
当然，如果要使用```monitor()```而尽量减少对主循环的影响，可降低采样率```motor.monitor_downsample = 100;```(默认值为10),此值表示每100次执行monitor函数才有一次输出，

除此之外也