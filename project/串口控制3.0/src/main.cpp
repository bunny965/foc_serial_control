/*
MKS DUAL FOC 闭环位置控制例程 测试库：SimpleFOC 2.1.1 测试硬件：MKS DUAL FOC V3.1
在串口窗口中输入：T+位置，就可以使得两个电机闭环转动
比如让两个电机都转动180°，则输入其弧度制：T3.14
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字
程序默认设置的供电电压为 12V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
默认PID针对的电机是 2804云台电机 ，使用自己的电机需要修改PID参数，才能实现更好效果
*/

#include <SimpleFOC.h>
#include <stdlib.h>
#include <vector>
using namespace std;

#define vctrl_mode 115//即串口接收为s时进入速度控制模式
#define pctrl_mode 112//串口输入为p时进入位置控制模式
#define interctrl_mode 105//串口输入为i时进入互控模式
#define monitor_mode 109//串口输入为m时可monitor
#define torctrl_mode 116//串口输入为t时进入力矩控制
#define other_command 111//开发模式，试行

// typedef struct tagPoint Point
// Point oPoint1={100,100,0};
// Point oPoint2;


extern int flag;
unsigned int cnt = 0;
// extern int cnt;
int serial_read;
void feedback(unsigned int clock,char text[]);
void ddd(unsigned int vlock);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//电机参数
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);
BLDCMotor motor1 = BLDCMotor(7);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(26,27,14,12);

//命令设置
float target_velocity = 0;
Commander command = Commander(Serial);
//void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void monitor_2motor();
void doMotor1(char* cmd){ command.motor(&motor, cmd); } //回调函数
void doMotor2(char* cmd){ command.motor(&motor1, cmd); }
void domonitor(char* cmd){monitor_2motor();}//motor.monitor()将会影响执行性能，降低FOC算法的采样频率
void domonitoring(char* cmd){motor.useMonitoring(Serial);}
//set start position
float angle_test_1=0;
float angle_test_0=0;


void setup() {
  //set motor flag
  motor1.motor_flag = 2; 
  motor.motor_flag = 1; 

  I2Cone.begin(19,18, 400000); 
  I2Ctwo.begin(23,5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  //连接motor对象与传感器对象
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //供电电压设置 [V]
  driver.voltage_power_supply = 12;
  driver.init();

  driver1.voltage_power_supply = 12;
  driver1.init();
  //连接电机和driver对象
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  //FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //运动控制模式设置
  motor.controller = MotionControlType::angle;
  motor1.controller = MotionControlType::angle;

  //速度PI环设置
  motor.PID_velocity.P = 0.1;
  motor1.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1;
  motor1.PID_velocity.I = 1;
  //角度P环设置 
  motor.P_angle.P = 20;
  motor1.P_angle.P = 20;
  //最大电机限制电机
  motor.voltage_limit = 1;
  motor1.voltage_limit = 1;
  
  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //设置最大速度限制
  motor.velocity_limit = 20;
  motor1.velocity_limit = 20;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  
  //初始化电机
  motor.init();
  motor1.init();
  //初始化 FOC
  motor.initFOC();
  motor1.initFOC();
  //初始化监视器显示
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
  motor.monitor_downsample = 2000;//改变
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
  motor1.monitor_downsample = 2000;//改变

  //添加命令
  command.add('A', doMotor1, "motor 1");
  command.add('B', doMotor2, "motor 2");
  command.add('C', domonitor, "do monitor");
  // command.add('M', domonitoring, "do monitor");
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  

}



void loop() {
  while(1){
    serial_read = Serial.read();
    feedback(120000,"choose mode:s/p/i/t");
    // Serial.println(serial_read);
    //don't ask why not use 'switch'!
    if(serial_read == vctrl_mode){
      Serial.println("control voltage");
      motor.controller = MotionControlType::velocity;
      motor1.controller = MotionControlType::velocity;
      motor.target = 0;
      motor1.target = 0;
      break;
    }

    else if(serial_read == pctrl_mode){
      Serial.println("control position");
      motor.controller = MotionControlType::angle;
      motor1.controller = MotionControlType::angle; 
      command.reset_target(&motor);
      Serial.println(motor.shaft_angle);
      command.reset_target(&motor1);
      break;
    }

    else if(serial_read == torctrl_mode){
      Serial.println("control voltage");
      motor.controller = MotionControlType::velocity;
      motor1.controller = MotionControlType::velocity;
      break;
    }

    else if(serial_read == torctrl_mode){
      Serial.println("control voltage");
      motor.controller = MotionControlType::torque;
      motor1.controller = MotionControlType::torque;
      /*如果需进一步设置此力矩控制是将电流/电压设置为target，
      TorqueControlType::voltage  //电压模式的目标力矩是 1V ， foc_current 或者 dc_current 是 1A
      TorqueControlType::dc_current
      TorqueControlType::foc_current
      */
      break;
    }

    /*
    @breif:开发模式，随意进入监控、速度控制、位置控制，不稳定，在应用时请使用以上命令遵循通讯标准
    */
    else if(serial_read == other_command){
      
    break;
    }

  }
  while(1){
    motor.loopFOC();
    motor1.loopFOC();
    motor.move();
    motor1.move();
    command.run();
    ddd(1000);
    // Serial.println(motor.shaft_angle);
    if(flag == 0){
      Serial.println("change the mode");
      flag = 1;
      break;
    }
  }

}
/**
  @param:null
  @breif:monitor callback function
*/
void monitor_2motor(){
  while(1){
  ddd(2000);
  motor.monitor();
  motor1.monitor();
  motor.loopFOC();
  motor1.loopFOC();
  motor.move();
  motor1.move();
  command.run();
  if(flag == 0){
    Serial.println("end monitor");
    break;
    }
  }
  
}
/**
 * @brief :反馈函数，设置每隔多少次执行此函数后会有一次反馈输出，避免因输出过快导致有用信息在视觉上被覆盖
 * @param clock 间隔次数
 * @param text 反馈内容
 */
void feedback(unsigned int clock,char text[]){
  if(cnt++ < clock) return;
  cnt = 0;
  Serial.println(text);
}
void ddd(unsigned int vlock){
    if(cnt++ < vlock) return;
  cnt = 0;
  Serial.println(motor.shaft_angle);
}
