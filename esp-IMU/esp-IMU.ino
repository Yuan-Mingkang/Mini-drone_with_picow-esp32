#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define I2C_CLK_FREQ 400000 // 400kHz
const u_int8_t IMUAddress = 0x68; //IMU地址

#ifndef APSSID
const char* APSSID = "PicoW";
const char* APPSW = "password";
#endif
#define UDP_PKT_MAX_SIZE 16
#define bat_pin 26
#define diode_vol_drop 0.12
unsigned int localPort = 8888;
char packetBuffer[UDP_PKT_MAX_SIZE + 1];//数据包缓冲区
WiFiUDP Udp;

#define MOT_TOP_LEFT 32
#define MOT_TOP_RIGHT 23
#define MOT_BOTTOM_LEFT 27
#define MOT_BOTTOM_RIGHT 19
#define Green_LED 17
#define Red_LED 16
// #define sensitivity 2



boolean auto_level = true; //自动水平仪开关

//声明全局变量
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte highByte, lowByte;
volatile int receiver_input_channel_1 = 0, receiver_input_channel_2 = 0, receiver_input_channel_3 = 0, receiver_input_channel_4 = 0;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;
int16_t acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust, yaw_level_adjust;

float acc_x, acc_y, acc_z ,hf_acc_x, hf_acc_y, hf_acc_z,acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer,flowtimer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4], acc_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float pid_output_height = 0;
float angle_roll_acc, angle_pitch_acc, angle_roll_gyro, angle_pitch_gyro, angle_yaw_gyro, angle_pitch, angle_roll,angle_yaw;
boolean gyro_angles_set;
float gyro_pitch_mdf,gyro_roll_mdf,gyro_yaw_mdf;
int cnt_kal=0;
float angle_pitch_kal,angle_roll_kal;

typedef struct
{
	float b0;
	float a1;
	float a2;
	float preout;
	float lastout;
}LPF2ndData_t;
LPF2ndData_t lpf_2ndx,lpf_2ndy,lpf_2ndz,lpf_2ndfx,lpf_2ndfy,lpf_2ndax,lpf_2nday,lpf_2ndaz;
//PID
float last_inside_pid_angular_err_roll,last_outside_pid_angle_err_roll,outside_pid_integral_roll,inside_pid_integral_roll;
float last_inside_pid_angular_err_pitch,last_outside_pid_angle_err_pitch,outside_pid_integral_pitch,inside_pid_integral_pitch;
float last_inside_pid_angular_err_yaw,last_outside_pid_angle_err_yaw,outside_pid_integral_yaw,inside_pid_integral_yaw;
float last_outside_pid_angle_err_height,outside_pid_integral_height;
uint16_t aim_height;

int pid_max_roll = 300;
int pid_max_pitch = pid_max_roll;
int pid_max_yaw = 300;
int pid_max_height=200;

float outside_pid_Kp_roll = 0.50;
float outside_pid_Ki_roll = 0.00;
float outside_pid_Kd_roll = 0.45;
float inside_pid_Kp_roll = 0.9;
float inside_pid_Ki_roll = 0.00;
float inside_pid_Kd_roll = 1.8;

// float outside_pid_Kp_pitch = 2.5;
// float outside_pid_Ki_pitch = 0;
// float outside_pid_Kd_pitch = 2.00;
// float inside_pid_Kp_pitch = 2.4;
// float inside_pid_Ki_pitch = 0.008;
// float inside_pid_Kd_pitch = 3.8;

float outside_pid_Kp_yaw = 8.4;
float outside_pid_Ki_yaw = 0.008;
float outside_pid_Kd_yaw = 0;
// float inside_pid_Kp_yaw = 3.0;
// float inside_pid_Ki_yaw = 0;
// float inside_pid_Kd_yaw = 4.5;
float outside_pid_Kp_pitch = outside_pid_Kp_roll;
float outside_pid_Ki_pitch = outside_pid_Ki_roll;
float outside_pid_Kd_pitch = outside_pid_Kd_roll;
float inside_pid_Kp_pitch = inside_pid_Kp_roll;
float inside_pid_Ki_pitch = inside_pid_Ki_roll;
float inside_pid_Kd_pitch = inside_pid_Kd_roll;

float outside_pid_Kp_height = 0.08;
float outside_pid_Ki_height = 0.02;
float outside_pid_Kd_height = 0.005;

int take_off_throttle=0;
int take_off_height = 0;
int pid_height = 0;

#define LIMIT( x,min,max ) ( ((x) < (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *3.14f *(t) ) ) ) *( (in) - (out) ))	//一阶低通滤波
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )

float height;
float fx_limit=5.0, fy_limit=5.0;
float sum_roll=0,sum_pitch=0,speed_roll,speed_pitch;
float gyro_lpf_roll=0,gyro_lpf_pitch=0;
float POS_CONTROL_LIMIT_MAX=50.0f,POS_CONTROL_LIMIT_MIN=50.0f;
float acc_roll=0,acc_pitch=0;
float speed_x, speed_y, acc_speed_x, acc_speed_y;
float acc_g,acc_b;

//设置程序

void setup(){
    Serial.begin(115200);//开启串口
    Wire.setClock(I2C_CLK_FREQ);//设置I2C时钟频率
    Wire.begin();
    ledcSetup(2,500,10); //设置PWM频率为500HZ
    ledcSetup(3,500,10);
    ledcSetup(4,500,10);
    ledcSetup(5,500,10);
    ledcAttachPin(MOT_TOP_LEFT,2);
    ledcAttachPin(MOT_TOP_RIGHT,3);
    ledcAttachPin(MOT_BOTTOM_LEFT,4);
    ledcAttachPin(MOT_BOTTOM_RIGHT,5);
    // ledcWriteRange(1000);//该函数用于改变PWM的值(占空比), 1000对应100%。    
    gyro_address = IMUAddress;      
    // pinMode(MOT_TOP_LEFT, OUTPUT);
    // pinMode(MOT_TOP_RIGHT, OUTPUT);
    // pinMode(MOT_BOTTOM_LEFT, OUTPUT);
    // pinMode(MOT_BOTTOM_RIGHT, OUTPUT);
    // pinMode(LED_BUILTIN, OUTPUT);
    pinMode(Green_LED, OUTPUT);
    pinMode(Red_LED, OUTPUT);
    pinMode(bat_pin,INPUT);
    ledcWrite(MOT_TOP_LEFT, 0);
    ledcWrite(MOT_TOP_RIGHT, 0);
    ledcWrite(MOT_BOTTOM_LEFT, 0);
    ledcWrite(MOT_BOTTOM_RIGHT, 0);
    Serial.print("C3");
    digitalWrite(Green_LED,LOW);
    digitalWrite(Red_LED,LOW);
    // digitalWrite(LED_BUILTIN, HIGH);   
    // float reading = analogRead(bat_pin) * 3.3 / 1023 * 1300 / 1000 + diode_vol_drop;
    // Serial.println(reading);
    // if(reading < 3.75)
    // {
    //   Serial.println("Low Battery");
    //   while(1)
    //   {
    //     digitalWrite(Red_LED, HIGH);
    //   }
    // } 
    set_gyro_registers();
    int cal_num = 500;

    //采集多个陀螺仪数据样本，以便确定陀螺仪的平均偏移量
    for (cal_int = 0; cal_int < cal_num; cal_int ++){
        if(cal_int % 15 == 0){
            // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            digitalWrite(Red_LED, !digitalRead(Red_LED));

        }

        gyro_signalen();//读取陀螺仪输出
        gyro_axis_cal[1] += gyro_axis[1];
        gyro_axis_cal[2] += gyro_axis[2];
        gyro_axis_cal[3] += gyro_axis[3];
        acc_axis_cal[1] += acc_axis[1];
        acc_axis_cal[2] += acc_axis[2];
        acc_axis_cal[3] += acc_axis[3];
        delay(4);//下一个循环之前等待3豪秒
        
    }

    //现在我们有 2000 个测量值，我们需要除以 2000 以获得平均陀螺仪偏移
    gyro_axis_cal[1] /= cal_num;
    gyro_axis_cal[2] /= cal_num;
    gyro_axis_cal[3] /= cal_num;
    acc_axis_cal[1] /= cal_num;
    acc_axis_cal[2] /= cal_num;
    acc_axis_cal[3] /= cal_num;

    // WiFi.mode(WIFI_AP);//设为接入点模式
    WiFi.softAP(APSSID, APPSW); 
    // WiFi.begin(APSSID, APPSW);
    // while(WiFi.status() != WL_CONNECTED){
    //     Serial.print('.');
    //     delay(500);
    // }
    Udp.begin(localPort);
    
    //等待接收器启动，并将油门调至较低位置
    while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
        int packetSize = Udp.parsePacket();
        if(packetSize){
            int n = Udp.read(packetBuffer, UDP_PKT_MAX_SIZE);
            packetBuffer[n] = '\0';
            char ch1[5], ch2[5], ch3[5],ch4[5];
            ch1[4] = '\0'; ch2[4] = '\0'; ch3[4] = '\0'; ch4[4] = '\0';
            for(int i=0; i<4; i++) {
                ch4[i] = packetBuffer[i];
                ch3[i] = packetBuffer[i+4];
                ch1[i] = packetBuffer[i+8];
                ch2[i] = packetBuffer[i+12];     
            }
            receiver_input_channel_1 = atoi(ch1); // ROLL
            receiver_input_channel_2 = atoi(ch2); // PITCH
            receiver_input_channel_3 = atoi(ch3); // THROTTLE
            receiver_input_channel_4 = atoi(ch4); // YAW

        }
        start ++;
        delay(3);
        if(start == 125){
            // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            digitalWrite(Red_LED, !digitalRead(Red_LED));
            start = 0;
        }
    }
    start = 0;
    // loop_timer = micros();//设置下一个循环的时
    //所有步骤准备完成，关闭led
    // digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(Red_LED, LOW);
    pid_yaw_setpoint = 0;
}
//主程序循环
void loop(){
    // while(WiFi.status() != WL_CONNECTED)
    // {
    //   Serial.println("WiFi断开！！！");
    //   // while(1)
    //   // {
    //   esc_1 -= 1;
    //   esc_2 -= 1;
    //   esc_3 -= 1;
    //   esc_4 -= 1;
    //   if(esc_1 <= 0 || esc_2 <= 0 || esc_3 <= 0 || esc_4 <= 0)
    //   {
    //     esc_1 = 0;
    //     esc_2 = 0;
    //     esc_3 = 0;
    //     esc_4 = 0;
    //   }
    //   analogWrite(MOT_TOP_RIGHT, esc_1);
    //   analogWrite(MOT_BOTTOM_RIGHT, esc_2);
    //   analogWrite(MOT_BOTTOM_LEFT, esc_3);
    //   analogWrite(MOT_TOP_LEFT, esc_4);
    //   // }
    // }
    int packetSize = Udp.parsePacket();
    if(packetSize) {
      int n = Udp.read(packetBuffer, UDP_PKT_MAX_SIZE);
      packetBuffer[n] = '\0';
      char ch1[5], ch2[5], ch3[5], ch4[5];
      ch1[4] = '\0'; ch2[4] = '\0'; ch3[4] = '\0'; ch4[4] = '\0';
      for(int i=0; i<4; i++) {
        ch4[i] = packetBuffer[i];
        ch3[i] = packetBuffer[i+4];
        ch1[i] = packetBuffer[i+8];
        ch2[i] = packetBuffer[i+12];     
      }
      receiver_input_channel_1 = atoi(ch1); // ROLL
      receiver_input_channel_2 = atoi(ch2); // PITCH
      receiver_input_channel_3 = atoi(ch3); // THROTTLE
      receiver_input_channel_4 = atoi(ch4); // YAW
    }

    acc_x = acc_x/8192;
    acc_y = acc_y/8192;
    acc_z = acc_z/8192;

    gyro_roll = gyro_roll/65.5;
    gyro_pitch = gyro_pitch/65.5;
    gyro_yaw = gyro_yaw/65.5;
    // Serial.print(gyro_roll);
    // Serial.print(',');
    // Serial.print(gyro_pitch);
    // Serial.print(',');
    // Serial.print(gyro_yaw);
    // Serial.print(',');
    // Serial.print(acc_x);
    // Serial.print(',');
    // Serial.print(acc_y);
    // Serial.print(',');
    // Serial.print(acc_z);
    // Serial.print('\n');
    // gyro_roll = LPF_2nd(&lpf_2ndx,gyro_roll,0.004,10);
    // gyro_pitch = LPF_2nd(&lpf_2ndy,gyro_pitch,0.004,10);
    // gyro_yaw = LPF_2n0d(&lpf_2ndz,gyro_yaw,0.004,10);
    gyro_yaw_input = (gyro_yaw_input * 0.7) + (gyro_yaw * 0.3);
    // Serial.print(gyro_yaw_input);
    // Serial.print(',');
    double dt = 0.004;
    // double dt = (double)(micros()-loop_timer)/1000;
    // loop_timer = micros();
    // 加速度计角度计算
    // acc_x = LPF_2nd(&lpf_2ndax,acc_x,0.004,10);
    // acc_y = LPF_2nd(&lpf_2nday,acc_y,0.004,10);
    // acc_z = LPF_2nd(&lpf_2ndaz,acc_z,0.004,10);
    acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));//计算总加速度计矢量
    // 防止 asin 函数产生 NaN
    if(abs(acc_y) < acc_total_vector){
        angle_pitch_acc = asin((float)acc_y/acc_total_vector)*57.296;//计算俯仰角
    }
    if(abs(acc_x) < acc_total_vector){
        angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;//计算横滚角
    }
    gyro_roll_input = (gyro_roll_input * 0.7) + (gyro_roll * 0.3);//gyro pid的输入是deg/sec
    gyro_pitch_input = (gyro_pitch_input * 0.7) + (gyro_pitch * 0.3);
    // Serial.print(gyro_roll_input);
    // Serial.print(',');
    angle_pitch_gyro += gyro_pitch_input*dt;
    angle_roll_gyro += gyro_roll_input*dt;
    angle_yaw_gyro += gyro_yaw_input*dt;
    // Serial.print(angle_yaw_gyro);
    // Serial.print('\n');
    if(receiver_input_channel_3<1050)
    {
      angle_pitch_gyro=0;
      angle_roll_gyro=0;
    }
    if(receiver_input_channel_3<1050)
    {
      angle_pitch_acc=0;
      angle_roll_acc=0;
    }
    // Serial.print(angle_roll_gyro);
    // Serial.print(',');
    // Serial.print(angle_roll_acc);
    // Serial.print('\n');
    angle_pitch = angle_pitch_gyro * 0.9996 + angle_pitch_acc * 0.0004; 
    angle_roll = angle_roll_gyro * 0.9996 + angle_roll_acc * 0.0004; 
    // angle_pitch = getAngle(angle_pitch_acc, gyro_pitch, dt);//第1个值为加速度角度，第二个值为陀螺仪角速度，度/s，最终输出角度
    // angle_roll = getAngle(angle_roll_acc, gyro_roll, dt);
    // Serial.print(angle_pitch);
    // Serial.print(',');
    // Serial.print(angle_roll);
    // Serial.print('\n');
    pitch_level_adjust = angle_pitch*2;
    roll_level_adjust = angle_roll*2;
   
    //为了启动电机：油门低，向右偏航（步骤1）
    if(receiver_input_channel_3 < 1100 && receiver_input_channel_4 > 1750)
    {
      start = 1;
      digitalWrite(Green_LED, HIGH);
      digitalWrite(Red_LED, LOW);
    }
    //当偏航杆回到中心位置时，启动发动机（步骤 2）
    if(start == 1 && receiver_input_channel_3 < 1100 && receiver_input_channel_4 < 1650){
        start = 2;
        //当四轴飞行器启动时，将陀螺仪俯仰角设置为等于加速度计俯仰角
        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        gyro_angles_set = true;//设置IMU为启动标志
        //重置 PID 控制器，实现无缓冲启动
        last_inside_pid_angular_err_roll=0;
        last_outside_pid_angle_err_roll=0;
        outside_pid_integral_roll=0;
        inside_pid_integral_roll=0;
        last_inside_pid_angular_err_pitch=0;
        last_outside_pid_angle_err_pitch=0;
        outside_pid_integral_pitch=0;
        inside_pid_integral_pitch=0;
        last_inside_pid_angular_err_yaw=0;
        last_outside_pid_angle_err_yaw=0;
        outside_pid_integral_yaw=0;
        inside_pid_integral_yaw=0;
    }
    
    //停止电机：油门低，偏航左
    if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1350)
    {
      start = 0;
      digitalWrite(Red_LED, HIGH);
      digitalWrite(Green_LED, LOW);
    }
    //以每秒度数为单位的 PID 设定点由滚动接收器输入决定
    pid_roll_setpoint = 0;
    // 为了获得更好的结果，我们需要 16us 的一点死区
    if(receiver_input_channel_1 > 1510)pid_roll_setpoint = 0.4*(receiver_input_channel_1 - 1510);
    else if(receiver_input_channel_1 < 1490)pid_roll_setpoint = 0.4*(receiver_input_channel_1 - 1490);
    pid_roll_setpoint -= roll_level_adjust;
    pid_roll_setpoint /= 1.0;//将 PID 滚动控制器的设定值除以 3，得出角度（度）
    //PID 设定点（以度/秒为单位）由桨距接收器输入确定,在除以 3 的情况下，最大俯仰速率约为每秒 164 度 ( (500-8)/3 = 164 d/s )。
    pid_pitch_setpoint = 0;
    if(receiver_input_channel_2 > 1510)pid_pitch_setpoint = 0.4*(receiver_input_channel_2 - 1510);
    else if(receiver_input_channel_2 < 1490)pid_pitch_setpoint = 0.4*(receiver_input_channel_2 - 1490);
    pid_pitch_setpoint -= pitch_level_adjust;
    pid_pitch_setpoint /= 1.0;
    // pid_yaw_setpoint = 0;
    if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
        if(receiver_input_channel_4 > 1510)
        {
          // pid_yaw_setpoint = (receiver_input_channel_4 - 1510);
          // pid_yaw_setpoint /= 12.0;
          pid_yaw_setpoint += 0.05;
        }
        else if(receiver_input_channel_4 < 1490)
        {
          // pid_yaw_setpoint = (receiver_input_channel_4 - 1490);
          // pid_yaw_setpoint /= 12.0;
          pid_yaw_setpoint -= 0.05;
        }
    }
    throttle = receiver_input_channel_3;
    calculate_pid_roll();
    calculate_pid_pitch();
    calculate_pid_yaw();
    if(start == 2){
        if(throttle > 1800) throttle = 1800;
        esc_1 = throttle + pid_output_pitch - pid_output_roll + 0.2*pid_output_yaw + 2.0*pid_output_height; //Calculate the pulse for esc 1 (front-right - CCW)
        esc_2 = throttle - pid_output_pitch - pid_output_roll - 0.2*pid_output_yaw + 2.0*pid_output_height; //Calculate the pulse for esc 2 (rear-right - CW)
        esc_3 = throttle - pid_output_pitch + pid_output_roll + 0.2*pid_output_yaw + 2.0*pid_output_height; //Calculate the pulse for esc 3 (rear-left - CCW)
        esc_4 = throttle + pid_output_pitch + pid_output_roll - 0.2*pid_output_yaw + 2.0*pid_output_height; //Calculate the pulse for esc 4 (front-left - CW)
        // Serial.print(throttle);
        // Serial.print(",");
        // Serial.print(pid_output_pitch);
        // Serial.print(",");
        // Serial.print(pid_output_roll);
        // Serial.print(",");
        // Serial.print(pid_output_yaw);
        // Serial.print("\n");
        if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
        if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
        if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
        if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

        if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
        if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
        if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
        if(esc_4 > 2000)esc_4 = 2000;      
    }
    else{
        esc_1 = 1000;
        esc_2 = 1000;
        esc_3 = 1000;
        esc_4 = 1000;
    }
    //由于角度计算，循环时间变得非常重要。 如果循环时间是
    //长于或短于4000us，角度计算关闭。 如果修改代码请确保
    //循环时间仍然是4000us，不再是了！ 更多信息请参见
    // Serial.println(micros() - loop_timer);
    if(micros() - loop_timer > 4050)
    {
      digitalWrite(Red_LED, HIGH);
    }
    else
    {
      digitalWrite(Red_LED, LOW);
    }
    //控制电机的所有信息均可用。
    //刷新率为250Hz。 这意味着电调需要每 2 毫秒产生一次脉冲。
    while(micros() - loop_timer < 4000);//等待4000us
    loop_timer = micros();
    
    esc_1 = map(esc_1, 1000, 2000, 0, 1000);
    esc_2 = map(esc_2, 1000, 2000, 0, 1000);
    esc_3 = map(esc_3, 1000, 2000, 0, 1000);
    esc_4 = map(esc_4, 1000, 2000, 0, 1000);
    Serial.print(esc_1);
    Serial.print(",");
    Serial.print(esc_2+1000);
    Serial.print(",");
    Serial.print(esc_3+2000);
    Serial.print(",");
    Serial.print(esc_4+3000);
    Serial.print("\n");
    ledcWrite(MOT_TOP_RIGHT, esc_1);
    ledcWrite(MOT_BOTTOM_RIGHT, esc_2);
    ledcWrite(MOT_BOTTOM_LEFT, esc_3);
    ledcWrite(MOT_TOP_LEFT, esc_4);
    //总有1000us的空闲时间。 因此，让我们做一些有用但非常耗时的事情。
    //获取当前陀螺仪和接收器数据并将其缩放为每秒度数以进行 pid 计算。
    gyro_signalen();
    // float reading = analogRead(bat_pin) * 3.3 / 1023 * 1300 / 1000 + diode_vol_drop;
    // Serial.println(reading);
    // if(reading<3.7)
    // {
    //   digitalWrite(Green_LED, !digitalRead(Green_LED));
    // }
}

//二阶低通滤波
//LPF2ndData_t:滤波器系数结构体；newdata:本次采样值；deltaT:采样周期； Fcut:截止频率;
float LPF_2nd(LPF2ndData_t* lpf_2nd,float newdata,float deltaT,float Fcut)
{
	float a = 1/(2*3.1415926*Fcut*deltaT);
	lpf_2nd->b0 = 1/(a*a+2*a+1);
	lpf_2nd->a1 = (2*a*a+2*a)/(a*a+2*a+1);
	lpf_2nd->a2 = (a*a)/(a*a+2*a+1);
	
	float lpf_2nd_data;
	lpf_2nd_data = newdata*lpf_2nd->b0+lpf_2nd->lastout*lpf_2nd->a1-lpf_2nd->preout*lpf_2nd->a2;
	lpf_2nd->preout = lpf_2nd->lastout;
	lpf_2nd->lastout = lpf_2nd_data;
	
	return lpf_2nd_data;
}

float Q_angle = 0.001f;//加速度计的过程噪声方差
float Q_bias = 0.003f;//处理陀螺仪偏差的噪声方差
float R_measure= 0.03f;//测量噪声方差 - 这实际上是测量噪声的方差
float angle = 0.0f;//卡尔曼滤波器计算出的角度 2x1 状态向量的一部分
float bias = 0.0f;// 卡尔曼滤波器计算出的陀螺仪偏差 2x1 状态向量的一部分
float rate; // 根据速率和计算出的偏差计算出的无偏速率 - 您必须调用 getAngle 来更新速率
float P[2][2] = {{0.0f,0.0f},{0.0f,0.0f}};// 误差协方差矩阵 - 这是一个 2x2 矩阵

float getAngle(float newAngle, float newRate, float dt)
{
  //离散卡尔曼滤波时间更新方程-时间更新（“预测”）
  //更新xhat-预测前方的状态
  //Step 1
  rate = newRate-bias;
  angle += dt * rate;
  // 更新估计误差协方差——提前投影误差协方差
  //Step 2
  P[0][0] += dt*(dt*P[1][1]-P[0][1]-P[1][0]+Q_angle);
  P[0][1] -= dt*P[1][1];
  P[1][0] -= dt*P[1][1];
  P[1][1] += Q_bias*dt;
  // 离散卡尔曼滤波器测量更新方程 - 测量更新（“当前”）
  // 计算卡尔曼增益
  //Step 4
  float S = P[0][0] + R_measure;//估计误差
  //Step 5
  float K[2];//卡尔曼增益
  K[0] = P[0][0]/S;
  K[1] = P[1][0]/S;
  // 计算角度和偏差 - 用测量 zk (newAngle) 更新估计值
  //Step 3
  float y = newAngle-angle;//角度差
  //Step 6
  angle += K[0]*y;
  bias += K[1]*y;
  // 计算估计误差协方差_更新误差协方差
  //Step 7
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

//读取陀螺仪输出的函数
void gyro_signalen(){
    //读取MPU6050
    Wire.beginTransmission(gyro_address);//启动与陀螺仪的通信,开始一次传输数据，发送一个I2C开始字符
    Wire.write(0x3B);//发送数据
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 14);//从陀螺仪请求14个byte
    
    //如果有数据读取UDP数据包
    int packetSize = Udp.parsePacket();//本函数用于检查是否有UDP数据包传入
    if(packetSize){
        int n = Udp.read(packetBuffer, UDP_PKT_MAX_SIZE);//在使用了参数buffer和len调用此函数，函数的返回值情况如下所述:设备没有接收到数据时，返回值为-1,设备接收到数据时，返回值为接收到的数据包的大小（单位是字节）
        packetBuffer[n] = '\0';
        char ch1[5], ch2[5], ch3[5], ch4[5];
        ch1[4] = '\0'; ch2[4] = '\0'; ch3[4] = '\0'; ch4[4] = '\0';
        for(int i=0; i<4; i++){
            ch4[i] = packetBuffer[i];
            ch3[i] = packetBuffer[i+4];
            ch1[i] = packetBuffer[i+8];
            ch2[i] = packetBuffer[i+12];
        }
        receiver_input_channel_1 = atoi(ch1);//roll
        receiver_input_channel_2 = atoi(ch2);//pitch
        receiver_input_channel_3 = atoi(ch3);//throttle
        receiver_input_channel_4 = atoi(ch4);//yaw
    }

    while(Wire.available() < 14);//等候14bytes被接收
    acc_axis[1] = Wire.read()<<8|Wire.read();//将低字节和高字节添加到 acc_x 变量中
    acc_axis[2] = Wire.read()<<8|Wire.read(); 
    acc_axis[3] = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();
    gyro_axis[1] = Wire.read()<<8|Wire.read();
    gyro_axis[2] = Wire.read()<<8|Wire.read();
    gyro_axis[3] = Wire.read()<<8|Wire.read();

    if(cal_int == 2000){
        gyro_axis[1] -= gyro_axis_cal[1];//校准之后再进行补偿
        gyro_axis[2] -= gyro_axis_cal[2];
        gyro_axis[3] -= gyro_axis_cal[3];
        acc_axis[1] -= acc_axis_cal[1];//校准之后再进行补偿
        acc_axis[2] -= acc_axis_cal[2];
        acc_axis[3] -= acc_axis_cal[3];
    }
    gyro_roll = gyro_axis[2];//将 gyro_roll 设置为存储在 EEPROM 中的正确轴。
    gyro_pitch = gyro_axis[1];//角速度
    gyro_yaw = gyro_axis[3];
    gyro_yaw *= -1;
    acc_x = acc_axis[1];//加速度
    acc_y = acc_axis[2];
    acc_z = acc_axis[3];
    acc_z *= -1;
}

void set_gyro_registers(){
  //Setup the MPU-6050
  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
  Wire.write(0x6B); // PWR_MGMT_1                                                    //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                    //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
  Wire.write(0x1B); // GYRO_CONFIG                                                      //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (1000dps full scale)
  Wire.endTransmission();                                                    //End the transmission with the gyro

  Wire.beginTransmission(gyro_address); 
  // Wire.write();    j加速度计频率                                 //Start communication with the address found during search.
  Wire.write(0x1C); // ACCEL_CONFIG                                                      //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();                                                    //End the transmission with the gyro

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
  Wire.write(0x1B);                                                          //Start reading @ register 0x1B
  Wire.endTransmission();                                                    //End the transmission
  Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
  while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
  if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
    digitalWrite(12,HIGH);                                                   //Turn on the warning led
    while(1)delay(10);                                                       //Stay in this loop for ever
  }

  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
  Wire.write(0x1A); // CONFIG                                                      //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                    //End the transmission with the gyro    
}

//外环输入：欧拉角（陀螺仪测量的倾斜角）
//外环输出：期望角速度
//内环输入：期望角速度 - 当前角速度（陀螺仪测量的角速度值）
//内环输出：电机转数（实际上输出的是PWM信号的平衡补偿值）

int roll_time = 0;
int pitch_time = 0;
void calculate_pid_roll()
{
  float outside_pid_angle_err_roll=0,outside_Kp_output_roll=0,outside_Ki_output_roll=0,outside_pid_output_roll=0;
  float inside_pid_angular_err_roll=0,inside_Kp_output_roll=0,inside_Ki_output_roll=0;
  // ------外环------//
  //当前角度误差=期望角度-当前角度
  outside_pid_angle_err_roll=(pid_roll_setpoint - angle_roll);
  //外环PID_P项=外环Kp*当前角度误差
  outside_Kp_output_roll=outside_pid_Kp_roll*outside_pid_angle_err_roll;
  //当前角度误差积分及其积分限幅
  if(throttle>1050) outside_pid_integral_roll+=outside_pid_angle_err_roll;
  else outside_pid_integral_roll=0;
  if(outside_pid_integral_roll>pid_max_roll) outside_pid_integral_roll=pid_max_roll;
  else if(outside_pid_integral_roll<-pid_max_roll) outside_pid_integral_roll=-pid_max_roll;
  // Serial.println(outside_pid_integral_roll);
  //外环PID_I项=外环Ki*当前角度误差积分
  outside_Ki_output_roll=outside_pid_Ki_roll*outside_pid_integral_roll;
  //外环PID输出=外环PID_P项+外环PID_I项
  outside_pid_output_roll=outside_Kp_output_roll+outside_Ki_output_roll+outside_pid_Kd_roll*(outside_pid_angle_err_roll-last_outside_pid_angle_err_roll);
  last_outside_pid_angle_err_roll = outside_pid_angle_err_roll;
  //------内环------//
  //当前角速度误差=外环PID输出-当前角速度（直接用陀螺仪输出）
  inside_pid_angular_err_roll=outside_pid_output_roll-gyro_roll_input;
  //内环PID_P项 = 内环Kp * 当前角速度误差
  inside_Kp_output_roll=inside_pid_Kp_roll*inside_pid_angular_err_roll;
  //当前角速度误差积分及其积分限幅
  if(throttle>1050) inside_pid_integral_roll+=inside_pid_angular_err_roll;
  else inside_pid_integral_roll=0;
  if(inside_pid_integral_roll>pid_max_roll) inside_pid_integral_roll=pid_max_roll;
  else if(inside_pid_integral_roll<-pid_max_roll) inside_pid_integral_roll=-pid_max_roll;
  //内环PID_I项 = 内环Ki * 当前角速度误差积分
  inside_Ki_output_roll=inside_pid_Ki_roll*inside_pid_integral_roll;
  //当前角速度微分（本次角速度误差 - 上次角速度误差）
  //内环PID_D项 = 内环Kd * 当前角速度的微分
  //内环PID输出 = 内环PID_P项 + 内环PID_I项 + 内环PID_D项
  pid_output_roll=inside_Kp_output_roll+inside_Ki_output_roll+inside_pid_Kd_roll*(inside_pid_angular_err_roll-last_inside_pid_angular_err_roll);
  last_inside_pid_angular_err_roll = inside_pid_angular_err_roll;
}


void calculate_pid_pitch()
{
  float outside_pid_angle_err_pitch=0,outside_Kp_output_pitch=0,outside_Ki_output_pitch=0,outside_pid_output_pitch=0;
  float inside_pid_angular_err_pitch=0,inside_Kp_output_pitch=0,inside_Ki_output_pitch=0;
    //------外环------//
  //当前角度误差=期望角度-当前角度
  outside_pid_angle_err_pitch=(pid_pitch_setpoint - angle_pitch);
  //外环PID_P项=外环Kp*当前角度误差
  outside_Kp_output_pitch=outside_pid_Kp_pitch*outside_pid_angle_err_pitch;
  //当前角度误差积分及其积分限幅
  if(throttle>1050) outside_pid_integral_pitch+=outside_pid_angle_err_pitch;
  else outside_pid_integral_pitch=0;
  if(outside_pid_integral_pitch>pid_max_pitch) outside_pid_integral_pitch=pid_max_pitch;
  else if(outside_pid_integral_pitch<-pid_max_pitch) outside_pid_integral_pitch=-pid_max_pitch;
  // Serial.println(outside_pid_integral_roll);
  //外环PID_I项=外环Ki*当前角度误差积分
  outside_Ki_output_pitch=outside_pid_Ki_pitch*outside_pid_integral_pitch;
  //外环PID输出=外环PID_P项+外环PID_I项
  outside_pid_output_pitch=outside_Kp_output_pitch+outside_Ki_output_pitch+outside_pid_Kd_pitch*(outside_pid_angle_err_pitch-last_outside_pid_angle_err_pitch);
  last_outside_pid_angle_err_pitch = outside_pid_angle_err_pitch;
  //------内环------//
  //当前角速度误差=外环PID输出-当前角速度（直接用陀螺仪输出）
  inside_pid_angular_err_pitch=outside_pid_output_pitch-gyro_pitch_input;
  //内环PID_P项 = 内环Kp * 当前角速度误差
  inside_Kp_output_pitch=inside_pid_Kp_pitch*inside_pid_angular_err_pitch;
  //当前角速度误差积分及其积分限幅
  if(throttle>1050) inside_pid_integral_pitch+=inside_pid_angular_err_pitch;
  else inside_pid_integral_pitch=0;
  if(inside_pid_integral_pitch>pid_max_pitch) inside_pid_integral_pitch=pid_max_pitch;
  else if(inside_pid_integral_pitch<-pid_max_pitch) inside_pid_integral_pitch=-pid_max_pitch;
  //内环PID_I项 = 内环Ki * 当前角速度误差积分
  inside_Ki_output_pitch=inside_pid_Ki_pitch*inside_pid_integral_pitch;
  //当前角速度微分（本次角速度误差 - 上次角速度误差）
  //内环PID_D项 = 内环Kd * 当前角速度的微分
  //内环PID输出 = 内环PID_P项 + 内环PID_I项 + 内环PID_D项
  pid_output_pitch=inside_Kp_output_pitch+inside_Ki_output_pitch+inside_pid_Kd_pitch*(inside_pid_angular_err_pitch-last_inside_pid_angular_err_pitch);
  last_inside_pid_angular_err_pitch = inside_pid_angular_err_pitch;
}
void calculate_pid_yaw()
{
  float outside_pid_angle_err_yaw=0,outside_Kp_output_yaw=0,outside_Ki_output_yaw=0,outside_pid_output_yaw=0;
  float inside_pid_angular_err_yaw=0,inside_Kp_output_yaw=0,inside_Ki_output_yaw=0;
  //------外环------//
  //当前角度误差=期望角度-当前角度
  outside_pid_angle_err_yaw=pid_yaw_setpoint - angle_yaw_gyro;
  // outside_pid_angle_err_yaw=gyro_yaw_input-pid_yaw_setpoint;
  //外环PID_P项=外环Kp*当前角度误差
  outside_Kp_output_yaw=outside_pid_Kp_yaw*outside_pid_angle_err_yaw;
  //当前角度误差积分及其积分限幅
  if(throttle>1050) outside_pid_integral_yaw+=outside_pid_angle_err_yaw;
  else outside_pid_integral_yaw=0;
  if(outside_pid_integral_yaw>pid_max_yaw) outside_pid_integral_yaw=pid_max_yaw;
  else if(outside_pid_integral_yaw<-pid_max_yaw) outside_pid_integral_yaw=-pid_max_yaw;
  // Serial.println(outside_pid_integral_roll);
  //外环PID_I项=外环Ki*当前角度误差积分
  outside_Ki_output_yaw=outside_pid_Ki_yaw*outside_pid_integral_yaw;
  //外环PID输出=外环PID_P项+外环PID_I项
  pid_output_yaw=outside_Kp_output_yaw+outside_Ki_output_yaw+outside_pid_Kd_yaw*(outside_pid_angle_err_yaw-last_outside_pid_angle_err_yaw);
  // outside_pid_output_yaw=outside_Kp_output_yaw+outside_Ki_output_yaw+outside_pid_Kd_yaw*(outside_pid_angle_err_yaw-last_outside_pid_angle_err_yaw);
  last_outside_pid_angle_err_yaw = outside_pid_angle_err_yaw;
  // //------内环------//
  // //当前角速度误差=外环PID输出-当前角速度（直接用陀螺仪输出）
  // inside_pid_angular_err_yaw=outside_pid_output_yaw-gyro_yaw;
  // //内环PID_P项 = 内环Kp * 当前角速度误差
  // inside_Kp_output_yaw=inside_pid_Kp_yaw*inside_pid_angular_err_yaw;
  // // Serial.print(inside_Kp_output_roll);
  // // Serial.print("\n");
  // //当前角速度误差积分及其积分限幅
  // if(throttle>1050) inside_pid_integral_yaw+=inside_pid_angular_err_yaw;
  // else inside_pid_integral_yaw=0;
  // if(inside_pid_integral_yaw>pid_max_yaw) inside_pid_integral_yaw=pid_max_yaw;
  // else if(inside_pid_integral_yaw<-pid_max_yaw) inside_pid_integral_yaw=-pid_max_yaw;
  // //内环PID_I项 = 内环Ki * 当前角速度误差积分
  // inside_Ki_output_yaw=inside_pid_Ki_yaw*inside_pid_integral_yaw;
  // //当前角速度微分（本次角速度误差 - 上次角速度误差）
  // //内环PID_D项 = 内环Kd * 当前角速度的微分
  // //内环PID输出 = 内环PID_P项 + 内环PID_I项 + 内环PID_D项
  // pid_output_yaw=inside_Kp_output_yaw+inside_Ki_output_yaw+inside_pid_Kd_yaw*(inside_pid_angular_err_yaw-last_inside_pid_angular_err_yaw);
  // last_inside_pid_angular_err_yaw = inside_pid_angular_err_yaw;
}


