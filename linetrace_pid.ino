/*
  SPDX-License-Identifier:MIT
  Copyright (C) 2022 Yusuke Yamasaki. All Rights Reserved.
*/

/* 参考サイトリンク
https://veresk.hatenablog.com/entry/2019/06/29/192525
https://skpme.com/211/
*/

#include <ros.h>
#include <intelligent_robot_b_2022/rosbagdata.h>

#define PPR 100
#define Pi 3.141592
#define WHEEL 0.071  // 車輪直径(7.1[cm] → 0.071[m])

#define KP 10
#define KI 0//14.727
#define KD 0//1.138

int PWM = 127;  // 初期デューティー比(未使用)
int L_PWMref = 0;  // 未使用
int R_PWMref = 0;  // 未使用

short Start = 1;  // 最初の処理_判定フラグ
unsigned int T, T_0, T_init;  // 時間設定用
float dt = 0.001;  // 1[ms] 微小時間（値設定に理由はない）

volatile long L_encoder = 0;  // エンコーダのカウント値
volatile int L_old_value = 0;
volatile long R_encoder = 0;
volatile int R_old_value = 0;

float L_x_0 = 0,  // 距離
      R_x_0 = 0, 
      L_x = 0, 
      R_x = 0;

float L_v = 0,  // 速度
      R_v = 0,
      v = 0;

float L_sensor_val = 0,  // 左４つのセンサ
      R_sensor_val = 0;  // 右４つのセンサ

float vel_target = 0.1, // 目標速度[m/s]
      L_sen_target = 540,  // センサ発光：700, センサ消灯：300 黒：発光、白：消灯
      R_sen_target = 540;

float L_sen_diff = 0,  // センサ値の差
      L_old_sen_diff = 0,  // 前回のセンサ値の差
      R_sen_diff = 0,
      R_old_sen_diff = 0,
      vel_diff = 0,
      old_vel_diff = 0;

float L_sen_integral = 0,  // 積分値
      R_sen_integral = 0,
      vel_integral = 0;

float v_pid=0, L_pid=0, R_pid=0;  // 操作量の記録(L R に v を足す処理になっている)
float L_old_pid=0, R_old_pid=0;  // 前回の操作量

float Circumference = WHEEL * Pi;  // 円周
float Cir_PPR = Circumference / PPR;  // １パルス分の円周

ros::NodeHandle nh;
intelligent_robot_b_2022::rosbagdata data;
ros::Publisher pub("linetrace_data", &data);

// 左エンコーダ値の更新(割り込み(タイマ割り込みではない))
void L_encoderUpdate()
{
  int L_valueA, L_valueB, L_value, L_rolate;

  L_valueA = digitalRead(18);
  L_valueB = digitalRead(19);

  L_value = (L_valueA << 1) | L_valueB;
  L_rolate = (L_old_value << 2) | L_value;

  if(L_rolate == 0b0010 || L_rolate == 0b0100 || L_rolate == 0b1011 || L_rolate == 0b1101){ L_encoder++; }
  if(L_rolate == 0b0001 || L_rolate == 0b0111 || L_rolate == 0b1000 || L_rolate == 0b1110){ L_encoder--; }

  L_old_value = L_value;
}

// 右エンコーダ値の更新(割り込み(タイマ割り込みではない))
void R_encoderUpdate()
{
  int R_valueA, R_valueB, R_value, R_rolate;

  R_valueA = digitalRead(18);
  R_valueB = digitalRead(19);

  R_value = (R_valueA << 1) | R_valueB;
  R_rolate = (R_old_value << 2) | R_value;

  if(R_rolate == 0b0010 || R_rolate == 0b0100 || R_rolate == 0b1011 || R_rolate == 0b1101){ R_encoder++; }
  if(R_rolate == 0b0001 || R_rolate == 0b0111 || R_rolate == 0b1000 || R_rolate == 0b1110){ R_encoder--; }

  R_old_value = R_value;
}

float v_PID_Control()  // 速度制御
{
  float p, i, d;

  old_vel_diff = vel_diff;  // 前回の差を記録

  vel_diff = v - vel_target;  // 現在速度v[m/s] - 目標速度

  vel_integral += (old_vel_diff + vel_diff) / 2 * dt;  // 積分値の記録
  
  p = KP * vel_diff;
  i = KI * vel_integral;
  d = KD * (vel_diff - old_vel_diff) / dt;

  v_pid = p + i + d;  // 操作量
}

float L_PID_Control()
{
  float L_p, L_i, L_d;

  L_old_pid = L_pid;

  L_old_sen_diff = L_sen_diff;

  L_sen_diff = L_sensor_val - L_sen_target;

  L_sen_integral += (L_old_sen_diff + L_sen_diff) / 2 * dt;

  L_p = KP * L_sen_diff;
  L_i = KI * L_sen_integral;
  L_d = KD * (L_sen_diff - L_old_sen_diff) / dt;

  L_pid = (L_p + L_i + L_d) + v_pid;  // 操作量＋速度制御の操作量
}

float R_PID_Control()
{
  float R_p, R_i, R_d;

  R_old_pid = R_pid;

  R_old_sen_diff = R_sen_diff;

  R_sen_diff = R_sensor_val - R_sen_target;

  R_sen_integral += (R_old_sen_diff + R_sen_diff) / 2 * dt;

  R_p = KP * R_sen_diff;
  R_i = KI * R_sen_integral;
  R_d = KD * (R_sen_diff - R_old_sen_diff) / dt;

  R_pid = (R_p + R_i + R_d) + v_pid;
}

void setup()
{
  // ROS Setup
  nh.initNode();
  nh.advertise(pub);

  //Serial
  Serial.begin(115200);

  //L_MOTOR
  pinMode(44, OUTPUT);  // DIR_ROL
  pinMode(6, OUTPUT);  // PWM
  //R_MOTOR
  pinMode(46, OUTPUT);  // DIR_ROL
  pinMode(7, OUTPUT);  // PWM

  //PhotoReflecter(LineTracer)
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  //L_ENCODER
  pinMode(18, INPUT);  // A相  yellow
  pinMode(19, INPUT);  // B相  white
  digitalWrite(18, HIGH);
  digitalWrite(19, HIGH);
  attachInterrupt(digitalPinToInterrupt(18), L_encoderUpdate, CHANGE);  // 割り込みの設定
  attachInterrupt(digitalPinToInterrupt(19), L_encoderUpdate, CHANGE);
  //R_ENCODER
  pinMode(3, INPUT);  // A相
  pinMode(2, INPUT);  // B相
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), R_encoderUpdate, CHANGE);  // 割り込みの設定
  attachInterrupt(digitalPinToInterrupt(2), R_encoderUpdate, CHANGE);

  delay(100);  // 適当に待つ
}

void loop()
{
  int L_val, R_val;
  // float s1 = analogRead(A0)*5/1023,
  //       s2 = analogRead(A1)*5/1023,
  //       s3 = analogRead(A2)*5/1023,
  //       s4 = analogRead(A3)*5/1023,
  //       s5 = analogRead(A4)*5/1023,
  //       s6 = analogRead(A5)*5/1023,
  //       s7 = analogRead(A6)*5/1023,
  //       s8 = analogRead(A7)*5/1023;
  float s1 = analogRead(A0),  // ライントレースのセンサの値を記録
        s2 = analogRead(A1),
        s3 = analogRead(A2),
        s4 = analogRead(A3),
        s5 = analogRead(A4),
        s6 = analogRead(A5),
        s7 = analogRead(A6),
        s8 = analogRead(A7);
  // MOTOR_init_Start
  if(Start == 1)
  {
    digitalWrite(44, HIGH);  // L_MOTOR_DIR_ROL
    digitalWrite(46, LOW);  // R_MOTOR_DIR_ROL

  //   for(int i = 0; i <= PWM; i++)
  //   {  // 基準 PWM を決める ＜ 127 ＞
  //     analogWrite(6, i);  // L_MOTOR_PWM
  //     analogWrite(7, i);  // R_MOTOR_PWM
  //     L_pid = i;
  //     R_pid = i;
  //   }
  //   //T_init = micros();
  //   //T = 0;
  }

  //T_0 = T;
  //T = micros() - T_init;  // プログラム動作時刻[ms] 最下位に１のズレ

  // 距離ｘ：エンコーダ値＊1パルス分の円周＝走行距離
  L_x_0 = L_x;
  R_x_0 = R_x;
  L_x = abs(Cir_PPR * L_encoder);  // １パルス分の円周＊エンコーダカウント値
  R_x = abs(Cir_PPR * R_encoder);

  // 速度ｄｘ：疑似微分
  L_v = (L_x - L_x_0) / dt;  // 進んだ距離の差/微小時間
  R_v = (R_x - R_x_0) / dt;

  v = (L_v + R_v) / 2;

  // センサの基準からの距離に応じて重み付け。左右別々に扱う。(200 = white , 700 = black)
  L_sensor_val = 0;
  R_sensor_val = 0;
  L_sensor_val += s1 * 10;  // ここもおかしそう
  L_sensor_val += s2 * 7;
  L_sensor_val += s3 * 4;  // ↑ 左
  L_sensor_val += s4 * 1;  // 真ん中
  R_sensor_val += s5 * 1;  // 真ん中
  R_sensor_val += s6 * 4;  // ↓ 右
  R_sensor_val += s7 * 7;
  R_sensor_val += s8 * 10;

  // 各制御値の更新。今回操作量の取得
  v_PID_Control();  // 操作量に閾値、丸めは行っていない
  L_PID_Control();
  R_PID_Control();

  // 前回操作量と今回操作量の差分＋前回操作量＝今回操作量
  L_val = L_old_pid + (L_pid - L_old_pid);  // 操作量の計算
  R_val = R_old_pid + (R_pid - R_old_pid);
  // analogWrite(6, map(L_val, -10000, 10000, 0, 55)); 
  // analogWrite(7, map(R_val, -10000, 10000, 0, 55));
  analogWrite(6, constrain(L_val, -55, 55));  // マイナスが前進になってない？
  analogWrite(7, constrain(R_val, -55, 55));  // 閾値を決めて、モータに操作量(デューティー比)を与えている(-55,55は実験途中の為)

  if(Start == 1){ Start = 0; }
  // 以下、Topic出力(しかし、上手く動いていない。rosbagが使えていない)
  data.L_encoder = L_encoder;
  data.R_encoder = R_encoder;
  data.L_x = L_x;
  data.R_x = R_x;
  data.L_v = L_v;
  data.R_v = R_v;
  data.v = v;

  pub.publish(&data);
  
}
