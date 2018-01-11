

#include <Wire.h>
#include "StewartPlateform.h"
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>   // 引用程式庫
#include "JY901.h"
/*
  Test on Uno R3.
  JY901   UnoR3
  TX <---> 0(Rx)
*/
//#define REMOTE_CTRL
#define BALANCE_CTRL
// 定義連接藍牙模組的序列埠
SoftwareSerial BT(8, 9); // 接收腳, 傳送腳

float Kp = 0.06;
float Ki =0;
float Kd = 0.025;

float pre_pitch;
float pre_roll;
float pre_yaw;
float pre_err_pitch;
float pre_err_roll;
float pre_err_yaw;
float integral_pitch = 0;
float integral_roll = 0;
unsigned long time_last;
myPoint::Point platepoint[6];
myPoint::Point motorarmpoint[6];
Stewart stewart;
String val;  // 儲存接收資料的變數
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  130
#define SERVOMID  330
#define SERVOMAX  530
double RADCONST = 57.3248408;
uint8_t servonum = 0;
double pulselen;
double angle[6] = { 0 };
double angle_last[6] = { 0 };
int x = 1;
float p = 0, r = 0;
String input_str;
////////////////////////////////////////////////
String readLine();
void getServoAngle(int i);
void DecodeSerial();
bool getNewPosition();
bool checkLength(float S, int i);
void ServoDrive();
void readGyro();
void balance_ctrl(float p_pitch, float p_roll);
void setup() {

  Serial.begin(9600);
  BT.begin(9600);
  JY901.attach(BT);
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  for (int num = 0; num < 6; num++)
  {
    angle[num] = 90 + 11 * pow(-1, num);
    angle_last[num] = angle[num];
    //pwm.setPWM(num+x, 0, SERVOMID);
  }
}
void loop() {
  // Drive each servo one at a time

  char ch = Serial.read();
  float decrease = 0.01;
  if (ch == '1')
    p = 0.15;
  if (ch == '2')
    p -= decrease;
  JY901.receiveSerialData();

#ifdef BALANCE_CTRL
  balance_ctrl(p , r);
#endif
  if (getNewPosition())
    ServoDrive();
  //delay(2);
  Serial.println();
}
void readGyro()
{
  float roll = JY901.getRoll();
  float pitch = JY901.getPitch();
  float yaw = JY901.getYaw();
  roll = roll / 180 * 3.14;
  pitch = pitch / 180 * 3.14;
  yaw = yaw / 180 * 3.14;
  if (abs(roll) <= 0.13)
    stewart.targetPitch = roll;
  if (abs(pitch) <= 0.13)
    stewart.targetRoll = pitch;
  //if (abs(yaw) <= 0.18)
  //stewart.targetYaw = yaw;
  //  Serial.print(roll); Serial.print(" "); Serial.print(pitch); Serial.print(" "); Serial.print(yaw);
}
void balance_ctrl(float p_pitch, float p_roll)
{

  float g_pitch = JY901.getRoll();
  float g_roll = JY901.getPitch();
  float g_yaw = JY901.getYaw();
  unsigned long time_now = millis();
  g_roll = g_roll / 180 * 3.14;
  g_pitch = g_pitch / 180 * 3.14;
  float err_pitch = p_pitch - g_pitch;
  float err_roll = p_roll - g_roll;
  float diff_time = float(time_now - time_last) * 0.001;
  // Serial.println(diff_time,4);

  float err_pitch_diff = (err_pitch - pre_err_pitch) / diff_time;
  float err_roll_diff = (err_roll - pre_err_roll) / diff_time;
  integral_pitch += err_pitch * diff_time;
  integral_roll += err_roll * diff_time;
//Serial.print(g_pitch*180/3.14, 3);// Serial.print(" "); Serial.println(err_roll, 3);
 // Serial.print(err_pitch, 3); Serial.print(" "); Serial.println(err_roll, 3);
  stewart.targetRoll += err_roll * Kp + err_roll_diff * Kd + integral_roll * Ki;
  stewart.targetPitch += err_pitch * Kp + err_pitch_diff * Kd + integral_pitch * Ki;
  Serial.println(diff_time);
  pre_pitch = g_pitch;
  pre_roll = g_roll;
  pre_err_pitch = err_pitch;
  pre_err_roll = err_roll;
  time_last = time_now;
}
String readLine() {
  String s = "";
  char c;
  while ((c = Serial.read()) != '\n') {
    s += c;
  }
  return s;
}
void ServoDrive()
{
  for (int i = 0; i < 6; i++)
  {
    angle[i] = 90 + angle[i] * pow(-1, i + 2);
    pulselen = map(angle[i], 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(i + x, 0, pulselen);
    // Serial.print(pulselen); Serial.print(" ");
  }
  //Serial.println();
}
void getServoAngle(int i)
{
  float Li = platepoint[i].getDistance(stewart.BasePoint[i]);
  float L = Li * Li - (stewart.servo[i].connect_length * stewart.servo[i].connect_length - stewart.servo[i].arm_length * stewart.servo[i].arm_length);
  float M = 2 * stewart.servo[i].arm_length * (platepoint[i].z - stewart.BasePoint[i].z);
  float N = 2 * stewart.servo[i].arm_length * (cos(stewart.servo[i].thida_S) * (platepoint[i].x - stewart.BasePoint[i].x) + sin(stewart.servo[i].thida_S) * (platepoint[i].y - stewart.BasePoint[i].y));
  float alfa = asin(L / sqrt(M * M + N * N)) - atan(N / M);
  stewart.servo[i].angle = alfa ;
  angle[i] = stewart.servo[i].angle * 180 / 3.14;
}
bool getNewPosition()
{
  float rr[9] = { 0 };
  float tt[3] = { 0 };
  stewart.getTransMat(rr, tt);
  for (int i = 0; i < 6; i++)
  {
    platepoint[i].x = rr[0] * stewart.PlatePoint[i].x + rr[1] * stewart.PlatePoint[i].y + rr[2] * stewart.PlatePoint[i].z;
    platepoint[i].y = rr[3] * stewart.PlatePoint[i].x + rr[4] * stewart.PlatePoint[i].y + rr[5] * stewart.PlatePoint[i].z;
    platepoint[i].z = rr[6] * stewart.PlatePoint[i].x + rr[7] * stewart.PlatePoint[i].y + rr[8] * stewart.PlatePoint[i].z;
    platepoint[i].x += tt[0];
    platepoint[i].y += tt[1];
    platepoint[i].z += tt[2];
  }
  for (int i = 0; i < 6; i++)
  {
    getServoAngle(i);
    // Serial.print(stewart.servo[i].angle);Serial.print("  ");
    float S = platepoint[i].getDistance(stewart.BasePoint[i]);
    if (checkLength(S, i))
    {
      for (int j = 0; j < 6; j++)
        stewart.servo[j].angle = stewart.servo[j].angle_pre;
      return false;
    }
    if (abs(stewart.servo[i].angle) > 70 * 3.14 / 180)
    {
      for (int j = 0; j < 6; j++)
        stewart.servo[j].angle = stewart.servo[j].angle_pre;
      return false;
    }
    if (isnan(stewart.servo[i].angle))
    {
      for (int j = 0; j < 6; j++)
        stewart.servo[j].angle = stewart.servo[j].angle_pre;
      return false;
    }
    //  printf("S%i=%f \n", i, S);
    stewart.servo[i].angle_pre = stewart.servo[i].angle;
  }
  // Serial.println();
  return true;
}
bool checkLength(float S, int i)
{
  float armz = stewart.BasePoint[i].z + stewart.servo[i].arm_length * sin(stewart.servo[i].angle);
  if (armz >= 0)
    if (S > stewart.servo[i].arm_length + stewart.servo[i].connect_length)
      return true;

  if (armz < 0)
    if (stewart.servo[i].connect_length - stewart.servo[i].arm_length > S)
      return true;

  return false;
}
