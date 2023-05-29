#include <Arduino.h>
#include <PS4Controller.h>
#include "esp_intr_alloc.h"
#include "m2006.h"
#include "as5600_tca9548a.h"
#include <math.h>

// PS4 Controller
#define PS4_ADDR "0C:B8:15:C5:1C:C4"

// エアシリンダー
#define OPEN_PIN 13 // 1
#define UPDOWN_PIN 12 // 2
#define RING_PIN 14 // 3

// I2C
#define SDA_PIN 21
#define SCL_PIN 22

#define DIR_PIN 21

#define rx 4
#define tx 5
#define CAN_SPEED 1000E3

TaskHandle_t thp[2];

int8_t headerByte = 0x55; // 0x55 = 85
int8_t addressByte = 0x00; // 0x00 = 0
int8_t commandByte1 = 0x00; // 0x00 = 0
int8_t commandByte2 = 0x00; // 0x00 = 0
int8_t commandByte3 = 0x00; // 0x00 = 0
int8_t checksum = 0x00; // 0x00 = 0

int8_t updwn_cmd = 0; // Up:1, Down:-1, Stop:0
int8_t rollr_cmd = 0;
int8_t rollr_spd_cmd = 0;
bool prev_bttn_state[6] = {false, false, false, false, false, false}; // Share, Options, Left, Right, Touchpad, R1

float l_x = 0.0; // 左スティックのX軸
float l_y = 0.0; // 左スティックのY軸
float r_x = 0.0; // 右スティックのX軸
float r_y = 0.0; // 右スティックのY軸

float lstick_x = l_x;
float lstick_y = l_y;
float rstick_x = r_x; 
float rstick_y = r_y;

// AS5600_TCA9548A
float offset1[4] = {0.0, 0.0, 0.0, 0.0}; // 静止時のオフセット値(4個分)
float offset2[4] = {0.0, 0.0, 0.0, 0.0}; // 回転時のオフセット値(4個分)
float current_angle[4] = {0.0, 0.0, 0.0, 0.0}; // AS5600の現在の角度(4個分) (0 to 360)

float A;
float B;
float C;
float D;

// M2006のCAN受信データ
int16_t mangle[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M2006の8個分の角度値(16bit)
int16_t mrpm[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M2006の8個分の回転数値(16bit)
int16_t mtorque[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M2006の8個分のトルク値(16bit)
int num = 0;
int id[8] = {0x201, 0x202, 0x203, 0x204, 0x205, 0x206, 0x207, 0x208};

float target_speed[4] = {0, 0, 0, 0};
float target_angle[4] = {0, 0, 0, 0};
int16_t rotate_rpm[4] = {0, 0, 0, 0};
int16_t target_rpm[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// PIDの計算用
float prev_time = 0.0;

// PID制御(速度制御)
int16_t error[8];
int16_t integral[8];
int16_t derivative[8];
int16_t pre_error[8];

// PID制御(角度制御)
int16_t error_angle[4];
int16_t integral_angle[4];
int16_t derivative_angle[4];
int16_t pre_error_angle[4];

float Kp_angle = 3.8; // 3.5 
float Ki_angle = 0.0008; 
float Kd_angle = 5;

// 3倍はやくする（angleよりも速くする）
float Kp = Kp_angle * 3;
float Ki = Ki_angle * 3;
float Kd = Kd_angle * 3;

// M2006
int16_t current_data[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // M2006の8個分の電流値(16bit)
uint8_t send_data1[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // M2006に送信するデータ(8bit) (ID:1-4)
uint8_t send_data2[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // M2006に送信するデータ(8bit) (ID:5-8)

// DifferentialDrive
int16_t w_w[4] = {0, 0, 0, 0}; // 移動方向の速度
int16_t w_a[4] = {0, 0, 0, 0}; // 方位方向の速度
int16_t d_a[4] = {0, 0, 0, 0}; // 方位方向の速度

int core0a_free_stack = 0;
int core0b_free_stack = 0;
int core1b_free_stack = 0;
int core1m_free_stack = 0;

void Core0a(void *args); // PS4 Controller
void Core0b(void *args);
void Core1b(void *args);

/* 何でうまくいかなかったがわからないため，もう一度書き直す */
/* ローラ，昇降を動かすだけのプログラム */
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(OPEN_PIN, OUTPUT);
  pinMode(UPDOWN_PIN, OUTPUT);
  pinMode(RING_PIN, OUTPUT);
  // I2Cのピンをプルアップ
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  // CAN
  while (!can_init(rx, tx, CAN_SPEED));

  PS4.begin(PS4_ADDR);
  Serial.println("> PS4: Started");
  Serial.println("> PS4: Press PS button to connect");
  while (!PS4.isConnected()) {
    Serial.println("> PS4: Connecting...");
    delay(1000);
  }
  
  Serial.println("> PS4: Connected");
  PS4.setLed(255, 255, 255);
  PS4.setRumble(200, 200);
  PS4.sendToController();
  delay(1000);
  PS4.setRumble(0, 0);
  PS4.sendToController();

  // AS5600_TCA9548A
  as5600_tca9548a_init(DIR_PIN);
  Serial.println("> AS5600_TCA9548A: Started.");
  as5600_tca9548a_get_offset(offset1);
  Serial.println("> AS5600_TCA9548A: Offset1: " + String(offset1[0]) + ", " + String(offset1[1]) + ", " + String(offset1[2]) + ", " + String(offset1[3]));
  offset1[0] = -39.90;
  offset1[1] = 23.99;
  offset1[2] = 73.74;
  offset1[3] = 110.74;

  xTaskCreatePinnedToCore(Core0a, "Core0a", 4096 / 1.5, NULL, 3, &thp[0], 0); // (タスク名, タスクのサイズ, パラメータ, 優先度, タスクハンドル, コア番号)
  xTaskCreatePinnedToCore(Core0b, "Core0b", 4096, NULL, 3, &thp[1], 0);
  xTaskCreatePinnedToCore(Core1b, "Core1b", 4096 / 1.5, NULL, 3, &thp[2], 1); 
}

void loop() {
  // M2006のデータを読むこむ(1つずつ読む)
  m2006_read_data(id[num], mangle, mrpm, mtorque);
  num = (num + 1) % 8;
  if (abs(sqrt(pow(lstick_x, 2) + pow(lstick_y, 2))) > 0 || abs(rstick_x) > 0) {
    A = lstick_x - (rstick_x * 1.0);
    B = lstick_x + (rstick_x * 1.0);
    C = lstick_y - (rstick_x * 1.0);
    D = lstick_y + (rstick_x * 1.0);

    target_speed[0] =  sqrt(pow(B, 2) + pow(C, 2));
    target_speed[1] =  sqrt(pow(B, 2) + pow(D, 2));
    target_speed[2] =  sqrt(pow(A, 2) + pow(D, 2));
    target_speed[3] =  sqrt(pow(A, 2) + pow(C, 2));

    target_angle[0] = -(atan2(B, C) * 180 / PI);
    target_angle[1] = -(atan2(B, D) * 180 / PI);
    target_angle[2] = -(atan2(A, D) * 180 / PI);
    target_angle[3] = -(atan2(A, C) * 180 / PI);

    for (int i = 0; i < 4; i++){
      if (target_angle[i] - current_angle[i] < -90) {
        target_angle[i] += 180;
        target_speed[i] = -target_speed[i];
      } else if (target_angle[i] - current_angle[i] > 90) {
        target_angle[i] -= 180;
        target_speed[i] = -target_speed[i];
      }
    }
  } else {
    for (int i = 0; i < 4; i++){
      target_speed[i] = 0;
      target_angle[i] = (int)current_angle[i];
    }
  }

  /******************************************
   * PID制御
   ******************************************/

  // 経過時間の計算
  float dt = millis() - prev_time;
  prev_time = millis();

  // PID制御(角度制御)
  for (int i = 0; i < 4; i++){
    error_angle[i] = target_angle[i] - current_angle[i];
    integral_angle[i] += error_angle[i] * dt; // 積分
    derivative_angle[i] = (error_angle[i] - pre_error_angle[i]) / dt; // 微分
    pre_error_angle[i] = error_angle[i]; // 前回の誤差を保存
  }

  // 電流値の計算(-3000 ~ 3000)
  for (int i = 0; i < 4; i++){
    rotate_rpm[i] = -(Kp_angle * error_angle[i] + Ki_angle * integral_angle[i] + Kd_angle * derivative_angle[i]);
    rotate_rpm[i] = constrain(rotate_rpm[i], -280, 280); // 回転速度の制限
  }

  for (int i = 0; i < 4; i++){
    target_rpm[0 + (i * 2)] = target_speed[i] - rotate_rpm[i];
    target_rpm[1 + (i * 2)] = -target_speed[i] - rotate_rpm[i];
  }

  // PID制御(速度制御)
  for (int i = 0; i < 8; i++){
    error[i] = target_rpm[i] - mrpm[i];
    integral[i] += error[i] * dt; // 積分
    derivative[i] = (error[i] - pre_error[i]) / dt; // 微分
    pre_error[i] = error[i]; // 前回の誤差を保存
  }

  // 電流値の計算(-3000 ~ 3000)
  for (int i = 0; i < 4; i++){
    current_data[0 + (i * 2)] = (Kp * error[0 + (i * 2)] + Ki * integral[0 + (i * 2)] + Kd * derivative[0 + (i * 2)]);
    current_data[1 + (i * 2)] = (Kp * error[1 + (i * 2)] + Ki * integral[1 + (i * 2)] + Kd * derivative[1 + (i * 2)]);

    current_data[0 + (i * 2)] = constrain(current_data[0 + (i * 2)], -3000, 3000); // -3000 ~ 3000
    current_data[1 + (i * 2)] = constrain(current_data[1 + (i * 2)], -3000, 3000); // -3000 ~ 3000
  }

  // M2006に送信するデータを作成
  m2006_make_data(current_data, send_data1, send_data2);
  // M2006にデータを送信
  m2006_send_data(send_data1, send_data2);

  // Diffencial Swerve Driveの式
  for (int i = 0; i < 4; i++){
    w_w[i] = (mrpm[0 + (i * 2)] - mrpm[1 + (i * 2)]) / 2; // 移動方向の速度
    w_a[i] = (mrpm[0 + (i * 2)] + mrpm[1 + (i * 2)]) / 2; // 方位方向の速度
  }

  core1m_free_stack = uxTaskGetStackHighWaterMark(NULL);
  delay(1);
}

void Core0a(void *args) {
  while (1){
    if (PS4.LatestPacket() && PS4.isConnected()){
      // 移動・回転
      l_x = PS4.LStickX() / 127.0; // -127 ~ 127
      l_y = PS4.LStickY() / 127.0; // -127 ~ 127
      r_x = PS4.RStickX() / 127.0; // -127 ~ 127
      r_y = PS4.RStickY() / 127.0; // -127 ~ 127

      if (l_x < 0.1 && l_x > -0.1) l_x = 0;
      if (l_y < 0.1 && l_y > -0.1) l_y = 0;
      if (r_x < 0.1 && r_x > -0.1) r_x = 0;
      if (r_y < 0.1 && r_y > -0.1) r_y = 0;

      l_x = l_x * abs(l_x) * abs(l_x);
      l_y = l_y * abs(l_y) * abs(l_y);
      r_x = r_x * abs(r_x) * abs(r_x);
      r_y = r_y * abs(r_y) * abs(r_y);

      if (PS4.L1()){
          l_x *= 0.4;
          l_y *= 0.4;
          r_x *= 0.15;
          r_y *= 0.15;
      }

      if (PS4.Up()){
        updwn_cmd = 1;
      }
      if (PS4.Down()){
        updwn_cmd = -1;
      }
      if (!PS4.Up() && !PS4.Down()){
        updwn_cmd = 0;
      }
      if (PS4.Circle()) {
        rollr_cmd = 1;
      }
      if (PS4.Triangle()) rollr_cmd = 2;
      if (PS4.Square()) rollr_cmd = 3;
      if (PS4.Cross()) { // 全て停止
        updwn_cmd = 0;
        rollr_cmd = 0;
        rollr_spd_cmd = 0;
        l_x = 0;
        l_y = 0;
        r_x = 0;
        r_y = 0;
      }
      // 回収機構（開閉）デフォルト：開
      if (PS4.Share() && prev_bttn_state[0] == false){
        digitalWrite(OPEN_PIN, !(digitalRead(OPEN_PIN)));
        PS4.setLed(255, 0, 0); // LEDを赤色
        PS4.sendToController();
        delay(500);
        PS4.setLed(255, 255, 255); // LEDを消灯
        PS4.sendToController();
      }
      // 回収機構（上下）条件：リング射出機構が閉じている，昇降機構が0度，回収機構が開いている
      if (PS4.Options() && prev_bttn_state[1] == false && digitalRead(RING_PIN) == false){
        if (digitalRead(UPDOWN_PIN) == true && digitalRead(OPEN_PIN) == true) {
          digitalWrite(UPDOWN_PIN, LOW);
        } else {
          digitalWrite(UPDOWN_PIN, HIGH);
        }
        PS4.setLed(255, 0, 0); // LEDを赤色
        PS4.sendToController();
        delay(500);
        PS4.setLed(255, 255, 255); // LEDを消灯
        PS4.sendToController();
      }
      // リング射出（回収機構が上がっている）
      if (PS4.R1() && prev_bttn_state[5] == false && digitalRead(UPDOWN_PIN) == false){
        digitalWrite(RING_PIN, !(digitalRead(RING_PIN)));
        PS4.setLed(255, 0, 0); // LEDを赤色
        PS4.sendToController();
        delay(500);
        PS4.setLed(255, 255, 255); // LEDを消灯
        PS4.sendToController();
      }
      if (PS4.Left() && prev_bttn_state[2] == false){
        rollr_spd_cmd -= 1;
      }
      if (PS4.Right() && prev_bttn_state[3] == false){
        rollr_spd_cmd += 1;
      }
      if (PS4.Touchpad() && prev_bttn_state[4] == false){
        Serial.println("> PS4: Touchpad");
      }
      prev_bttn_state[0] = PS4.Share();
      prev_bttn_state[1] = PS4.Options();
      prev_bttn_state[2] = PS4.Left();
      prev_bttn_state[3] = PS4.Right();
      prev_bttn_state[4] = PS4.Touchpad();
      prev_bttn_state[5] = PS4.R1();     
    } else {
      updwn_cmd = 0;
      rollr_cmd = 0;
      rollr_spd_cmd = 0;
      l_x = 0;
      l_y = 0;
      r_x = 0;
      r_y = 0;
    }
    lstick_x = l_x * 380.0;
    lstick_y = l_y * 380.0;
    rstick_x = r_x * 200.0;
    rstick_y = r_y * 200.0;
    core0a_free_stack = uxTaskGetStackHighWaterMark(NULL);
    delay(1);
  }
}

void Core0b(void *args) {
  while (1){
    commandByte1 = updwn_cmd;
    commandByte2 = rollr_cmd;
    commandByte3 = rollr_spd_cmd;
    checksum = int8_t(headerByte + addressByte + commandByte1 + commandByte2 + commandByte3);
    Serial2.write(headerByte);
    Serial2.write(addressByte);
    Serial2.write(commandByte1); // 昇降機構
    Serial2.write(commandByte2); // ローラー速度設定
    Serial2.write(commandByte3); // ローラー速度増減
    Serial2.write(checksum);
    Serial2.flush(); // 送信が完了するまで待機
    core0b_free_stack = uxTaskGetStackHighWaterMark(NULL);
    delay(10);
  }
}

void Core1b(void *args) {
  while (1) {
    as5600_tca9548a_get_current_angle(current_angle, offset1, offset2);
    // Serial.print(String(current_angle[0]) + ", " + String(current_angle[1]) + ", " + String(current_angle[2]) + ", " + String(current_angle[3]));
    // Serial.print(" | ");
    // Serial.print(String(w_w[0]) + ", " + String(w_w[1]) + ", " + String(w_w[2]) + ", " + String(w_w[3]));
    // Serial.print(" | ");
    // Serial.print(String(core0a_free_stack) + ", " + String(core0b_free_stack) + ", " + String(core1b_free_stack) + ", " + String(core1m_free_stack));
    // Serial.println();
    core1b_free_stack = uxTaskGetStackHighWaterMark(NULL);
    delay(10);
  }
}