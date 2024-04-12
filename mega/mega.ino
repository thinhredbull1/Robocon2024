#include <Wire.h>
#include "profile.h"
Profile forward_x;
Profile forward_y;
Profile rotate;
float x_pos = 0;
float y_pos = 0;
float theta_pos = 0;
float coord_now[3] = { 0, 0, 0 };
int posPrev[] = { 0, 0 };
long pos[] = { 0, 0, 0, 0 };
long pos_delta[] = { 0, 0, 0, 0 };
float speed_x_max = 120;
int speed_desired[4];
bool start_run = 0;
float dx = 0;
float dy = 0;
bool start_send = 0;
float d_theta = 0;
float p[3] = { 0.885, 0.88, 1.95 };
float d[3] = { 28.5, 28.5, 8.2 };
float output_speed[] = { 0, 0, 0 };
float last_e[3]={0,0,0};
float error[3] = { 0, 0, 0 };
void move_gripper(int speed)
{
  if(speed==0){
    digitalWrite(pwm,0);
    digitalWrite(dir,0);
    return;
  }
  bool dir_=speed>0?1:0;
  if(dir_){
    analogWrite(pwm,speed);
    digitalWrite(dir,0);
  }
  else{
    analogWrite(dir,abs(speed));
    digitalWrite(pwm,0);
  }
}
void position_compute(float des_coord[]) {
  float d_input[3] = { 0, 0, 0 };
  for (int i = 0; i < 2; i++) {
    error[i] += des_coord[i] - coord_now[i];
    d_input[i] = error[i] - last_e[i];
    last_e[i] = error[i];
    output_speed[i] = p[i] * error[i] + d_input[i] * d[i];
  }
  error[2] += des_coord[2] - coord_now[2];
  d_input[2] = error[2] - last_e[2];
  last_e[2] = error[2];
  output_speed[2] = p[2] * error[2] + d_input[2] * d[2];
}
void reset_all() {
  for (int i = 0; i < 3; i++) {
    error[i] = 0;
    last_e[i] = 0;
    output_speed[i] = 0;
  }
}
// speed_ = (delta*cm_per_count)/delta_time // cm/s -> delta=(speed_*dt)/cm_per_count
void calculate_speed_robot(float x, float y, float theta, float angle_robot_now) {
  // x -> cm/s y-> cm/s
  float wheel_ang[] = { 45, 315, 135, 225 };
  float sin_wheel[4];
  float cos_wheel[4];
  float delta_now[4];
  float cm_s[4];
  for (int i = 0; i < 4; i++) {
    wheel_ang[i] += theta_pos;
    wheel_ang[i] = wheel_ang[i] * PI / 180.0;
    sin_wheel[i] =  -sin(wheel_ang[i]);
    cos_wheel[i] = cos(wheel_ang[i]);
    cm_s[i] = x * sin_wheel[i] + y * cos_wheel[i] + theta * R;
    speed_desired[i] = roundFloat(delta_cvt(cm_s[i]));
    if (i < 2) speed_desired[i] = -speed_desired[i];
  }
}
void update_pid()
{
  forward_x.update();
  forward_y.update();
  rotate.update();
  // Serial.println(forward_x.speed());
  float speed_x = forward_x.speed();
  float speed_y = forward_y.speed();
  float turn = rotate.speed();
  float real_speed_X = 0;
  float real_speed_Y = 0;
  float real_speed_theta = 0;
  float des_coord[3] = { forward_x.increment(), forward_y.increment(), rotate.increment()};
  coord_now[0] = dx;
  coord_now[1] = dy;
  coord_now[2] = d_theta;
  // float des_coord[3] = { forward_x.position(), forward_y.position(), rotate.increment()};
  // coord_now[0] = x_pos;
  // coord_now[1] = y_pos;
  // coord_now[2] = d_theta;
  position_compute(des_coord);
  real_speed_X += speed_x;
  real_speed_Y += speed_y;
  real_speed_theta += turn;
  real_speed_X += output_speed[0];
  real_speed_Y += output_speed[1];
  real_speed_theta += output_speed[2];
  calculate_speed_robot(real_speed_X, real_speed_Y, real_speed_theta * PI / 180.0, 0);
}
void send_motor() {
  // ;
  Wire.beginTransmission(0x01);
  sendByte(speed_desired[M0]);
  sendByte(speed_desired[M1]);
  Wire.endTransmission();
  Wire.beginTransmission(0x02);
  sendByte(speed_desired[M2]);
  sendByte(speed_desired[M3]);
  Wire.endTransmission();
}
void rec_motor() {
  float wheel_ang[] = { 45, 315, 135, 225 };
  Wire.requestFrom(1, 8);
  //   while (Wire.available() < 8);
  pos_delta[M0] = receiveLong();
  pos_delta[M1] = receiveLong();
  //   while (Wire.available() < 8);
  Wire.requestFrom(2, 8);
  pos_delta[M2] = receiveLong();
  pos_delta[M3] = receiveLong();
  float sin_wheel[4];
  float cos_wheel[4];
  float delta_now[4];
  float cm_s[4];
  dx = 0;
  dy = 0;
  d_theta = 0;
  for (int i = 0; i < 4; i++) {
    if (i < 2) pos_delta[i] = -pos_delta[i];
    pos[i] += pos_delta[i];
    wheel_ang[i] += theta_pos;
    wheel_ang[i] = wheel_ang[i] * PI / 180.0;
    sin_wheel[i] = -sin(wheel_ang[i]);
    cos_wheel[i] = cos(wheel_ang[i]);
    cm_s[i] = (pos_delta[i] * cm_per_count);
    dx += 0.5 * cm_s[i] * sin_wheel[i];
    dy += 0.5 * cm_s[i] * cos_wheel[i];
    d_theta += (0.25 / R) * cm_s[i];
    // speed_desired[i]=delta_cvt();
    // Serial.println(speed_desired[i]);
  }
  d_theta = d_theta * 180 / PI;
  update_pid();
  send_motor();
  x_pos += dx;
  y_pos += dy;
  theta_pos += d_theta;
  static uint8_t count_print = 0;
  if (debug_speed) {
    if (start_send) {
      if (!forward_x.is_finished() || !forward_y.is_finished() || !rotate.is_finished()) {
        Serial.print(dx);
        Serial.print(",");
        Serial.print(forward_x.increment());
        Serial.print(",");
        Serial.print(dy);
        Serial.print(",");
        Serial.println(forward_y.increment());
      } else {
        Serial.print(dx);
        Serial.print(",");
        Serial.print(forward_x.increment());
        Serial.print(",");
        Serial.print(dy);
        Serial.print(",");
        Serial.println(forward_y.increment());
        Serial.println("done");
        Serial.print(x_pos);
        Serial.print(" ");
        Serial.print(y_pos);
        Serial.print(" ");
        Serial.println(theta_pos);
        start_send = 0;
        x_pos = 0;
        y_pos = 0;
        theta_pos = 0;
        forward_x.reset();
        forward_y.reset();
        rotate.reset();
      }
    }
  }
}
void setup() {
  // put your setup code here, to run once:
  Wire.begin();  // join i2c bus
  Wire.setClock(400000);
  Serial.begin(9600);
  Serial.setTimeout(200);
  // if (!bno.begin()) {
  //   Serial.print("No BNO055 detected");
  //   while (1)
  //     ;
  // }

  pinMode(29, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  pinMode(pwm,OUTPUT);
  pinMode(dir,OUTPUT);
  // calculate_speed_robot(60, 60, 0,0);
  delay(3000);  // wait i2c setup
}

void loop() {
  static unsigned long time_rec = millis();
  static unsigned long time_send = millis();
  static unsigned long time_stop = millis();

  if (Serial.available()) {
    String c = Serial.readString();
    int index_test = c.indexOf(",");
    int index_kp_desired = c.indexOf(":");
  if (index_test != -1) {
      int index_2 = c.indexOf("#");
      float x_start = c.substring(0, index_test).toFloat();
      float y_start = c.substring(index_test + 1, index_2).toFloat();
      float rotate_start = c.substring(index_2 + 1).toFloat();
      forward_x.start(x_start, 60, 0, 200);
      forward_y.start(y_start, 60, 0, 200);
      rotate.start(rotate_start, 90, 0, 120);
      start_send = 1;
    } else if (index_kp_desired != -1) {
      int index_cal = c.indexOf("#");
      int index_cal_cal = c.indexOf("*");
      if (index_cal != -1) {
        p[0] = c.substring(0, index_kp_desired).toFloat();
        d[0] = c.substring(index_kp_desired + 1, index_cal).toFloat();
        p[1] = c.substring(index_cal + 1, index_cal_cal).toFloat();
        d[1] = c.substring(index_cal_cal + 1).toFloat();
        Serial.print(p[0]);
        Serial.print(" ");
        Serial.print(d[0]);
        Serial.print(" ");
        Serial.print(p[1]);
        Serial.print(" ");
        Serial.println(d[1]);
        reset_all();
      }
    }
  }
 

  // callFunctionPeriodically(send_motor, LOOP_SEND, time_send);
  callFunctionPeriodically(rec_motor, LOOP_REC, time_rec);
}
