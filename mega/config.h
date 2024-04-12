#ifndef CONFIG_H
#define CONFIG_H
#include<Arduino.h>
#define up 1
#define down 0
#define M0 0
#define M1 1
#define M2 2
#define M3 3
#define R 20.15 // do duoc < thuc te --> giam
#define ENCODER_TOTAL 264.85 /// do duoc < thuc te -> giam 
#define WHEEL_DIAMETER 12 // 
#define min_cm_s 20
#define min_cm_y 25
const float cm_per_count = (PI * WHEEL_DIAMETER) / ENCODER_TOTAL;  // cm/count banh
const float LOOP_CONTROL=0.01;
#define delta_cvt(speed) (speed * LOOP_CONTROL) / cm_per_count             // cm/s -> delta send
bool debug_speed=1;
const float LOOP_REC = 40.0;
// const float LOOP_SEND = 40.0;
const float LOOP_INTERVAL=(LOOP_REC/1000.0);
const float LOOP_CVT=LOOP_INTERVAL/LOOP_CONTROL;
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
const int pwm=5; //{10,11}
const int dir=6;
typedef void (*CallbackFunction)();
void callFunctionPeriodically(CallbackFunction functionToCall, unsigned long intervalTime, unsigned long &previousMillis) {
  unsigned long currentMillis = millis();  //
  if (currentMillis - previousMillis >= intervalTime) {
    functionToCall();
    previousMillis = currentMillis;
  }
}
float roundFloat(float number) {
  if (number > 0) {
    return floor(number + 0.5);
  } else {
    return ceil(number - 0.5);
  }
}
void sendByte(int value) {
  for (int k = 0; k < 2; k++) {
    byte out = (value >> 8 * (1 - k)) & 0xFF;
    Wire.write(out);
  }
}
long receiveLong() {
  long outValue;
  for (int k = 0; k < 4; k++) {
    byte nextByte = Wire.read();
    outValue = (outValue << 8) | nextByte;
  }
  return outValue;
}
#endif
