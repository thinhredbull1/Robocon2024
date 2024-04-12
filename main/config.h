#include<Arduino.h>
class SimplePID{
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral,last_u;
  public:
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
     void reset_all()
    {
        eintegral=0;
        eprev=0;
    } 
    void setParams(float kpIn, float kiIn, float kdIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
      reset_all();
    }
    float compute(int value, int target, float deltaT){
      if(target==0){
        reset_all();
        return 0;
      }
      int e = target - value;
      float dedt = (e-eprev)/(deltaT);
      if(abs((int)last_u) >= umax && (((e >= 0) && (eintegral >= 0)) || ((e < 0) && (eintegral < 0))))
      {
        eintegral = eintegral;
      }
      else{
        eintegral+=e*deltaT;
      }
      eintegral=constrain(eintegral,-120,120);
      float u = kp*e + kd*dedt + ki*eintegral;
      if( u > umax)u = umax;
      else if(u<-umax)u=-umax;
      last_u=u;
      eprev = e;
      return u;
    }
    float GetKp(){return kp;}
    float GetKi(){return ki;}
    float GetKd(){return kd;}
   
};
#define NMOTORS 2
#define M0 0
#define M1 1
const bool test_ff=0;
const int enca[] = {4,2};
const int encb[]= {5,3};
const int pwm[] = {10,9}; //{10,11}
const int dir[] = {8,7};

const float LOOP_FREQUENCY=100.0;
const float speed_ff=10.0; // 100 Hz
const bool serial_tune = 0;
const int time_run_test=2000;
// const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
// const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const unsigned long micros_interval=(1.0/LOOP_FREQUENCY)*1e6;


