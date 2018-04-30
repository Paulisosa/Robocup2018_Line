#include "mbed.h"

#define MASK_SIGN 0x80000000

union speed_data{
  float val;
  int32_t dir;
};


class MotorDC
{
  public:
    /** Create and initializate MotorDC instance
    * @param pin_pwm A PwmOut pin, driving the H-bridge enable line to control the speed
    * @param pin_dir0 A DigitalOut, set direction motor pin0
    * @param pin_dir1 A DigitalOut, set direction motor pin1
    */
    MotorDC(PinName pin_pwm, PinName pin_dir0, PinName pin_dir1);
    /**
     * Set the speed of the Motor
     * @param fspeed speed between -1.0 and 1.0
     */
    void setSpeed(float fspeed);
    float operator =(float fspeed);

  private:

    union speed_data speed;
    PwmOut pwmout;
    DigitalOut dir_motor0;
    DigitalOut dir_motor1;
};
