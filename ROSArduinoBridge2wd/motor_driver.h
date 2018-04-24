/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  // Pins can be changed
  // However, ENABLE PINS must be able to support AnalogWrite
  #define RIGHT_MOTOR_BACKWARD 11
  #define LEFT_MOTOR_BACKWARD 12
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 5
  #define LEFT_MOTOR_ENABLE 6
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
