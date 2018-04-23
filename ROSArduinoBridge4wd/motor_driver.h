/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 8
  #define LEFT_MOTOR_BACKWARD  7
  #define RIGHT_MOTOR_FORWARD  13
  #define LEFT_MOTOR_FORWARD   12

  #define RIGHT_MOTOR_ENABLE 6
  #define LEFT_MOTOR_ENABLE 9
  
  #define RIGHT2_MOTOR_ENABLE 10
  #define LEFT2_MOTOR_ENABLE 11
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed, int left2Speed, int right2Speed);
