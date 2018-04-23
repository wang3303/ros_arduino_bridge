/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
  	pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT); 
  	pinMode(LEFT_MOTOR_BACKWARD, OUTPUT); 
  	pinMode(RIGHT_MOTOR_FORWARD, OUTPUT); 
  	pinMode(LEFT_MOTOR_FORWARD, OUTPUT); 
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
    
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == RIGHT) { 
      if      (reverse == 0) { digitalWrite(RIGHT_MOTOR_FORWARD, LOW); digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH); analogWrite(RIGHT_MOTOR_ENABLE,spd);}
      else if (reverse == 1) { digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); analogWrite(RIGHT_MOTOR_ENABLE,spd);}
    }
    else if (i == LEFT) {
      if      (reverse == 0) { digitalWrite(LEFT_MOTOR_FORWARD, LOW); digitalWrite(LEFT_MOTOR_BACKWARD, HIGH); analogWrite(LEFT_MOTOR_ENABLE,spd);}
      else if (reverse == 1) { digitalWrite(LEFT_MOTOR_BACKWARD, LOW); digitalWrite(LEFT_MOTOR_FORWARD, HIGH); analogWrite(LEFT_MOTOR_ENABLE,spd);}
    }
    else if (i == RIGHT2) { // set second wheel speed with same direction
      analogWrite(RIGHT2_MOTOR_ENABLE,spd);
    }
    else if (i == LEFT2) {
      analogWrite(LEFT2_MOTOR_ENABLE,spd);
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed, int left2Speed, int right2Speed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
    setMotorSpeed(LEFT2, left2Speed);
    setMotorSpeed(RIGHT2, right2Speed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
