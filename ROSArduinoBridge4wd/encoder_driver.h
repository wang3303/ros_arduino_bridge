/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER

  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  #define LEFT2_ENC_PIN_A PD4  //pin 4
  #define LEFT2_ENC_PIN_B PD5   //pin 5

  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5

  #define RIGHT2_ENC_PIN_A PC0  //pin A0
  #define RIGHT2_ENC_PIN_B PC1   //pin A1

  
  
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

