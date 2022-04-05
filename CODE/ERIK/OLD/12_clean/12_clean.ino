



// ______ LIBRARYS______

//#include <Adafruit_MCP3008.h>
#include <Encoder.h>
#include <PID_v1.h>






// ______ DEFINITIONS ______

/*
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;
*/





// ______ GLOBAL VARIABLES ______


// SPEDOMETER

  int enc_one_diff = 0 ;
  int enc_two_diff = 0 ;
  float wheel_one_speed = 0 ;
  float wheel_two_speed = 0 ;
  float linear_one_diff = 0 ;
  float linear_two_diff = 0 ;
  unsigned int time_diff = 0 ;
  unsigned int spd_chk_prev = 0 ;
  int previous_enc_one_value = 0 ;
  int previous_enc_two_value = 0 ;
  unsigned int previous_speed_time = 0 ;



// PID CONTROLLERS

  double whl_1_vl_Kp = 20 ;
  double whl_1_vl_Ki = 30 ;
  double whl_1_vl_Kd = 0 ;
  
  double whl_1_vl_setpoint = 3 ;
  double whl_1_vl_input = 0 ;
  double whl_1_vl_output = 0 ;



// HARDWARE PINS

  /*
  const short RF_CS = A4 ;
  const short ADC_1_CS = A3 ;
  const short ADC_2_CS = A2 ;
  */
  
  const unsigned short M1_IN_1 = 2 ;
  const unsigned short M1_IN_2 = 3 ;
  const unsigned short M2_IN_1 = 5 ;
  const unsigned short M2_IN_2 = 4 ;
  
  const unsigned short M1_ENC_A = 6 ;
  const unsigned short M1_ENC_B = 7 ;
  const unsigned short M2_ENC_A = 8 ;
  const unsigned short M2_ENC_B = 9 ;






// _______ OBJECTS _______

Encoder enc1( M1_ENC_A, M1_ENC_B ) ;
Encoder enc2( M2_ENC_A, M2_ENC_B ) ;

PID whl_1_vl_PID( &whl_1_vl_input, &whl_1_vl_output, &whl_1_vl_setpoint, whl_1_vl_Kp, whl_1_vl_Ki, whl_1_vl_Kd, DIRECT ) ;







// ______ FUNCTION DECLARATION ______

void speedometor() ;








// ______ SETUP ______

void setup() 
{

  // BEGIN SERIAL
  Serial.begin(9600) ;

  // RADIO?
  /*
  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH); // Without this the nRF24 will write to the SPI bus while the ADC's are also talking
  */

  // ADC INITIATION
  /*
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  
  */

  // PID SETTINGS
  whl_1_vl_PID.SetMode( AUTOMATIC ) ;
  whl_1_vl_PID.SetOutputLimits ( -255, 255 ) ;
  whl_1_vl_PID.SetSampleTime( 65 ) ;

} // <-- setup()












// _____________ MAIN _____________                                <---------- M A I N ---------<

void loop() 
{

  // CALCULATE SPEED OF WHEELS
  if( ( millis() - spd_chk_prev ) > 60 ) speedometor() ;

  // MAP SETPOINT VALUE
  //whl_1_vl_setpoint = map( whl_1_vl_setpoint , -257 , 245 , -255 , 255  ) ;

  // MAP INPUT VALUE
  //whl_1_vl_input = (float)map( (float)wheel_one_speed , (float)-257 , (float)245 , (float)-255 , (float)255 ) ;  // <-- This is probabbly not a line of best fit mapping.
  whl_1_vl_input = wheel_one_speed ;


  // COMPUTE PID OUTPUT
  whl_1_vl_PID.Compute() ;






  // COMMAND MOTORS
  if ( whl_1_vl_output < 0 ) 
  {
      analogWrite( M1_IN_1 , whl_1_vl_output ) ;
      analogWrite( M1_IN_2 , 0 ) ;
  }
  else
  {
      analogWrite( M1_IN_1 , 0 ) ;
      analogWrite( M1_IN_2 , whl_1_vl_output ) ;  
  }




Serial.println() ;
  
/*
  // PRINT
  Serial.print( "cm/s: " ) ; 
  Serial.print( wheel_one_speed, 3 ) ; 
  Serial.println("\t") ;
*/
  // PRINT
  Serial.print( "whl_1_vl Set: " ) ; 
  Serial.print( whl_1_vl_setpoint, 3 ) ; 
  //Serial.print("\t") ;
/*
  // PRINT
  Serial.print( "whl_1_vl In: " ) ; 
  Serial.print( whl_1_vl_input, 3 ) ; 
  Serial.print("\t") ;
*/

  // PRINT
  Serial.print( "       whl_1_vl Out: " ) ; 
  Serial.print( whl_1_vl_output, 3 ) ; 
  //Serial.print("\t") ;

  // PRINT
  Serial.print( "       error_vl: " ) ; 
  Serial.print( whl_1_vl_setpoint - whl_1_vl_input, 3 ) ; 
  //Serial.print("\t") ;


  // DELAY FOR SPEDOMETOR
  //delay( 60 );



} // <-- main()                                                     <---------- E N D ---------< 



















// __________________________ FUNCTION LAND __________________________
// ___________________________________________________________________





// _________ SPEDOMETOR () _________

void speedometor()
{

  // DIFFERENCE IN TIME
  time_diff = micros() - previous_speed_time ;


  // DIFFERENCE IN POSITION
  enc_one_diff = enc1.read() - previous_enc_one_value ;
  enc_two_diff = enc2.read() - previous_enc_two_value ;


  //CONVERT TO CM
  linear_one_diff = (float)enc_one_diff * (float)10 / (float)360 * (float)1000000 ;
  linear_two_diff = (float)enc_two_diff * (float)10 / (float)360 * (float)1000000 ;


  // CALCULATE SPEED
  wheel_one_speed = (float)linear_one_diff / (float)time_diff ;
  wheel_two_speed = (float)linear_two_diff / (float)time_diff * (float)-1;


  // UPDATE ENCODER 'PREVIOUS' VALUES
  previous_enc_one_value = enc1.read() ;
  previous_enc_two_value = enc2.read() ;
  

  // UPDATE TIMESTAMP
  previous_speed_time = micros() ;


  // UPDATE FUNCTION TIMESTAMP
  spd_chk_prev = millis() ;


} // <--- spedometor()
