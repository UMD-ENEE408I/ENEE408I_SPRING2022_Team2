// COMPENSATES COMMAND VS PERFORMANCE


// ______ LIBRARYS______

#include <Adafruit_MCP3008.h>
#include <Encoder.h>




// ______ DEFINITIONS ______

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;





// ______ GLOBAL VARIABLES ______


short a = 0 ;
float error_line = 0 ;
//float error_speed = 0 ;
float P_correction_line = 0 ;
float P_correction_speed = 0 ;
unsigned int sample_time = 100 ;
uint16_t light_bar_values[ 16 ] ;
unsigned int last_time_stamp = 0 ;
float light_bar_values_normalized[ 13 ] ;
unsigned int previous_speed_time = 0 ;
unsigned int previous_enc_one_value = 0 ;
unsigned int previous_enc_two_value = 0 ;
float error_wheel_one_speed = 0 ;
float error_wheel_two_speed = 0 ;

float P_correction_wheel_one_speed = 0 ;
float P_correction_wheel_two_speed = 0 ;

float wheel_one_speed = 0 ;
float wheel_two_speed = 0 ;

float motor_one_command = 0 ;
float motor_two_command = 0 ;

float motor_one_request = 0 ;
float motor_two_request = 0 ;

float Kp_line = 10 ;
float Kp_speed = 30 ;

const short ADC_1_CS = A3 ;
const short ADC_2_CS = A2 ;
const short RF_CS = A4 ;

const unsigned short M1_IN_1 = 2;
const unsigned short M1_IN_2 = 3;
const unsigned short M2_IN_1 = 5;
const unsigned short M2_IN_2 = 4;

const unsigned short M1_ENC_A = 6;
const unsigned short M1_ENC_B = 7;
const unsigned short M2_ENC_A = 8;
const unsigned short M2_ENC_B = 9;






// _______ OBJECTS _______

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);






// ______ FUNCTION DECLARATION ______

void light_bar_sensor() ;
void error_calculator_line() ;
void print_statistics() ;
void proportion_control_line() ;
void speedometor() ;
void error_calculator_speed() ;
void proportion_control_speed() ;





// ______ SETUP ______

void setup() 
{
  
  Serial.begin(9600);

  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH); // Without this the nRF24 will write to the SPI bus while the ADC's are also talking

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  

}







// ______ MAIN ______                                <---------- M A I N ---------<

void loop() 
{


  // CYCLE CONTROLER PER UNIT OF TIME
  if( ( millis() - last_time_stamp ) >= sample_time )
  {
  
      // UPDATE TIME STAMP
      last_time_stamp =  millis() ;
    
      // GRAB LIGHT BAR SENSOR VALUES
      light_bar_sensor() ;
    
      // CALCULATE LINE ERROR
      error_clalculator_line() ;

      // PROPORTIONAL CONTROL LINE
      proportion_control_line() ;

      // CALCULATE SPEED
      speedometor() ;

      // CALCULATE SPEED ERROR
      error_calculator_speed() ; 

      // PROPORTIONAL CONTROL SPEED
      proportion_control_speed() ;
  
      // INTEGRAL CONTROL
      //integral_control() ;

      // DERIVATIVE CONTROL
      //derivative_control() ;    




      // APPLY PID CONTROL OF REQUEST
      motor_one_request = motor_one_request - P_correction_line ;
      motor_two_request = motor_two_request + P_correction_line ;


      // HARD-LINED REQUEST VALUE LIMITS
      if ( motor_one_request > 1.5 )  motor_one_request = 1.5 ;
      if ( motor_one_request < 0 )  motor_one_request = 0 ;

      if ( motor_two_request > 1.5 )  motor_two_request = 1.5 ;
      if ( motor_two_request < 0 )  motor_two_request = 0 ;  


      // APPLY CONTROL VALUES
      motor_one_command = motor_one_request + P_correction_wheel_one_speed ;
      motor_two_command = motor_two_request + P_correction_wheel_two_speed ;


      // APPLY HARD-LINED CONTROL VALUE LIMITS
      if ( motor_one_command > 60 )  motor_one_command = 60 ;
      if ( motor_one_command < 0 )  motor_one_command = 0 ;

      if ( motor_two_command > 60 )  motor_two_command = 60 ;
      if ( motor_two_command < 0 )  motor_two_command = 0 ;      



      // SEND OUT COMMANDS TO MOTORS
      analogWrite( M1_IN_1 , 0 ) ;
      analogWrite( M1_IN_2 , 0 ) ;
      
      analogWrite( M2_IN_1 , 0 ) ;
      analogWrite( M2_IN_2 , 0 ) ;      



      // PRINT STATISTICS 
      print_statistics() ;


  } // <--if()

 // delay(25);
  
} // <-- main()










// ____________________ FUNCTION LAND ____________________
// _______________________________________________________



// _________ LIGHT BAR SENSOR () _________


void light_bar_sensor()
{


  // LOCAL VARIABLES 
  uint16_t adc1_buf[8] ;
  uint16_t adc2_buf[8] ;
  //uint16_t light_bar_values[ 16 ] ;


  
  // READ IN VALUES
  for ( a = 0 ; a < 8 ; a++ ) 
  {
      adc1_buf[ a ] = adc1.readADC( a ) ;
      adc2_buf[ a ] = adc2.readADC( a ) ;   
  }



  // COMBINE INTO ONE ARRAY
  light_bar_values[ 12 ] = adc1_buf[ 0 ] ;
  light_bar_values[ 11 ] = adc2_buf[ 0 ] ;
  light_bar_values[ 10 ] = adc1_buf[ 1 ] ;
  light_bar_values[ 9 ] = adc2_buf[ 1 ] ;
  light_bar_values[ 8 ] = adc1_buf[ 2 ] ;
  light_bar_values[ 7 ] = adc2_buf[ 2 ] ;
  light_bar_values[ 6 ] = adc1_buf[ 3 ] ;
  light_bar_values[ 5 ] = adc2_buf[ 3 ] ;
  light_bar_values[ 4 ] = adc1_buf[ 4 ] ;
  light_bar_values[ 3 ] = adc2_buf[ 4 ] ;
  light_bar_values[ 2 ] = adc1_buf[ 5 ] ;
  light_bar_values[ 1 ] = adc2_buf[ 5 ] ;
  light_bar_values[ 0 ] = adc1_buf[ 6 ] ;



} // <-- light_bar_sensor()









// ______ ERROR CALCULATOR LINE () ______

void error_clalculator_line()
{

  // LOCAL VARIABLES
  int light_bar_values_sum = 0 ;
  //float light_bar_values_normalized[ 13 ] ; // <-- GLOBAL
  float light_bar_values_normalized_leftside_sum = 0 ;
  float light_bar_values_normalized_rightside_sum = 0 ;
  //float error = 0 ; // <-- GLOBAL


  // FIND THE SUM OF THE ARRAY
  for ( a = 0 ; a < 13 ; a++ ) 
  {
      light_bar_values_sum = light_bar_values_sum + light_bar_values[ a ] ;   
  }


  // NORMALIZE BY DIVIDING BY THE SUM OF THE ARRAY
  for ( a = 0 ; a < 13 ; a++ )
  {
      light_bar_values_normalized[ a ] = (float)light_bar_values[ a ] / (float)light_bar_values_sum ;   
  }


  // ADD UP LEFT SIDE
  for ( a = 0 ; a < 6 ; a++ )
  {
      light_bar_values_normalized_leftside_sum = light_bar_values_normalized_leftside_sum + light_bar_values_normalized[ a ] ;   
  }


  // ADD UP RIGHT SIDE
  for ( a = 7 ; a < 13 ; a++ )
  {
      light_bar_values_normalized_rightside_sum = light_bar_values_normalized_rightside_sum + light_bar_values_normalized[ a ] ;     
  }


  // CALCULATE ERROR
  error_line = light_bar_values_normalized_rightside_sum - light_bar_values_normalized_leftside_sum ;



} // <-- error_clalculator_line()










// ______ PROPORTION CONTROL ()______

void proportion_control_line()
{
    
    // MULTIPLY ERROR WITH Kp GAIN CONSTANT
    P_correction_line = Kp_line * error_line ;

} // <--- proportional_control_line()















// ______ SPEDOMETOR ()______

void speedometor()
{

  // VARIABLES
  unsigned int time_diff = 0 ;
  // unsigned int previous_speed_time = 0 ; // <-- GLOBAL
  int enc_one_diff = 0 ; // <-- GLOBAL
  int enc_two_diff = 0 ; // <-- GLOBAL
  //unsigned int previous_enc_one_value = 0 ; // <-- GLOBAL
  //unsigned int previous_enc_two_value = 0 ; // <-- GLOBAL
  float linear_one_diff = 0 ;
  float linear_two_diff = 0 ;
  //unsigned int wheel_one_speed = 0 ; // <-- GLOBAL
  //unsigned int wheel_two_speed = 0 ; // <-- GLOBAL



  // DIFFERENCE IN TIME
  time_diff = millis() - previous_speed_time ;


  // DIFFERENCE IN POSITION
  enc_one_diff = enc1.read() - previous_enc_one_value ;
  enc_two_diff = enc2.read() - previous_enc_two_value ;


  //CONVERT TO CM
  linear_one_diff = (float)enc_one_diff * (float)10 / (float)360 * (float)1000 ;
  linear_two_diff = (float)enc_two_diff * (float)10 / (float)360 * (float)1000 ;


  // CALCULATE SPEED
  wheel_one_speed = (float)linear_one_diff / (float)time_diff ;
  wheel_two_speed = (float)linear_two_diff / (float)time_diff * (float)-1;


  // UPDATE ENCODER
  previous_enc_one_value = enc1.read() ;
  previous_enc_two_value = enc2.read() ;
  

  // UPDATE TIMESTAMP
  previous_speed_time = millis() ;
  

  // RESET TIME_DIFF
  time_diff = 0 ;





} // <--- spedometor()
















// ______ ERROR CALCULATOR SPEED () ______

void error_calculator_speed()
{

  // CALCULATE SPEED ERROR
  error_wheel_one_speed = motor_one_request - wheel_one_speed ;
  error_wheel_two_speed = motor_two_request - wheel_two_speed ;


} // <-- error_clalculator_speed()










// ______ PROPORTION CONTROL SPEED ()______

void proportion_control_speed()
{
    
    // MULTIPLY SPEED ERROR WITH Kp GAIN CONSTANT
    P_correction_wheel_one_speed = Kp_speed * error_wheel_one_speed ;
    P_correction_wheel_two_speed = Kp_speed * error_wheel_two_speed ;
    

} // <--- proportional_control_line()














// ______ PRINT STATISTICS () ______

void print_statistics()
{

/*
 
  // PRINT LIGHT BAR ARRAY VALUES
  for ( a = 0 ; a < 13 ; a++ ) 
  {
      Serial.print( light_bar_values[ a ] ) ; 
      Serial.print("\t") ;    
  }

  
  // NEW LINE
  Serial.println() ;


  // PRINT NORMALIZED LIGHT BAR ARRAY VALUES
  for ( a = 0 ; a < 13 ; a++ ) 
  {
      Serial.print( light_bar_values_normalized[ a ] ) ; 
      Serial.print("\t") ;    
  }
  
*/

/*
  // PRINT ERROR LINE
  Serial.print( "Err_ln: " ) ; 
  Serial.print( error_line , 4 ) ; 
  Serial.print("\t") ; 
*/

  // PRINT ERROR SPEED
  Serial.print( "Err_one_speed: " ) ; 
  Serial.print( error_wheel_one_speed , 4 ) ; 
  //Serial.print("\t") ; 
  
/*
  // PRINT ERROR SPEED
  Serial.print( "Err_two_speed: " ) ; 
  Serial.print( error_wheel_two_speed , 4 ) ; 
  Serial.print("\t") ; 
  

  // PROPORTIONAL CORRECTION LINE
  Serial.print( "P_cor_ln: " ) ; 
  Serial.print( P_correction_line ) ; 
  Serial.print("\t") ;
*/

  // PROPORTIONAL CORRECTION LINE
  Serial.print( "P_cor_speed: " ) ; 
  Serial.print( P_correction_speed ) ; 
  //Serial.print("\t") ;

  // MOTOR_ONE_REQUEST
  Serial.print( "Mtr_rqst_one: " ) ; 
  Serial.print( motor_one_request ) ; 
  //Serial.print("\t") ; 

  // MOTOR_ONE_COMMAND
  Serial.print( "Mtr_cmd_one: " ) ; 
  Serial.print( motor_one_command/10 ) ; 
  //Serial.print("\t") ; 
  
/*
  // WHEEL SPEED
  Serial.print( "Whl_spd_one: " ) ; 
  Serial.print( wheel_one_speed ) ; 
  Serial.print("\t") ;

  // MOTOR_TWO_REQUEST
  Serial.print( "Mtr_rqst_two: " ) ; 
  Serial.print( motor_two_request ) ; 
  Serial.print("\t") ; 

  // MOTOR_TWO_COMMAND
  Serial.print( "Mtr_cmd_two: " ) ; 
  Serial.print( motor_two_command ) ; 
  Serial.print("\t") ; 

  // WHEEL SPEED
  Serial.print( "Whl_spd_two: " ) ; 
  Serial.print( wheel_two_speed ) ; 
  Serial.print("\t") ;
*/

  // NEW LINE
  Serial.println() ;


} // <-- print_statistics()
