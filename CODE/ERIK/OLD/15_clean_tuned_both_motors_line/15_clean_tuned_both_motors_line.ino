// INPROGRESS
// THOUGHTS:
// PRELOAD SETPOINTS FOR NEGATIVE ROOM. <-- CROSS THIS BRIDGE WHEN WE GET THERE. 
// PUT BACK IN SERIAL READ IN ORDER TO TUNE TURNING. POSSIBLE SET UP TUNNING IN SOME SORT OF WAY WHERE THE ROBOT IS STATIONARY. THEN APPLY MOVING FORWARD.


// QUESTIONS FOR LEVI:
// 1. HOW MUCH SLOWE DOES THE LINE LOOP NEED TO BE.
// 2. MOTORS DO NOT MOVE BACKWARDS.
// 3. THE LOOSE ENCODER WIRE.



// ______ LIBRARYS______

#include <PID_v1.h>
#include <Encoder.h>
#include <Adafruit_MCP3008.h>






// ______ GLOBAL VARIABLES by Function ______



// PID LF CONTROL

  unsigned int prev_line_err_time = 0 ;




// PID VL CONTROL

  float twinky_one = 0 ;
  float twinky_two = 0 ;
  float twinky_one_speed = 0.15 ;
  float twinky_two_speed = 0.15 ;
  unsigned int prev_twinky_time = 0 ;




// LINE ERROR CALC

  int a = 0 ;
  unsigned int light_values_left_sum = 0 ;
  unsigned int light_values_right_sum = 0 ;
  unsigned int light_values[ 16 ] ; // <-- ARRAY OF 16 INCASE OF GOING OUT OF ARRAY BOUNDS SO AS TO NOT RIGHT RANDOM INFORMATION IN RANDOM PLACES IN MEMORY.
  int line_error = 0 ;




// PRINT OUT

  unsigned int prev_print_time = 0 ;




// PID CONTROLLERS

  double whl_1_vl_Kp = 00.9500 ;
  double whl_1_vl_Ki = 60.0000 ;
  double whl_1_vl_Kd = 00.00205 ;

  double whl_2_vl_Kp = 00.9500 ;
  double whl_2_vl_Ki = 60.0000 ;
  double whl_2_vl_Kd = 00.00205 ;

  double line_err_Kp = 00.0001 ;
  double line_err_Kp_test = 00.0001 ;
  double line_err_Ki = 00.0000 ;
  double line_err_Kd = 00.0001 ;
 
  double whl_1_vl_setpoint = 0 ;
  double whl_1_vl_feedback = 0 ;
  double whl_1_vl_output = 0 ;
 
  double whl_2_vl_setpoint = 0 ;
  double whl_2_vl_feedback = 0 ;
  double whl_2_vl_output = 0 ;

  double line_err_setpoint = 0 ;
  double line_err_feedback = 0 ;
  double line_err_output = 0 ;

  float twinky_max = 0.15 ;
  float twinky_min = -0.15 ;







// HARDWARE PINS

  const short RF_CS = A4 ;
  const short ADC_1_CS = A3 ;
  const short ADC_2_CS = A2 ;

  const unsigned short M1_IN_1 = 2 ;
  const unsigned short M1_IN_2 = 3 ;
  const unsigned short M2_IN_1 = 5 ;
  const unsigned short M2_IN_2 = 4 ;
  
  const unsigned short M1_ENC_A = 6 ;
  const unsigned short M1_ENC_B = 7 ;
  const unsigned short M2_ENC_A = 8 ;
  const unsigned short M2_ENC_B = 9 ;







// _______ OBJECTS _______

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

Encoder enc1( M1_ENC_A, M1_ENC_B ) ;
Encoder enc2( M2_ENC_A, M2_ENC_B ) ;

PID whl_1_vl_PID( &whl_1_vl_feedback, &whl_1_vl_output, &whl_1_vl_setpoint, whl_1_vl_Kp, whl_1_vl_Ki, whl_1_vl_Kd, DIRECT ) ;
PID whl_2_vl_PID( &whl_2_vl_feedback, &whl_2_vl_output, &whl_2_vl_setpoint, whl_2_vl_Kp, whl_2_vl_Ki, whl_2_vl_Kd, DIRECT ) ;
PID line_err_PID( &line_err_feedback, &line_err_output, &line_err_setpoint, line_err_Kp, line_err_Ki, line_err_Kd, DIRECT ) ;





// ______ FUNCTION DECLARATION ______

void printout() ;
void speedometor() ;
void command_motors() ;
void pid_vl_control() ;
void pid_lf_control() ;
void line_error_calc() ;
void light_sensor_read() ;





// ______ SETUP ______

void setup() 
{

  // BEGIN SERIAL
  Serial.begin(9600) ;

  // RADIO?
  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH); // Without this the nRF24 will write to the SPI bus while the ADC's are also talking
  
  // ADC INITIATION
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  
  
  // PID SETTINGS
  whl_1_vl_PID.SetMode( AUTOMATIC ) ;
  whl_1_vl_PID.SetOutputLimits ( -255, 255 ) ;
  whl_1_vl_PID.SetSampleTime( 1 ) ;

  whl_2_vl_PID.SetMode( AUTOMATIC ) ;
  whl_2_vl_PID.SetOutputLimits ( -255, 255 ) ;
  whl_2_vl_PID.SetSampleTime( 1 ) ;

  line_err_PID.SetMode( AUTOMATIC ) ;
  line_err_PID.SetOutputLimits ( -1 , 1 ) ;
  line_err_PID.SetSampleTime( 1 ) ;


} // <-- setup()












// _____________ MAIN _____________                                <---------- M A I N ---------<

void loop() 
{


    // LINE FOLLOW PID CONTROL
    if( ( millis() - prev_line_err_time ) > 40 ) pid_lf_control() ;  // <-- NOT SURE HOW MUCH SLOWER THIS LOOP SHOULD BE.
  
    // MOTOR PID CONTROL
    if( ( millis() - prev_twinky_time ) > 20 ) pid_vl_control() ;
  
    // PRINTOUT
    if( ( millis() - prev_print_time ) > 100 ) printout() ; 



} // <-- main()                                                     <---------- E N D ---------< 



















// ___________________________________________ FUNCTION LAND ___________________________________________
// _____________________________________________________________________________________________________










// ______ PID LF CONTROL () ______

void pid_lf_control()
{
  
        // UPDATE TIME STAMP
        prev_line_err_time = millis() ;

        // CALCULATE LINE ERROR RAW VALUE
        line_error_calc() ;

        // COPY IN ERROR VALUE
        line_err_feedback = line_error ;

        // COMPUTE PID LF OUTPUT
        line_err_PID.Compute() ;
        
        // SCALE TWINKY SPEED FOR LEFT WHEEL M1
        //twinky_one_speed = twinky_max * line_err_output ;
        twinky_one_speed = twinky_max * line_error * line_err_Kp_test ;

        // SCALE TWINKY SPEEED FOR RIGHT WHEEL M2
        // twinky_two_speed = twinky_max * line_err_output ;  // <-- DO NOT NEED TO MAKE NEGATIVE BECAUSE MOTOR II IS ALREADY MOUNTED BACKWARDS?


} // <-- pid_lf_control()
















// ______ PID VL CONRTOL () ______

void pid_vl_control()
{
  
    // MOVE THE TWINKY FORWARD 
    twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed ;  
    twinky_two = twinky_two + ( millis() - prev_twinky_time ) * twinky_two_speed ;
  
    // UPDATE TIME STAMP
    prev_twinky_time = millis() ;
  
    // COPY WHEELSPEED VALUE
    whl_1_vl_setpoint = twinky_one ;
    whl_2_vl_setpoint = twinky_two ;
  
    // GRAB FEEDBACK TICKS
    whl_1_vl_feedback = enc1.read() ;
    whl_2_vl_feedback = enc2.read() * -1 ;
  
    // COMPUTE PID VL OUTPUT
    whl_1_vl_PID.Compute() ;
    whl_2_vl_PID.Compute() ;
  
    // COMMAND MOTORS
    command_motors() ;


} // <-- pid_vl_control()





















// ______ LINE ERROR CALC () ______

void line_error_calc()
{

    // READ IN LIGHT VALUES
    light_sensor_read() ;
  
    // ZERO-OUT SUMS
    light_values_left_sum = 0 ;
    light_values_right_sum = 0 ;
   
    // ADD UP LEFT SIDE
    for ( a = 0 ; a < 6 ; a++ ) light_values_left_sum = light_values_left_sum + light_values[ a ] ;   
  
    // ADD UP RIGHT SIDE
    for ( a = 7 ; a < 13 ; a++ ) light_values_right_sum = light_values_right_sum + light_values[ a ] ;
  
    // CALCULATE ERROR
    line_error = light_values_left_sum - light_values_right_sum ;


} // <-- line_error()













void light_sensor_read()
{

    // LOCAL VARIABLES 
    int adc1_buf[8] ;
    int adc2_buf[8] ;
  
  
    // READ IN VALUES
    for ( a = 0 ; a < 8 ; a++ ) 
    {
        adc1_buf[ a ] = adc1.readADC( a ) ;
        adc2_buf[ a ] = adc2.readADC( a ) ;   
    }
 
  
    // COMBINE INTO ONE ARRAY
    light_values[ 12 ] = adc1_buf[ 0 ] ;
    light_values[ 11 ] = adc2_buf[ 0 ] ;
    light_values[ 10 ] = adc1_buf[ 1 ] ;
    light_values[ 9 ] = adc2_buf[ 1 ] ;
    light_values[ 8 ] = adc1_buf[ 2 ] ;
    light_values[ 7 ] = adc2_buf[ 2 ] ;
    light_values[ 6 ] = adc1_buf[ 3 ] ;
    light_values[ 5 ] = adc2_buf[ 3 ] ;
    light_values[ 4 ] = adc1_buf[ 4 ] ;
    light_values[ 3 ] = adc2_buf[ 4 ] ;
    light_values[ 2 ] = adc1_buf[ 5 ] ;
    light_values[ 1 ] = adc2_buf[ 5 ] ;
    light_values[ 0 ] = adc1_buf[ 6 ] ;


} // <-- light_sensor_read()












// _________ COMMAND MOTORS () _________

void command_motors()
{
  
    // IF PID_1 FORWARD    
    if ( whl_1_vl_output > 0 ) 
    {
        analogWrite( M1_IN_1 , 0 ) ;
        analogWrite( M1_IN_2 , whl_1_vl_output ) ;
    }
    
    // IF PID_1 REVERSE
    else
    {
        analogWrite( M1_IN_1 , -whl_1_vl_output * -1 ) ;
        analogWrite( M1_IN_2 , 0 ) ;  
    }
  
  
  
    // IF PID_2 FORWARD    
    if ( whl_2_vl_output > 0 ) 
    {
        analogWrite( M2_IN_1 , 0 ) ;
        analogWrite( M2_IN_2 , whl_2_vl_output ) ;
    }
    
    // IF PID_2 REVERSE
    else
    {
        analogWrite( M2_IN_1 , whl_2_vl_output * -1 ) ;
        analogWrite( M2_IN_2 , 0 ) ;  
    }


} // <--- command_motors()













// _________ PRINTOUT () _________

void printout()
{

    // UPDATE TIME STAMP FOR FUNCTION
    prev_print_time = millis () ;


    Serial.println() ;
    

  // PRINT LIGHT BAR ARRAY VALUES
  for ( a = 0 ; a < 13 ; a++ ) 
  {
      Serial.print( light_values[ a ] ) ; 
      Serial.print("   ") ;    
  }


    Serial.println() ;


    // PRINT
    //Serial.print( " tick_count: " ) ; 
    //Serial.print( enc1.read() ) ; 

   
    // PRINT
    //Serial.print( "       whl_1_vl_set: " ) ; 
    //Serial.print( whl_1_vl_setpoint, 3 ) ; 

  
    // PRINT
    //Serial.print( "       whl_1_vl_in: " ) ; 
    //Serial.print( whl_1_vl_feedback, 3 ) ; 


    // PRINT
    //Serial.print( "       whl_2_vl_in: " ) ; 
    //Serial.print( whl_2_vl_feedback, 3 ) ;

/*
    // PRINT
    Serial.print( "       whl_1_vl_Kp: " ) ; 
    Serial.print( whl_1_vl_Kp , 4 ) ; 


    // PRINT
    Serial.print( "       whl_1_vl_Ki: " ) ; 
    Serial.print( whl_1_vl_Ki , 4 ) ; 


    // PRINT
    Serial.print( "       whl_1_vl_Kd: " ) ; 
    Serial.print( whl_1_vl_Kd , 8 ) ; 
*/

    // PRINT
    Serial.print( "       err_v1: " ) ; 
    Serial.print( whl_1_vl_setpoint - whl_1_vl_feedback, 1 ) ; 

    
    // PRINT
    Serial.print( "       err_v2: " ) ; 
    Serial.print( whl_2_vl_setpoint - whl_2_vl_feedback, 1 ) ; 


    // PRINT
    Serial.print( "       whl_1_out: " ) ; 
    Serial.print( whl_1_vl_output, 1 ) ; 


    // PRINT
    Serial.print( "       whl_2_out: " ) ; 
    Serial.print( whl_2_vl_output, 1 ) ; 



    // PRINT
    Serial.print( "       ln_err: " ) ; 
    Serial.print( line_error ) ; 


    // PRINT
    Serial.print( "       ln_err_out: " ) ; 
    Serial.print( line_err_output, 4 ) ; 
    

    // PRINT
    Serial.print( "       ln_err_out_test: " ) ; 
    Serial.print( line_error * line_err_Kp_test , 4 ) ; 
   



} // <--- printout()
