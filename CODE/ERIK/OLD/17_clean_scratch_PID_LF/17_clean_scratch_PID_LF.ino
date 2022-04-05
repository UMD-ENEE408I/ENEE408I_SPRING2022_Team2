// INPROGRESS
// THOUGHTS:
// PRELOAD SETPOINTS FOR NEGATIVE ROOM. <-- CROSS THIS BRIDGE WHEN WE GET THERE. 
// PUT BACK IN SERIAL READ IN ORDER TO TUNE TURNING. POSSIBLE SET UP TUNNING IN SOME SORT OF WAY WHERE THE ROBOT IS STATIONARY. THEN APPLY MOVING FORWARD.


// QUESTIONS FOR LEVI:
// 1. HOW MUCH SLOWE DOES THE LINE LOOP NEED TO BE.

// 3. THE LOOSE ENCODER WIRE.



// ______ LIBRARYS______

#include <Encoder.h>
#include <Adafruit_MCP3008.h>






// ______ GLOBAL VARIABLES by Function ______




  float twinky_one_speed = 0.25 ;
  float twinky_two_speed = 0.25 ;



// PID LF CONTROL

  unsigned int prev_line_err_time = 0 ;
  float twinky_max = 0.25 ;
  float twinky_min = -0.25 ;



// PID VL CONTROL

  float twinky_one = 0 ;
  float twinky_two = 0 ;
  unsigned int prev_twinky_time = 0 ;




// LINE LF PID CALC

  float line_lf_PID_err = 0 ;
  float line_lf_PID_err_prev = 0 ;
  float line_lf_PID_P = 0 ;
  float line_lf_PID_I = 0 ;
  float line_lf_PID_D = 0 ;
  float line_lf_PID_out = 0 ;
  float line_lf_PID_feedback = 0 ;
  unsigned int line_lf_PID_D_time_prev = 0 ;

  float line_lf_PID_KP = 00.0000 ;
  float line_lf_PID_KI = 00.0000 ;
  float line_lf_PID_KD = 00.0000 ;




// WHL 1 VL PID CALC

  float whl_1_vl_PID_err = 0 ;
  float whl_1_vl_PID_err_prev = 0 ;
  float whl_1_vl_PID_P = 0 ;
  float whl_1_vl_PID_I = 0 ;
  float whl_1_vl_PID_D = 0 ;
  float whl_1_vl_PID_out = 0 ;
  float whl_1_vl_PID_feedback = 0 ;
  unsigned int whl_1_vl_PID_D_time_prev = 0 ;

  float whl_1_vl_PID_KP = 00.9000 ;
  float whl_1_vl_PID_KI = 00.0180 ;
  float whl_1_vl_PID_KD = 20.0000 ;





// WHL 2 VL PID CALC

  float whl_2_vl_PID_err = 0 ;
  float whl_2_vl_PID_err_prev = 0 ;
  float whl_2_vl_PID_P = 0 ;
  float whl_2_vl_PID_I = 0 ;
  float whl_2_vl_PID_D = 0 ;
  float whl_2_vl_PID_out = 0 ;
  float whl_2_vl_PID_feedback = 0 ;
  unsigned int whl_2_vl_PID_D_time_prev = 0 ;

  float whl_2_vl_PID_KP = 00.9000 ;
  float whl_2_vl_PID_KI = 00.0180 ;
  float whl_2_vl_PID_KD = 20.0000 ;





// LINE ERROR CALC

  int a = 0 ;
  unsigned int light_values_left_sum = 0 ;
  unsigned int light_values_right_sum = 0 ;
  unsigned int light_values[ 16 ] ;
  int line_error = 0 ;




// PRINT OUT

  unsigned int prev_print_time = 0 ;






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





// ______ FUNCTION DECLARATION ______

void printout() ;
void command_motors() ;
void pid_vl_control() ;
void pid_lf_control() ;
void line_error_calc() ;
void light_sensor_read() ;
void whl_1_vl_PID_calc() ;
void whl_2_vl_PID_calc() ;
void line_lf_PID_calc() ;


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


} // <-- setup()












// _____________ MAIN _____________                                <---------- M A I N ---------<

void loop() 
{


    // CHECK INPUT FROM SERIAL
    serial_input() ;


    // LINE FOLLOW PID CONTROL
    if( ( millis() - prev_line_err_time ) > 40 ) 
    {
        pid_lf_control() ;
        prev_line_err_time = millis() ;
    }
    
    // MOTOR PID CONTROL
    if( ( millis() - prev_twinky_time ) > 20 )
    { 
        pid_vl_control() ; 
        prev_twinky_time = millis() ;
    }

    // PRINTOUT
    if( ( millis() - prev_print_time ) > 20 )
    {     
      printout() ; 
      prev_print_time = millis () ;
    }



} // <-- main()                                                     <---------- E N D ---------< 



















// ___________________________________________ FUNCTION LAND ___________________________________________
// _____________________________________________________________________________________________________










// ______ PID LF CONTROL () ______

void pid_lf_control()
{

        // CALCULATE LINE ERROR RAW VALUE
        line_error_calc() ;

        // COMPUTE PID LF OUTPUT
        line_lf_PID_calc() ;
        
        // SLOW TWINKY SPEED FOR RIGHT WHEEL M2
        twinky_two_speed = twinky_max - line_lf_PID_out ;

        // SCALE TWINKY SPEEED FOR RIGHT WHEEL M2
        twinky_one_speed = twinky_max + line_lf_PID_out ;  // <-- DO NOT NEED TO MAKE NEGATIVE BECAUSE MOTOR II IS ALREADY MOUNTED BACKWARDS?


} // <-- pid_lf_control()














// ______ LINE LF PID CALC () ______

void line_lf_PID_calc()
{

    // PROPORTIONAL
    line_lf_PID_P = line_error * line_lf_PID_KP ;

    
    // INTEGRAL
    line_lf_PID_I = line_lf_PID_I + line_error * line_lf_PID_KI ;
    if( line_lf_PID_I > 255 ) line_lf_PID_I = 255 ;
    if( line_lf_PID_I < -255 ) line_lf_PID_I = -255 ;

    
    // DERIVATIVE
    line_lf_PID_D = ( ( line_error - line_lf_PID_err_prev ) / (float)( millis() - line_lf_PID_D_time_prev ) ) * line_lf_PID_KD ;
    line_lf_PID_err_prev = line_lf_PID_D ;
    line_lf_PID_D_time_prev = millis () ;


    // SUMMATION
    line_lf_PID_out = line_lf_PID_P + line_lf_PID_I + line_lf_PID_D ;
    if( line_lf_PID_out > twinky_max ) line_lf_PID_out = twinky_max ;
    if( line_lf_PID_out < twinky_min ) line_lf_PID_out = twinky_min ;    
    

} // <-- line_lf_PID_calc()















// ______ PID VL CONTROL () ______

void pid_vl_control()
{
  
    // MOVE THE TWINKY FORWARD 
    twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed ;  
    twinky_two = twinky_two + ( millis() - prev_twinky_time ) * twinky_two_speed ;
    //twinky_two = 0 ;
    
    // GRAB FEEDBACK TICKS
    whl_1_vl_PID_feedback = enc1.read() ;
    whl_2_vl_PID_feedback = enc2.read() * -1 ;
  
    // COMPUTE PID VL OUTPUT
    whl_1_vl_PID_calc() ;
    whl_2_vl_PID_calc() ;
  
    // COMMAND MOTORS
    command_motors() ;


} // <-- pid_vl_control()

















// ______ WHL 1 VL PID CALC () ______

void whl_1_vl_PID_calc()
{

    // ERROR
    whl_1_vl_PID_err = twinky_one - whl_1_vl_PID_feedback ;


    // PROPORTIONAL
    whl_1_vl_PID_P = whl_1_vl_PID_err * whl_1_vl_PID_KP ;

    
    // INTEGRAL
    whl_1_vl_PID_I = whl_1_vl_PID_I + whl_1_vl_PID_err * whl_1_vl_PID_KI ;
    if( whl_1_vl_PID_I > 255 ) whl_1_vl_PID_I = 255 ;
    if( whl_1_vl_PID_I < -255 ) whl_1_vl_PID_I = -255 ;

    
    // DERIVATIVE
    whl_1_vl_PID_D = ( ( whl_1_vl_PID_err - whl_1_vl_PID_err_prev ) / (float)( millis() - whl_1_vl_PID_D_time_prev ) ) * whl_1_vl_PID_KD ;
    whl_1_vl_PID_err_prev = whl_1_vl_PID_D ;
    whl_1_vl_PID_D_time_prev = millis () ;


    // SUMMATION
    whl_1_vl_PID_out = whl_1_vl_PID_P + whl_1_vl_PID_I + whl_1_vl_PID_D ;
    if( whl_1_vl_PID_out > 255 ) whl_1_vl_PID_out = 255 ;
    if( whl_1_vl_PID_out < -255 ) whl_1_vl_PID_out = -255 ;    
    

} // <-- whl_1_vl_PID_calc()
















// ______ WHL 2 VL PID CALC () ______

void whl_2_vl_PID_calc()
{

    // ERROR
    whl_2_vl_PID_err = twinky_two - whl_2_vl_PID_feedback ;


    // PROPORTIONAL
    whl_2_vl_PID_P = whl_2_vl_PID_err * whl_2_vl_PID_KP ;

    
    // INTEGRAL
    whl_2_vl_PID_I = whl_2_vl_PID_I + whl_2_vl_PID_err * whl_2_vl_PID_KI ;
    if( whl_2_vl_PID_I > 255 ) whl_2_vl_PID_I = 255 ;
    if( whl_2_vl_PID_I < -255 ) whl_2_vl_PID_I = -255 ;

    
    // DERIVATIVE
    whl_2_vl_PID_D = ( ( whl_2_vl_PID_err - whl_2_vl_PID_err_prev ) / (float)( millis() - whl_2_vl_PID_D_time_prev ) ) * whl_2_vl_PID_KD ;
    whl_2_vl_PID_err_prev = whl_2_vl_PID_D ;
    whl_2_vl_PID_D_time_prev = millis () ;


    // SUMMATION
    whl_2_vl_PID_out = whl_2_vl_PID_P + whl_2_vl_PID_I + whl_2_vl_PID_D ;
    if( whl_2_vl_PID_out > 255 ) whl_2_vl_PID_out = 255 ;
    if( whl_2_vl_PID_out < -255 ) whl_2_vl_PID_out = -255 ;    
    

} // <-- whl_2_vl_PID_calc()





















// _________ SERIAL INPUT () _________

void serial_input()
{

    // CHECK FOR SERIAL
    while (Serial.available() > 0) 
    {
        // GRAB INCOMING CHARACTERS
        char incomingCharacter = Serial.read() ;

       // PICK ACTION ACORDING TO CHAR 
       switch (incomingCharacter) 
       {

          // INCREASE Kp
          case '1':
              //whl_1_vl_PID_KP = whl_1_vl_PID_KP + 0.05 ;
              line_lf_PID_KP = line_lf_PID_KP + 0.00005 ;
          break;


          // DECREASE Kp
          case '2':
              //whl_1_vl_PID_KP = whl_1_vl_PID_KP - 0.05 ;
              line_lf_PID_KP = line_lf_PID_KP - 0.00005 ;
          break;





          // INCREASE Ki
          case '3':
              //whl_1_vl_PID_KI = whl_1_vl_PID_KI + 0.005 ;
              line_lf_PID_KI = line_lf_PID_KI + 0.000005 ;
          break;


          // DECREASE Ki
          case '4':
              //whl_1_vl_PID_KI = whl_1_vl_PID_KI - 0.005 ;
              line_lf_PID_KI = line_lf_PID_KI - 0.000005 ;
          break;





          // INCREASE Kd
          case '5':
              //whl_1_vl_PID_KD = whl_1_vl_PID_KD + 0.5 ;
              line_lf_PID_KD = line_lf_PID_KD + 0.05 ;
          break;


          // DECREASE Kp
          case '6':
              //whl_1_vl_PID_KD = whl_1_vl_PID_KD - 0.5 ;
              line_lf_PID_KD = line_lf_PID_KD - 0.05 ;
          break;

          
        } // <-- switch ()

    } // <-- while()

} // <--- serial_input()




















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
    if ( whl_1_vl_PID_out > 0 /*whl_1_vl_output > 0*/ ) 
    {
        analogWrite( M1_IN_1 , 0 ) ;
        analogWrite( M1_IN_2 , whl_1_vl_PID_out /*whl_1_vl_output*/ ) ;
    }
    
    // IF PID_1 REVERSE
    else
    {
        analogWrite( M1_IN_1 , whl_1_vl_PID_out *-1 /*whl_1_vl_output * -1*/ ) ; // <-- ONE MUST HARD-CODE FLIP THE MAGNITUDE OF THE PWM INTO POSITIVE BECAUSE THE PWM RANGE IS 0 --> 255 (ALL POSITIVE).
        analogWrite( M1_IN_2 , 0 ) ;  
    }

  
  
  
    // IF PID_2 FORWARD    
    if ( whl_2_vl_PID_out > 0 ) 
    {
        analogWrite( M2_IN_1 , 0 ) ;
        analogWrite( M2_IN_2 , whl_2_vl_PID_out ) ;
    }
    
    // IF PID_2 REVERSE
    else
    {
        analogWrite( M2_IN_1 , whl_2_vl_PID_out * -1 ) ;
        analogWrite( M2_IN_2 , 0 ) ;  // <-- ONE MUST HARD-CODE FLIP THE MAGNITUDE OF THE PWM INTO POSITIVE BECAUSE THE PWM RANGE IS 0 --> 255 (ALL POSITIVE).
    }


} // <--- command_motors()













// _________ PRINTOUT () _________

void printout()
{


    Serial.println() ;
    
/*
  // PRINT LIGHT BAR ARRAY VALUES
  for ( a = 0 ; a < 13 ; a++ ) 
  {
      Serial.print( light_values[ a ] ) ; 
      Serial.print("   ") ;    
  }


    Serial.println() ;
*/

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
    Serial.print( "     twy_one: " ) ; 
    Serial.print( twinky_one ) ; 

    
    // PRINT
    Serial.print( "     twy_one_spd: " ) ; 
    Serial.print( twinky_one_speed ) ; 



    // PRINT
    Serial.print( "     err_PID_v1: " ) ; 
    Serial.print( whl_1_vl_PID_err, 2 ) ; 


    // PRINT
    Serial.print( "     err_PID_v2: " ) ; 
    Serial.print( whl_2_vl_PID_err, 2 ) ; 


    // PRINT
    Serial.print( "     err_line: " ) ; 
    Serial.print( line_error ) ; 

/*    
    // PRINT
    Serial.print( "     err_v2: " ) ; 
    Serial.print( whl_2_vl_setpoint - whl_2_vl_feedback, 1 ) ; 
*/


    // PRINT
    Serial.print( "     whl_1_PID_out: " ) ; 
    Serial.print( whl_1_vl_PID_out, 2 ) ; 



    // PRINT
    Serial.print( "     whl_2_PID_out: " ) ; 
    Serial.print( whl_2_vl_PID_out, 2 ) ; 



    // PRINT
    Serial.print( "     line_lf_PID_out: " ) ; 
    Serial.print( line_lf_PID_out, 2 ) ; 





    // PRINT
    Serial.print( "     line_lf_PID_P: " ) ; 
    Serial.print( line_lf_PID_P, 2 ) ; 


    // PRINT
    Serial.print( "     line_lf_PID_I: " ) ; 
    Serial.print( line_lf_PID_I, 2 ) ; 


    // PRINT
    Serial.print( "     line_lf_PID_D: " ) ; 
    Serial.print( line_lf_PID_D, 2 ) ; 


    // PRINT
    Serial.print( "     line_lf_KP: " ) ; 
    Serial.print( line_lf_PID_KP , 4 ) ; 


    // PRINT
    Serial.print( "     line_lf_KI: " ) ; 
    Serial.print( line_lf_PID_KI , 4 ) ; 


    // PRINT
    Serial.print( "     line_lf_KD: " ) ; 
    Serial.print( line_lf_PID_KD , 8 ) ; 






/*
    // PRINT
    Serial.print( "     whl_1_vl_PID_P: " ) ; 
    Serial.print( whl_1_vl_PID_P, 2 ) ; 


    // PRINT
    Serial.print( "     whl_1_vl_PID_I: " ) ; 
    Serial.print( whl_1_vl_PID_I, 2 ) ; 


    // PRINT
    Serial.print( "     whl_1_vl_PID_D: " ) ; 
    Serial.print( whl_1_vl_PID_D, 2 ) ; 


    // PRINT
    Serial.print( "     whl_1_vl_KP: " ) ; 
    Serial.print( whl_1_vl_PID_KP , 4 ) ; 


    // PRINT
    Serial.print( "     whl_1_vl_KI: " ) ; 
    Serial.print( whl_1_vl_PID_KI , 4 ) ; 


    // PRINT
    Serial.print( "     whl_1_vl_KD: " ) ; 
    Serial.print( whl_1_vl_PID_KD , 8 ) ; 

*/



/*
    // PRINT
    Serial.print( "     whl_2_out: " ) ; 
    Serial.print( whl_2_vl_output, 1 ) ; 
*/
/*
    // PRINT
    Serial.print( "     ln_err: " ) ; 
    Serial.print( line_error ) ; 


    // PRINT
    Serial.print( "     ln_err_out: " ) ; 
    Serial.print( line_err_output, 4 ) ; 
*/
    
/*
    // PRINT
    Serial.print( "     ln_err_out_test: " ) ; 
    Serial.print( line_error * line_err_Kp_test , 4 ) ; 
*/   


} // <--- printout()
