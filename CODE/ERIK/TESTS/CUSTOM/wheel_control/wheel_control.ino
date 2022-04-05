



// ______ LIBRARYS______

#include <Encoder.h>
#include <Adafruit_MCP3008.h>






// ______ GLOBAL VARIABLES by Function ______


  // MAIN

    unsigned int prev_vl_time = 0 ;
    unsigned int prev_print_time = 0 ;



  // PID VL CONTROL

    float twinky_one = 0 ;
    float twinky_one_speed = 0.35 ;
    unsigned int prev_twinky_time = 0 ;



  // WHL 1 VL PID CALC

    unsigned int previous_twinky_time = 0 ;

    float whl_1_vl_PID_err = 0 ;
    float whl_1_vl_PID_err_prev = 0 ;
    float whl_1_vl_PID_P = 0 ;
    float whl_1_vl_PID_I = 0 ;
    float whl_1_vl_PID_D = 0 ;
    float whl_1_vl_PID_out = 0 ;
    float whl_1_vl_PID_feedback = 0 ;
    unsigned int whl_1_vl_PID_D_time_prev = 0 ;
  
    float whl_1_vl_KP = 0.9000 ;
    float whl_1_vl_KI = 00.0200 ;
    float whl_1_vl_KD = 20.0000 ;








// ______ HARDWARE PINS ______

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
void whl_1_vl_PID_calc() ;










// __________ SETUP __________

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

/*  1. Check for input (used for tunning)
 *  2. Advance the twinky forward and have the PID follow.  <-- THE MAIN MEAT AND POTATOS
 *  3. Print out stats.
 *  REPEAT
 */



    // CHECK INPUT FROM SERIAL
    serial_input() ;



  
    // WHEEL VELOCITY CONTROL
    if( ( millis() - prev_vl_time ) > 20 )   // <-- "Only runs if 20 milliseconds have transpired since the last time it ran." In this case, it is also your SAMPLE RATE for the descrite PID control.
    {
      
      // UPDATE TIME STAMP
      prev_vl_time = millis () ;

      // WHEEL CONTROL
      pid_vl_control() ;
    }




  
    // PRINTOUT
    if( ( millis() - prev_print_time ) > 100 )  // <-- "Only runs if 20 milliseconds have transpired since the last time it ran."
    {
      
      // UPDATE TIME STAMP
      prev_print_time = millis () ;

      // PRINTOUT
      printout() ; 
      
    }



} // <-- main()                                                     <---------- E N D ---------< 



















// ___________________________________________ FUNCTION LAND ___________________________________________
// _____________________________________________________________________________________________________




/* This function advances the twinky forward, and then calls the PID controlor to have the wheel folow the twinky.
 * REMEMBER - All you do is move the twinky, the PID moves the wheel.
 *    AGAIN - You do not directly move the wheel, the PID controler does, you move the twinky where you want the wheel to go.
 * 
 */


// ______ PID VL CONRTOL () ______

void pid_vl_control()
{
  
    // MOVE THE TWINKY FORWARD 
    twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed ;  
  
    // UPDATE TIME STAMP
    prev_twinky_time = millis() ;
  
    // GRAB FEEDBACK TICKS
    whl_1_vl_PID_feedback = enc1.read() ;
  
    // COMPUTE PID VL OUTPUT
    whl_1_vl_PID_calc() ;
  
    // COMMAND MOTORS
    command_motors() ;


} // <-- pid_vl_control()














/* This function simply calculates the PID for the weel.
 * 
 */

// ______ WHL 1 VL PID CALC () ______

void whl_1_vl_PID_calc()
{

    // ERROR (How far away is the wheel from the twinky?)
    whl_1_vl_PID_err = twinky_one - whl_1_vl_PID_feedback ;
    

    // ___ PROPORTIONAL ___
    whl_1_vl_PID_P = whl_1_vl_PID_err * whl_1_vl_KP ;

    
    // ___ INTEGRAL ___
    whl_1_vl_PID_I = whl_1_vl_PID_I + whl_1_vl_PID_err * whl_1_vl_KI ;
    if( whl_1_vl_PID_I > 255 ) whl_1_vl_PID_I = 255 ;
    if( whl_1_vl_PID_I < -255 ) whl_1_vl_PID_I = -255 ;

    
    // ___ DERIVATIVE ___
    whl_1_vl_PID_D = ( ( whl_1_vl_PID_err - whl_1_vl_PID_err_prev ) / (float)( millis() - whl_1_vl_PID_D_time_prev ) ) * whl_1_vl_KD ;
    whl_1_vl_PID_err_prev = whl_1_vl_PID_D ;
    whl_1_vl_PID_D_time_prev = millis () ;


    // SUMMATION
    whl_1_vl_PID_out = whl_1_vl_PID_P + whl_1_vl_PID_I + whl_1_vl_PID_D ;
    if( whl_1_vl_PID_out > 255 ) whl_1_vl_PID_out = 255 ;
    if( whl_1_vl_PID_out < -255 ) whl_1_vl_PID_out = -255 ;    
    

} // <-- whl_1_vl_PID_calc()












/* This function simply lets one tune the P I D coeficients in real time so that there result can be instantly observed.
 * 
 */

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
              whl_1_vl_KP = whl_1_vl_KP + 0.05 ;
          break;


          // DECREASE Kp
          case '2':
              whl_1_vl_KP = whl_1_vl_KP - 0.05 ;
          break;





          // INCREASE Ki
          case '3':
              whl_1_vl_KI = whl_1_vl_KI + 0.005 ;
          break;


          // DECREASE Ki
          case '4':
              whl_1_vl_KI = whl_1_vl_KI - 0.005 ;
          break;





          // INCREASE Kd
          case '5':
              whl_1_vl_KD = whl_1_vl_KD + 0.5 ;
          break;


          // DECREASE Kp
          case '6':
              whl_1_vl_KD = whl_1_vl_KD - 0.5 ;
          break;

          
        } // <-- switch ()

    } // <-- while()

} // <--- serial_input()

















/* This function simply sends out the command from the PID output to the motors.
 * 
 */
 
// _________ COMMAND MOTORS () _________

void command_motors()
{
  
    // IF PID_1 FORWARD    
    if ( whl_1_vl_PID_out >= 0 ) 
    {
        analogWrite( M1_IN_1 , 0 ) ;
        analogWrite( M1_IN_2 , whl_1_vl_PID_out ) ;
    }
    
    // IF PID_1 REVERSE
    else
    {
        analogWrite( M1_IN_1 , whl_1_vl_PID_out * -1 ) ;
        analogWrite( M1_IN_2 , 0 ) ;  
    }


} // <--- command_motors()
















/* This function simply keeps printing all in one place for development (though sometime it is helpful to put print statmennts in the code).
 *  Suppress all print functionality, even the print setup code, for when you are really ready to perform for the final. Printing slows things waaaaaaay down.
 * 
 */



// _________ PRINTOUT () _________

void printout()
{


    // PRINT LN
    Serial.println( " " ) ;


    // PRINT
    Serial.print( "     err_PID_v1: " ) ; 
    Serial.print( whl_1_vl_PID_err, 2 ) ; 
    Serial.print("\t"); 
    
    // PRINT
    Serial.print( "     whl_1_PID_out: " ) ;    
    Serial.print( whl_1_vl_PID_out, 2 ) ; 
    Serial.print("\t"); 

/*
    // PRINT
    Serial.print( "     whl_1_vl_PID_P: " ) ; 
    Serial.print( whl_1_vl_PID_P, 2 ) ; 
    Serial.print("\t"); 

    // PRINT
    Serial.print( "     whl_1_vl_PID_I: " ) ; 
    Serial.print( whl_1_vl_PID_I, 2 ) ; 
    Serial.print("\t"); 

    // PRINT
    Serial.print( "     whl_1_vl_PID_D: " ) ; 
    Serial.print( whl_1_vl_PID_D, 2 ) ; 
    Serial.print("\t"); 
*/

    // PRINT
    Serial.print( "     whl_1_vl_KP: " ) ; 
    Serial.print( whl_1_vl_KP , 4 ) ; 
    Serial.print("\t"); 

    // PRINT
    Serial.print( "     whl_1_vl_KI: " ) ; 
    Serial.print( whl_1_vl_KI , 4 ) ; 
    Serial.print("\t"); 

    // PRINT
    Serial.print( "     whl_1_vl_KD: " ) ; 
    Serial.print( whl_1_vl_KD , 8 ) ; 
    Serial.print("\t"); 





} // <--- printout()
