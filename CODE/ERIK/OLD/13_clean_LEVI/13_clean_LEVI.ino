



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



// MAIN

  float integrated_input = 0 ;
  unsigned int prev_integrated_time = 0 ;




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

  double whl_1_vl_Kp = 0.25 ;
  double whl_1_vl_Ki = 0 ;
  double whl_1_vl_Kd = 0 ;
  
  double whl_1_vl_setpoint = 1 ;
  double whl_1_vl_feedback = 0 ;
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

PID whl_1_vl_PID( &whl_1_vl_feedback, &whl_1_vl_output, &whl_1_vl_setpoint, whl_1_vl_Kp, whl_1_vl_Ki, whl_1_vl_Kd, DIRECT ) ;







// ______ FUNCTION DECLARATION ______

void printout() ;
void speedometor() ;
void serial_input() ;
void command_motors() ;







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
  whl_1_vl_PID.SetSampleTime( 1 ) ;

} // <-- setup()










// _____________ MAIN _____________                                <---------- M A I N ---------<

void loop() 
{


    // CHECK INPUT FROM SERIAL
    serial_input() ;



      // ______ PID CONTROL ______
  
        // CONTROL LOOP TIMER
        if( ( millis() - prev_integrated_time ) > 60 )
        {
            // INTEGRATE VALUE
            integrated_input = integrated_input + ( millis() - prev_integrated_time ) ;  

            // UPDATE TIME STAMP
            prev_integrated_time = millis() ;
        
            // COPY WHEELSPEED VALUE
            whl_1_vl_setpoint = integrated_input ;

            // GRAB FEEDBACK TICKS
            whl_1_vl_feedback = enc1.read() ;
      
            // COMPUTE PID OUTPUT
            whl_1_vl_PID.Compute() ;

            // COMMAND MOTORS
            command_motors() ;
        
        } // <-- if()

  
    // PRINTOUT
    printout() ;



} // <-- main()                                                     <---------- E N D ---------< 



















// ___________________________________________ FUNCTION LAND ___________________________________________
// _____________________________________________________________________________________________________











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
              whl_1_vl_Kp = whl_1_vl_Kp + 0.5 ;
          break;


          // DECREASE Kp
          case '2':
              whl_1_vl_Kp = whl_1_vl_Kp - 0.5 ;
          break;

          
        } // <-- switch ()

    } // <-- while()

} // <--- serial_input()
















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


  // UPDATE FUNCTION TIMER TIMESTAMP
  spd_chk_prev = millis() ;


} // <--- spedometor()




















// _________ COMMAND MOTORS () _________

void command_motors()
{
    // IF FORWARD    
    if ( whl_1_vl_output > 0 ) 
    {
        analogWrite( M1_IN_1 , 0 ) ;
        analogWrite( M1_IN_2 , whl_1_vl_output ) ;
    }
    
    // IF REVERSE
    else
    {
        analogWrite( M1_IN_1 , -whl_1_vl_output ) ;
        analogWrite( M1_IN_2 , 0 ) ;  
    }

} // <--- command_motors()













// _________ PRINTOUT () _________

void printout()
{

    Serial.println() ;
    
  
    // PRINT
    Serial.print( " tick_count: " ) ; 
    Serial.print( enc1.read() ) ; 
  
  
    // PRINT
    Serial.print( "       whl_1_vl_set: " ) ; 
    Serial.print( whl_1_vl_setpoint, 3 ) ; 

  
    // PRINT
    Serial.print( "       whl_1_vl_in: " ) ; 
    Serial.print( whl_1_vl_feedback, 3 ) ; 
  
  
    // PRINT
    Serial.print( "       whl_1_vl_out: " ) ; 
    Serial.print( whl_1_vl_output, 3 ) ; 
  
 
    // PRINT
    Serial.print( "       error_vl: " ) ; 
    Serial.print( whl_1_vl_setpoint - whl_1_vl_feedback, 3 ) ; 


    // PRINT
    Serial.print( "       whl_1_vl Kp: " ) ; 
    Serial.print( whl_1_vl_Kp , 3 ) ; 


} // <--- printout()
