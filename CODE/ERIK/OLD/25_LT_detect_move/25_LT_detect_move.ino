
// THOUGHTS:
// PRELOAD SETPOINTS FOR NEGATIVE ROOM. <-- CROSS THIS BRIDGE WHEN WE GET THERE. 


// THINGS TO DO:
/*

- Abstract D detect and D move
- The rest of the speacial cases.
- Double check the gate holds for D_move.
- Auto calibrating bin threshold.
- Add coupon for right turn final detection.
- Gradually stop wheels when stop command.
- Inspect why wheel two delays in rotating in D-move. <--- FIX FIX FIX! THE ISSUE IS THAT THE TWINKY IS STILL COUNTING AS A FUNCTION OF TIME.
- CORNER CASE OF A STUB OF WHITE TAPE CONTINUING ON AT THE END OF A RT.
- Move the speciale cases in a way so that the other stuff does not have to calculate if being overiden.

*/





// QUESTIONS FOR LEVI:
// 1. THE LOOSE ENCODER WIRE.




// ______ LIBRARYS______

#include <Encoder.h>
#include <Adafruit_MCP3008.h>






// ______ GLOBAL VARIABLES by Function ______


// OVERALL

  float twinky_one_speed_const = 0.40 ;
  float twinky_two_speed_const = twinky_one_speed_const ;

  float twinky_one_speed = twinky_one_speed_const ;
  float twinky_two_speed = twinky_one_speed_const ;


// SPECIAL DETECT

  bool light_values_bin[ 16 ] ;
  bool d_detected = 0 ;
  int d_enc_one_prev = 0 ;
  int d_enc_two_prev = 0 ;
  bool d_enc_cupon = 1 ;

  bool rt_enc_cupon = 1 ;
  bool rt_initial_latch = 0 ;
  bool rt_final_latch = 0 ;
  int rt_enc_one_initial = 0 ;
  int rt_enc_two_initial = 0 ;
  bool rt_initial_detect = 0 ;
  bool rt_final_detect = 0 ;
  bool go_for_rt = 0 ;  // <-- WILL BECOME A FUNCTIN CALL TO MICHAELS MODULE?
  bool go_for_st = 1 ;  // <-- WILL BECOME A FUNCTIN CALL TO MICHAELS MODULE?
  bool rt_move_latch = 0 ;
  int rt_enc_one_final_detect = 0 ;
  int rt_enc_two_final_detect = 0 ;
  bool rt_wheel_one_mv_complt = 0 ;
  bool rt_position_final_gate = 1 ;
  bool rtd_detect = 0 ;
  int rt_enc_two_final_detect_update = 0 ;
  int rt_enc_one_final_detect_update = 0 ;
  bool rt_mv_rot_one_cmplt = 0 ;
  bool rt_mv_rot_two_cmplt = 0 ;
  bool rt_mv_fwd_one_cmplt = 0 ;
  bool rt_mv_fwd_two_cmplt = 0 ;



// PID LF CONTROL

  unsigned int prev_line_err_time = 0 ;
  float twinky_max = twinky_one_speed ;
  float twinky_min = twinky_one_speed * -1 ;



// PID VL CONTROL

  float twinky_one = 0 ;
  float twinky_two = 0 ;
  unsigned int prev_twinky_time = 0 ;

  float twinky_one_d = 0 ;
  float twinky_two_d = 0 ;
  bool d_mv_cmplt = 0 ;
  bool d_mv_fwd_one_cmplt = 0 ;
  bool d_mv_fwd_two_cmplt = 0 ;
  bool d_mv_rot_one_cmplt = 0 ;
  bool d_mv_rot_two_cmplt = 0 ;
  bool d_mv_latch = 0 ;



// LINE LF PID CALC

  float line_lf_PID_err = 0 ;
  float line_lf_PID_err_prev = 0 ;
  float line_lf_PID_P = 0 ;
  float line_lf_PID_I = 0 ;
  float line_lf_PID_D = 0 ;
  float line_lf_PID_out = 0 ;
  float line_lf_PID_feedback = 0 ;
  unsigned int line_lf_PID_D_time_prev = 0 ;

  float line_lf_PID_KP = 00.0005 ;
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
void special_detect() ;







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



    // SPECIAL CASE DETECTION
    special_detect() ;

    

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
    if( ( millis() - prev_print_time ) > 250 )
    {     
      printout() ; 
      prev_print_time = millis () ;
    }



} // <-- main()                                                     <---------- E N D ---------< 



















// ___________________________________________ FUNCTION LAND ___________________________________________
// _____________________________________________________________________________________________________











// ______ SPECIAL DETECT () ______

void special_detect()
{
    
    // OBTAIN BINARY ARRAY
    for ( a = 0 ; a < 13 ; a++ ) 
    {
        // THRESHOLD
        if( light_values[ a ] <= 680 ) light_values_bin[ a ]  = 1 ;   
        else light_values_bin[ a ] = 0 ;
    }



 



    
    
    // RIGHT_TURN DETECT

    // LOOK FOR RT SIGNITURE   0 0 0 0 X X 1 1 1 1 1 1 X
    if( light_values_bin[ 6 ] == 1 && light_values_bin[ 7 ] == 1 && light_values_bin[ 8 ] == 1 && light_values_bin[ 9 ] == 1 && light_values_bin[ 10 ] == 1 && light_values_bin[ 11 ] == 1 && light_values_bin[ 0 ] == 0 && light_values_bin[ 1 ] == 0 && light_values_bin[ 2 ] == 0 && light_values_bin[ 3 ] == 0 ) 
    {
        // YES RT SIGNATURE
        if( d_detected == 0 ) rt_initial_detect = 1 ;   
    }

    // NO RT SIGNITURE
    else rt_initial_detect = 0 ;
        



    // IF DETECTED
    if( rt_initial_detect == 1 ) 
    {
         
        // UPDATE TIME_STAMP PER CUPON
        if( rt_enc_cupon == 1 ) 
        {
          // USE UP CUPON
          rt_enc_cupon = 0 ;

          // UPDATE POSTIONS
          rt_enc_one_initial = enc1.read() ;
          rt_enc_two_initial = enc2.read() * -1 ;
          
        }


        // IF HAS BEEN A RIGHT TURN FOR SOME ROTATION
        if( ( ( enc1.read() - rt_enc_one_initial > 10 ) || ( ( enc2.read() * -1 ) - rt_enc_two_initial > 10 ) ) && ( rt_initial_latch == 0 ) )
        {

            // GRAB INITIAL LATCH
            rt_initial_latch = 1 ; 
              
        }

    } // <-- if( rt_initial_detected )





    // IF INITIAL LATCH AND FINAL RT SGNATURE   0 0 0 0 0 0 0 0 0 0 0 0 0 
    else if( rt_initial_latch == 1 &&  light_values_bin[ 0 ] == 0 && light_values_bin[ 1 ] == 0 && light_values_bin[ 2 ] == 0 && light_values_bin[ 3 ] == 0 && light_values_bin[ 4 ] == 0 && light_values_bin[ 5 ] == 0 && light_values_bin[ 6 ] == 0 && light_values_bin[ 7 ] == 0 && light_values_bin[ 8 ] == 0 && light_values_bin[ 9 ] == 0 && light_values_bin[ 10 ] == 0 && light_values_bin[ 11 ] == 0 && light_values_bin[ 12 ] == 0 ) 
    {
        // GRAB FINAL LATCH
        rt_final_detect = 1 ;  

        // DEAD RIGHT DETECTED
        rtd_detect = 1 ; 
    }

 /*   
    // IF INITIAL LATCH AND FINAL RT SGNATURE   0 0 0 0 X 1 1 1 X 0 0 0 0 
    else if( rt_initial_latch == 1 &&  light_values_bin[ 0 ] == 0 && light_values_bin[ 1 ] == 0 && light_values_bin[ 2 ] == 0 && light_values_bin[ 3 ] == 0 && ( light_values_bin[ 5 ] == 1 || light_values_bin[ 6 ] == 1 || light_values_bin[ 7 ] == 1 ) && light_values_bin[ 9 ] == 0 && light_values_bin[ 10 ] == 0 && light_values_bin[ 11 ] == 0 && light_values_bin[ 12 ] == 0 ) 
    {
        // GRAB FINAL LATCH
        rt_final_detect = 1 ;  

        //Serial.print( "   IN_FD   " ) ;
    }

*/
    // IF ABOVE FINAL SIGNATURES IS NOT DETECTED AFTER SOME ROTATION
    else if ( ( ( enc1.read() - rt_enc_one_initial ) > 100 ) && ( ( ( enc2.read() * -1 ) - rt_enc_two_initial ) > 100 ) )
    {

      //Serial.print( "   IN_CPN   " ) ;
      
      // RESET GATE
      rt_enc_cupon = 1 ;
      
    }




    // IF FINAL DETECT AND GO FOR RIGHT TURN
    if( rt_final_detect == 1 && ( go_for_rt == 1 || rtd_detect == 1 ) )         // <-- function calls to desicsion module in place of variables.
    {

        //Serial.print( "   IN_FD&GFT   " ) ;

        // LATCH RIGHT MOVE
        rt_move_latch = 1 ;

        // UPDATE WHEEL POSITION UPON LATCH
        if( rt_position_final_gate == 1 )
        {
    
            //Serial.print( "   IN_GATE   " ) ;
    
            // CLOSE GATE
            rt_position_final_gate = 0 ;
    
            // UPDATE POSITIONS
            rt_enc_one_final_detect = enc1.read() ;
            rt_enc_two_final_detect = enc2.read() * -1 ;  

          
            // ASSIGN GATE VALUES
            //rt_mv_cmplt = 0 ;
            //rt_mv_fwd_one_cmplt = 0 ;
            //rt_mv_fwd_two_cmplt = 0 ;
            //rt_mv_rot_one_cmplt = 0 ;
            //rt_mv_rot_two_cmplt = 0 ;
              
        }
            
    }  // <-- if( rt_dect and go_st )
    


    // ELSE IF FINAL DETECT AND GO STRAIT
    else if( rt_final_detect == 1 && go_for_st == 1 )    // <-- function calls to desicsion module in place of variables.
    {
        // RESET RT DETECT CONDITIONS
        rt_enc_cupon = 1 ;
        rt_initial_detect = 0 ;
        rt_initial_latch = 0 ;
        rt_final_detect = 0 ;
        rtd_detect = 0 ;
      
    }















   
    // DEAD_END DETECT
    for ( a = 0 ; a < 13 ; a++ ) 
    {
        // LOOK FOR ALL ZEROS
        if( light_values_bin[ a ] == 1 ) 
        {
            // NO ALL ZEROS DETECTED
            d_detected = 0 ; 
            break ;   
        }

        // ALL ZEROS DETECTED
        else d_detected = 1 ;
        
    } // <-- for(a)


    // IF DETECTED
    if( d_detected == 1 && rt_initial_latch == 0 && rt_initial_detect == 0 && rtd_detect == 0 ) 
    {
         
        // UPDATE TIME_STAMP PER CUPON
        if( d_enc_cupon == 1 ) 
        {
          // USE UP CUPON
          d_enc_cupon = 0 ;

          // UPDATE POSTIONS
          d_enc_one_prev = enc1.read() ;
          d_enc_two_prev = enc2.read() * -1 ;
          
        }


        // IF HAS BEEN A DEAD END FOR SOME ROTATION
        if( ( ( enc1.read() - d_enc_one_prev > 40 ) || ( ( enc2.read() * -1 ) - d_enc_two_prev > 40 ) ) && ( d_mv_latch == 0 ) )
        {

            // GRAB LATCH
            d_mv_latch = 1 ;
          
            // ASSIGN GATE VALUES
            d_mv_cmplt = 0 ;
            d_mv_fwd_one_cmplt = 0 ;
            d_mv_fwd_two_cmplt = 0 ;
            d_mv_rot_one_cmplt = 0 ;
            d_mv_rot_two_cmplt = 0 ;
            
            // CAPTURE CURRENT TWINKY POSITIONS
            twinky_one_d = twinky_one ;
            twinky_two_d = twinky_two ;
                
            
        } // <-- if( enc - prev )

    } // <-- if( d_detected )

    
    // RESET CUPON
    else d_enc_cupon = 1 ;

















    // RIGHT_DEAD DETECT


    
    
    
    // LEFT_TURN DETECT





    // LEFT_DEAD DETECT


    // CROSS DETECT

        

} // <-- special_detect()

























// ______ PID VL CONTROL () ______

void pid_vl_control()
{
  
    // NORMAL LINE FOLLOW
    if( d_mv_latch == 0 && rt_initial_detect == 0 && rt_initial_latch == 0 && rt_move_latch == 0 ) 
    {
        twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed ;  
        twinky_two = twinky_two + ( millis() - prev_twinky_time ) * twinky_two_speed ;
    }
    
    // ELSE IF DEAD_END MOVE
    else if( d_mv_latch == 1 )
    { 

        // IF FWD TURN IS NOT COMPLETE
        if ( d_mv_fwd_one_cmplt == 0 || d_mv_fwd_two_cmplt == 0 )
        {
            // TURN WHEEL ONE FORWARD A SPECIFIC AMOUNT
            if( ( twinky_one - ( twinky_one_d + 360 ) ) < 0 ) twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed ;  
            else 
            {
              // UPDATE POSITION
              twinky_one_d = twinky_one ;

              // UPDATE GATE FLAG
              d_mv_fwd_one_cmplt = 1 ;
            }

            // TURN WHEEL TWO FORWARD A SPECIFIC AMOUNT
            if( ( twinky_two - ( twinky_two_d + 360 ) ) < 0 ) twinky_two = twinky_two + ( millis() - prev_twinky_time ) * twinky_two_speed ;
            else 
            {
              // UPDATE POSITION
              twinky_two_d = twinky_two ;

              // UPDATE GATE FLAG
              d_mv_fwd_two_cmplt = 1 ;
            }
            
        } // <-- if(d_fwd_I && d_fwd_II)


        // IF ROTATION IS NOT COMPLETE
        if ( ( d_mv_rot_one_cmplt == 0 || d_mv_rot_two_cmplt == 0 ) && ( d_mv_fwd_one_cmplt == 1 && d_mv_fwd_two_cmplt == 1 ) )
        {
            // TURN WHEEL ONE FORWARD A SPECIFIC AMOUNT
            if( ( twinky_one - ( twinky_one_d + 450 ) ) < 0 ) twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed ;  
            else d_mv_rot_one_cmplt = 1 ;

            // TURN WHEEL TWO BACKWARDS A SPECIFIC AMOUNT
            if( ( twinky_two - ( twinky_two_d - 450 ) ) > 0 ) twinky_two = twinky_two - ( millis() - prev_twinky_time ) * twinky_two_speed ;
            else d_mv_rot_two_cmplt = 1 ;

     
            // IF COMPLETE RESET GATE FLAGS
            if( ( d_mv_rot_one_cmplt == 1 ) && ( d_mv_rot_two_cmplt == 1 ) )
            {
                /*
                d_mv_fwd_one_cmplt = 1 ;
                d_mv_fwd_two_cmplt = 1 ;
                d_mv_rot_one_cmplt = 1 ;
                d_mv_rot_two_cmplt = 1 ;
                */
                d_mv_latch = 0 ;
                d_enc_cupon = 1 ;
            }              
            
        } // <-- if(d_mv_cmplt)
        
    } // <-- else if(d_dect)













    // ELSE IF RT MOVE
    else if( rt_initial_detect == 1 || rt_initial_latch == 1 || rt_move_latch == 1 )
    { 

        //Serial.print( "   IN_RT_M   " ) ;

        // IF RT DETECT KEEP STRAIGHT
        if( ( rt_initial_detect == 1 || rt_initial_latch == 1 ) && ( rt_move_latch == 0 ) )
        {


            // KEEP STRAIGHT
            twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed_const ;
            twinky_two = twinky_two + ( millis() - prev_twinky_time ) * twinky_two_speed_const ;

          
        } // <-- if(go rt)


        // ELSE IF GO FOR RT
        if( rt_move_latch == 1 )
        {

            // MANUALLY MOVE FORWARD A CERTAIN DISTANCE
            // IF FWD TURN IS NOT COMPLETE
            if ( rt_mv_fwd_one_cmplt == 0 || rt_mv_fwd_two_cmplt == 0 )
            {
                // TURN WHEEL ONE FORWARD A SPECIFIC AMOUNT
                if( enc1.read() - rt_enc_one_final_detect < 180 ) twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed ;  
                else 
                {
                  // UPDATE POSITION
                  rt_enc_one_final_detect_update = enc1.read() ;
    
                  // UPDATE GATE FLAG
                  rt_mv_fwd_one_cmplt = 1 ;
                }
    
                // TURN WHEEL TWO FORWARD A SPECIFIC AMOUNT
                if( ( enc2.read() * -1 ) - rt_enc_two_final_detect < 180 ) twinky_two = twinky_two + ( millis() - prev_twinky_time ) * twinky_two_speed ;
                else 
                {
                  // UPDATE POSITION
                  rt_enc_two_final_detect_update = ( enc2.read() * -1 ) ;
    
                  // UPDATE GATE FLAG
                  rt_mv_fwd_two_cmplt = 1 ;

                  //Serial.print( "   IN_FWD2_CPT   " ) ;
                }
                
            } // <-- if(d_fwd_I && d_fwd_II)


    
            // IF ROTATION IS NOT COMPLETE
            if ( ( rt_mv_rot_one_cmplt == 0 || rt_mv_rot_two_cmplt == 0 ) && ( rt_mv_fwd_one_cmplt == 1 && rt_mv_fwd_two_cmplt == 1 ) )
            {
                // TURN WHEEL ONE FORWARD A SPECIFIC AMOUNT
                if( enc1.read() - rt_enc_one_final_detect_update < 180 ) twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed ;  
                else rt_mv_rot_one_cmplt = 1 ;
    
                // TURN WHEEL TWO BACKWARDS A SPECIFIC AMOUNT
                if( ( enc2.read() * -1 ) - rt_enc_two_final_detect_update > -180 ) twinky_two = twinky_two - ( millis() - prev_twinky_time ) * twinky_two_speed ;
                else rt_mv_rot_two_cmplt = 1 ;
    
         
                // IF COMPLETE RESET GATE FLAGS
                if( ( rt_mv_rot_one_cmplt == 1 ) && ( rt_mv_rot_two_cmplt == 1 ) )
                {

                   // UPDATE GATE FLAG
                    rt_wheel_one_mv_complt = 0 ;
  
                    // REMOVE RT MOVE LATCH
                    rt_move_latch = 0 ;
  
                    // RESET RT DETECT CONDITIONS
                    rt_enc_cupon = 1 ;
                    rt_initial_detect = 0 ;
                    rt_initial_latch = 0 ;
                    rt_final_detect = 0 ;
                    rt_position_final_gate = 1 ;
                    rtd_detect = 0 ;
                    rt_mv_rot_one_cmplt = 0 ;
                    rt_mv_rot_two_cmplt = 0 ;
                    rt_mv_fwd_one_cmplt = 0 ;
                    rt_mv_fwd_two_cmplt = 0 ;       
     
                }              
                
            } // <-- if(d_mv_cmplt)

          
        } // <-- if(go rt)


    } // <-- else if(rt dect or latch)

















    
    // GRAB FEEDBACK TICKS
    whl_1_vl_PID_feedback = enc1.read() ;
    whl_2_vl_PID_feedback = enc2.read() * -1 ;
  
    // COMPUTE PID VL OUTPUT
    whl_1_vl_PID_calc() ;
    whl_2_vl_PID_calc() ;
  
    // COMMAND MOTORS
    command_motors() ;


} // <-- pid_vl_control()




























// ______ PID LF CONTROL () ______

void pid_lf_control()
{


        // CALCULATE LINE ERROR RAW VALUE
        line_error_calc() ;

        // COMPUTE PID LF OUTPUT
        line_lf_PID_calc() ;
        
        // SUBTRACT TWINKY SPEED FOR RIGHT WHEEL M2
        if( line_lf_PID_out >= 0 ) twinky_two_speed = twinky_max - line_lf_PID_out ;

        // SUBTRACT TWINKY SPEEED FOR LEFT WHEEL M1
        else twinky_one_speed = twinky_max - ( line_lf_PID_out * -1 ) ;

        // OVERRIDE FOR SPECIAL CASE OF DEAD_END TURN AROUND
        if( d_mv_latch == 1 ) twinky_two_speed = twinky_max ;
        if( d_mv_latch == 1 ) twinky_one_speed = twinky_max ;
        if( rt_move_latch == 1 ) twinky_one_speed = twinky_one_speed_const ;
        if( rt_move_latch == 1 ) twinky_two_speed = twinky_two_speed_const ;
        
          


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
              line_lf_PID_KP = line_lf_PID_KP + 0.0005 ;
          break;


          // DECREASE Kp
          case '2':
              //whl_1_vl_PID_KP = whl_1_vl_PID_KP - 0.05 ;
              line_lf_PID_KP = line_lf_PID_KP - 0.0005 ;
          break;





          // INCREASE Ki
          case '3':
              //whl_1_vl_PID_KI = whl_1_vl_PID_KI + 0.005 ;
              line_lf_PID_KI = line_lf_PID_KI + 0.00005 ;
          break;


          // DECREASE Ki
          case '4':
              //whl_1_vl_PID_KI = whl_1_vl_PID_KI - 0.005 ;
              line_lf_PID_KI = line_lf_PID_KI - 0.00005 ;
          break;





          // INCREASE Kd
          case '5':
              //whl_1_vl_PID_KD = whl_1_vl_PID_KD + 0.5 ;
              line_lf_PID_KD = line_lf_PID_KD + 0.5 ;
          break;


          // DECREASE Kp
          case '6':
              //whl_1_vl_PID_KD = whl_1_vl_PID_KD - 0.5 ;
              line_lf_PID_KD = line_lf_PID_KD - 0.5 ;
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
    if ( whl_1_vl_PID_out >= 0 ) 
    {
        analogWrite( M1_IN_1 , 0 ) ;
        //analogWrite( M1_IN_2 , 0 /*whl_1_vl_PID_out*/ ) ;
        analogWrite( M1_IN_2 , whl_1_vl_PID_out ) ;
    }
    
    // IF PID_1 REVERSE
    else
    {
        //analogWrite( M1_IN_1 , 0 /*whl_1_vl_PID_out * -1*/ ) ;
        analogWrite( M1_IN_1 , whl_1_vl_PID_out * -1 ) ;// <-- ONE MUST HARD-CODE FLIP THE MAGNITUDE OF THE PWM INTO POSITIVE BECAUSE THE PWM RANGE IS 0 --> 255 (ALL POSITIVE).
        analogWrite( M1_IN_2 , 0 ) ;  
    }

  
  
  
    // IF PID_2 FORWARD    
    if ( whl_2_vl_PID_out >= 0 ) 
    {
        analogWrite( M2_IN_1 , 0 ) ;
        //analogWrite( M2_IN_2 , 0 /*whl_2_vl_PID_out*/ ) ;
        analogWrite( M2_IN_2 , whl_2_vl_PID_out ) ;
    }
    
    // IF PID_2 REVERSE
    else
    {
        analogWrite( M2_IN_1 , 0 /*whl_2_vl_PID_out * -1*/ ) ;
        analogWrite( M2_IN_1 , whl_2_vl_PID_out * -1 ) ;
        //analogWrite( M2_IN_2 , 0 ) ;  // <-- ONE MUST HARD-CODE FLIP THE MAGNITUDE OF THE PWM INTO POSITIVE BECAUSE THE PWM RANGE IS 0 --> 255 (ALL POSITIVE).
    }


} // <--- command_motors()













// _________ PRINTOUT () _________

void printout()
{


    Serial.println() ;
    

  // PRINT LIGHT BAR ARRAY VALUES
  for ( a = 0 ; a < 13 ; a++ ) 
  {
      Serial.print( light_values_bin[ a ] ) ; 
      Serial.print("   ") ;    
  }


    //Serial.println() ;




    // PRINT
    Serial.print( "    rt_int_detect: " ) ; 
    Serial.print( rt_initial_detect ) ; 


    // PRINT
    Serial.print( "    rt_int_lch: " ) ; 
    Serial.print( rt_initial_latch ) ; 


    // PRINT
    Serial.print( "    rt_cpn: " ) ; 
    Serial.print( rt_enc_cupon ) ; 


    // PRINT
    Serial.print( "    rt_fn_detect: " ) ; 
    Serial.print( rt_final_detect ) ; 


    // PRINT
    Serial.print( "    go_for_st: " ) ; 
    Serial.print( go_for_st ) ; 


    // PRINT
    Serial.print( "    go_for_rt: " ) ; 
    Serial.print( go_for_rt ) ; 


    // PRINT
    Serial.print( "    rt_mv_latch: " ) ; 
    Serial.print( rt_move_latch ) ; 


    // PRINT
    Serial.print( "    rt_wh1_fwd_cpt: " ) ; 
    Serial.print( rt_mv_fwd_one_cmplt ) ; 
    
    
    // PRINT
    Serial.print( "    rt_wh2_fwd_cpt: " ) ; 
    Serial.print( rt_mv_fwd_two_cmplt ) ; 


    // PRINT
    Serial.print( "    rt_wh1_rot_cpt: " ) ; 
    Serial.print( rt_mv_rot_one_cmplt ) ; 
    
    
    // PRINT
    Serial.print( "    rt_wh2_rot_cpt: " ) ; 
    Serial.print( rt_mv_rot_two_cmplt ) ; 


    // PRINT
    Serial.print( "    Enc1: " ) ; 
    Serial.print( enc1.read() ) ; 


    // PRINT
    Serial.print( "    Enc2: " ) ; 
    Serial.print( enc2.read() * -1 ) ; 


    // PRINT
    Serial.print( "    t_one: " ) ; 
    Serial.print( twinky_one ) ; 


    // PRINT
    Serial.print( "    t_two: " ) ; 
    Serial.print( twinky_two ) ; 

    
    // PRINT
    Serial.print( "     whl_1_PID_out: " ) ; 
    Serial.print( whl_1_vl_PID_out, 2 ) ; 


    // PRINT
    Serial.print( "     whl_2_PID_out: " ) ; 
    Serial.print( whl_2_vl_PID_out, 2 ) ; 


    // PRINT
    Serial.print( "    whl2_math: " ) ; 
    Serial.print( ( enc2.read() * -1 ) - rt_enc_two_final_detect_update ) ; 


/*

    // PRINT
    Serial.print( "    rt_whl_one_compt: " ) ; 
    Serial.print( rt_wheel_one_mv_complt ) ; 


    // PRINT
    Serial.print( "    rt_fin_pos_gate: " ) ; 
    Serial.print( rt_position_final_gate ) ; 


    // PRINT
    Serial.print( "    rt_enc_one_int: " ) ; 
    Serial.print( rt_enc_one_initial ) ; 


    // PRINT
    Serial.print( "    rt_enc_two_int: " ) ; 
    Serial.print( rt_enc_two_initial ) ; 


    // PRINT
    Serial.print( "    rt_enc_one_fin: " ) ; 
    Serial.print( rt_enc_one_final_detect ) ; 


    // PRINT
    Serial.print( "    rt_enc_two_fin: " ) ; 
    Serial.print( rt_enc_two_final_detect ) ; 
*/
    








/*

    // PRINT
    Serial.print( "    d_detected: " ) ; 
    Serial.print( d_detected ) ; 


    // PRINT
    Serial.print( "    d_mv_latch: " ) ; 
    Serial.print( d_mv_latch ) ; 


    // PRINT
    Serial.print( "    d_enc_cupon: " ) ; 
    Serial.print( d_enc_cupon ) ; 


    // PRINT
    Serial.print( "    d_mv_cmplt: " ) ; 
    Serial.print( d_mv_cmplt ) ; 


    // PRINT
    Serial.print( "    d_mv_fwd_one_cmplt: " ) ; 
    Serial.print( d_mv_fwd_one_cmplt ) ; 


    // PRINT
    Serial.print( "    d_mv_fwd_two_cmplt: " ) ; 
    Serial.print( d_mv_fwd_two_cmplt ) ; 


    // PRINT
    Serial.print( "    d_mv_rot_one_cmplt: " ) ; 
    Serial.print( d_mv_rot_one_cmplt ) ; 


    // PRINT
    Serial.print( "    d_mv_rot_two_cmplt: " ) ; 
    Serial.print( d_mv_rot_two_cmplt ) ; 
    

    // PRINT
    Serial.print( "    enc1 - d_prev: " ) ; 
    Serial.print( enc1.read() - d_enc_one_prev ) ; 


    // PRINT
    Serial.print( "    enc2 - d_prev: " ) ; 
    Serial.print( enc2.read() - d_enc_two_prev ) ; 


    // PRINT
    Serial.print( "    t_one_math: " ) ; 
    Serial.print( twinky_one - ( twinky_one_d + 450 ) ) ; 


    // PRINT
    Serial.print( "    t_two_math: " ) ; 
    Serial.print( twinky_two - ( twinky_two_d - 450 ) ) ; 


    // PRINT
    Serial.print( "    t_one: " ) ; 
    Serial.print( twinky_one ) ; 


    // PRINT
    Serial.print( "    t_two: " ) ; 
    Serial.print( twinky_two ) ; 

*/


/*

    // PRINT
    Serial.print( "    one_spd: " ) ; 
    Serial.print( twinky_one_speed ) ; 


    // PRINT
    Serial.print( "    two_spd: " ) ; 
    Serial.print( twinky_two_speed ) ; 


*/










/*

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



*/








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

/*


    // PRINT
    Serial.print( "     twy_one: " ) ; 
    Serial.print( twinky_one ) ; 

    
    // PRINT
    Serial.print( "     twy_one_spd: " ) ; 
    Serial.print( twinky_one_speed ) ; 


        // PRINT
    Serial.print( "     twy_two: " ) ; 
    Serial.print( twinky_two ) ; 

    
    // PRINT
    Serial.print( "     twy_two_spd: " ) ; 
    Serial.print( twinky_two_speed ) ; 



    // PRINT
    Serial.print( "     err_PID_v1: " ) ; 
    Serial.print( whl_1_vl_PID_err, 2 ) ; 


    // PRINT
    Serial.print( "     err_PID_v2: " ) ; 
    Serial.print( whl_2_vl_PID_err, 2 ) ; 


    // PRINT
    Serial.print( "     err_line: " ) ; 
    Serial.print( line_error ) ; 
*/

/*    
    // PRINT
    Serial.print( "     err_v2: " ) ; 
    Serial.print( whl_2_vl_setpoint - whl_2_vl_feedback, 1 ) ; 
*/

/*
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

*/




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
