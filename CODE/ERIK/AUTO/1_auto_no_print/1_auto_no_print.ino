


// ______ LIBRARYS______

#include <Encoder.h>
#include <Adafruit_MCP3008.h>






// ______ GLOBAL VARIABLES by Function ______


  // OVERALL
  int a = 0 ;
  
  float twinky_speed_const = 0.45 ;
  //float twinky_two_speed_const = twinky_one_speed_const ;

  float twinky_one_speed = twinky_speed_const ;
  float twinky_two_speed = twinky_speed_const ;



  // SPECIAL DETECT
  int decision = 1 ;  // <-- ONE IS A NUTRAL DECISION
  bool rdy_for_dec = 0 ;
  bool hault = 0 ;
  bool out_detect = 0 ;

    // SAMPLE BIN
    int light_values_bin[ 16 ] ;
    bool cur_bin_aray[ 16 ] ;
  
    // COMPARE BIN
    bool prev_bin_aray[ 16 ] ;
    bool last_in_bin_aray[ 16 ] ;
    bool comp_bin_diff = 0 ;
    bool comp_bin = 0 ;
    bool comp_bin_last = 0 ;
  
    // OBSERVE IN BIN
    bool rst_gate = 1 ;
    int prev_enc_one = 0 ;
    int prev_enc_two = 0 ;
    short last_det_in_sig = 0 ;
  
    // SUM BIN ARAY
    short sum_bin = 0 ;

    // DET IN SIG
    bool flag_ln = 0 ;
    bool flag_f = 0 ;
    bool flag_de = 0 ;
    bool flag_r = 0 ;
    bool flag_l = 0 ;
    bool flag_t = 0 ;

    // DET OUT SIG 
    bool flag_dr = 0 ;
    bool flag_dl = 0 ;
    bool flag_dt = 0 ;
    bool flag_or = 0 ;
    bool flag_ol = 0 ;
    bool flag_ot = 0 ;

    // DECISION APLY
    bool go_r_move = 0 ;
    bool go_l_move = 0 ;
    bool go_d_move = 0 ;
    bool go_f_move = 0 ;





  // MOVE MANUAL
  int left_amount = 0 ;
  int right_amount = 0 ;
  bool man_mv_comp = 0 ;
  bool left_comp = 0 ;
  bool right_comp = 0 ;



/*

  // MOVE COMMANDS
    bool rt_wheel_one_mv_complt = 0 ;
    bool rt_wheel_two_mv_complt = 0 ;
    bool rt_mv_rot_one_cmplt = 0 ;
    bool rt_mv_rot_two_cmplt = 0 ;
    bool rt_mv_fwd_one_cmplt = 0 ;
    bool rt_mv_fwd_two_cmplt = 0 ;
*/


  // PID LF CONTROL
  unsigned int prev_line_err_time = 0 ;
  //float twinky_max = twinky_one_speed ;    // <-- CONSOLIDATE. GET RID OF THIS TWINKY MAX STUFF.
  //float twinky_min = twinky_one_speed * -1 ;



  // LV ADJUST
  float b = 0 ;
  float adjustment = 0 ;




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
  //float line_lf_PID_KP = 00.0005 ;
  float line_lf_PID_KP = twinky_speed_const / (float)3200 ;
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
  float whl_1_vl_PID_KP = 00.9500 ;
  float whl_1_vl_PID_KI = 00.0026 ;
  float whl_1_vl_PID_KD = 46.0000 ;



  // WHL 2 VL PID CALC
  float whl_2_vl_PID_err = 0 ;
  float whl_2_vl_PID_err_prev = 0 ;
  float whl_2_vl_PID_P = 0 ;
  float whl_2_vl_PID_I = 0 ;
  float whl_2_vl_PID_D = 0 ;
  float whl_2_vl_PID_out = 0 ;
  float whl_2_vl_PID_feedback = 0 ;
  unsigned int whl_2_vl_PID_D_time_prev = 0 ;
  float whl_2_vl_PID_KP = 00.9500 ;
  float whl_2_vl_PID_KI = 00.0026 ;
  float whl_2_vl_PID_KD = 46.0000 ;



  // LINE ERROR CALC
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
void sample_bin() ;
void compare_bin() ;
void update_prev_aray() ;
void observe_in_bin() ;
void observe_out_bin() ;
void sum_bin_aray() ;
short det_in_sig() ;
void det_out_sig() ;
void decision_aply() ;
void decision_recv() ;
void hault_motors() ;
void move_manual( int , int ) ;
void light_values_adjust() ;


// ______ SETUP ______

void setup() 
{

  // BEGIN SERIAL
  //Serial.begin(9600) ;

  // RADIO?
  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH); // Without this the nRF24 will write to the SPI bus while the ADC's are also talking
  
  // ADC INITIATION
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  


/*
  // GET SERIAL PRINTING FIRST
  for( a = 0 ; a < 25 ; a++ )
  {
//Serial.print( "   twy_two_spd " ) ;
////Serial.println( twinky_two_speed ) ;
    delay(100) ; 
  }

*/




} // <-- setup()












// _____________ MAIN _____________                                <---------- M A I N ---------<

void loop() 
{


    //LISTEN FOR SERIAL
    serial_input() ;



    // CHECK FOR SPECIAL CASE
    special_detect() ;     // <-- ADD A CONDITION THAT FIRST CHECKS FOR SPECIAL MOVE COMPLETION SO AS TO NOT INTERUPT THE SPECIAL MOVE.
    

   
    pid_lf_control() ;
    prev_line_err_time = millis() ; // <-- NOTE HOW THE TIME UPDATE MUST INDEED COME AFTER OTHERWISE IT WOULD BE EQUAL BEFORE GOING INTO THE FUNCTION.


    
    // MOTOR PID CONTROL
    if( ( millis() - prev_twinky_time ) >= 20 )
    { 
        pid_vl_control() ; 
        prev_twinky_time = millis() ;  // <-- NOTE HOW THE TIME UPDATE MUST INDEED COME AFTER OTHERWISE IT WOULD BE EQUAL BEFORE GOING INTO THE FUNCTION.
    }
    
/*

    // PRINTOUT
    if( ( millis() - prev_print_time ) >= 100 )
    {     
      
      printout() ; 
      
      prev_print_time = millis () ;  // <-- NOTE HOW THE TIME UPDATE MUST INDEED COME AFTER OTHERWISE IT WOULD BE EQUAL BEFORE GOING INTO THE FUNCTION.
    }

*/


//delay( 500 ) ;





} // <-- main()                                                     <---------- E N D ---------< 



















// ___________________________________________ FUNCTION LAND ___________________________________________
// _____________________________________________________________________________________________________







// ______ SPECIAL DETECT () ______   // <-- NOTE: WILL NOT IMPEED LINE FOLLOW.

void special_detect()
{

//printout() ; 
////Serial.println( " " ) ;
////Serial.println( "* * * * * * ___IN_SPEC_()___ * * * * * *" ) ;



// PRINT LIGHT BAR ARRAY VALUES
////Serial.println("[ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]") ;
//Serial.print("[ ") ;
for ( a = 0 ; a < 13 ; a++ ) 
{
    //Serial.print( cur_bin_aray[ a ] ) ; 
    //Serial.print("  ") ;    
}

////Serial.println("]") ;




//Serial.print( "   Ot_Det: " ) ;
////Serial.println( out_detect ) ;
//Serial.print( "   Ry_dec: " ) ;
////Serial.println( rdy_for_dec ) ;
//Serial.print( "   Hlt: " ) ;
////Serial.println( hault ) ;


    // IF NOT LOOKING FOR OUT BIN SIG
    if( out_detect == 0 ) observe_in_bin() ;

    // ELSE OBSERVE OUT BIN
    else observe_out_bin() ;

    // DECISION CHECK
    if ( rdy_for_dec == 1 || hault == 1 ) 
    {

        // HAULT MOTORS HERE  <----------- NEED TO DO THIS BECUASE THE HAULT COMMAND WILL NOT GO THROUGH DURING THE 
        hault_motors();
        
        // WAIT FOR DECISION
        decision_recv() ;

        // APLY DECISION
        decision_aply() ;

        // TOE-HEEL TWINKYS
        twinky_one = enc1.read() ;
        twinky_two = enc2.read() * -1 ;

        // CHILL OUT PID's
        whl_1_vl_PID_P = 0 ;
        whl_1_vl_PID_I = 0 ;
        whl_1_vl_PID_D = 0 ;
        whl_1_vl_PID_out = 0 ;
        
        whl_2_vl_PID_P = 0 ;
        whl_2_vl_PID_I = 0 ;
        whl_2_vl_PID_D = 0 ;
        whl_2_vl_PID_out = 0 ;

        prev_twinky_time = millis() ;
    
    }




} // <-- special_detect()










// ______ HAULT_MOTORS () ______

void hault_motors()
{


    // OVERIDE PID OUTPUTS AS ZERO
    whl_1_vl_PID_out = 0 ;
    whl_2_vl_PID_out = 0 ;

    // CALL MOTOR COMMAND
    command_motors() ;


} // <--- hault_motors()
















// ______ SAMPLE_BIN () ______

void sample_bin()
{

    // READ IN LIGHT VALUES
    light_sensor_read() ;
  
    // OBTAIN BINARY ARRAY
    for ( a = 0 ; a < 13 ; a++ ) 
    {
        // THRESHOLD
        if( light_values[ a ] <= 400 ) cur_bin_aray[ a ]  = 1 ;      // <--- SUGGEST CHANGING THRESHOLD AS A FUNCTION OF POSTION ON LIGHT BAR. BUT THAT MAY REQUIRE SPECIAL DETECTION OF WHEN PERPENDICUALR TO THE TAPE. ACTUALLY THIS DOES NOT MATTER FOR THE BINARY CASE.
        else cur_bin_aray[ a ] = 0 ;
    }

} // <--- sample_bin()










// ______ COMPARE_BIN () ______

void compare_bin()
{


//printout() ; 
////Serial.println( " " ) ;
////Serial.println( "___IN_COMPARE_()___" ) ;


    // RESET COMPARER VARIABLES
    comp_bin = 0 ;
    comp_bin_diff = 0 ;
    comp_bin_last = 0 ;


    // _________ COMPARE BIN _________
    for ( a = 0 ; a < 13 ; a++ ) 
    {
        // READ CURRENT BIN
        if( cur_bin_aray[ a ] != prev_bin_aray[ a ] ) // JUST COMPARE WITH WHAT WE ARE LOOKING AT FOR LOCK-ON
        {
            // NOT THE SAME
            comp_bin_diff = 1 ;
        }

    }

    // IF NO DIFFERENCE FOUND AT END OF FOR LOOP
    if( a >= 12 && comp_bin_diff != 1 ) comp_bin = 1 ;

    // ELSE A DIFERENSE WAS FOUND
    else comp_bin = 0 ;





    // IF OUT BIN OPERATION
    if( out_detect == 1 )
    {
        // COMPARE LAST IN BIN
        for ( a = 0 ; a < 13 ; a++ ) 
        {
            // READ CURRENT BIN ARAY
            if( cur_bin_aray[ a ] != last_in_bin_aray[ a ] ) // COMPARES AGAINST THE LAST BIN ARRAY FROM THE IN OBSERVE
            {
                // NOT THE SAME
                comp_bin_last = 0 ;
    
                // RETURN
                return ;
            }
        }

        // IF NO DIFFERENCE FOUND AT END OF FOR LOOP
        comp_bin_last = 1 ;

    }




//Serial.print( "   Cmp_bin: " ) ;
////Serial.println( comp_bin ) ;
//Serial.print( "   Ot_Det: " ) ;
////Serial.println( out_detect ) ;
//Serial.print( "   Cmp_bin_lst: " ) ;
////Serial.println( comp_bin_last ) ;
    

} // <--- compare_bin()

















// ______ UPDATE_PREV_ARAY () ______

void update_prev_aray()
{
  
    // COMPARE BIN
    for ( a = 0 ; a < 13 ; a++ ) 
    {
        // READ CURRENT BIN ARAY
        prev_bin_aray[ a ] = cur_bin_aray[ a ] ;
        
        // IF NOT DOING AN OUT DETECT KEEP A RECORD OF THE LAST ARAY
        if( out_detect == 0 ) last_in_bin_aray[ a ] = cur_bin_aray[ a ] ;
    }
    

} // <--- update_prev_aray()
















// ______ OBSERVE_IN_BIN () ______

void observe_in_bin()
{


//printout() ; 
////Serial.println( " " ) ;
////Serial.println( "___IN_OBSRV_IN_()___" ) ;

    // SAMPLE THE BIN ARRAY
    sample_bin() ;

    // COMPARE THE BIN ARRAY
    compare_bin() ;

    // UPDATE PREVIOUS ARRAY
    update_prev_aray() ;

////Serial.println( " " ) ;
////Serial.println( "___BACK_IN_OBSRV_IN_()___" ) ;
//Serial.print( "   Cmp_bin: " ) ;
////Serial.println( comp_bin ) ;
//Serial.print( "   rst_gt: " ) ;
////Serial.println( rst_gate ) ;
//Serial.print( "   P_ec_one: " ) ;
////Serial.println( prev_enc_one ) ;

    // UPDATE POSITION
    if( comp_bin == 1 && rst_gate == 1 )
    {
        // RESET RST_GATE
        rst_gate = 0 ;

        // UPDATE POSTIONS
        prev_enc_one = enc1.read() ;
        prev_enc_two = enc2.read() * -1 ;
      
    } // <-- if( bin )


    // IF NOT THE SAME
    if( comp_bin == 0 )
    {
        // RESET RST_GATE
        rst_gate = 1 ;
    }


    // ELSE IF UN_INTURUPTED DETECTION
    else if( ( enc1.read() - prev_enc_one > 10 ) || ( ( enc2.read() * -1 ) - prev_enc_two > 10 ) )
    {
/*
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
*/   
      
        
        // RESET RST_GATE
        rst_gate = 1 ;

        // DETERMINE INITIAL SIGNATURE AND STORE IT FOR LATER COMMPARISON
        last_det_in_sig = det_in_sig() ;
        

    } // <-- else_if( enc - prev )

    
} // <--- observe_in_bin()















// ______ OBSERVE_OUT_BIN () ______

void observe_out_bin()
{

//printout() ; 
////Serial.println( " " ) ;
////Serial.println( "___IN_OBSRV_OUT_()___" ) ;


    // SAMPLE THE BIN ARRAY
    sample_bin() ;

    // COMPARE THE BIN ARRAY
    compare_bin() ;  
    
    // UPDATE PREVIOUS ARRAY
    update_prev_aray() ;

////Serial.println( " " ) ;
////Serial.println( "___BACK_IN_OBSRV_OUT_()___" ) ;
//Serial.print( "   Cmp_bin: " ) ;
////Serial.println( comp_bin ) ;
//Serial.print( "   Cmp_bin_lst: " ) ;
////Serial.println( comp_bin_last ) ;
//Serial.print( "   rst_gt: " ) ;
////Serial.println( rst_gate ) ;
//Serial.print( "   P_ec_one: " ) ;
////Serial.println( prev_enc_one ) ;


    // UPDATE POSITION
    if( comp_bin == 1 && comp_bin_last == 0 && rst_gate == 1 )  // <-- NOT SURE ABOUT THIS LOGIC.
    {
        // RESET RST_GATE
        rst_gate = 0 ;

        // UPDATE POSTIONS
        prev_enc_one = enc1.read() ;
        prev_enc_two = enc2.read() * -1 ;
      
    } // <-- if( bin )


// HERE IS WHERE I WOULD COMPARE TO SEE IF WHAT IS FOUND IS THE SAME AS THE IN SIGNATURE.
// PASS THE CURRENT ARRAY TO THE DETERMIND IN BIN AND COMPARE ITS RETURN WITH ITS PREVIOUS RETURN.


    // IF NOT THE SAME OR THE SAME SIG AS IN AND NOT FINISH
    if( ( ( comp_bin == 0 ) || ( comp_bin_last == 1 ) || ( det_in_sig() == last_det_in_sig ) ) && ( flag_f == 0 ) ) // <-- NOT SURE ABOUT THIS LOGIC.
    {
        // RESET RST_GATE
        rst_gate = 1 ;
    }



    // ELSE IF UN_INTURUPTED DETECTION
    else if( ( enc1.read() - prev_enc_one > 10 ) || ( ( enc2.read() * -1 ) - prev_enc_two > 10 ) )
    {
/*
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
        ////Serial.println( "  * * * * * * * * * L O C K  * * * * * L O C K  * * * * * * * L O C K * * * * * * * * * L O C K " ) ;
*/
      
        // RESET RST_GATE
        rst_gate = 1 ;

        // RESET FINAL DETECT
        out_detect = 0 ;

        // DETERMINE OUT SIGNATURE
        det_out_sig() ;
  
        
    } // <-- else_if( enc - prev )


} // <--- observe_out_bin()













// ______ SUM_BIN_ARAY () ______

void sum_bin_aray()
{

    // RESET SUM
    sum_bin = 0 ;
  
    // ADD UP ARRAY
    for ( a = 0 ; a < 13 ; a++ ) 
    {
        // READ CURRENT BIN ARAY
        sum_bin = sum_bin + cur_bin_aray[ a ] ;
    }


} // <-- sum_bin_aray()













    

// ______ DET_IN_SIG () ______

short det_in_sig()
{

//printout() ; 
////Serial.println( " " ) ;
////Serial.println( "___IN_DET_IN_SIG_()___" ) ;
// PRINT LIGHT BAR ARRAY VALUES
////Serial.println("[ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]") ;
//Serial.print("[ ") ;
for ( a = 0 ; a < 13 ; a++ ) 
{
    //Serial.print( cur_bin_aray[ a ] ) ; 
    //Serial.print("  ") ;    
}

////Serial.println("]") ;

    // SUM BIN ARAY
    sum_bin_aray() ;


//Serial.print( "  Sum_bin: " ) ;
////Serial.println( sum_bin ) ;


    // RESET DECISION
    decision = 0 ;



    // PREVENT FROM EXECUTING IF IN OUT DETECT
    if( out_detect == 0 )
    {
        // RESET FLAGS       <-- THIS FUNCTION COMES BEFORE OUT DETECTION SO IT IS RESONABLE TO RESET THEM HERE.
        //flag_ln = 0 ; // <-- THIS SHOULD "STICK" UNLESS SOMETHING ELSE IS DETECTED     
        flag_f = 0 ;
        flag_de = 0 ;
        flag_l = 0 ;
        flag_r = 0 ;
        flag_t = 0 ;
        flag_ol = 0 ;
        flag_or = 0 ;
        flag_ot = 0 ;
    }  





    // ADD CODE HERE TO CHECK IF THE SUM IS CONTINUOIS, THAT IS, FURTHER SMARTS THAT IT IS INDEED A LINE AND NOT JUST  A SUM OF RANDOM STUFF.
    //                    [ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]
    // CHECK FOR LINE     [ 0  0  0  X  X  1  X  1  X  X  0  0  0 ]  // <-- NOTE: 5 OR 7
    if( ( sum_bin <= 4 ) && ( ( sum_bin >= 2 ) ) /*&& ( cur_bin_aray[ 0 ] == 0 && cur_bin_aray[ 1 ] == 0 && cur_bin_aray[ 2 ] == 0 && ( cur_bin_aray[ 5 ] == 1  || cur_bin_aray[ 7 ] == 1 ) && cur_bin_aray[ 10 ] == 0 && cur_bin_aray[ 11 ] == 0 && cur_bin_aray[ 12 ] == 0 )*/ )
    {
        
        // PREVENT FROM EXECUTING IF IN OUT DETECT
        if( out_detect == 0 )
        {
            // FLAG FOR LINE FOLLOW
            flag_ln = 1 ;
            
            // READY FOR DECISION
            rdy_for_dec = 0 ;
    
            // AUTOMATICALLY DECIDE TO LINE FOLLOW
            //decision = 1 ;     
        }


        //Serial.print( "  Return: " ) ;
        ////Serial.println( "1" ) ;

        // RETURN
        return 1 ;
    }
  

    //                    [ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]
    // CHECK FOR FINISH   [ 0  0  X  X  1  1  1  1  1  X  X  0  0 ]
    else if( ( sum_bin >= 3 && sum_bin <= 7 ) && ( cur_bin_aray[ 0 ] == 0 && cur_bin_aray[ 1 ] == 0 && cur_bin_aray[ 4 ] == 1 && cur_bin_aray[ 5 ] == 1 && cur_bin_aray[ 6 ] == 1 && cur_bin_aray[ 7 ] == 1 && cur_bin_aray[ 8 ] == 1 && cur_bin_aray[ 11 ] == 0 && cur_bin_aray[ 12 ] == 0 ) )
    {

        // PREVENT FROM EXECUTING IF IN OUT DETECT
        if( out_detect == 0 )
        {   
            // FLAG FOR FINISH
            flag_f = 1 ;
    
            // READY FOR DECISION
            rdy_for_dec = 1 ;
    
            // RESET LF FLAG
            flag_ln = 0 ;
        }
        
        //Serial.print( "  Return: " ) ;
        ////Serial.println( "5" ) ;

        // RETURN
        return 5 ;
    }


// SOMEWHERE WE NEED TO DECLARE A DEFAULT DRIVE FORWARD TO IGNORE THE LN FOLLOW TO MOVE FORWARD. MAYBE THAT CAN HAPPEN BY SIIMPLY TOGGLING OFF THE LN FLAG IF ANOTHER DETECTED. FOR EXAMPLE, A MOVE FORWARD BOOLIAN.



    //                    [ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]
    // CHECK FOR DEAD END [ 0  0  0  0  0  0  0  0  0  0  0  0  0 ]
    else if( cur_bin_aray[ 0 ] == 0 && cur_bin_aray[ 1 ] == 0 && cur_bin_aray[ 3 ] == 0 && cur_bin_aray[ 4 ] == 0 && cur_bin_aray[ 5 ] == 0 && cur_bin_aray[ 6 ] == 0 && cur_bin_aray[ 7 ] == 0 && cur_bin_aray[ 8 ] == 0 && cur_bin_aray[ 9 ] == 0 && cur_bin_aray[ 10 ] == 0 && cur_bin_aray[ 11 ] == 0 && cur_bin_aray[ 12 ] == 0 ) 
    {

        // PREVENT FROM EXECUTING IF IN OUT DETECT
        if( out_detect == 0 )
        { 
            // FLAG FOR DEAD END
            flag_de = 1 ;
    
            // READY FOR DECISION
            rdy_for_dec = 1 ;
    
            // AUTOMATICALLY DECIDE TO TURN_AROUND
            //decision = 4 ;
    
            // RESET LF FLAG
            flag_ln = 0 ;
        }
        
        //Serial.print( "  Return: " ) ;
        ////Serial.println( "9" ) ;

        // RETURN
        return 9 ;
    }

    
    //                    [ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]
    // CHECK FOR RIGHT    [ 0  0  0  X  X  X  1  1  1  1  1  1  X ]
    else if( cur_bin_aray[ 0 ] == 0 && cur_bin_aray[ 1 ] == 0 && cur_bin_aray[ 6 ] == 1 && cur_bin_aray[ 7 ] == 1 && cur_bin_aray[ 8 ] == 1 && cur_bin_aray[ 9 ] == 1 && cur_bin_aray[ 10 ] == 1 && cur_bin_aray[ 11 ] == 1 )
    {

        // PREVENT FROM EXECUTING IF IN OUT DETECT
        if( out_detect == 0 )
        { 
            // FLAG FOR RIGHT
            flag_r = 1 ;
    
            // OUT INSPECT
            out_detect = 1 ;
    
            // RESET LF FLAG
            flag_ln = 0 ;
        }

        //Serial.print( "  Return: " ) ;
        ////Serial.println( "2" ) ;

        // RETURN
        return 2 ;
    }


    //                    [ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]
    // CHECK FOR LEFT     [ X  1  1  1  1  1  1  X  X  X  0  0  0 ]
    else if( cur_bin_aray[ 1 ] == 1 && cur_bin_aray[ 2 ] == 1 && cur_bin_aray[ 3 ] == 1 && cur_bin_aray[ 4 ] == 1 && cur_bin_aray[ 5 ] == 1 && cur_bin_aray[ 6 ] == 1 && cur_bin_aray[ 10 ] == 0 && cur_bin_aray[ 11 ] == 0 && cur_bin_aray[ 12 ] == 0 )
    {

        // PREVENT FROM EXECUTING IF IN OUT DETECT
        if( out_detect == 0 )
        {
            // FLAG FOR LEFT
            flag_l = 1 ;
    
            // OUT INSPECT
            out_detect = 1 ;
    
            // RESET LF FLAG
            flag_ln = 0 ;
        }
        
        //Serial.print( "  Return: " ) ;
        ////Serial.println( "3" ) ;

        // RETURN
        return 3 ;
    }


    //                    [ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]
    // CHECK FOR TEE      [ X  X  1  1  1  1  1  1  1  1  1  X  X ]
    else if( ( sum_bin >= 7 ) && ( cur_bin_aray[ 2 ] == 1 && cur_bin_aray[ 3 ] == 1 && cur_bin_aray[ 4 ] == 1 && cur_bin_aray[ 5 ] == 1 && cur_bin_aray[ 6 ] == 1 && cur_bin_aray[ 7 ] == 1 && cur_bin_aray[ 8 ] == 1 && cur_bin_aray[ 9 ] == 1 ) && cur_bin_aray[ 10 ] == 1 ) 
    {   

        // PREVENT FROM EXECUTING IF IN OUT DETECT
        if( out_detect == 0 )
        {
            // FLAG FOR TEE
            flag_t = 1 ;
    
            // OUT INSPECT
            out_detect = 1 ;
    
            // RESET LF FLAG
            flag_ln = 0 ;
        }
            

        //Serial.print( "  Return: " ) ;
        ////Serial.println( "6" ) ;


        // RETURN
        return 6 ;
    }


    // UNDERTERMINED
    else 
    {
        ////Serial.println( "  Undetermined " ) ;

        //Serial.print( "  Return: " ) ;
        ////Serial.println( "0" ) ;

        // PREVENT FROM EXECUTING IF IN OUT DETECT
        if( out_detect == 0 )
        {
            // HAULT
            //hault = 1 ;
            hault = 0 ;
      
            // LN FLAG
            //flag_ln = 0 ;
            flag_ln = 1 ;
        }
        
        // RETURN
        return 0 ;
    }



// THE FOLLOWING PRINT STATMENTS NEVER GET EXECUTED

//Serial.print( "   Ry_dec: " ) ;
////Serial.println( rdy_for_dec ) ;
//Serial.print( "  Decsn: " ) ;
////Serial.println( decision ) ;
//Serial.print( "  Hault: " ) ;
////Serial.println( hault ) ;
//Serial.print( "  FLAG_LN: " ) ;
////Serial.println( flag_ln ) ;
//Serial.print( "  FLAG_DE: " ) ;
////Serial.println( flag_de ) ;
//Serial.print( "  FLAG_R: " ) ;
////Serial.println( flag_r ) ;
//Serial.print( "  FLAG_L: " ) ;
////Serial.println( flag_l ) ;
//Serial.print( "  FLAG_T: " ) ;
////Serial.println( flag_t ) ;
//Serial.print( "  FLAG_OR: " ) ;
////Serial.println( flag_or ) ;
//Serial.print( "  FLAG_OL: " ) ;
////Serial.println( flag_ol ) ;
//Serial.print( "  FLAG_OT: " ) ;
////Serial.println( flag_ot ) ;
//Serial.print( "  FLAG_F: " ) ;
////Serial.println( flag_f ) ;






} // <--- det_in_sig()













// ______ DET_OUT_SIG () ______     // <-- WE CAN USE THIS BOLLIAN TO DO THE MOVE FORWARD COMMAND IN THE CASES WHERE IT IS NEEDED IN DET_IN_SIG ABOVE.

void det_out_sig()
{




//Serial.print( "  __________  BEFORE _________ " ) ;
//Serial.print( "   Ry_dec: " ) ;
////Serial.println( rdy_for_dec ) ;
//Serial.print( "  Decsn: " ) ;
////Serial.println( decision ) ;
//Serial.print( "  Hault: " ) ;
////Serial.println( hault ) ;
//Serial.print( "  FLAG_LN: " ) ;
////Serial.println( flag_ln ) ;
//Serial.print( "  FLAG_DE: " ) ;
////Serial.println( flag_de ) ;
//Serial.print( "  FLAG_R: " ) ;
////Serial.println( flag_r ) ;
//Serial.print( "  FLAG_L: " ) ;
////Serial.println( flag_l ) ;
//Serial.print( "  FLAG_T: " ) ;
////Serial.println( flag_t ) ;
//Serial.print( "  FLAG_OR: " ) ;
////Serial.println( flag_or ) ;
//Serial.print( "  FLAG_OL: " ) ;
////Serial.println( flag_ol ) ;
//Serial.print( "  FLAG_OT: " ) ;
////Serial.println( flag_ot ) ;
//Serial.print( "  FLAG_F: " ) ;
////Serial.println( flag_f ) ;
////Serial.println( " " ) ;
////Serial.println( " " ) ;
////Serial.println( " " ) ;





//printout() ; 
////Serial.println( " " ) ;
////Serial.println( "___IN_DET_OUT_SIG_()___" ) ;
// PRINT LIGHT BAR ARRAY VALUES
////Serial.println("[ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]") ;
//Serial.print("[ ") ;
for ( a = 0 ; a < 13 ; a++ ) 
{
    //Serial.print( cur_bin_aray[ a ] ) ; 
    //Serial.print("  ") ;    
}

////Serial.println("]") ;

    // SUM BIN ARAY
    sum_bin_aray() ;    // <-- THIS IS NOT BEING USED?

    // RESET OUT DETECT FLAG
    out_detect = 0 ;

  
    //                    [ 0  1  2  3  4  5  6  7  8  9  10 11 12 ]
    // CHECK FOR DEAD END [ 0  0  0  0  0  0  0  0  0  0  0  0  0 ]
    if( cur_bin_aray[ 0 ] == 0 && cur_bin_aray[ 1 ] == 0 && cur_bin_aray[ 3 ] == 0 && cur_bin_aray[ 4 ] == 0 && cur_bin_aray[ 5 ] == 0 && cur_bin_aray[ 6 ] == 0 && cur_bin_aray[ 7 ] == 0 && cur_bin_aray[ 8 ] == 0 && cur_bin_aray[ 9 ] == 0 && cur_bin_aray[ 10 ] == 0 && cur_bin_aray[ 11 ] == 0 && cur_bin_aray[ 12 ] == 0 ) 
    {
      // DERMINE OUT SIGNATURE
      if( flag_r == 1 ) flag_dr = 1 ;
      if( flag_l == 1 ) flag_dl = 1 ;
      if( flag_t == 1 ) flag_dt = 1 ;
         
    }


    else 
    {
      // DERMINE OUT SIGNATURE
      if( flag_r == 1 /*&& ( sum_bin <= 4 )*/ ) flag_or = 1 ;
      if( flag_l == 1 /*&& ( sum_bin <= 4 )*/ ) flag_ol = 1 ;
      if( flag_t == 1 /*&& ( sum_bin <= 4 )*/ ) flag_ot = 1 ;
         
    }


    // READY FOR DECISION
    rdy_for_dec = 1 ;




//Serial.print( "  Rdy_f_decsn: " ) ;
////Serial.println( rdy_for_dec ) ;
//Serial.print( "  Hault: " ) ;
////Serial.println( hault ) ;
//Serial.print( "  FLAG_LN: " ) ;
////Serial.println( flag_ln ) ;
//Serial.print( "  FLAG_DE: " ) ;
////Serial.println( flag_de ) ;
//Serial.print( "  FLAG_R: " ) ;
////Serial.println( flag_r ) ;
//Serial.print( "  FLAG_L: " ) ;
////Serial.println( flag_l ) ;
//Serial.print( "  FLAG_T: " ) ;
////Serial.println( flag_t ) ;
//Serial.print( "  FLAG_OR: " ) ;
////Serial.println( flag_or ) ;
//Serial.print( "  FLAG_OL: " ) ;
////Serial.println( flag_ol ) ;
//Serial.print( "  FLAG_OT: " ) ;
////Serial.println( flag_ot ) ;
//Serial.print( "  FLAG_F: " ) ;
////Serial.println( flag_f ) ;


  

} // <--- det_out_sig()














// ______ DECISION_APLY () ______

void decision_aply()
{


// PRETEND PRINT
////Serial.println( " " ) ;
////Serial.println( "___DICISION_APLY () ____" ) ;

//Serial.print( "  Decsn: " ) ;
////Serial.println( decision ) ;
//Serial.print( "  FLAG_R: " ) ;
////Serial.println( flag_r ) ;
//Serial.print( "  FLAG_L: " ) ;
////Serial.println( flag_l ) ;


    // RESET READY FOR DECISION
    rdy_for_dec = 0 ;


    // TOE HEEL MATCH TWINKY       // MAYBE BETTER CLASIFY THIS WITH THE MOVE COMMANDS
    twinky_one = enc1.read() ;
    twinky_two = enc2.read() ;





    // PRE_DECISION HOUSE KEEPING
    if( decision != 0 )
    {
        // RELAESE HAULT
        hault = 0 ;
    }






    // IF DECISION IS ZERO HAULT
    if( decision == 0 ) hault = 1 ;  // <-- DECISION 0 AS HAULT, THAT WAY IT DEFAULTS TO A HAULT IF THE DECISION HAS NOT BEEN UPDATED.



    // ELSE IF GO STRAIT
    else if( decision == 1 && ( flag_ln == 1 || flag_or == 1 || flag_ol == 1 || flag_ot == 1 ) )
    {

        // DO NOTHING AND RESET FLAGS
        flag_or = 0 ;
        flag_ol = 0 ;
        flag_ot = 0 ;
    }



    // ELSE IF TURN RIGHT
    else if( decision == 2 && ( flag_r == 1 || flag_t == 1 ) )
    {

        // PRETEND PRINT  
        ////Serial.println( " " ) ;
        ////Serial.println( "    TURNING RIGHT" ) ;
        
        // RESET FLAGS    <-- THESE ARE THE ONLY FLAGS THAT COULD BE ON THAT IS WHY THEY ARE THE ONLY ONES BEING RESET.
        flag_or = 0 ;
        flag_ot = 0 ;
        
        //go_r_move = 1 ;   // <-- WHERE DO ALL OF THESE COMMANDS GET USED?
  
        // DELAY FOR MOVE BY HAND
        //delay( 500 ) ;



        // MOVE PATTERN
        move_manual( 210 , 210 ) ;
        //delay(2000) ;
        move_manual( 205 , -205 ) ;
        //delay(2000) ;

 
        // PRETEND PRINT
        ////Serial.println( " " ) ;
        ////Serial.println( "    TR_COMPLETE" ) ;
  
        // USE UP DECISION BACK TO NUTRAL
        decision = 1 ;     
    }


    // ELSE IF TURN LEFT
    else if( decision == 3 && ( flag_l == 1 || flag_t == 1 ) )
    {
      
        // PRETEND PRINT
        ////Serial.println( " " ) ;
        ////Serial.println( "    TURNING LEFT" ) ;
        
        // RESET FLAGS    <-- THESE ARE THE ONLY FLAGS THAT COULD BE ON THAT IS WHY THEY ARE THE ONLY ONES BEING RESET.
        flag_ol = 0 ;
        flag_ot = 0 ;

        
        // MOVE PATTERN
        move_manual( 210 , 210 ) ;
        move_manual( -210 , 210 ) ;

        
        //go_l_move = 1 ;
  
        // DELAY FOR MOVE BY HAND
        //delay( 5000 ) ;
  
        // PRETEND PRINT
        ////Serial.println( " " ) ;
        ////Serial.println( "    TL_COMPLETE" ) ;

        // USE UP DECISION BACK TO NUTRAL
        decision = 1 ;
    }



    // ELSE IF TURN AROUND
    else if( decision == 4 )
    {
  
        // PRETEND PRINT
        ////Serial.println( " " ) ;
        ////Serial.println( "    TURNING AROUND" ) ;
        
        //go_d_move = 1 ;

        // MOVE PATTERN
        move_manual( 280 , 280 ) ;
        //delay(500) ;
        move_manual( 430 , -430 ) ;

        // DELAY FOR MOVE BY HAND
        //delay( 5000 ) ;
  
        // PRETEND PRINT
        ////Serial.println( " " ) ;
        ////Serial.println( "    TU_COMPLETE" ) ;

        // USE UP DECISION BACK TO NUTRAL
        decision = 1 ;
    }


    // ELSE IF FINISH
    else if( decision == 5 && flag_f == 1 ) // <-- DONT WASTE TIME IMPLIMENTING THIS UNTIL ALL ELSE IS DONE.
    {
        // PRETEND PRINT
        ////Serial.println( " " ) ;
        ////Serial.println( "    FINISH" ) ;
        
        //go_f_move = 1 ;
        //hault = 1 ;  
/*
        // MOVE PATTERN
        move_manual( 700 , 700 ) ;
        move_manual( 470 , -470 ) ;
*/

        // HAULT
        hault = 1 ;
        hault_motors() ;
        while(1){ } ;

  
        // DELAY FOR MOVE BY HAND
        //delay( 5000 ) ;
  
        // PRETEND PRINT
        ////Serial.println( " " ) ;
        ////Serial.println( "    F_COMPLETE" ) ;

        // USE UP DECISION BACK TO NUTRAL
        decision = 1 ;
    }


    else 
    {
        // HOLD FOR PROPER DECISION     <-- I DON'T THINK THIS IS NEEDED BECAUSE IT WILL HOLD THE TURN FLAGS UNTIL 10 ENCODER TICKS PASSES, AT WHICH POINT IT WILL THEN CALL DET_IN_SIG AND RESET THE FLAGS.
        //decision_hold = 1 ;
  
        // HAULT
        hault =  1 ;   // <-- NOTE: THE OR OL OT FLAGS DO NOT GET RESET BECAUSE WE ARE WAITING FOR COMMAND. 
  
        // IF THE PLACE ON THE MAP DOES NOT ACOMMODATE THE COMMAND
        ////Serial.println( " " ) ;
        ////Serial.println( " I am disinclined to aquest that request. " ) ;
      
    }




//Serial.print( "  Hault: " ) ;
////Serial.println( hault ) ;
//Serial.print( "  FLAG_LN: " ) ;
////Serial.println( flag_ln ) ;
//Serial.print( "  FLAG_DE: " ) ;
////Serial.println( flag_de ) ;
//Serial.print( "  FLAG_R: " ) ;
////Serial.println( flag_r ) ;
//Serial.print( "  FLAG_L: " ) ;
////Serial.println( flag_l ) ;
//Serial.print( "  FLAG_T: " ) ;
////Serial.println( flag_t ) ;
//Serial.print( "  FLAG_OR: " ) ;
////Serial.println( flag_or ) ;
//Serial.print( "  FLAG_OL: " ) ;
////Serial.println( flag_ol ) ;
//Serial.print( "  FLAG_OT: " ) ;
////Serial.println( flag_ot ) ;
//Serial.print( "  FLAG_F: " ) ;
////Serial.println( flag_f ) ;





    // USE UP DECISION BACK TO NUTRAL
    //decision = 1 ;



// NOTE: UNUSED FLAGS MAY NOT GET RECET?  <-- ALL FLAGS GET RESET FOR IN_DETECTION




} // <--- decision_aply()















// ______ DESICION_RESV () ______

void decision_recv()
{
  
    // PRETEND PRINT
    ////Serial.println( " " ) ;
    ////Serial.println( "___IN_DISC_RESV ()___" ) ;
    ////Serial.println( "...waiting for input..." ) ;




    // IF TEE OR RIGHT GO RIGHT
    if( ( flag_t == 1 ) || ( flag_r == 1 ) ) decision = 2 ;

    // IF OPEN LEFT GO STRAIT
    else if ( flag_ol == 1 ) decision = 1 ;

    // IF LEFT GO LEFT
    else if( ( flag_l == 1 ) && ( flag_ol == 0 ) ) decision = 3 ;
    
    // IF DEAD END TURN AROUND
    else if( ( flag_de == 1 ) && ( flag_r == 0 ) ) decision = 4 ;

    // IF FINISH STOP
    else if( flag_f == 1 ) hault = 1 ;

    // ELSE LINE FOLLOW
    else decision = 1 ;







/*
    // CHECK FOR SERIAL
    while ( Serial.available() == 0 )
    {
      // DO NOTHING
      hault = 1 ;   // <-- IT WILL BE INTERESTING TO SEE HOW THIS IS HANDLED, SPECIFIACALLY HOW IT IS SET BACK TO ZERO.
    } 
    
    // GRAB INCOMING CHARACTERS
    decision = Serial.parseInt() ;    // <-- COULD PUT A WHILE LOOP HERE TO PREVENT PROGRESS RIGHT HERE, BUT THAT IS NOT AS THUROUGH AS THECKING IF IT IS A VALID MOVE DESPITE BEING A VALID NUMBER.


            
    // BURN UP FLOATING CHARACTERS
    Serial.read() ;
    Serial.read() ;
    Serial.read() ;
    Serial.read() ;

*/

    //Serial.print( "  Decision:" ) ;   
    ////Serial.println( decision ) ; 


} // <--- decision_check()





















// THIS FUNCTION IS THE TWINKY SPEED COMMMITY, THIS IS WHERE IT ALL GETS ARGUED WHAT SPEED THE TWINKY SHOULD BE. POSIBLY WILL ABSTRACT AS A SEPERATE FUNCTION ONCE THERE IS TIME TO DO SO.


// ______ PID LF CONTROL () ______

void pid_lf_control()
{


        // CALCULATE LINE ERROR RAW VALUE
        line_error_calc() ;

        // COMPUTE PID LF OUTPUT
        line_lf_PID_calc() ;



        // IF NOT IN ANY SPECIAL SITUAION. JUST NORMAL LINE FOLLOW.
        if( out_detect == 0 && hault == 0 )
        {
            // SUBTRACT TWINKY SPEED FOR RIGHT WHEEL M2
            if( line_lf_PID_out >= 0 ) twinky_two_speed = twinky_speed_const - line_lf_PID_out ;
    
            // SUBTRACT TWINKY SPEEED FOR LEFT WHEEL M1
            else twinky_one_speed = twinky_speed_const - ( line_lf_PID_out * -1 ) ;  
        }


        
        // ELSE IF IN OUT DETECT. MOVE RAW FORWARD.
        else if( out_detect == 1 )
        {
            // DO NOTHING IN ORDER TO MATCH THE CURRENT TWINK SPEED? WHAT ABOUT CORNER CASES?

            twinky_one_speed = twinky_speed_const / 3.00 ;
            twinky_two_speed = twinky_speed_const / 3.00 ;
                    
        }


        
/*
//Serial.print( "   twy_two_spd " ) ;
////Serial.println( twinky_two_speed ) ;

*/
        
/*
        // OVERRIDE FOR SPECIAL CASE OF DEAD_END TURN AROUND
        if( d_mv_latch == 1 ) twinky_two_speed = twinky_max ;
        if( d_mv_latch == 1 ) twinky_one_speed = twinky_max ;
        if( rt_move_latch == 1 ) twinky_one_speed = twinky_one_speed_const ;
        if( rt_move_latch == 1 ) twinky_two_speed = twinky_two_speed_const ;
*/        
          


} // <-- pid_lf_control()
















// ______ MOVE MANUAL () ______

void move_manual( int left_amount, int right_amount )
{


bool left_go_fwd = 0 ;
bool right_go_fwd = 0 ;

    // RESET
    man_mv_comp = 0 ;
    left_comp = 0 ;
    right_comp = 0 ;


    // ADAPT COMMAND AMOUNTS
    left_amount = left_amount + enc1.read() ;
    right_amount = right_amount + enc2.read() * -1 ;



    // _____ DEPRESURIZE _____
    
    // TOE-HEEL TWINKYS
    twinky_one = enc1.read() ;
    twinky_two = enc2.read() * -1 ;

    // CHILL OUT PID's
    whl_1_vl_PID_P = 0 ;
    whl_1_vl_PID_I = 0 ;
    whl_1_vl_PID_D = 0 ;
    whl_1_vl_PID_out = 0 ;
    
    whl_2_vl_PID_P = 0 ;
    whl_2_vl_PID_I = 0 ;
    whl_2_vl_PID_D = 0 ;
    whl_2_vl_PID_out = 0 ;

    


if( enc1.read() <= left_amount ) left_go_fwd = 1 ;
else left_go_fwd = 0 ;

if( ( enc2.read() * -1 ) <= right_amount ) right_go_fwd = 1 ;
else right_go_fwd = 0 ;






//delay(3000) ;



prev_twinky_time = millis() ;

// MOVE UNTIL COMPLETION FLAG
while( man_mv_comp == 0 )
{


    // SAMPLE CADENCE
    if( ( millis() -  prev_twinky_time ) >= 20 )
    {


////Serial.println( " " ) ;
////Serial.println( " " ) ;
////Serial.println( " " ) ;


/*//Serial.print( "tw_1: " ) ;
////Serial.println( twinky_one ) ;*/
//Serial.print( "tw_2: " ) ;
////Serial.println( twinky_two ) ;
/*//Serial.print( "L_amt: " ) ;
////Serial.println( left_amount ) ;
//Serial.print( "enc1: " ) ;
////Serial.println( enc1.read() ) ;
//Serial.print( "L_dif: " ) ;
////Serial.println( left_amount - enc1.read() ) ;*/
//Serial.print( "R_amt: " ) ;
////Serial.println( right_amount ) ;
//Serial.print( "enc2: " ) ;
////Serial.println( enc2.read() * -1 ) ;
//Serial.print( "R_dif: " ) ;
////Serial.println( right_amount - ( enc2.read() * -1 ) ) ;
//Serial.print( "R_fwd: " ) ;
////Serial.println( right_go_fwd ) ;



    
    
        // ____________________ ADVANCE TWINKYS ____________________


        // IF FORWARD MOVE
        if( ( enc1.read() <= left_amount ) && ( left_comp == 0 ) && ( left_go_fwd == 1 ) )
        {
            // UGLY CODE
            //if( ( left_amount - enc1.read() ) < 5 ) left_comp = 1 ;

////Serial.println( "YES IT WAS LESS" ) ;
          
            // IF NOT TO DESTINED AMOUNT THEN KEEP ADVANCING
            if( enc1.read() < left_amount ) twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_speed_const ;  
            //if( enc1.read() >= left_amount ) left_comp = 1 ;
            //else left_comp = 1 ;
        }

        else if( left_go_fwd == 1 ) left_comp = 1 ;
        

        // IF BACKWARD MOVE
        if( ( enc1.read() >= left_amount ) && ( left_comp == 0 ) && ( left_go_fwd == 0 ) )
        {    
            // IF NOT TO DESTINED AMOUNT THEN KEEP ADVANCING
            if( enc1.read() > left_amount ) twinky_one = twinky_one - ( millis() - prev_twinky_time ) * twinky_speed_const ;  
            //if( enc1.read() <= left_amount ) left_comp = 1 ;
            //else right_comp = 1 ;
        }

        else if( left_go_fwd == 0 ) left_comp = 1 ;
      



        // IF FORWARD MOVE
        if( ( ( enc2.read() * -1 ) < right_amount ) && ( right_comp == 0 ) && ( right_go_fwd == 1 ) )
        {  
            // UGLY CODE
            //if( ( right_amount - ( enc2.read() * -1 ) ) < 5 ) right_comp = 1 ;
          
            // TURN WHEEL TWO FORWARD A SPECIFIC AMOUNT
            if( ( enc2.read() * -1 ) < right_amount ) twinky_two = twinky_two + ( millis() - prev_twinky_time ) * twinky_speed_const ;
            //else if( ( enc2.read() * -1 ) >= right_amount ) right_comp = 1 ;
            //else right_comp = 1 ;
        }

        else if( right_go_fwd == 1 ) right_comp = 1 ;
        


        // IF BACKWARD MOVE
        if( ( ( enc2.read() * -1 ) > right_amount ) && ( right_comp == 0 ) && ( right_go_fwd == 0 ) )
        {
            // TURN WHEEL TWO FORWARD A SPECIFIC AMOUNT
            if( ( enc2.read() * -1 ) > right_amount ) twinky_two = twinky_two - ( millis() - prev_twinky_time ) * twinky_speed_const ;
            //if( ( enc2.read() * -1 ) <= right_amount ) right_comp = 1 ;
            //else right_comp = 1 ;       
        }

        else if( right_go_fwd == 0 ) right_comp = 1 ;

        




        // ____________________ CHECK FOR COMPLETION ____________________
    
        // IF BOTH WHEELS ARE COMPLETE MARK AS COMPLETE
        if( ( left_comp == 1 ) && ( right_comp == 1 ) ) man_mv_comp = 1 ;


    
    
    
    
        // ____________________ CALCULATE PID ____________________
    
    
        // GRAB FEEDBACK TICKS
        whl_1_vl_PID_feedback = enc1.read() ;
        whl_2_vl_PID_feedback = enc2.read() * -1 ;
      
        // COMPUTE PID VL OUTPUT
        whl_1_vl_PID_calc() ;
        whl_2_vl_PID_calc() ;
    
    
    
    
    
    
    
        // ____________________ COMMAND MOTORS ____________________
    
      
        // COMMAND MOTORS
        command_motors() ;
    
    

    
    
        // UPDATE TIME
        prev_twinky_time = millis() ;






//Serial.print( "man_mv_comp: " ) ;
////Serial.println( man_mv_comp ) ;    
//Serial.print( "L_comp: " ) ;
////Serial.println( left_comp ) ;
//Serial.print( "R_comp: " ) ;
////Serial.println( right_comp ) ;




    

    } // <-- if()


  
}  // <-- while()




} // <-- move_manual()





















// ______ PID VL CONTROL () ______

void pid_vl_control()
{

    // SUMBIN
    sum_bin_aray() ;


    // IF ON THE LINE
    if( ( sum_bin <= 4 ) && ( ( sum_bin >= 2 ) ) ) 
    {
        twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_one_speed ;  
        twinky_two = twinky_two + ( millis() - prev_twinky_time ) * twinky_two_speed ;
    }

    // ELSE MOVE STRAIT
    else
    {

        // DEBUG
        for( a = 0 ; a <= 100 ; a++ )
        {
            ////Serial.println( " IT WORRKED " ) ;
            ////Serial.println( " IT WORRKED " ) ;
            ////Serial.println( " IT WORRKED " ) ;
        }

      
        twinky_one = twinky_one + ( millis() - prev_twinky_time ) * twinky_speed_const/3.00 ;  
        twinky_two = twinky_two + ( millis() - prev_twinky_time ) * twinky_speed_const/3.00 ;
    }

    
    
    // GRAB FEEDBACK TICKS
    whl_1_vl_PID_feedback = enc1.read() ;
    whl_2_vl_PID_feedback = enc2.read() * -1 ;
  
    // COMPUTE PID VL OUTPUT
    whl_1_vl_PID_calc() ;
    whl_2_vl_PID_calc() ;
  
    // COMMAND MOTORS
    command_motors() ;


} // <-- pid_vl_control()



































// ______ LINE LF PID CALC () ______

void line_lf_PID_calc()
{

    // PROPORTIONAL
    line_lf_PID_P = line_error * line_lf_PID_KP ;

    
    // INTEGRAL
    line_lf_PID_I = line_lf_PID_I + line_error * 40 * line_lf_PID_KI ;
    if( line_lf_PID_I > 255 ) line_lf_PID_I = 255 ;
    if( line_lf_PID_I < -255 ) line_lf_PID_I = -255 ;

    
    // DERIVATIVE
    line_lf_PID_D = ( ( line_error - line_lf_PID_err_prev ) / (float)( millis() - line_lf_PID_D_time_prev ) ) * line_lf_PID_KD ;
    line_lf_PID_err_prev = line_error ;
    line_lf_PID_D_time_prev = millis () ;


    // SUMMATION
    line_lf_PID_out = line_lf_PID_P + line_lf_PID_I + line_lf_PID_D ;
    if( line_lf_PID_out > twinky_speed_const ) line_lf_PID_out = twinky_speed_const ;
    if( line_lf_PID_out < ( twinky_speed_const * -1 ) ) line_lf_PID_out = twinky_speed_const * -1 ;    

    
/*
////Serial.println( " " ) ;
//Serial.print( "   line_error " ) ;
////Serial.println( line_error ) ;
//Serial.print( "   line_lf_PID_KP " ) ;
////Serial.println( line_lf_PID_KP ) ;
//Serial.print( "   lnf_PID_P " ) ;
////Serial.println( line_lf_PID_P ) ;
//Serial.print( "   lnf_PID_I " ) ;
////Serial.println( line_lf_PID_I ) ;
//Serial.print( "   lnf_PID_D " ) ;
////Serial.println( line_lf_PID_D ) ;
//Serial.print( "   lnf_PID_D " ) ;
////Serial.println( line_lf_PID_out ) ;
*/



} // <-- line_lf_PID_calc()
























// ______ WHL 1 VL PID CALC () ______

void whl_1_vl_PID_calc()
{

    // ERROR
    whl_1_vl_PID_err = twinky_one - whl_1_vl_PID_feedback ;


    // PROPORTIONAL
    whl_1_vl_PID_P = whl_1_vl_PID_err * whl_1_vl_PID_KP ;

    
    // INTEGRAL
    whl_1_vl_PID_I = whl_1_vl_PID_I + whl_1_vl_PID_err * 20 * whl_1_vl_PID_KI ;
    if( whl_1_vl_PID_I > 255 ) whl_1_vl_PID_I = 255 ;
    if( whl_1_vl_PID_I < -255 ) whl_1_vl_PID_I = -255 ;

    
    // DERIVATIVE
    whl_1_vl_PID_D = ( ( whl_1_vl_PID_err - whl_1_vl_PID_err_prev ) / (float)( millis() - whl_1_vl_PID_D_time_prev ) ) * whl_1_vl_PID_KD ;
    whl_1_vl_PID_err_prev = whl_1_vl_PID_err ;
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
    whl_2_vl_PID_I = whl_2_vl_PID_I + whl_2_vl_PID_err * 20 * whl_2_vl_PID_KI ;
    if( whl_2_vl_PID_I > 255 ) whl_2_vl_PID_I = 255 ;
    if( whl_2_vl_PID_I < -255 ) whl_2_vl_PID_I = -255 ;

    
    // DERIVATIVE
    whl_2_vl_PID_D = ( ( whl_2_vl_PID_err - whl_2_vl_PID_err_prev ) / (float)( millis() - whl_2_vl_PID_D_time_prev ) ) * whl_2_vl_PID_KD ;
    whl_2_vl_PID_err_prev = whl_2_vl_PID_err ;
    whl_2_vl_PID_D_time_prev = millis () ;


    // SUMMATION
    whl_2_vl_PID_out = whl_2_vl_PID_P + whl_2_vl_PID_I + whl_2_vl_PID_D ;
    if( whl_2_vl_PID_out > 255 ) whl_2_vl_PID_out = 255 ;
    if( whl_2_vl_PID_out < -255 ) whl_2_vl_PID_out = -255 ;    
    

} // <-- whl_2_vl_PID_calc()

































// _________ SERIAL INPUT () _________

void serial_input()
{

int decision_fly = 0 ;
char burner ;

    // CHECK FOR SERIAL
    while (Serial.available() > 0) 
    {
        // GRAB INCOMING CHARACTERS
        decision_fly = Serial.parseInt() ; 
        burner = Serial.read() ; 
        burner = Serial.read() ;
        //burner = Serial.parseInt() ; 



        // DECISION IS TO TURN AROUND
        if( decision_fly == 4 )
        {
            //move_manual( 280 , 280 ) ;
            //hault_motors() ;
            move_manual( 470 , -470 ) ;

            // TOE-HEEL TWINKYS
            twinky_one = enc1.read() ;
            twinky_two = enc2.read() * -1 ;
    
            // CHILL OUT PID's
            whl_1_vl_PID_P = 0 ;
            whl_1_vl_PID_I = 0 ;
            whl_1_vl_PID_D = 0 ;
            whl_1_vl_PID_out = 0 ;
            
            whl_2_vl_PID_P = 0 ;
            whl_2_vl_PID_I = 0 ;
            whl_2_vl_PID_D = 0 ;
            whl_2_vl_PID_out = 0 ;

            line_lf_PID_P = 0 ;
            line_lf_PID_I = 0 ;
            line_lf_PID_D = 0 ;
            line_lf_PID_out = 0 ;
    
    
            prev_twinky_time = millis() ;
            
        }



        // DECISION IS TO HAULT
        else if( decision_fly == 9 )
        {

            // HAULT
            hault == 1 ;

            // TOE-HEEL TWINKYS
            twinky_one = enc1.read() ;
            twinky_two = enc2.read() * -1 ;
    
            // CHILL OUT PID's
            whl_1_vl_PID_P = 0 ;
            whl_1_vl_PID_I = 0 ;
            whl_1_vl_PID_D = 0 ;
            whl_1_vl_PID_out = 0 ;
            
            whl_2_vl_PID_P = 0 ;
            whl_2_vl_PID_I = 0 ;
            whl_2_vl_PID_D = 0 ;
            whl_2_vl_PID_out = 0 ;

            line_lf_PID_P = 0 ;
            line_lf_PID_I = 0 ;
            line_lf_PID_D = 0 ;
            line_lf_PID_out = 0 ;
    
            prev_twinky_time = millis() ;
            
            // HAULT MOTORS HERE  <----------- NEED TO DO THIS BECUASE THE HAULT COMMAND WILL NOT GO THROUGH DURING THE 
            hault_motors();


            // WAIT FOR DECISION
            burner = Serial.read() ;
            burner = Serial.read() ;
            decision_recv() ;

        }


        burner = Serial.read() ; 
        burner = Serial.read() ;




    } // <-- while()

} // <--- serial_input()

























// ______ LIGHT VALUES ADJUST () ______

void light_values_adjust()
{

/*
    // SPACE
    ////Serial.println( " " ) ;
    ////Serial.println( " " ) ;
    ////Serial.println( " " ) ;
    //Serial.print( "Adjustment:    " ) ; 
*/  

    for ( a = 0 ; a < 13 ; a++ ) 
    {

        // MATH REFERENCE
        if( a == 0 ) b = 6 ;
        if( a == 1 ) b = 5 ;
        if( a == 2 ) b = 4 ;
        if( a == 3 ) b = 3 ;
        if( a == 4 ) b = 2 ;
        if( a == 5 ) b = 1 ;
        if( a == 6 ) b = 0 ;
        if( a == 7 ) b = 1 ;
        if( a == 8 ) b = 2 ;
        if( a == 9 ) b = 3 ;
        if( a == 10 ) b = 4 ;
        if( a == 11 ) b = 5 ;
        if( a == 12 ) b = 6 ;


        // CALCULAATE ADJUSTMENT
        //adjustment = ( 1.00 / 50000.00 ) * (float)pow( b , 6 ) + 1.00 ;
        //adjustment = ( 1.00 / 3000.00 ) * (float)pow( b , 4.5 ) + 1.00 ;
        //adjustment = ( 1.00 / 10000.00 ) * (float)pow( b , 5.8 ) + 1.00 ;   // <--- THIS WITH 3200 IS FAILRY GOOD FOR SPEED
        //adjustment = ( 1.00 / 10000.00 ) * (float)pow( b , 6.2 ) + 1.00 ;
        //adjustment = ( 1.00 / 50.00 ) * (float)pow( b , 3 ) + 1.00 ;
        adjustment = ( 1.00 / 3.00 ) * (float)pow( b , 2 ) + 1.00 ;  // <--- THIS WITH 3000 I THINK WAS THE RIGHT COMOBO FOR CONTROL
        //adjustment = ( 1.00 / 0.5 ) * (float)pow( b , 2 ) + 1.00 ;
        //adjustment = ( 1.00 / 2.0 ) * (float)pow( b , 2 ) + 1.00 ;





        
/*
        //Serial.print( "Snsr " ) ; 
        //Serial.print( a ) ; 
        //Serial.print(": ");
*/
        
/*        // PRINT
        //Serial.print( adjustment , 4 ) ; 
        //Serial.print("\t");
*/
        // MULTIPLY ADJUSTMENT
        light_values[ a ] = light_values[ a ] * adjustment ; 
            
    }




} // <-- light_values_adjust()






































// ______ LINE ERROR CALC () ______

void line_error_calc()
{

    // READ IN LIGHT VALUES
    light_sensor_read() ;


    // ADJUST ARRAY
    light_values_adjust() ;

  
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
        //analogWrite( M2_IN_1 , 0 /*whl_2_vl_PID_out * -1*/ ) ;
        analogWrite( M2_IN_1 , whl_2_vl_PID_out * -1 ) ;
        analogWrite( M2_IN_2 , 0 ) ;  // <-- ONE MUST HARD-CODE FLIP THE MAGNITUDE OF THE PWM INTO POSITIVE BECAUSE THE PWM RANGE IS 0 --> 255 (ALL POSITIVE).
    }


} // <--- command_motors()













// _________ PRINTOUT () _________

void printout()
{


    ////Serial.println() ;
    
/*
  // PRINT LIGHT BAR ARRAY VALUES
  for ( a = 0 ; a < 13 ; a++ ) 
  {
      //Serial.print( light_values_bin[ a ] ) ; 
      //Serial.print("   ") ;    
  }
*/

    //////Serial.println() ;


/*

    // PRINT
    //Serial.print( "    rt_int_detect: " ) ; 
    //Serial.print( rt_initial_detect ) ; 


    // PRINT
    //Serial.print( "    rt_int_lch: " ) ; 
    //Serial.print( rt_initial_latch ) ; 


    // PRINT
    //Serial.print( "    rt_cpn: " ) ; 
    //Serial.print( rt_enc_cupon ) ; 


    // PRINT
    //Serial.print( "    rt_fn_detect: " ) ; 
    //Serial.print( rt_final_detect ) ; 


    // PRINT
    //Serial.print( "    go_for_st: " ) ; 
    //Serial.print( go_for_st ) ; 


    // PRINT
    //Serial.print( "    go_for_rt: " ) ; 
    //Serial.print( go_for_rt ) ; 


    // PRINT
    //Serial.print( "    rt_mv_latch: " ) ; 
    //Serial.print( rt_move_latch ) ; 


    // PRINT
    //Serial.print( "    rt_wh1_fwd_cpt: " ) ; 
    //Serial.print( rt_mv_fwd_one_cmplt ) ; 
    
    
    // PRINT
    //Serial.print( "    rt_wh2_fwd_cpt: " ) ; 
    //Serial.print( rt_mv_fwd_two_cmplt ) ; 


    // PRINT
    //Serial.print( "    rt_wh1_rot_cpt: " ) ; 
    //Serial.print( rt_mv_rot_one_cmplt ) ; 
    
    
    // PRINT
    //Serial.print( "    rt_wh2_rot_cpt: " ) ; 
    //Serial.print( rt_mv_rot_two_cmplt ) ; 


    // PRINT
    //Serial.print( "    Enc1: " ) ; 
    //Serial.print( enc1.read() ) ; 


    // PRINT
    //Serial.print( "    Enc2: " ) ; 
    //Serial.print( enc2.read() * -1 ) ; 


    // PRINT
    //Serial.print( "    t_one: " ) ; 
    //Serial.print( twinky_one ) ; 


    // PRINT
    //Serial.print( "    t_two: " ) ; 
    //Serial.print( twinky_two ) ; 

    
    // PRINT
    //Serial.print( "     whl_1_PID_out: " ) ; 
    //Serial.print( whl_1_vl_PID_out, 2 ) ; 


    // PRINT
    //Serial.print( "     whl_2_PID_out: " ) ; 
    //Serial.print( whl_2_vl_PID_out, 2 ) ; 


    // PRINT
    //Serial.print( "    whl2_math: " ) ; 
    //Serial.print( ( enc2.read() * -1 ) - rt_enc_two_final_detect_update ) ; 
*/

/*

    // PRINT
    //Serial.print( "    rt_whl_one_compt: " ) ; 
    //Serial.print( rt_wheel_one_mv_complt ) ; 


    // PRINT
    //Serial.print( "    rt_fin_pos_gate: " ) ; 
    //Serial.print( rt_position_final_gate ) ; 


    // PRINT
    //Serial.print( "    rt_enc_one_int: " ) ; 
    //Serial.print( rt_enc_one_initial ) ; 


    // PRINT
    //Serial.print( "    rt_enc_two_int: " ) ; 
    //Serial.print( rt_enc_two_initial ) ; 


    // PRINT
    //Serial.print( "    rt_enc_one_fin: " ) ; 
    //Serial.print( rt_enc_one_final_detect ) ; 


    // PRINT
    //Serial.print( "    rt_enc_two_fin: " ) ; 
    //Serial.print( rt_enc_two_final_detect ) ; 
*/
    








/*

    // PRINT
    //Serial.print( "    d_detected: " ) ; 
    //Serial.print( d_detected ) ; 


    // PRINT
    //Serial.print( "    d_mv_latch: " ) ; 
    //Serial.print( d_mv_latch ) ; 


    // PRINT
    //Serial.print( "    d_enc_cupon: " ) ; 
    //Serial.print( d_enc_cupon ) ; 


    // PRINT
    //Serial.print( "    d_mv_cmplt: " ) ; 
    //Serial.print( d_mv_cmplt ) ; 


    // PRINT
    //Serial.print( "    d_mv_fwd_one_cmplt: " ) ; 
    //Serial.print( d_mv_fwd_one_cmplt ) ; 


    // PRINT
    //Serial.print( "    d_mv_fwd_two_cmplt: " ) ; 
    //Serial.print( d_mv_fwd_two_cmplt ) ; 


    // PRINT
    //Serial.print( "    d_mv_rot_one_cmplt: " ) ; 
    //Serial.print( d_mv_rot_one_cmplt ) ; 


    // PRINT
    //Serial.print( "    d_mv_rot_two_cmplt: " ) ; 
    //Serial.print( d_mv_rot_two_cmplt ) ; 
    

    // PRINT
    //Serial.print( "    enc1 - d_prev: " ) ; 
    //Serial.print( enc1.read() - d_enc_one_prev ) ; 


    // PRINT
    //Serial.print( "    enc2 - d_prev: " ) ; 
    //Serial.print( enc2.read() - d_enc_two_prev ) ; 


    // PRINT
    //Serial.print( "    t_one_math: " ) ; 
    //Serial.print( twinky_one - ( twinky_one_d + 450 ) ) ; 


    // PRINT
    //Serial.print( "    t_two_math: " ) ; 
    //Serial.print( twinky_two - ( twinky_two_d - 450 ) ) ; 


    // PRINT
    //Serial.print( "    t_one: " ) ; 
    //Serial.print( twinky_one ) ; 


    // PRINT
    //Serial.print( "    t_two: " ) ; 
    //Serial.print( twinky_two ) ; 

*/


/*

    // PRINT
    //Serial.print( "    one_spd: " ) ; 
    //Serial.print( twinky_one_speed ) ; 


    // PRINT
    //Serial.print( "    two_spd: " ) ; 
    //Serial.print( twinky_two_speed ) ; 


*/










/*

    // PRINT
    ////Serial.print( " tick_count: " ) ; 
    ////Serial.print( enc1.read() ) ; 

   
    // PRINT
    ////Serial.print( "       whl_1_vl_set: " ) ; 
    ////Serial.print( whl_1_vl_setpoint, 3 ) ; 

  
    // PRINT
    ////Serial.print( "       whl_1_vl_in: " ) ; 
    ////Serial.print( whl_1_vl_feedback, 3 ) ; 


    // PRINT
    ////Serial.print( "       whl_2_vl_in: " ) ; 
    ////Serial.print( whl_2_vl_feedback, 3 ) ;



*/








/*
    // PRINT
    //Serial.print( "       whl_1_vl_Kp: " ) ; 
    //Serial.print( whl_1_vl_Kp , 4 ) ; 


    // PRINT
    //Serial.print( "       whl_1_vl_Ki: " ) ; 
    //Serial.print( whl_1_vl_Ki , 4 ) ; 


    // PRINT
    //Serial.print( "       whl_1_vl_Kd: " ) ; 
    //Serial.print( whl_1_vl_Kd , 8 ) ; 
*/


// ____________ LF __________________

/*



    // PRINT
    //Serial.print( "err_line: " ) ; 
    //Serial.print( line_error ) ; 
    //Serial.print("\t") ;


    
    // PRINT
    //Serial.print( "whl_1_PID_out: " ) ;    
    //Serial.print( whl_1_vl_PID_out, 2 ) ; 
    //Serial.print("\t") ; 


    // PRINT
    //Serial.print( "whl_2_PID_out: " ) ;    
    //Serial.print( whl_2_vl_PID_out, 2 ) ; 
    //Serial.print("\t") ; 


    // PRINT
    //Serial.print( "err_PID_v1: " ) ; 
    //Serial.print( whl_1_vl_PID_err, 2 ) ; 
    //Serial.print("\t") ;


    // PRINT
    //Serial.print( "err_PID_v2: " ) ; 
    //Serial.print( whl_2_vl_PID_err, 2 ) ; 
    //Serial.print("\t") ;



    // PRINT
    //Serial.print( "twy_one: " ) ; 
    //Serial.print( twinky_one ) ;
    //Serial.print("\t") ; 


    // PRINT
    //Serial.print( "t_spd_c: " ) ; 
    //Serial.print( twinky_speed_const ) ; 
    //Serial.print("\t") ;

    
    // PRINT
    //Serial.print( "twy_one_spd: " ) ; 
    //Serial.print( twinky_one_speed ) ; 
    //Serial.print("\t") ;


    // PRINT
    //Serial.print( "twy_two: " ) ; 
    //Serial.print( twinky_two ) ; 
    //Serial.print("\t") ;


    // PRINT
    //Serial.print( "t_spd_c: " ) ; 
    //Serial.print( twinky_speed_const ) ; 
    //Serial.print("\t") ;

    
    // PRINT
    //Serial.print( "twy_two_spd: " ) ; 
    //Serial.print( twinky_two_speed ) ; 
    //Serial.print("\t") ;



    // PRINT
    //Serial.print( "ln_PID_out: " ) ; 
    //Serial.print( line_lf_PID_out ) ; 
    //Serial.print("\t") ;
*/




/*    
    // PRINT
    //Serial.print( "     err_v2: " ) ; 
    //Serial.print( whl_2_vl_setpoint - whl_2_vl_feedback, 1 ) ; 
*/

/*
    // PRINT
    //Serial.print( "     whl_1_PID_out: " ) ; 
    //Serial.print( whl_1_vl_PID_out, 2 ) ; 



    // PRINT
    //Serial.print( "     whl_2_PID_out: " ) ; 
    //Serial.print( whl_2_vl_PID_out, 2 ) ; 



    // PRINT
    //Serial.print( "     line_lf_PID_out: " ) ; 
    //Serial.print( line_lf_PID_out, 2 ) ; 





    // PRINT
    //Serial.print( "     line_lf_PID_P: " ) ; 
    //Serial.print( line_lf_PID_P, 2 ) ; 


    // PRINT
    //Serial.print( "     line_lf_PID_I: " ) ; 
    //Serial.print( line_lf_PID_I, 2 ) ; 


    // PRINT
    //Serial.print( "     line_lf_PID_D: " ) ; 
    //Serial.print( line_lf_PID_D, 2 ) ; 


    // PRINT
    //Serial.print( "     line_lf_KP: " ) ; 
    //Serial.print( line_lf_PID_KP , 4 ) ; 


    // PRINT
    //Serial.print( "     line_lf_KI: " ) ; 
    //Serial.print( line_lf_PID_KI , 4 ) ; 


    // PRINT
    //Serial.print( "     line_lf_KD: " ) ; 
    //Serial.print( line_lf_PID_KD , 8 ) ; 

*/




/*
    // PRINT
    //Serial.print( "     whl_1_vl_PID_P: " ) ; 
    //Serial.print( whl_1_vl_PID_P, 2 ) ; 


    // PRINT
    //Serial.print( "     whl_1_vl_PID_I: " ) ; 
    //Serial.print( whl_1_vl_PID_I, 2 ) ; 


    // PRINT
    //Serial.print( "     whl_1_vl_PID_D: " ) ; 
    //Serial.print( whl_1_vl_PID_D, 2 ) ; 


    // PRINT
    //Serial.print( "     whl_1_vl_KP: " ) ; 
    //Serial.print( whl_1_vl_PID_KP , 4 ) ; 


    // PRINT
    //Serial.print( "     whl_1_vl_KI: " ) ; 
    //Serial.print( whl_1_vl_PID_KI , 4 ) ; 


    // PRINT
    //Serial.print( "     whl_1_vl_KD: " ) ; 
    //Serial.print( whl_1_vl_PID_KD , 8 ) ; 

*/



/*
    // PRINT
    //Serial.print( "     whl_2_out: " ) ; 
    //Serial.print( whl_2_vl_output, 1 ) ; 
*/
/*
    // PRINT
    //Serial.print( "     ln_err: " ) ; 
    //Serial.print( line_error ) ; 


    // PRINT
    //Serial.print( "     ln_err_out: " ) ; 
    //Serial.print( line_err_output, 4 ) ; 
*/
    
/*
    // PRINT
    //Serial.print( "     ln_err_out_test: " ) ; 
    //Serial.print( line_error * line_err_Kp_test , 4 ) ; 
*/   


} // <--- printout()
