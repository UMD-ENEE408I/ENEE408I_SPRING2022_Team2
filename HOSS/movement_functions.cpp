#include "definitions.hpp"





void Encoder_Print(){
  Serial.print(enc1_value);
  Serial.print("\t");
  Serial.print(enc2_value);
  Serial.println();
}

void read_Light_bar(){

  /*
  //int adc1_buf[8]; // could make these extern, update: I did
  //int adc2_buf[8];

  int t_start = micros();
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
  int t_end = micros();

  for (int i = 0; i < 8; i++) {
    Serial.print(adc1_buf[i]); 
    Serial.print("\t");
    Serial.print(adc2_buf[i]); 
    Serial.print("\t");
  }
  Serial.print(t_end - t_start);
  Serial.println();
  */

  adc_buf[11] = adc1.readADC(0);
  adc_buf[10] = adc2.readADC(0);
  adc_buf[9] = adc1.readADC(1);
  adc_buf[8] = adc2.readADC(1);
  adc_buf[7] = adc1.readADC(2);
  adc_buf[6] = adc2.readADC(2);
  //adc_buf[6] = adc1.readADC(3);
  adc_buf[5] = adc2.readADC(3);
  adc_buf[4] = adc1.readADC(4);
  adc_buf[3] = adc2.readADC(4);
  adc_buf[2] = adc1.readADC(5);
  adc_buf[1] = adc2.readADC(5);
  adc_buf[0] = adc1.readADC(6);

/*//Sanity check
  for(int i = 0; i < 12; i++){
    Serial.print(adc_buf[i]);
    Serial.print('\t');
  }
  Serial.println();
*/

  for(int i = 0; i < 6; i++){
    LightBar_Left_Sum += adc_buf[i];
  }
  for(int i = 6; i < 12; i++){
    LightBar_Right_Sum += adc_buf[i];
  }


  delay(100);
}





void M1_backward() {
  ledcWrite(M1_IN_1_CHANNEL, M1_PWM_VALUE);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}

void M1_forward() {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, M1_PWM_VALUE);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}

void M2_backward() {
  ledcWrite(M2_IN_1_CHANNEL, M2_PWM_VALUE);// this one M2_PWM_VALUE
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M2_forward() {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, M2_PWM_VALUE);// this one M2_PWM_VALUE
}

void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}





void pid_v1_control(){
  //get desired input distance

  twinky_one = twinky_one + (current_time - prev_twinky_time)*twinky_one_speed; // multiply by some constant to keep pushing up the twinky_one distance
  twinky_two = twinky_two + (current_time - prev_twinky_time)*twinky_two_speed; // multiply by some constant to keep pushing up the twinky_one distance 

  // COMPUTE PID VL OUTPUT
  whl_1_2_vl_PID_calculation();
  motor_move();
}


void whl_1_2_vl_PID_calculation(){
  // ERROR for whl1 and whl2
  whl1_vl_PID_error = twinky_one - enc1_value; //this should be the desired - the current_read, twinky_one has been pushed up to a desired position.
  whl2_vl_PID_error = twinky_two - enc2_value;

  // PROPORTIONAL for whl1 and whl2
  whl1_vl_PID_P = whl1_vl_PID_error * whl1_vl_PID_KP; // error times kP constant
  whl2_vl_PID_P = whl2_vl_PID_error * whl2_vl_PID_KP;

  
  // INTEGRAL for whl1 and whl2
  whl1_vl_PID_I +=  whl1_vl_PID_error * whl1_vl_PID_KI * (float)(current_time - whl1_vl_PID_D_time_prev); // I accumulates with the error times the kI constant
  if(whl1_vl_PID_I > 255) whl1_vl_PID_I = 255;               // cam this possibly go over 255?
  if(whl1_vl_PID_I < -255) whl1_vl_PID_I = -255;

  whl2_vl_PID_I += whl2_vl_PID_error * whl2_vl_PID_KI * (float)(current_time - whl2_vl_PID_D_time_prev); // I accumulates with the error times the kI constant
  if(whl2_vl_PID_I > 255) whl2_vl_PID_I = 255;               // cam this possibly go over 255?
  if(whl2_vl_PID_I < -255) whl2_vl_PID_I = -255;


  // DERIVATIVE for whl1 and whl2
  whl1_vl_PID_D = ( (whl1_vl_PID_error - whl1_vl_PID_error_prev)/(float)(current_time - whl1_vl_PID_D_time_prev) ) * whl1_vl_PID_KD;
  whl1_vl_PID_error_prev = whl1_vl_PID_error; // should this be whl1_vl_PID_error? not whl1_vl_PID_D
  whl1_vl_PID_D_time_prev = current_time;

  whl2_vl_PID_D = ( (whl2_vl_PID_error - whl2_vl_PID_error_prev) / (float)(current_time - whl2_vl_PID_D_time_prev) ) * whl2_vl_PID_KD;
  whl2_vl_PID_error_prev = whl2_vl_PID_error;
  whl2_vl_PID_D_time_prev = current_time;

  // SUMMATION for whl1 and whl2
  whl1_vl_PID_out = whl1_vl_PID_P + whl1_vl_PID_I + whl1_vl_PID_D;
  if(whl1_vl_PID_out >= 255) whl1_vl_PID_out = 255;
  if(whl1_vl_PID_out <= -255) whl1_vl_PID_out = -255;

  whl2_vl_PID_out = whl2_vl_PID_P + whl2_vl_PID_I + whl2_vl_PID_D;
  if(whl2_vl_PID_out >= 255) whl2_vl_PID_out = 255;
  if(whl2_vl_PID_out <= -255) whl2_vl_PID_out = -255;    
  /*
  Serial.print(whl2_vl_PID_error);
  Serial.print(" ");
  Serial.print(whl2_vl_PID_P);
  Serial.print(" ");
  Serial.print(whl2_vl_PID_I);
  Serial.print(" ");
  Serial.print(whl2_vl_PID_D);
  Serial.print(" ");
  Serial.print(whl2_vl_PID_out);
  Serial.println();
  */
}


void motor_move(){ 
  M1_PWM_VALUE = static_cast<int>(whl1_vl_PID_out);
  M2_PWM_VALUE = static_cast<int>(whl2_vl_PID_out);

  // IF PID_1 FORWARD    
  if(whl1_vl_PID_out >= 0){
    M1_forward();

  }else{  // IF PID_1 REVERSE
    M1_PWM_VALUE = M1_PWM_VALUE * -1;
    M1_backward(); 
  }

  // IF PID_2 FORWARD    
  if (whl2_vl_PID_out >= 0) {
    M2_forward();

  }else{ // IF PID_2 REVERSE
    M2_PWM_VALUE = M2_PWM_VALUE * -1;
    M2_backward();
  }

}





void pid_lf_control(){
  //find error
  LightBar_Left_Sum = 0;
  LightBar_Right_Sum =  0;
  read_Light_bar();
  //line_follow_PID_KP = 
  //find error, left minus right, want to keep them equal as possible
  line_PID_error = LightBar_Left_Sum - LightBar_Right_Sum;
  Serial.print(line_PID_error);

  //PROPORTIONAL
  line_follow_PID_P = line_PID_error * line_follow_PID_KP;
  //INTEGRAL
  line_follow_PID_I += (float)(line_PID_error) * (float)(current_time - prev_line_follow_time) * line_follow_PID_KI;
  if(line_follow_PID_I > 255) line_follow_PID_I = 255;
  if(line_follow_PID_I < -255) line_follow_PID_I = -255;

  //DERIVATIVE
  line_follow_PID_D = ((float)(line_PID_error - line_PID_error_prev) / (float)(current_time - prev_line_follow_time)) * line_follow_PID_KD;
  line_PID_error_prev = line_PID_error;

  // SUMMATION
  line_follow_PID_out = line_follow_PID_P + line_follow_PID_I + line_follow_PID_D;
  if(line_follow_PID_out > twinky_max) line_follow_PID_out = twinky_max;
  if(line_follow_PID_out < twinky_min) line_follow_PID_out = twinky_min;


  //Now check if line_follow_PID_out, if negative then mouse has gone to the right of white line?, slow down the left motor?
  Serial.print(" ");
  Serial.println(line_follow_PID_out);
  if(line_follow_PID_out >= 0){
    twinky_two_speed = twinky_max - line_follow_PID_out; //SHOW ERIK
  }else{
    twinky_one_speed = twinky_max - (-1 * line_follow_PID_out); //SHOW ERIK
  }

}








































