#include "definitions.hpp"


//################################
// Replace with your network credentials

WiFiClient client; // extern   
String rec_Message = ""; // extern
char holder; // extern                                   // FOR WIFI
bool client_Flag = false; // extern
const char* ssid = "ARRIS-93FA"; // extern
const char* password = "BSY89A602856"; // extern
const uint16_t port = 8000; // extern
const char* host = "192.168.0.14"; // extern
//################################



//################################
Adafruit_MCP3008 adc1; // extern
Adafruit_MCP3008 adc2; // extern                        //FOR LIGHT BAR
const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;
int* adc1_buf = (int*) malloc(sizeof(int)*8); // extern  or could do "new int[8];"
int adc2_buf[8]; // extern 
int* adc_buf = (int*) malloc(sizeof(int)*12); //extern
//################################



//################################
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;                       // FOR ENCODER
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;
long enc1_value = 0; // extern
long enc2_value = 0; // extern
//################################


//################################
const unsigned int M1_IN_1 = 13; // extern
const unsigned int M1_IN_2 = 12; // extern
const unsigned int M2_IN_1 = 25; // extern
const unsigned int M2_IN_2 = 14; // extern            

const unsigned int M1_IN_1_CHANNEL = 1; // extern
const unsigned int M1_IN_2_CHANNEL = 2; // extern
const unsigned int M2_IN_1_CHANNEL = 3; // extern                   // FOR MOTORS
const unsigned int M2_IN_2_CHANNEL = 4; // extern

const unsigned int M1_I_SENSE = 35; // extern
const unsigned int M2_I_SENSE = 34; // extern

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;  // we can write PWM wave with max of 2^8 - 1 = 255

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120; // extern
unsigned int M1_PWM_VALUE = 0;  // extern
unsigned int M2_PWM_VALUE = 0;  // extern
//################################




//#################################
unsigned long prev_twinky_time = 0; // extern
float twinky_one = 0; // extern
float twinky_two = 0; // extern
float twinky_one_speed = 0.3; // extern
float twinky_two_speed = twinky_one_speed; // extern                                  

float whl1_vl_PID_error = 0; // extern  
float whl2_vl_PID_error = 0; // extern 

float whl1_vl_PID_P = 0; // extern 
float whl2_vl_PID_P = 0; // extern 

float whl1_vl_PID_I = 0; // extern                    //For Motors PID Control Loop
float whl2_vl_PID_I = 0; // extern 

float whl1_vl_PID_D = 0; // extern 
float whl2_vl_PID_D = 0; // extern 

float whl1_vl_PID_KP = .25; // extern 2
float whl2_vl_PID_KP = .25; // extern 2

float whl1_vl_PID_KI = 0.0; // extern 0.0026
float whl2_vl_PID_KI = 0.0; // extern 

float whl1_vl_PID_KD = 0; // extern 46
float whl2_vl_PID_KD = 0; // extern 

float whl1_vl_PID_error_prev = 0; // extern 
float whl2_vl_PID_error_prev = 0; // extern 

unsigned long whl1_vl_PID_D_time_prev = 0; // extern 
unsigned long whl2_vl_PID_D_time_prev = 0; // extern 

float whl1_vl_PID_out = 0; // extern 
float whl2_vl_PID_out = 0; // extern 

unsigned long current_time = 0; // extern 

bool foward_Flag = true;
//#################################



//#################################
unsigned long prev_line_follow_time = 0; // extern 
unsigned int LightBar_Left_Sum = 0; // extern 
unsigned int LightBar_Right_Sum = 0; // extern 
int line_PID_error = 0; // extern 
//float line_follow_PID_KP = 00.0005; // extern 
float line_follow_PID_KP = twinky_one_speed/(2500.00);
float line_follow_PID_KI = 0.0; // extern 
float line_follow_PID_KD = 0; // extern 
float line_follow_PID_P = 0; // extern 
float line_follow_PID_I = 0; // extern                                           //FOR LINE FOLLOW PID LOOP 
float line_follow_PID_D = 0; // extern 
int line_PID_error_prev = 0; // extern 
float line_follow_PID_out = 0;
float twinky_max = twinky_one_speed;
float twinky_min = twinky_one_speed * -1;
//#################################















//##################################################################################################
void setup() {
  pinMode(14,OUTPUT);
  digitalWrite(14,LOW);
  delay(100);

  Serial.begin(115200);

  /*
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  */
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS); 
 

  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);
  
  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);
}










//##################################################################################################

void loop(){
  delay(2000);
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  //twinky_one_speed *= -1; //to reverse direction
  //twinky_two_speed *= -1;


  prev_twinky_time = millis();
  prev_line_follow_time = millis();
  while(true){
    
    //send_and_recieve_message_to_client();
    //Serial.println("scope check and FINAL -->> " + rec_Message);
    //if(rec_Message == "Right\n"){
    //  Serial.println("HEEEEEEEELLLLLO");
    //}

    //read_Light_bar();
    //Serial.println("SCOPE Check ");
    //Serial.print(adc1_buf[0]);
    //Serial.println(adc2_buf[0]);
    //delay(1000);

    //enc1_value = enc1.read();
    //enc2_value = enc2.read();
    //Encoder_Print();
    //-------------------------------------------------------------------
    //-------------above is testing--------------------------------------
    //-------------------------------------------------------------------
    if(Serial.available()){
              // GRAB INCOMING CHARACTERS
        char incomingCharacter = Serial.read();

       // PICK ACTION ACORDING TO CHAR 
       switch (incomingCharacter) 
       {

          // INCREASE Kp
          case '1':
              whl1_vl_PID_KP = whl1_vl_PID_KP + 0.05 ;
          break;

          // DECREASE Kp
          case '2':
              whl1_vl_PID_KP = whl1_vl_PID_KP - 0.05 ;
          break;

          // INCREASE Ki
          case '3':
              whl1_vl_PID_KI = whl1_vl_PID_KI + 0.00005 ;
          break;


          // DECREASE Ki
          case '4':
              whl1_vl_PID_KI = whl1_vl_PID_KI - 0.00005 ;
          break;

          // INCREASE Kd
          case '5':
              whl1_vl_PID_KD = whl1_vl_PID_KD + 2 ;
          break;


          // DECREASE Kp
          case '6':
              whl1_vl_PID_KD = whl1_vl_PID_KD - 2 ;
          break;

          
        } // <-- switch ()
    }







    current_time = millis();

    
    //Motor control PID loop
    if((current_time - prev_twinky_time) > 20){
      enc2_value = enc2.read()*-1; // should be -1.
      enc1_value = enc1.read(); // This should be in pid_v1_control() but since enc1 and enc2 cannot be extern I have to read() here.
      

      pid_v1_control();
      //Encoder_Print();

      prev_twinky_time = current_time;
    }
    


    /*
    //Line follow PID loop
    if((current_time - prev_line_follow_time) > 40){ // we desire to keep the middle three under 500, 

      pid_lf_control();
      prev_line_follow_time = current_time;
    }
    */



    

    /*
    //Now add logic to halt, back up with PID control, and send/recieve message, then switch case and do operation.


    */




    //delay(6000);
    //client_Flag = true;
  }
}