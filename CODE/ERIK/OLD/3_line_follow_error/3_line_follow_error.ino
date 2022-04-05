// READS IN VLAUES INTO SINGLE ARRAY AND FLIPS THEM SO THEY APEAR LEFT TO RIGHT IN THE SERIAL MONITOR (ANATOMICALY)


// ______ LIBRARYS______

#include <Adafruit_MCP3008.h>





// ______ DEFINITIONS ______

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;





// ______ GLOBAL VARIABLES ______


short a = 0 ;
float error = 0 ;
uint16_t light_bar_values[ 16 ] ;
float light_bar_values_normalized[ 13 ] ;

const short ADC_1_CS = A3 ;
const short ADC_2_CS = A2 ;
const short RF_CS = A4 ;





// ______ FUNCTION DECLARATION ______

void light_bar_sensor() ;
void error_calculator() ;
void print_statistics() ;



// ______ SETUP ______

void setup() 
{
  
  Serial.begin(9600);

  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH); // Without this the nRF24 will write to the SPI bus while the ADC's are also talking

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  

}







// ______ MAIN ______   <------- M A I N ------<

void loop() 
{

  // CHECK LIGHT BAR SENSOR
  light_bar_sensor() ;

  // CALCULATE ERROR
  error_clalculator() ;


  // PRINT STATISTICS 
  print_statistics() ;
    

  delay(25);
  
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
  //light_bar_values[ 2 ] = adc1_buf[ 6 ] ;
  //light_bar_values[ 1 ] = adc1_buf[ 7 ] ;
  //light_bar_values[ 0 ] = adc1_buf[ 7 ] ;




} // <-- light_bar_sensor()









// ______ ERROR CALCULATOR () ______

void error_clalculator()
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
  error = light_bar_values_normalized_rightside_sum - light_bar_values_normalized_leftside_sum ;




} // <-- error_clalculator()











// ______ PRINT STATISTICS () ______

void print_statistics()
{

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


  // PRINT ERROR
  Serial.print( error , 4 ) ; 


  // NEW LINE
  Serial.println() ;


} // <-- print_statistics()
