// NORMALIZE READ IN VALUES



// ______ LIBRARYS______

#include <Adafruit_MCP3008.h>





// ______ DEFINITIONS ______

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;





// ______ GLOBAL VARIABLES ______

const short ADC_1_CS = A3 ;
const short ADC_2_CS = A2 ;
const short RF_CS = A4 ;





// ______ FUNCTION DECLARATION ______

void light_bar_sensor() ;





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
  

  delay(25);
  
} // <-- main()










// ____________________ FUNCTION LAND ____________________
// _______________________________________________________



// _________ LIGHT BAR SENSOR () _________


void light_bar_sensor()
{

  // VARIABLES 

  short a = 0 ;
  //short b = 0 ;
  //int adc1_read_sum = 0 ;
  //int adc2_read_sum = 0 ;
  uint16_t adc1_buf[8] ;
  uint16_t adc2_buf[8] ;
  uint16_t light_bar_values[ 16 ] ;

  //int t_start = micros();
  
/*
  // READ IN LIGHT BAR
  for( a = 0 ; a < 8 ; a++ )
  {    
      for ( b = 0 ; b < 8 ; b++ ) 
      {
     */   
          /*
          // READ INTO ONE ARRAY
          if ( i <= 4 )  adc1_buf[i] = adc1.readADC(i) ;
          else adc2_buf[i] = adc2.readADC(i) ;
          */
/*    
          adc1_buf[ a ][ b ] = adc1.readADC( b ) ;
          adc2_buf[ a ][ b ] = adc2.readADC( b ) ;
    
          
      } // <-- for(a)
      
  } // <-- for(b)

*/



/*
      for ( a = 0 ; a < 8 ; a++ ) 
      {

          adc1_read_sum = 0 ;
          adc2_read_sum = 0 ;

          for ( b = 0 ; b < 1 ; b++ )
          {
          
            adc1_read_sum = adc1_read_sum + adc1.readADC( a ) ;  
            adc2_read_sum = adc2_read_sum + adc2.readADC( a ) ;   
                 
          }

          adc1_buf[ a ] = adc1_read_sum / 1 ;
          adc2_buf[ a ] = adc2_read_sum / 1 ;
    
          
      } // <-- for(a)

*/

      for ( a = 0 ; a < 8 ; a++ ) 
      {

          adc1_buf[ a ] = adc1.readADC( a ) ;
          adc2_buf[ a ] = adc2.readADC( a ) ;
    
          
      } // <-- for(a)






      light_bar_values[ 0 ] = adc1_buf[ 0 ] ;
      light_bar_values[ 1 ] = adc2_buf[ 0 ] ;
      light_bar_values[ 2 ] = adc1_buf[ 1 ] ;
      light_bar_values[ 3 ] = adc2_buf[ 1 ] ;
      light_bar_values[ 4 ] = adc1_buf[ 2 ] ;
      light_bar_values[ 5 ] = adc2_buf[ 2 ] ;
      light_bar_values[ 6 ] = adc1_buf[ 3 ] ;
      light_bar_values[ 7 ] = adc2_buf[ 3 ] ;
      light_bar_values[ 8 ] = adc1_buf[ 4 ] ;
      light_bar_values[ 9 ] = adc2_buf[ 4 ] ;
      light_bar_values[ 10 ] = adc1_buf[ 5 ] ;
      light_bar_values[ 11 ] = adc2_buf[ 5 ] ;
      light_bar_values[ 12 ] = adc1_buf[ 6 ] ;
      light_bar_values[ 13 ] = adc1_buf[ 6 ] ;
      light_bar_values[ 14 ] = adc1_buf[ 7 ] ;
      light_bar_values[ 15 ] = adc1_buf[ 7 ] ;
      



/*


  // COMBINE INTO SINGLE ARRAY
  for ( a = 0 ; a < 16 ; a = a + 2 ) 
  {

      light_bar_values[ a ] = adc1_buf[ a ] ;
      light_bar_values[ a + 1 ] = adc2_buf[ a ] ;



      
  }
*/


  // PRINT LIGHT BAR ARRAY VALUES
  for ( a = 0 ; a < 16 ; a++ ) 
  {

      Serial.print( light_bar_values[ a ] ) ; 
      Serial.print("\t");
      //Serial.print(adc2_buf[a]); Serial.print("\t");
      
  }


/*


  //int t_end = micros();
  
  for (int i = 0; i < 8; i++) 
  {
      Serial.print(adc1_buf[i]); Serial.print("\t");
      Serial.print(adc2_buf[i]); Serial.print("\t");
  }

*/
/*
  //Serial.print(t_end - t_start);
  Serial.println();
  */

} // <-- light_bar_sensor()
