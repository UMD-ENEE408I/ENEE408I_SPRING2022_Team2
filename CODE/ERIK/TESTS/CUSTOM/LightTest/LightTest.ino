

// ______ LIBRARYS______

#include <Adafruit_MCP3008.h>





// ______ DEFINITIONS ______

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;





// ______ VARIABLES ______

const unsigned int ADC_1_CS = A3;
const unsigned int ADC_2_CS = A2;

const unsigned int RF_CS = A4;





// ______ SETUP ______

void setup() 
{
  
  Serial.begin(9600);

  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH); // Without this the nRF24 will write to the SPI bus while the ADC's are also talking

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  

}





// ______ MAIN ______

void loop() 
{
  
  int adc1_buf[8];
  int adc2_buf[8];

  int t_start = micros();
  
  for (int i = 0; i < 8; i++) 
  {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
  
  int t_end = micros();

  for (int i = 0; i < 8; i++) 
  {
    Serial.print(adc1_buf[i]); Serial.print("\t");
    Serial.print(adc2_buf[i]); Serial.print("\t");
  }

  Serial.print(t_end - t_start);
  Serial.println();

  delay(100);
  
} // <-- main()
