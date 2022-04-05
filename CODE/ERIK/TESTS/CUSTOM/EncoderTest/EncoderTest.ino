
// THIS CODE SHOWCASES THE ENCODER FUNCTIONALITY 


// _______ LIBRAYS _______

#include <Encoder.h>





// _______ VARIABLES ______

const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;




// _______ OBJECTS _______

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);





// _______ SETUP _______

void setup() 
{ 
  
    // BEGIN SERIAL
    Serial.begin(9600);

    // WAIT UNTIL SERIAL IS ESTABLISHED
    while (!Serial) 
    {
        delay(1);
    }

    
    //analogWrite(2, 255);
    //analogWrite(3, 0);
    //analogWrite(4, 255);
    //analogWrite(5, 0);
}





// _______ MAIN _______

void loop() 
{
  
  Serial.print(enc1.read());
  Serial.print("\t");
  Serial.print(enc2.read());
  Serial.println();
  delay(100);


  
  // TRY TO MISS A TIK
  for ( unsigned long a = 0 ; a < 1000 ; a++ ) 
  {
      a = a + a ;   
  }
  
  
} // <-- main()
