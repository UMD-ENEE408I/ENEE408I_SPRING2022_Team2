
// _______ LIBRAYS _______

#include <Encoder.h>





// _______ ENC VARIABLES ______

const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;


// _______ OBJECTS _______

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

int Map[20][20] = {};



// _______ POS VARIABLES _______

int position_x = 10;
int position_y = 10;


int current1 = enc1.read();
int current2 = enc2.read();
int orientation = 1;

int unit = 1000;

int difference = 0;


void setup() {
  
    Serial.begin(9600);

    // WAIT UNTIL SERIAL IS ESTABLISHED
    while (!Serial) 
    {
        delay(1);
    }

    Map[position_y][position_x] = 1;
    
}

void loop() {

  // Straight
  
  if (enc1.read() > unit + current1 && enc2.read() < current2 - unit){
    current1 = enc1.read();
    current2 = enc2.read();
    Map[position_y][position_x] = 2;
    if (orientation == 1){
     position_y--; 
     Map[position_y][position_x] = 1;
    }
    else if (orientation == 2){
     position_x++; 
     Map[position_y][position_x] = 1;
    }
    else if (orientation == 4){
     position_x--;
     Map[position_y][position_x] = 1;
    }
    else if (orientation == 3){
      position_y++;
      Map[position_y][position_x] = 1;
    }
    
    
  }

  difference = enc1.read() - abs(enc2.read());

  // Right
  
  if (difference > 250 && difference < 500){
    orientation = 2;
  }

  else if (difference > 700 && difference < 950){
    orientation = 3;
  }

  else if (difference > 1050 && difference < 1300){
    orientation = 4;
  }

  else if (difference > 1400 || (difference < 50 && difference > -50)){
    orientation = 1;
  }


  // Left

  if (difference < -250 && difference > -500){
    orientation = 4;
  }

  else if (difference < -700 && difference > -950){
    orientation = 3;
  }

  else if (difference < -1050 && difference > -1300){
    orientation = 2;
  }

  else if (difference < -1400){
    orientation = 1;
  }


  
  for (int i = 0; i < 20; i++){
    for (int j = 0; j < 20; j++){
      Serial.print(Map[i][j]);
    }
    Serial.print('\n');
  }

  Serial.print(enc1.read());
  Serial.print("\t");
  Serial.print(enc2.read());
  Serial.print("\t");
  Serial.print(current1);
  Serial.print("\t");
  Serial.print(current2);
  Serial.println();
  Serial.println(orientation);
  Serial.println(difference);
  delay(10);

  Serial.print('\n');
  Serial.print('\n');
  Serial.print('\n');
  delay(1000);
}
