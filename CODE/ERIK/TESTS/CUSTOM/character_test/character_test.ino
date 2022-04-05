

int decision = 0 ;
char incomingCharacter ;


void setup() 
{
  
  // BEGIN SERIAL
  Serial.begin(9600) ;

}
















void loop() 
{


    // TITLE
    Serial.println( " * * * *   B E G I N   * * * * " ) ;

 
    // PRETEND PRINT
    Serial.println( " " ) ;
    Serial.println( "waiting for input..." ) ;


    // CHECK FOR SERIAL
    while ( Serial.available() == 0 )
    {
      // DO NOTHING
      //hault = 1 ;   // <-- IT WILL BE INTERESTING TO SEE HOW THIS IS HANDLED, SPECIFIACALLY HOW IT IS SET BACK TO ZERO.
    } 

    // PRINT OUT
    Serial.println( " " ) ;
    Serial.println( "parsing in integer..." ) ;


    // BRING IN INTEGER
    decision = Serial.parseInt() ;


    // BURN OFF EXTRA CHARACTERS
    Serial.parseInt() ;


    // PRINT OUT DECISION VLAUE
    Serial.print( "Decision: " ) ;   
    Serial.println( decision ) ; 


    // PRINT OUT
    Serial.print( "Delay...." ) ;

    // DELAY
    delay( 2000 );


    // PRINT OUT
    Serial.println( " " ) ;
    Serial.println( " " ) ;
    Serial.println( " " ) ;
    Serial.println( " " ) ;
    Serial.println( " " ) ;
    Serial.println( " " ) ;
    Serial.println( " " ) ;










/*
    
    if( Serial.available() > 0 )
    {
        // GRAB INCOMING CHARACTERS
        incomingCharacter = Serial.read() ;
        
        Serial.print( "  Msg recieved:" ) ;   
        Serial.println( incomingCharacter ) ; 


        // PICK ACTION ACORDING TO CHAR 
        switch ( incomingCharacter ) 
        {
        
            case '1':
            {
            decision = 1 ; 
            break;
            }
            
            case '2':
            {
            decision = 2 ;
            break;
            }
            
            case '3':
            decision = 3 ;
            break;
            
            
            case '4':
            decision = 4 ;
            break;
            
            
            case '5':
            decision = 5 ;
            break;
            
            
            case '6':
            decision = 6 ;
            break;
            
        
        } // <-- switch ()

*/




/*
        while( ( incomingCharacter != '0' ) || ( incomingCharacter != '1' ) || ( incomingCharacter != '2' )  )
        {

            if( Serial.available() > 0 )
            {
                incomingCharacter = Serial.read() ;          
            }
            
          //Serial.print( "  Nonsense. Try Again:  " ) ; 
           
        }

        if( incomingCharacter == '0' ) decision = 0 ;
        else if( incomingCharacter == '1' ) decision = 1 ;
        else if( incomingCharacter == 2 ) decision = 2 ;
        else if( incomingCharacter == '3' ) decision = 3 ;
        else if( incomingCharacter == '4' ) decision = 4 ;
        else if( incomingCharacter == '5' ) decision = 5 ;
        else if( incomingCharacter == '6' ) decision = 6 ;
*/

    
   // } // <-- if()



} // <-- MAIN
