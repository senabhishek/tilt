/* 
 
 T/LT Copyright 2014
 
 Alarm detection code 
 */




/* Play Melody
 * -----------
 *
 * Program to play melodies stored in an array, it requires to know
 * about timing issues and about how to play tones.
 *
 * The calculation of the tones is made following the mathematical
 * operation:
 *
 *       timeHigh = 1/(2 * toneFrequency) = period / 2
 *
 * where the different tones are described as in the table:
 *
 * note         frequency       period  PW (timeHigh)  
 * c            261 Hz          3830    1915    
 * d            294 Hz          3400    1700    
 * e            329 Hz          3038    1519    
 * f            349 Hz          2864    1432    
 * g            392 Hz          2550    1275    
 * a            440 Hz          2272    1136    
 * b            493 Hz          2028    1014   
 * C            523 Hz          1912    956
 *
 * (cleft) 2005 D. Cuartielles for K3
 */



/*Includes*/
#include "alarm.h" 



/*Global variables */

int alarm_signal=1;


//light configurations
int light_front=13;
int light_right=12;
int light_left=11;
int light_back=10;

//sound configuration
int speakerOut = 9; //piezo speaker               
int speakerOut2 = 8; // loud speaker
byte names[] = {
  'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C'};  
int tones[] = {
  1915, 1700, 1519, 1432, 1275, 1136, 1014, 956};
byte melody[] = "2d2a1f2c2d2a2d2c2f2d2a2c2d2a1f2c2d2a2a2g2p8p8p8p";
// count length: 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
//                                10                  20                  30
int count = 0;
int count2 = 0;
int count3 = 0;
int MAX_COUNT = 24;
int statePin = LOW;



/*==============================================================================
 * SETUP()
 *============================================================================*/
void setup() {
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(light_front, OUTPUT);
  pinMode(light_left, OUTPUT);
  pinMode(light_right, OUTPUT);
  pinMode(light_back, OUTPUT);  
  pinMode(speakerOut2, OUTPUT);  
}


void sound_activation (void);
void sound_activation2 (boolean state);
void light_activation (boolean state);
void alarm_handler (boolean alarm_state);



/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop() 
{
  //check if alarm is signaled 
//    alarm_handler();  //alarm on state
   
   if(alarm_signal==1)
      alarm_handler(true);  //alarm on state
   else 
     alarm_handler(false); //alarm off state
   
   
  //sound_activation(); 
  //light_activation();  




}

void alarm_handler (boolean alarm_state){

 static int timer_count=0;  
 static boolean toggle=alarm_state;
  
  if(alarm_state==true){
     light_activation(toggle);
     toggle=!toggle;
     timer_count ++;
  }
  else{ 
     light_activation(false);
  
}
sound_activation2(alarm_state);
     //test: just make alarm go off
      if(timer_count>=50){
      alarm_signal=0;
      }  
  

}



/*function that handles the alarm trigger */
/*void alarm_handler (void){
  
 static int timer_count=0;  
 static boolean toggle=true;
 
 //detect alarm 

  sound_activation2(true); // turn on lights
  light_activation(true); // keep toggling lights
  
  //keep toggling lights while alarm is on and keep buzzer on
  if(alarm_signal==1){
     sound_activation2(true);
     light_activation(!toggle);
     timer_count ++;
     Serial.println(timer_count); 
     Serial.println(alarm_signal);
     Serial.println("Alarm testing ");
      
     //test: just make alarm go off
      if(timer_count>=5){
      alarm_signal=0;
      }  
}
  else
    //alarm is off, take off buzzer    
    sound_activation2(false);
    light_activation(false);


  
  //signal interrupt 


}
*/

void light_activation (boolean state ){
    digitalWrite(light_front, state);   // set the LED to state
  digitalWrite(light_right, state);   // set the LED to state
  digitalWrite(light_left, state);   // set the LED to state
  digitalWrite(light_back, state);   // set the LED to state
  delay(100);              // wait a lil while. 1000 is one second
}



void sound_activation2 (boolean state){
  digitalWrite(speakerOut2,state);
  //delay(1000);
}







void sound_activation (void){
  // int time_cnt=0 ;   
  analogWrite(speakerOut, 0);    
  for (count = 0; count < MAX_COUNT; count++) {
    statePin = !statePin;

    //digitalWrite(ledPin, statePin);

    for (count3 = 0; count3 <= (melody[count*2] - 48) * 30; count3++) {
      for (count2=0;count2<8;count2++) {
        if (names[count2] == melody[count*2 + 1]) {      
          analogWrite(speakerOut,2000);
          delayMicroseconds(tones[count2]);
          analogWrite(speakerOut, 0);
          delayMicroseconds(tones[count2]);
        }
        if (melody[count*2 + 1] == 'p') {
          // make a pause of a certain size
          analogWrite(speakerOut, 0);
          delayMicroseconds(500);
        }
      }
    }
  }





}










