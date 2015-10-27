/*

Wiring Diagram for QTI Sensors:
Arduino          Sensor
D7               QTI4 - Far left
D6               QTI3 - Mid left
D5               QTI2 - Mid right
D4               QTI1 - Far right

Wiring Diagram for Servos:
Arduino          Servo
D13              Left servo
D12              Right servo

*/

#include <Servo.h>                           // Use the Servo library (included with Arduino IDE)  

Servo servoL;                                // Define the left and right servos
Servo servoR;

int t = 30;

void setup()
{
  Serial.begin(9600);                        // Set up Arduino Serial Monitor at 9600 baud
  servoL.attach(13);                         // Attach (programmatically connect) servos to pins on Arduino
  servoR.attach(12);

}

String pinCheck(long farleft, long midleft, long midright,long farright){
  int pin4;
int pin5;
int pin6;
int pin7;

  if(farleft > t){
  pin4=1;
}
else{
  pin4=0;
}
if(midleft > t){
  pin5=1;
}
else{
  pin5=0;
}
if(midright > t){
  pin6=1;
}
else{
  pin6=0;
}
if(farright > t){
  pin7=1;
}
else{
  pin7=0;
}
return String(pin4) + String(pin5) + String(pin6) + String(pin7);
}

long RCTime(int sensorIn){
   long duration = 0;
   pinMode(sensorIn, OUTPUT);     
   digitalWrite(sensorIn, HIGH);  
   delay(1);                      
   pinMode(sensorIn, INPUT);      
   digitalWrite(sensorIn, LOW);   
   while(digitalRead(sensorIn)){  
      duration++;
   }
   return duration;
}

void loop()
{

int farleft = RCTime(7); 
int midleft = RCTime(6); 
int midright = RCTime(5);
int farright =RCTime(4);
String pins = pinCheck(farleft, midleft,midright,farright);

  int vL, vR;

  Serial.println(pins);

if(pins == "1000"){
  vL = -100;                             // -100 to 100 indicate course correction values
  vR = 100;
}
else if( pins == "1100"){
   vL = 0;
   vR = 100;
}
else if(pins == "0100"){
   vL = 50;
      vR = 100;
}
else if (pins == "0110"){
  vL = 100;
      vR = 100;
}
else if (pins == "0010"){
       vL = 100;
      vR = 50;
}
else if (pins == "0011"){
    vL = 100;
      vR = 0;
}
else if (pins == "0001"){
  vL = 100;
  vR = -100;
}
else if (pins == "1111"){
  vL = 100;
  vR = 100;
}
else if (pins == "0000"){
  vL = 0;
  vR = 0;
}

  servoL.writeMicroseconds(1500 + vL);      // Steer robot to recenter it over the line
  servoR.writeMicroseconds(1500 - vR);
  
  delay(50);                                // Delay for 50 milliseconds (1/20 second)
}
