boolean cross = 0;
boolean go = 1;
boolean firstType = 1;
int measurements[] = {0, 0, 0, 0, 0};
#include <Servo.h>                           // Use the Servo library (included with Arduino IDE) 
#include <SoftwareSerial.h>
const int TxPin = 8;
SoftwareSerial mySerial = SoftwareSerial(255, TxPin);
boolean printed = 0;
boolean line = 0;


int initialRead = 0;
Servo servoL;                                // Define the left and right servos
Servo servoR; 

int t = 20;
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>


#include <SoftwareSerial.h>
//#define Rx 2 // DOUT to pin 10
//#define Tx 3 // DIN to pin 11
SoftwareSerial Xbee (10, 11);



/* Assign a unique ID to this sensor at the same currLineCount */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
int avgVal;
int count = 0;
long sum;
boolean found = false;
int currLineCount = 0;
int star;
int deathstarLocation;


// Perform these steps with the Arduino is first powered on
void setup()
{
  Serial.begin(9600); // Set to No line ending;
  Xbee.begin(9600); // type a char, then hit enter
  delay(500);
  pinMode(TxPin, OUTPUT);
  digitalWrite(TxPin, HIGH);

  Serial.begin(9600);                        // Set up Arduino Serial Monitor at 9600 baud
  mySerial.begin(9600);
  delay(100);
  servoL.attach(13);                         // Attach (programmatically connect) servos to pins on Arduino
  servoR.attach(12);

  mySerial.write(12);                 // Clear
  mySerial.write(17);                 // Turn backlight on
  delay(5);                           // Required delay
  mySerial.print("Squad 3 is GO!");  // First line
  mySerial.write(18);                 // Turn backlight off

  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  // displaySensorDetails();
}

void wait(){
  if(deathstarLocation!=1){
    int supposedNum = deathstarLocation+10;
    while(true){
      servoL.writeMicroseconds(1500);
      servoR.writeMicroseconds(1500);
      delay(40);  
      if(Xbee.available()){
        Serial.println("We're trying");
        int received = Xbee.read();
        Serial.print(received);
        int printout = received - 48;
        if(printout == supposedNum ){
          break; // return    
        }
      }
      else{
        Serial.print("not avaliable");
      }
    }  
  }
}



String pinCheck(long farleft, long midleft, long midright, long farright) {
  int pin4;
  int pin5;
  int pin6;
  int pin7;

  if (farleft > t) {
    pin4 = 1;
  }
  else {
    pin4 = 0;
  }
  if (midleft > t) {
    pin5 = 1;
  }
  else {
    pin5 = 0;
  }
  if (midright > t) {
    pin6 = 1;
  }
  else {
    pin6 = 0;
  }
  if (farright > t) {
    pin7 = 1;
  }
  else {
    pin7 = 0;
  }
  return String(pin4) + String(pin5) + String(pin6) + String(pin7);
}

long RCcurrLineCount(int sensorIn) {
  long duration = 0;
  pinMode(sensorIn, OUTPUT);
  digitalWrite(sensorIn, HIGH);
  delay(1);
  pinMode(sensorIn, INPUT);
  digitalWrite(sensorIn, LOW);
  while (digitalRead(sensorIn)) {
    duration++;
  }
  return duration;
}

boolean five = false;
void loop()
{
  if (go == 1) {
    if(deathstarLocation == 5 && five==false){
      servoL.detach();
      servoR.detach();
   servoL.attach(13);                         // Attach (programmatically connect) servos to pins on Arduino
  servoR.attach(12);
      servoL.writeMicroseconds(1600);
    servoR.writeMicroseconds(1400);
    delay(900);
    servoL.writeMicroseconds(1600);
    servoR.writeMicroseconds(1600);
    delay(750);
       servoL.writeMicroseconds(1600);      
    servoR.writeMicroseconds(1400);
     delay(300);
     firstType=0;
     five = true;
     Serial.println("here");
    }
     else if (firstType == 1) {
      movement();
    }
    else {
     // wait();
      moveTwo();
    }
  }
  else {
    servoL.writeMicroseconds(1500);      // Steer robot to recenter it over the line
    servoR.writeMicroseconds(1500);
  }
}

void moveTwo() {

  int farleft = RCcurrLineCount(7); // replace this with the pin you attatched the WHITE wire of the LEFT qti sensor too
  int midleft = RCcurrLineCount(6); // replace this with the pin you attatched the WHITE wire of the RIGHT qti sensor too
  int midright = RCcurrLineCount(5);
  int farright = RCcurrLineCount(4);
  String pins = pinCheck(farleft, midleft, midright, farright);

  // Display result of D4-D7 pins in Serial Monitor

  // Determine how to steer based on state of the four QTI sensors

  int vL, vR;

  //Serial.println(found);


  if (pins == "1000") {
    vL = -100;                             // -100 to 100 indicate course correction values
    vR = 100;
    line = 0;
  }
  else if ( pins == "1100") {
    vL = 0;
    vR = 100;
    line = 0;
  }
  else if (pins == "0100") {
    vL = 50;
    vR = 100;
    line = 0;
  }
  else if (pins == "0110") {
    vL = 100;
    vR = 100;
    line = 0;
  }
  else if (pins == "0010") {
    vL = 100;
    vR = 50;
    line = 0;
  }
  else if (pins == "0011") {
    vL = 100;
    vR = 0;
    line = 0;
  }
  else if (pins == "0001") {
    vL = 100;
    vR = -100;
    line = 0;
  }
  else if (pins == "1111" && line == 0 ) {
    Serial.println("here");
    vL = 100;
    vR = 100;
    line = 1;
    servoL.writeMicroseconds(1500);      // Steer robot to recenter it over the line
    servoR.writeMicroseconds(1500);
    currLineCount++;
    delay(500);
    if (currLineCount == 6 - deathstarLocation) {
      for(int highfive = 0; highfive<40 ; highfive++){
            servoL.writeMicroseconds(1500);
            servoR.writeMicroseconds(1500);
            delay(40);
            char sentback = deathstarLocation + 59; // 48+10+1
            Xbee.print(sentback);
        }
        servoL.detach();
        servoR.detach();
        while(true){
          
        mySerial.print(deathstarLocation);
          
        }
      go = 0;
    }
  }
  else if (pins == "1111" && line == 1) {
    vL = 100;
    vR = 100;
  }

  else if (pins == "0000") {
    vL = 0;
    vR = 0;
  }


  servoL.writeMicroseconds(1500 + vL);      // Steer robot to recenter it over the line
  servoR.writeMicroseconds(1500 - vR);


  delay(40);

}

void movement() {
  int farleft = RCcurrLineCount(7); // replace this with the pin you attatched the WHITE wire of the LEFT qti sensor too
  int midleft = RCcurrLineCount(6); // replace this with the pin you attatched the WHITE wire of the RIGHT qti sensor too
  int midright = RCcurrLineCount(5);
  int farright = RCcurrLineCount(4);
  String pins = pinCheck(farleft, midleft, midright, farright);

  // Display result of D4-D7 pins in Serial Monitor

  // Determine how to steer based on state of the four QTI sensors

  int vL, vR;

  //Serial.println(found);


  if (pins == "1000") {
    vL = -100;                             // -100 to 100 indicate course correction values
    vR = 100;
    line = 0;
  }
  else if ( pins == "1100") {
    vL = 0;
    vR = 100;
    line = 0;
  }
  else if (pins == "0100") {
    vL = 50;
    vR = 100;
    line = 0;
  }
  else if (pins == "0110") {
    vL = 100;
    vR = 100;
    line = 0;
  }
  else if (pins == "0010") {
    vL = 100;
    vR = 50;
    line = 0;
  }
  else if (pins == "0011") {
    vL = 100;
    vR = 0;
    line = 0;
  }
  else if (pins == "0001") {
    vL = 100;
    vR = -100;
    line = 0;
  }
  else if (pins == "1111" && line == 0 ) {
    vL = 100;
    vR = 100;
    line = 1;
    servoL.writeMicroseconds(1500);      // Steer robot to recenter it over the line
    servoR.writeMicroseconds(1500);
    currLineCount++;
    if (found == 0 && currLineCount <= 5) {
      magnet();
    }
  }
  else if (pins == "1111" && line == 1) {
    vL = 100;
    vR = 100;
  }

  else if ( pins == "0000" && found == 1 && cross == 0) {
   // wait();

    servoL.writeMicroseconds(1600);
    servoR.writeMicroseconds(1400);
    delay(900);
    servoL.writeMicroseconds(1600);
    servoR.writeMicroseconds(1600);
    delay(750);
    cross = 1;
    servoL.writeMicroseconds(1600);
    servoR.writeMicroseconds(1400);
    delay(100);
    currLineCount = 0;
    firstType = 0;
    line = 1;
    return;
  }

  else if (pins == "0000" && cross == 1) {
    vL = 0;
    vR = 0;
  }


  servoL.writeMicroseconds(1500 + vL);      // Steer robot to recenter it over the line
  servoR.writeMicroseconds(1500 - vR);


  delay(40);
}
int hasPrinted = 0;



void magnet() {

  for (int iter = 0; iter < 20; iter++) {
    sensors_event_t event;
    mag.getEvent(&event);
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = 0.22;
    heading += declinationAngle;
    // Correct for when signs are reversed.
    if (heading < 0) {
      heading += 2 * PI;
    }
    // Check for wrap due to addition of declination.
    if (heading > 2 * PI) {
      heading -= 2 * PI;
    }
    // Convert radians to degrees for readability.
    float magValue = heading * 180 / M_PI;


    if (measurements[currLineCount] == 0) {
      measurements[currLineCount] = magValue;
    }
    else {
int curr = measurements[currLineCount]; http: //qklnk.co/Z90qFm
      int avg = (curr + magValue) / 2;
      measurements[currLineCount] = avg;
    }
  
    delay(50);
  }



  if (currLineCount == 1 && measurements[currLineCount] < 240) {
    mySerial.write(12);                 // Clear
    mySerial.write(17);                 // Turn backlight on
    delay(5);                           // Required delay
    mySerial.print("Star at 1");  // First line
    deathstarLocation = 1;
    found = 1;
  }


  else if (measurements[currLineCount] < ((-1 * measurements[currLineCount - 1]*.06) + measurements[currLineCount - 1])) {
    mySerial.write(12);                 // Clear
    mySerial.write(17);                 // Turn backlight on
    delay(5);                           // Required delay
    mySerial.print(currLineCount);  // First line
    deathstarLocation = currLineCount;
    found = 1;

  }

  else {
    mySerial.write(12);                 // Clear
    mySerial.write(17);                 // Turn backlight on
    delay(5);                           // Required delay
    mySerial.print("not here");  // First line
    mySerial.write(18);

  }


}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(200);
}

void printDisp() {

}


