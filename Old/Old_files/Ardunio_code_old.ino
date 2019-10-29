
//OBS OBS this code is obsolete with current system
//Arduino now runs StandardFirmata file from examples library in IDE



#include <Servo.h>
#define INPUT_SIZE 100
char inputString[INPUT_SIZE];
bool stringComplete = false;
int Speeds[5] = {90,90,90,90,90};
int i = 0;
int j = 0;
char* separator;

Servo hoved_motor;
Servo styrbord_motor;
Servo babord_motor;
Servo styrbord_servo;
Servo babord_servo;


void setup() {
  // initialize serial:
  Serial.begin(9600);
  // setter motorer og servo til sin respektive pin
  hoved_motor.attach(11);
  styrbord_motor.attach(10);
  babord_motor.attach(9);
  styrbord_servo.attach(6);
  babord_servo.attach(5);
}

void loop() {
  if (stringComplete) {      
      separator = strtok(inputString, ":;");
      
     for(int i = 0; i < (sizeof(Speeds) / sizeof(Speeds[0])) ; i++) {
        
        Speeds[i] = atoi(separator);
        separator = strtok(NULL, ":;");  
    }      
    //setter fart til motorer og servoer
    hoved_motor.write(Speeds[0]);
    styrbord_motor.write(Speeds[1]);
    babord_motor.write(Speeds[2]);
    styrbord_servo.write(Speeds[3]);
    babord_servo.write(Speeds[4]);
    //
    stringComplete = false;
    i = 0;
  }
}

void serialEvent() { //alt riktig, returnerer det den får
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    inputString[j] = inChar;
    j++;
    if (inChar == ';') {
      stringComplete = true;
      j = 0;
    }
  }
}
