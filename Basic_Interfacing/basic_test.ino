/* Program to test basic interfacing of Aruino with HC05 bluetooth modules,
 *  3x servo motors and a Sharp IR sensor.
 *  
 *  Date: 27/05/2017
 */

#include <Servo.h>
#include <SharpIR.h>

#define ir A0
#define model 1080
// ir: the pin where your sensor is attached
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)

SharpIR SharpIR(ir, model);

//Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

int servo_num = 0;
int servo_pos = 0;
int prev_servo_pos = 0;  
int separator_idx = 0;    //Index of separator character ',' in inputString

//Range of servo numbers
int min_servo_num = 1;
int max_servo_num = 2;

//Range of servo values
int min_servo_pos = 0;
int max_servo_pos = 170;

int distance = 0;  //Distance in cm received from SharpIR.distance()

void setup(){
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  //initialize servos
  servo1.attach(7);
  servo2.attach(8);
  servo3.attach(9);
}

void loop(){
  if(stringComplete){ //when a newline char is received 
    //Serial.println(inputString);

    separator_idx = inputString.indexOf(',');
    if(separator_idx < 0){  //indexOf() returns -1 if char not present in string
      Serial.println("Invalid format! Enter as: servo_num,position ");
    }
    else{
       //get servo number and position as interger from inputString
       servo_num = inputString.substring(0, separator_idx).toInt();
       servo_pos = inputString.substring(separator_idx + 1).toInt();

       Serial.print("servo is: ");
       Serial.println(servo_num);
       
       Serial.print("position is: ");
       Serial.println(servo_pos);
       
       if((servo_num <= max_servo_num)and(servo_num >= min_servo_num)){
         //if servo number is in proper range then,
         if((servo_pos <= max_servo_pos)and(servo_pos >= min_servo_pos)){
           //if servo position is also in range then write values.
           if(servo_num == 1)
             servo1.write(servo_pos);
           if(servo_num == 2)
             servo2.write(servo_pos);
           if(servo_num == 3)
             servo3.write(servo_pos);
         }
         else{ //error in servo position
           Serial.print("Invalid servo position! Enter as servo_num,position. Possible values from ");
           Serial.print(min_servo_pos);
           Serial.print(" to ");
           Serial.println(max_servo_pos);
         }
       }
       else{ //error in servo number
         Serial.print("Invalid servo number! Enter as servo_num,position. Possible values from ");
         Serial.print(min_servo_num);
         Serial.print(" to ");
         Serial.println(max_servo_num);
       }
    }
    // clear the string:
       inputString = "";
       stringComplete = false; 
  }

  distance = SharpIR.distance();  //get distance

  //map the distance values ranging (8cm, 50cm) to servo position
  servo_pos = map(distance, 8, 50, min_servo_pos, max_servo_pos);

  if(servo_pos != prev_servo_pos){ //don't write same values again and again.
    
    if((servo_pos <= max_servo_pos)and(servo_pos >= min_servo_pos)){
       //if servo position is in range then write values.
       servo3.write(servo_pos);
     }
     prev_servo_pos = servo_pos;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent(){
  while(Serial.available()){
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if(inChar == '\n'){
      stringComplete = true;
    }
  }
}

