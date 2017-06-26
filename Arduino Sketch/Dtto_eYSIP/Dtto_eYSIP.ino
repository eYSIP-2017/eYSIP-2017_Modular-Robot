/*
 * Authors: Srijal Poojari, Madhav Wagh
 * Description: Our program created for the Dtto modular robot based off 
 * the original work by Alberto(https://hackaday.io/project/9976-dtto-explorer-modular-robot)
 * This standalone program includes code for both master and slave modules.
 * 
 * Function List: setup(), loop(), get_module_num(), assign_module_num(),
 * listen_bluetooth(), listen_RF(), hook_attach(), snake(), set_angle(), escape(),
 * attach_servos(), detach_servos(), variation().
 */
 
#include <SoftwareSerial.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Adafruit_VL53L0X.h"

/*********************** Change before Upload *****************************/
#define DTTO_TYPE 0   //0 for MASTER module; 1 for the rest of SLAVE modules
int reverse = 1;      //used to negate angles in sinusoidal motion
#define HAS_SENSOR 0  //whether vl53l0x sensor is added or not
/**************************************************************************/

#if HAS_SENSOR
Adafruit_VL53L0X lox = Adafruit_VL53L0X();    //define lox object for VL53l0x
#endif

#define CE_PIN 2   //RF24 Chip Enable pin
#define CSN_PIN 4  //RF24 Chip Select Not pin

int SERVO_CENTER_M = 77;    //Male hinge servo center position, change as per your servo
int SERVO_CENTER_F = 77;     //Female hinge servo center position, change as per your servo

#define SERVO_CENTER_HOOK 90  //Servo hook positions, change as per your servo
#define SERVO_MIN_HOOK 0
#define SERVO_MAX_HOOK 150

//variables for sinusoidal control
float speed_factor = 0.002; //Speed factor
int amp_factor = 30;      //Amplitude factor (basically max angle from center position to either sides)
unsigned long t = 0;      //Time auxiliary variables for time varying sine wave
unsigned long t_start = 0;


byte reset = 1;   //used to indicate starting point(in time) for sine wave

//bluetooth definitions
SoftwareSerial bluetooth(7, 8); //Creating software serial for bluetooth module (RX,TX)
String bt_rx_data;    //buffer for bluetooth received data

//RF24 definitions
RF24 radio(CE_PIN, CSN_PIN); //radio object for RF24 class
const uint64_t pipes[2] = { 0xE8E8F0F0E1LL, 0xF0F0F0F0D2LL };  //2 pipe addresses defined.
                                                               //1 for MASTER->SLAVES, 2 for SLAVES->MASTER
char rf_tx_data[8];   //buffers for transmit and receive data
char rf_rx_data[8];

String final_rx_data; //Final Received Data buffer which holds the command received, common for master as well as slave
                      //This is used to decide various actions performed in the main loop

int my_num = 1;   //Id of the module
int total_module_num = 1;    //Total number of modules

//Create servo objects
Servo servo_male;
Servo servo_female;
Servo servo_base;
Servo servo_right;
Servo servo_left;
//Note: 'Left' and 'Right' is defined with the Arduino USB port facing upward
//and male module is forward when viewed from the top as the reference.

//Angle variables used to set servo angles. Default values are defined below.
int servo_angle_male = 77;
int servo_angle_female = 77;
int servo_angle_base = 90;
int servo_angle_right = 90;
int servo_angle_left = 90;

bool stop_flag = false;    //Flag raised by sensor detection to indicate obstacle

void setup() {
  Serial.begin(9600);
  delay(1000);

  radio.begin();

#if DTTO_TYPE                           //If SLAVE,
  radio.openWritingPipe(pipes[1]);    //define channels
  radio.openReadingPipe(1, pipes[0]);
  get_module_num();                   //get id, stored in my_num

#else                                   //If MASTER.
  bluetooth.begin(9600);
  bluetooth.println("Reset");

  radio.openWritingPipe(pipes[0]);    //define channels
  radio.openReadingPipe(1, pipes[1]);
  radio.startListening();             //and start listening
#endif

#if HAS_SENSOR
  if (!lox.begin()) {
    bluetooth.println(F("Failed to boot VL53L0X"));
  }
#endif
}

void loop() {
#if DTTO_TYPE       //If slave,
  listen_RF();    //listen for commands from master module via radio
#else                       //If master,
  assign_module_num();    //check if new modules asking for id and assign number
  listen_bluetooth();     //listen for commands
#endif

  switch (final_rx_data[1]) {    //Various blocks run based on commands in final_rx_data
  bluetooth.println(final_rx_data);
  case 'r': //Run, sinusoidal control
  #if HAS_SENSOR
    if (!stop_flag) {
      snake();
    } else {
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      int distance = 9999;
      if (measure.RangeStatus != 4) { // phase failures have incorrect data
        distance = measure.RangeMilliMeter - 30;
        //bluetooth.print("Distance (mm): ");
        //bluetooth.println(distance);
        if (distance < 90) {
          stop_flag = true;
          //stop_flag_final=1;
          detach_servos();

          radio.stopListening();
          radio.write("ae", 2);
          radio.startListening();
        }

        else {
          stop_flag = false;
          //stop_flag_final=0;
          attach_servos();
          radio.stopListening();
          radio.write("arm", 3);
          radio.startListening();
        }

      } else {
        stop_flag = false;
        //stop_flag_final=0;
        attach_servos();
        //bluetooth.println(" out of range ");
        radio.stopListening();
        radio.write("arm", 3);
        radio.startListening();

      }
      //delay(100);
    }
  #else
    snake();
  #endif

    break;
  case 'e':  //End all movement
    escape();
    break;
  case 's':  //Set angle manually
    set_angle();
    break;
  case 'h':  //attach module(hook)
    hook_attach();
    break;
  default:
    break;
  }
}

void get_module_num() {    //slave modules get module number from master module
  int pass = 1234;   //pass to confirm request for id number
  detach_servos();   //detach servos to avoid power fluctuations which can affect communication

  while (my_num == 1) {  //while id not received from master, i.e. my_num is still 1
    radio.stopListening();   //stop listening before writing request
    radio.write(&pass, sizeof(int));
    radio.startListening();  //listen for response

    //Wait 500ms for a response. RF24 communication usually takes place within microseconds.
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while (!radio.available() && !timeout) {
      //If no response and timeout disabled for >500ms, timeout occurred
      if (millis() - started_waiting_at > 500)
        timeout = true;
    }

    if (!timeout) {  //If no timeout, which means data received in RF24 buffer
      radio.read(&my_num, sizeof(int));   //read id number
    }
  }
}

void assign_module_num() {   //master assigns id number to slave modules
  if (radio.available()) { //If some data received,
    int got_pass;
    radio.read(&got_pass, sizeof(int));
    if (got_pass == 1234) {  //If received data is 1234, module is asking for number.
      total_module_num++;  //total connected modules counter
      bluetooth.println("Module total: " + String(total_module_num));

      radio.stopListening();    //stop listening and send id to slave module
      radio.write(&total_module_num, sizeof(int));
      radio.startListening();

    }
  }
}

void listen_bluetooth() {    //listens commands from user via bluetooth

  if (bluetooth.available() > 0) {
    detach_servos();    //detach servos to avoid power fluctuations which can affect communication
    bt_rx_data = bluetooth.readStringUntil('\0');    //read data from buffer as a string
    bluetooth.println("Received: " + bt_rx_data);

    if (bt_rx_data[0] == '1') {         //first character '1' means command is just for master
      final_rx_data = bt_rx_data;
    } else if (bt_rx_data[0] == 'a') {  //'a' means command for master as well as all slaves
      final_rx_data = bt_rx_data;

      bt_rx_data.toCharArray(rf_tx_data, 8);   //convert received bluetooth command(String) to
                                         //char array for rf transmission
      radio.stopListening();
      radio.write(rf_tx_data, 8);
      radio.startListening();
    } else {                            //command just for slave modules
      bt_rx_data.toCharArray(rf_tx_data, 8);    //convert received bluetooth command(String) to
                                                      //char array for rf transmission
      radio.stopListening();
      radio.write(rf_tx_data, 8);
      radio.startListening();
    }
  }
}

void listen_RF() {
  if (radio.available()) {
    detach_servos();    //detach servos to avoid power fluctuations which can affect communication
    while (radio.available()) {
      //fetch the data payload
      radio.read(rf_rx_data, 8);
    }

    if ((rf_rx_data[0] == (my_num + 48)) || (rf_rx_data[0] == 'a')) {
      //If command is for me particularly or for all modules.
      //(my_num + 48) converts int to ASCII equivalent for char comparison

      final_rx_data = String(rf_rx_data);
    }
  }
}

void hook_attach() {
  switch (final_rx_data[2]) {
  case 'b':    //For base,
    servo_base.attach(10);
    if (final_rx_data[3] == '0')         //center position, usually enough for detaching
      servo_angle_base = SERVO_CENTER_HOOK;
    else if (final_rx_data[3] == '1')    //attach
      servo_angle_base = SERVO_MIN_HOOK;
    else if (final_rx_data[3] == '2')    //maximum detach
      servo_angle_base = SERVO_MAX_HOOK;

    servo_base.write(servo_angle_base);   // write the changed angle
    break;
  case 'r':    //For right sided servo,
    servo_right.attach(3);
    if (final_rx_data[3] == '0')
      servo_angle_right = SERVO_CENTER_HOOK;
    else if (final_rx_data[3] == '1')
      servo_angle_right = SERVO_MIN_HOOK;
    else if (final_rx_data[3] == '2')
      servo_angle_right = SERVO_MAX_HOOK;

    servo_right.write(servo_angle_right);
    break;
  case 'l':    //For left sided servo,
    servo_left.attach(9);
    if (final_rx_data[3] == '0')
      servo_angle_left = SERVO_CENTER_HOOK;
    else if (final_rx_data[3] == '1')
      servo_angle_left = SERVO_MIN_HOOK;
    else if (final_rx_data[3] == '2')
      servo_angle_left = SERVO_MAX_HOOK;

    servo_left.write(servo_angle_left);
    break;
  default:
    break;
  }
  delay(150);
  detach_servos();
}

void snake() {
  attach_servos();
  if (reset) {    //If first run of loop,
    t_start = millis();
  }
  t = millis() - t_start;
  reset = 0;
  
  float male_joint_angle;
  float female_joint_angle;
  float angle_variation;    //difference between male and female angle values.
  
  switch (final_rx_data[2]) {
    
  case 'm':    //motion with male module facing front
     
    /* male_angle = amp_factor * sin(theta + phase + pi/2) = amp_factor * cos(theta + phase)
     * female_angle = amp_factor * sin(theta + phase)
     * 
     * cos and sin is interchanged for reverse motion. This can be directly done using the 'reverse' variable(int).
     * 
     * theta = time * speed_factor
     * phase = 180*module_nume ,i.e. it negates the sign of cos and sin for every alternate module,
     * since sin(a + pi) = -sin(a) and cos(a + pi) = -cos(a)
     * 
     * amplitude factor scales the sin and cos values to degrees.
     */
     
    male_joint_angle = amp_factor * sin((speed_factor * t) + (my_num * PI)
                        + (1 - reverse) * PI / 2);
    female_joint_angle = amp_factor * sin((speed_factor * t) + (my_num * PI)
                        + (reverse * PI / 2));

    servo_angle_male = SERVO_CENTER_M + male_joint_angle;    //Apply the offsets from center to get servo angle
    servo_angle_female = SERVO_CENTER_F + female_joint_angle;

#if HAS_SENSOR
    angle_variation = variation(male_joint_angle, female_joint_angle);
    //bluetooth.println(angle_variation);

    VL53L0X_RangingMeasurementData_t measure;

    if (servo_angle_male > SERVO_CENTER_M
        && servo_angle_female > (SERVO_CENTER_F - 2)
        && servo_angle_female < (SERVO_CENTER_F + 2)) {
      // bluetooth.println(servo_angle_male);
      // bluetooth.println(servo_angle_female);

      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      int distance = 9999;

      if (measure.RangeStatus != 4) { // phase failures have incorrect data
        distance = measure.RangeMilliMeter - 30;
        // bluetooth.print("Distance (mm): ");
        // bluetooth.println(distance);

        if (distance < 50) {
          stop_flag = true;
          //  bluetooth.println("Stop Flag=1");

          servo_angle_male = SERVO_CENTER_M;
          servo_angle_female = SERVO_CENTER_F;

          if (servo_male.read() < servo_angle_male) {
            for (int i = servo_male.read(); i <= servo_angle_male;
                i++) {
              // bluetooth.println(i);
              servo_male.write(i);
              delay(30);
            }
          } else {
            for (int i = servo_male.read(); i >= servo_angle_male;
                i--) {
              // bluetooth.println(i);
              servo_male.write(i);
              delay(30);
            }

          }

          if (servo_female.read() < servo_angle_female) {
            for (int i = servo_female.read();
                i <= servo_angle_female; i++) {
              servo_female.write(i);
              delay(30);
            }
          } else {
            for (int i = servo_female.read();
                i >= servo_angle_female; i--) {
              // bluetooth.println(i);
              servo_female.write(i);
              delay(30);
            }
          }
        }
      } 
    }
#endif

    servo_male.write(servo_angle_male);
    servo_female.write(servo_angle_female);
    break;
    
  case 'f':    //motion with female module facing front
    male_joint_angle = amp_factor
        * sin(
            (speed_factor * t) - (my_num * PI)
                + (1 - reverse) * PI / 2);
    female_joint_angle = amp_factor
        * sin((speed_factor * t) - ((my_num * PI) + reverse * PI / 2));

    servo_angle_male = SERVO_CENTER_M + male_joint_angle;
    servo_angle_female = SERVO_CENTER_F + female_joint_angle;

#if HAS_SENSOR
    angle_variation = variation(male_joint_angle, female_joint_angle);
    //bluetooth.println(angle_variation);

    // VL53L0X_RangingMeasurementData_t measure;

    if (servo_angle_male > SERVO_CENTER_M
        && servo_angle_female > (SERVO_CENTER_F - 2)
        && servo_angle_female < (SERVO_CENTER_F + 2)) {
      // bluetooth.println("posM & posF");
      // bluetooth.println(servo_angle_male);
      // bluetooth.println(servo_angle_female);

      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      int distance = 9999;

      if (measure.RangeStatus != 4) { // phase failures have incorrect data
        distance = measure.RangeMilliMeter - 30;
        // bluetooth.print("Distance (mm): ");
        // bluetooth.println(distance);

        if (distance < 50) {
          stop_flag = true;
          //  bluetooth.println("Stop Flag=1");

          servo_angle_male = SERVO_CENTER_M;
          servo_angle_female = SERVO_CENTER_F;

          if (servo_male.read() < servo_angle_male) {
            for (int i = servo_male.read(); i <= servo_angle_male;
                i++) {
              // bluetooth.println(i);
              servo_male.write(i);
              delay(30);
            }
          } else {
            for (int i = servo_male.read(); i >= servo_angle_male;
                i--) {
              // bluetooth.println(i);
              servo_male.write(i);
              delay(30);
            }

          }

          if (servo_female.read() < servo_angle_female) {
            for (int i = servo_female.read();
                i <= servo_angle_female; i++) {
              servo_female.write(i);
              delay(30);
            }
          } else {
            for (int i = servo_female.read();
                i >= servo_angle_female; i--) {
              // bluetooth.println(i);
              servo_female.write(i);
              delay(30);
            }
          }
        }
      } 
    }
#endif

    servo_male.write(servo_angle_male);
    servo_female.write(servo_angle_female);
    break;
  default:
    break;
  }
  //detachServos();
}

void set_angle() {
  bluetooth.println(final_rx_data);

  int angle = final_rx_data.substring(3, 6).toInt();
  bluetooth.print("Angle: " + String(angle) + "\n\r");
  attach_servos();
  if ((angle <= 180) && (angle >= 0)) {
    switch (final_rx_data[2]) {
    case 'm':
      servo_male.write(angle);
      break;
    case 'f':
      servo_female.write(angle);
      break;
    default:
      break;
    }
  }
  final_rx_data = "";
}
void escape() {
  t = 0;
  detach_servos();
  reset = 0;
}

void attach_servos() {
  servo_male.attach(6);
  servo_female.attach(5);
  servo_base.attach(10);
  servo_right.attach(3);
  servo_left.attach(9);
}

void detach_servos() {
  servo_male.detach();
  servo_female.detach();
  servo_base.detach();
  servo_right.detach();
  servo_left.detach();
}

float variation(float a, float b) { // absolute difference between 2 values.
  float a_abs, b_abs, d_ab;
  a_abs = abs(a);
  b_abs = abs(b);

  d_ab = abs(a_abs - b_abs);
  return d_ab;
}
