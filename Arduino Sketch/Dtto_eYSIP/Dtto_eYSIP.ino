/*
 * Authors: Srijal Poojari, Madhav Wagh
 * Description: Our program created for the Dtto modular robot for eYSIP 2017.
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

#include <Wire.h>
#include <VL53L0X.h>

/*********************** Change before Upload *****************************/
#define DTTO_TYPE 1   //0 for MASTER module; 1 for the rest of SLAVE modules
int reverse = 1;      //used to negate angles in sinusoidal motion
#define HAS_SENSOR 0  //whether vl53l0x sensor is added or not
#define MODULE_BUILD_NUM 3 //physical id of the module. Used to define servo positions as
                           //each servo has different 'true physical angle' for angles written by servo.write()
                           //this is mainly because of modifications done to increase ranges by adding external resistances
/* example:
 * If angles in code go from: -90 ---  0 --- 90,
 * it actually translates to  130 --- 77 --- 26(example values).
 * i.e. to set 90 degrees on the servo, you actually have to write '26' because
 * of the modifications we did to increase servo range.
 */
/**************************************************************************/

#if HAS_SENSOR
VL53L0X tof_sensor;
#endif

#define CE_PIN 2   //RF24 Chip Enable pin
#define CSN_PIN 4  //RF24 Chip Select Not pin


//------------------------ Servo calibration -----------------------------//
//angles found out for each servo after modifications to increase range
int servo_center_positions_m[] = {74, 77, 80, 77};   //male center, 0 degrees
int servo_min_positions_m[] = {22, 26, 28, 26};      //min postition, 90 degrees
int servo_max_positions_m[] = {128, 133, 137, 133};  //max postition, -90 degrees

int servo_center_positions_f[] = {70, 74, 80, 74};   //male center, 0 degrees
int servo_min_positions_f[] = {20, 28, 28, 22};      //min postition, 90 degrees
int servo_max_positions_f[] = {128, 130, 137, 130};  //max postition, -90 degrees

#define SERVO_CENTER_M  servo_center_positions_m[MODULE_BUILD_NUM]    //Male hinge servo positions
#define SERVO_MIN_M  servo_min_positions_m[MODULE_BUILD_NUM]
#define SERVO_MAX_M  servo_max_positions_m[MODULE_BUILD_NUM]

#define SERVO_CENTER_F  servo_center_positions_f[MODULE_BUILD_NUM]     //Female hinge servo positions
#define SERVO_MIN_F  servo_min_positions_f[MODULE_BUILD_NUM]
#define SERVO_MAX_F  servo_max_positions_f[MODULE_BUILD_NUM]

//Servo hook positions, change as per your servo
#define SERVO_CENTER_HOOK 90  //neutral position
#define SERVO_MIN_HOOK 0      //hooked postion
#define SERVO_MAX_HOOK 150    //detach hook
//-----------------------------------------------------------------------//

//arrays to store mapped angles from software angle: -90---0---90 to the physical angles
//Values are mapped and stored beforehand as mapping it everytime when required is slower.
int mapped_angles_m[181];
int mapped_angles_f[181];

//variables for sinusoidal control
float speed_factor = 0.002; //Speed factor
int amp_factor = 40;      //Amplitude factor (basically max angle from center position to either sides)
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

int wheel_run_count = 0;   //number of times the run_wheel() function is run.
                           //Used to switch betwwen the wheel stages.
                           
char sign;   //Sign of angle retrieved from commands

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
  Wire.begin();

  tof_sensor.init();
  tof_sensor.setTimeout(500);

  // reduce timing budget to 20 ms (default is about 33 ms)
  // greater the budget, higher the accuracy but takes more time.
  tof_sensor.setMeasurementTimingBudget(20000);
#endif

  //map all software angles to true physical angles and store them.
  //this saves the calculation time when software angles are written
  for(int angle = -90; angle <= 0; angle++){   
    mapped_angles_m[angle + 90] = map(angle, -90, 0, SERVO_MAX_M, SERVO_CENTER_M);
    //angles go from -90 to 90, and array index goes from 0 to 180,i.e. angle+90
    mapped_angles_f[angle + 90] = map(angle, -90, 0, SERVO_MAX_F, SERVO_CENTER_F);
  }

  for(int angle = 1; angle <= 90; angle++){
    mapped_angles_m[angle + 90] = map(angle, 0, 90, SERVO_CENTER_M, SERVO_MIN_M);
    mapped_angles_f[angle + 90] = map(angle, 0, 90, SERVO_CENTER_F, SERVO_MIN_F);
  }

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
    if (!stop_flag) {    //If stop_flag not raised
      snake();
    }
    else {    //Else, bot stopped, check if obstacle has been cleared

      int distance = tof_sensor.readRangeSingleMillimeters() - 30;
        //bluetooth.print("Distance (mm): ");
        //bluetooth.println(distance);
        if (distance < 90) {    //IF closer than 90mm, obstacle still present, remain still
          stop_flag = true;
          detach_servos();

          radio.stopListening();
          radio.write("ae", 2);    //Also command all bots to stop
          radio.startListening();
        }
        else {    // obstacle cleared, continue motion.
          stop_flag = false;
          attach_servos();

          radio.stopListening();
          radio.write("arm", 3);   //command other modules to resume motion
          radio.startListening();
        }
      //delay(100);
    }
  #else    //If no sensor, normal motion.
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

  case 'n':   //'NOT' angle. Basically switches angle between 2 states, either 0<->90 or 0<->-90
              //This will be clear once you understand the wheel motion, explained on github.
    sign = final_rx_data[3];
    int angle;

    if (sign == '-'){   //If '-' specified by user, angle is negative
       angle = final_rx_data.substring(4, 6).toInt();    //get angle parameter and convert to int.
       angle = -1 * angle;
    }
    else{
       angle = final_rx_data.substring(3, 5).toInt();    //get angle parameter and convert to int.
    }

    //angle = 90, not_male(90) switches between 0 and 90
    //angle = -90, not_male(-90) switches between 0 and -90

    switch (final_rx_data[2]){
       case 'm':   //male
         not_male(angle);
         break;
       case 'f':
         not_female(angle);
         break;

       default:
         break;
    }
    final_rx_data = "";  //do not repeat command

    break;

#if !DTTO_TYPE   //Command only for master module
  case 'w':  //wheel
     switch (final_rx_data[2]){
       case 'p':   //prepare wheel postion
         prepare_wheel();
         final_rx_data = "";  //do not repeat
         break;
       case 'r':  //run wheel
         run_wheel();
         break;
       default:
         break;
     }
     break;
#endif

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
    }
    else if (bt_rx_data[0] == 'a') {  //'a' means command for master as well as all slaves
      final_rx_data = bt_rx_data;

      bt_rx_data.toCharArray(rf_tx_data, 8);   //convert received bluetooth command(String) to
                                         //char array for rf transmission
      radio.stopListening();
      radio.write(rf_tx_data, 8);
      radio.startListening();
    }
    else {                            //command just for slave modules
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

  //float angle_variation;    //difference between male and female angle values.

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

    servo_angle_male = amp_factor * sin((speed_factor * t) + (my_num * PI)
                        + (1 - reverse) * PI / 2);
    servo_angle_female = amp_factor * sin((speed_factor * t) + (my_num * PI)
                        + (reverse * PI / 2));

#if HAS_SENSOR
    //angle_variation = variation(male_joint_angle, female_joint_angle);
    
    if (servo_angle_male < 0 && servo_angle_female > -2 && servo_angle_female <  2) {
     //condition for front face to face forward is that the half module containing the sensor(female)
     //has its hinge angle at the center(+2/-2) and the other half must be inclined upward.

      // bluetooth.println(servo_angle_male);
      // bluetooth.println(servo_angle_female);

      int distance = tof_sensor.readRangeSingleMillimeters() - 30;
      
//      bluetooth.print("Distance (mm): ");
//      bluetooth.println(distance);

      if (distance < 50) {    //If object closer than 50mm,
          stop_flag = true;    //raise stop flag
          //  bluetooth.println("Stop Flag=1");

          //restore hinges to center position
          modified_write(servo_male, 0, 0);
          modified_write(servo_female, 0, 1);

          radio.stopListening();
          radio.write("asm0", 4);
          delay(500);
          radio.write("asf0", 4);
          delay(500);
          radio.startListening();
       }
    }
#endif

    modified_write(servo_male, servo_angle_male, 0);
    modified_write(servo_female, servo_angle_female, 1);
    break;

  case 'f':    //motion with female module facing front
    servo_angle_male = amp_factor * sin((speed_factor * t) - (my_num * PI)
                                         + (1 - reverse) * PI / 2);
    servo_angle_female = amp_factor * sin((speed_factor * t) - ((my_num * PI)
                                         + reverse * PI / 2));

#if HAS_SENSOR
    //angle_variation = variation(male_joint_angle, female_joint_angle);

    if (servo_angle_male > 0 && servo_angle_female > -2 && servo_angle_female < 2) {
     //condition for front face to face forward is that the half module containing the sensor(female)
     //has its hinge angle at the center(+2/-2) and the other half must be inclined upward.

      // bluetooth.println(servo_angle_male);
      // bluetooth.println(servo_angle_female);

         int distance = tof_sensor.readRangeSingleMillimeters() - 30;
//         bluetooth.print("Distance (mm): ");
//         bluetooth.println(distance);

        if (distance < 50) {
          stop_flag = true;
          //  bluetooth.println("Stop Flag=1");

          //restore hinges to center position
          modified_write(servo_male, 0, 0);
          modified_write(servo_female, 0, 1);

          radio.stopListening();
          radio.write("asm0", 4);
          delay(500);
          radio.write("asf0", 4);
          delay(500);
          radio.startListening();
         
        }
    }
#endif

    modified_write(servo_male, servo_angle_male, 0);
    modified_write(servo_female, servo_angle_female, 1);

    break;
  default:
    break;
  }
  //detachServos();
}

void set_angle() {    //set a paricular angle to hinge servos
  //bluetooth.println(final_rx_data);

  //Two types of commands can be: 1sm90, 1sm-90
  //So for negative angles we must check for the '-' character
  sign = final_rx_data[3];
  int angle;

  if (sign == '-'){
    angle = final_rx_data.substring(4, 6).toInt();    //get angle parameter and convert to int.
    angle = -1 * angle;
  }
  else{
    angle = final_rx_data.substring(3, 5).toInt();    //get angle parameter and convert to int.
  }

  bluetooth.print("Angle: " + String(angle) + "\n\r");
  attach_servos();
  if ((angle <= 90) && (angle >= -90)) {    //If angle within range
    switch (final_rx_data[2]) {
    case 'm':    //for male hinge
      modified_write(servo_male, angle, 0);
      break;
    case 'f':    //for female hinge
      modified_write(servo_female, angle, 1);
      break;
    default:
      break;
    }
  }
  final_rx_data = "";
}

void escape() {    // stop all motion
  t = 0;
  detach_servos();
  reset = 1;
  wheel_run_count = 0;
}

void attach_servos() {    //attach servos to their pins
  servo_male.attach(6);
  servo_female.attach(5);
  servo_base.attach(10);
  servo_right.attach(3);
  servo_left.attach(9);
}

void detach_servos() {    //detach servos from thier pins
  servo_male.detach();
  servo_female.detach();
  servo_base.detach();
  servo_right.detach();
  servo_left.detach();
}

void modified_write(Servo &servo, int angle, int m_or_f){
  /* translates ideal physical angles from (-90,90) to angles required
   * by modified servos to set the required angle.
   * eg. -90 --- 0 --- 90 translates to 130 --- 77 --- 26
   * i.e. to set 90 degrees on the servo, you actually have to write '26' because
   * of the modifications we did to increase servo range.
   */

  if (m_or_f == 0){    //male
    int true_angle = mapped_angles_m[angle + 90];   //fetch angle from array
    //bluetooth.println(true_angle);
    servo.write(true_angle);
  }
  else if (m_or_f == 1){    //female
    int true_angle = mapped_angles_f[angle + 90];
    //bluetooth.println(true_angle);
    servo.write(true_angle);
  }
}

void prepare_wheel(){  //set the modules at their starting positions
  attach_servos();
  
  //straighten module 1: 0,0
  modified_write(servo_male, 0, 0);
  modified_write(servo_female, 0, 1);

  radio.stopListening();

  //module2: 90,-90
  radio.write("2sm90", 5);
  delay(1000);
  radio.write("2sf-90", 6);
  delay(1000);

  //straigthen module 3: 0,0
  radio.write("3sm0", 4);
  delay(1000);
  radio.write("3sf0", 4);
  delay(1000);

  //module 4: 90, -90
  radio.write("4sm90", 5);
  delay(1000);
  radio.write("4sf-90", 6);
  delay(1000);

  radio.startListening();
}

void run_wheel(){
  attach_servos();

  if(wheel_run_count % 2){   //retain male, NOT female
    radio.stopListening();
    radio.write("anf-90", 6);
    radio.startListening();

    not_female(-90);
  }
  else{    //retain female, NOT male
    radio.stopListening();
    radio.write("anm90", 5);
    radio.startListening();

    not_male(90);
  }
  delay(500);

  wheel_run_count++;
}

void not_male(int side_angle){
  attach_servos();    //Make sure to attach servos
  int current_angle = servo_male.read();
  int speed_delay = 20;

  if (current_angle == SERVO_CENTER_M){  //If servo at center,
    if(side_angle > 0){                  //go to the appropriate side angle.
      for(int angle = 0; angle <= side_angle; angle++){
        modified_write(servo_male, angle, 0);
        delay(speed_delay);
      }
    }
    else{
      for(int angle = 0; angle >= side_angle; angle--){
        modified_write(servo_male, angle, 0);
        delay(speed_delay);
      }
    }
  }
  else if(current_angle == SERVO_MIN_M || current_angle == SERVO_MAX_M){  //If at the sides,
    if(side_angle > 0){  //go to the center
      for(int angle = side_angle; angle >= 0; angle-- ){
        modified_write(servo_male, angle, 0);
        delay(speed_delay);
      }
    }
    else{
      for(int angle = side_angle; angle <= 0; angle++){
        modified_write(servo_male, angle, 0);
        delay(speed_delay);
      }
    }
  }
}

void not_female(int side_angle){
  attach_servos();    //Make sure to attach servos
  int current_angle = servo_female.read();
  int speed_delay = 20;

  if (current_angle == SERVO_CENTER_F){   //If servo at center,
    if(side_angle > 0){                   //go to the appropriate side angle.
      for(int angle = 0; angle <= side_angle; angle++){
        modified_write(servo_female, angle, 1);
        delay(speed_delay);
      }
    }
    else{
      for(int angle = 0; angle >= side_angle; angle--){
        modified_write(servo_female, angle, 1);
        delay(speed_delay);
      }
    }
  }
  else if(current_angle == SERVO_MIN_F || current_angle == SERVO_MAX_F){   //If at the sides,
    if(side_angle > 0){                                                   //go to the center
      for(int angle = side_angle; angle >= 0; angle--){
        modified_write(servo_female, angle, 1);
        delay(speed_delay);
      }
    }
    else{
      for(int angle = side_angle; angle <= 0; angle++){
        modified_write(servo_female, angle, 1);
        delay(speed_delay);
      }
    }
  }
}

float variation(float a, float b) { // absolute difference between 2 values.
  float a_abs, b_abs, d_ab;
  a_abs = abs(a);
  b_abs = abs(b);

  d_ab = abs(a_abs - b_abs);
  return d_ab;
}

