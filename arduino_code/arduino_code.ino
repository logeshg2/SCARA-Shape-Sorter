/*
  ### TODO ###
    1. Find correct link lengths
    2. phiAngleToSteps -> step ratio calculator joint 3
    2. Find Z-geared stepper motors gear ratio and find the zDistancetoStep ratio (IMP)
    3. Calibrate the robot's limitswitches and homing() -> set motor ticks
    4. Servo motor control and homing()
*/


//arduino code for SCARA_Robot (Modified: 31-08-2024)
// Using Arduino Mega 2560
//Check: (If Not working)
/*
 * limit switches values
 * 5v DC supply issue(encoder) ***IMP***
 * type conversion issue
 */
  
#include <AccelStepper.h>
#include <math.h>
#include <Servo.h>

// limit switches
#define limitSwitch1 12 // J1 
#define limitSwitch2 13 // J2
#define limitSwitch3 9 // J3
#define limitSwitch4 A3 // Z-axis top
// #define limitSwitch5 A0 // for the limit switch in the end effector (to find when it touches the parcel)

// Serovo pins
Servo myservo;          // servo for gripper
#define servoPin A1  // 180 -> close & 0 -> open

// stepper motor pins
AccelStepper stepper1(1, 2, 5);  // J1           // (Type:driver, STEP, DIR)
AccelStepper stepper2(1, 3, 6);  // J2
AccelStepper stepper3(1, 12, 13);  // J3
AccelStepper stepper4(1, 4, 7);  // Z axis

//*********** There is a change in length of link2 (L2) ***********
double L1 = 228; // link 1
double L2 = 136.5; // link 2 
double theta1, theta2, z;

long stepper1Position, stepper2Position, stepper3Position, stepper4Position;  // before double was used

const float theta1AngleToSteps = 44.444444; //old = 44.444444 
const float theta2AngleToSteps = 35.555555; //old = 35.555555 
const float phiAngleToSteps = 10; //35.555555;
const float zDistanceToSteps = 500; // [***IMP*** current motor takes 800 steps for 1 revolution] { see the below explanation }
/* ***IMP***
  > 800 steps for 1 rev
  > 1 rev is 1.6mm movement in Z
  > 1mm movement in Z is 500 steps
*/

/*const float zDistanceToSteps = 200; // WIP // 800 steps per mm -> according to calculation (total lenght of workspace in z-axis is 553mm) but 463 from bottom >>> 200 final
 1 rev -> 1.6mm movement in lead screw
 1 rev -> 350 ticks(WIP)
 const float zDistanceToSteps = 218.75;  // 218.75 ticks -> 1mm (movement)
*/

byte inputValue[5];
int k = 0;

String content = "";
String cont_from_py = "";
int data[10];

int theta1Array[100];
int theta2Array[100];
int phiArray[100];
int zArray[100];
int positionsCounter = 0;
int gripperArray[100];


void setup() {
  Serial.begin(115200);
  // while (!Serial) {
  //   ; // wait for serial port to connect.
  // }
  Serial.print("Serial connected to python");
  
  // Limitswitch Pins
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  pinMode(limitSwitch3, INPUT_PULLUP);
  pinMode(limitSwitch4, INPUT_PULLUP);

  // Servo Pin
  myservo.attach(servoPin);
  
  // Stepper motors max speed
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);
  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);
  stepper4.setMaxSpeed(4000);
  stepper4.setAcceleration(2000);

  // delay(500);
  //data[5] = 450; // z-axis (after homing the bot moves to height 450)
  new_homing();
}

void loop() {
  // getting data from python script: 
  while (Serial.available() == 0){
    ;//pass
  }
  content = Serial.readStringUntil('\n');  // Read the incoming data from Processing
  cont_from_py = content;   //Read from Python
  // Extract the data from the string and put into separate integer variables (data[] array)
  for (int i = 0; i < 10; i++) {
    int index = content.indexOf(","); // locate the first ","
    data[i] = atol(content.substring(0, index).c_str()); //Extract the number from start to the ","
    content = content.substring(index + 1); //Remove the number from the string
  }
  
  /*
     data[0] - SAVE button status
     data[1] - RUN button status
     data[2] - Joint 1 angle
     data[3] - Joint 2 angle
     data[4] - Joint 3 angle
     data[5] - Z position
     data[6] - Gripper value
     data[7] - Speed value
     data[8] - Acceleration value
  */

  // Movement: 
  stepper1Position = int(data[2] * theta1AngleToSteps);
  stepper2Position = int(data[3] * theta2AngleToSteps);
  stepper3Position = int(data[4] * phiAngleToSteps);
  stepper4Position = data[5] * zDistanceToSteps; // z-stepper motor
  Serial.print(stepper4Position);
  
  stepper1.setSpeed(data[7]);
  stepper2.setSpeed(data[7]);
  stepper3.setSpeed(data[7]);
  stepper4.setSpeed(data[7]);

  stepper1.setAcceleration(data[8]);
  stepper2.setAcceleration(data[8]);
  stepper3.setAcceleration(data[8]);
  stepper4.setAcceleration(data[8]);

  stepper1.moveTo(stepper1Position);
  stepper2.moveTo(stepper2Position);
  stepper3.moveTo(stepper3Position);
  stepper4.moveTo(stepper4Position);
  // Serial.print(data[5] * zDistanceToSteps);
  
  // First movement of joints (L1, L2 and L3):   
  while (stepper1.currentPosition() != stepper1Position || stepper2.currentPosition() != stepper2Position) { 
    // if (limitSwitch1 == 1 || limitSwitch2 == 1){
    //  break;
    // }
    // Serial.println(stepper2.currentPosition());
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  
  // Second movement of z-axis (stepper motor)
  while (stepper4.currentPosition() != stepper4Position){
    // Serial.println(stepper4.currentPosition());
    // if (limitSwitch4 == 1){
    //   break;
    // }
    stepper4.run();
  }
 
  // Servo Control (gripper)
  myservo.write(data[6]);
  delay(10);
  
  // Confirmation for the movement
  Serial.print(cont_from_py);
  //Serial.print(stepper4.currentPosition());
  delay(200);
}

void serialFlush() {
  while (Serial.available() > 0) {  //while there are characters in the serial buffer, because Serial.available is >0
    Serial.read();         // get one character
  }
}


void new_homing(){
  myservo.write(0);               // WIP -> default opening angle is 0
  delay(10);

  // Homing Stepper4 (Z-axis)
  while (digitalRead(limitSwitch4) != 0) {
    stepper4.setSpeed(1500);
    stepper4.runSpeed();
    stepper4.setCurrentPosition(125000); // When limit switch pressed set position to 0 steps     // old->17000 , 55300 -->1mm = 200
  }
  delay(20);
  stepper4.moveTo(100000);
  while (stepper4.currentPosition() != 100000) {
    stepper4.run();
  }

  // Homing Stepper3 (J3) (WIP)
  // while (digitalRead(limitSwitch3) != 1) {
  //   stepper3.setSpeed(1100);
  //   stepper3.runSpeed();
  //   stepper3.setCurrentPosition(1662); // When limit switch pressed set position to 0 steps
  // }
  stepper3.setCurrentPosition(1662);
  delay(20);
  stepper3.moveTo(0);  
  while (stepper3.currentPosition() != 0) {
    stepper3.run();
  }

  // Homing Stepper2 (J2)
  // while (digitalRead(limitSwitch2) != 0) {
  //   stepper2.setSpeed(-1200);
  //   stepper2.runSpeed();
  //   stepper2.setCurrentPosition(-5840); // When limit switch pressed set position to -5940 steps
  // }
  stepper2.setCurrentPosition(-5840);
  delay(20);
  stepper2.moveTo(0);
  while (stepper2.currentPosition() != 0) {
    stepper2.run();
  }

  // Homing Stepper1 (J1)
  // while (digitalRead(limitSwitch1) != 0) {
  //   stepper1.setSpeed(1200);
  //   stepper1.runSpeed();
  //   stepper1.setCurrentPosition(4200); // When limit switch pressed set position to 0 steps //old-> -4000
  // }
  stepper1.setCurrentPosition(4200);
  delay(20);
  stepper1.moveTo(0);
  while (stepper1.currentPosition() != 0) {
    stepper1.run();
  }
  //Serial.print(stepper4.currentPosition());
  Serial.print("Homed");
}

void homing() {
  //Serial.print("Inside homing()");
  // Homing Servo motor (gripper)
  myservo.write(0);               // WIP -> default opening angle is 0
  delay(10);

  // Homing Stepper4 (Z-axis)
  while (digitalRead(limitSwitch4) != 0) {
    stepper4.setSpeed(1500);
    stepper4.runSpeed();
    stepper4.setCurrentPosition(125000); // When limit switch pressed set position to 0 steps     // old->17000 , 55300 -->1mm = 200
  }
  delay(20);
  stepper4.moveTo(100000);
  while (stepper4.currentPosition() != 100000) {
    stepper4.run();
  }

  // Homing Stepper3 (J3) (WIP)
  while (digitalRead(limitSwitch3) != 1) {
    stepper3.setSpeed(1100);
    stepper3.runSpeed();
    stepper3.setCurrentPosition(1662); // When limit switch pressed set position to 0 steps
  }
  delay(20);
  stepper3.moveTo(0);  
  while (stepper3.currentPosition() != 0) {
    stepper3.run();
  }

  // Homing Stepper2 (J2)
  while (digitalRead(limitSwitch2) != 0) {
    stepper2.setSpeed(-1200);
    stepper2.runSpeed();
    stepper2.setCurrentPosition(-5840); // When limit switch pressed set position to -5940 steps
  }
  delay(20);
  stepper2.moveTo(0);
  while (stepper2.currentPosition() != 0) {
    stepper2.run();
  }

  // Homing Stepper1 (J1)
  while (digitalRead(limitSwitch1) != 0) {
    stepper1.setSpeed(1200);
    stepper1.runSpeed();
    stepper1.setCurrentPosition(4200); // When limit switch pressed set position to 0 steps //old-> -4000
  }
  delay(20);
  stepper1.moveTo(0);
  while (stepper1.currentPosition() != 0) {
    stepper1.run();
  }
  //Serial.print(stepper4.currentPosition());
  Serial.print("Homed");
}