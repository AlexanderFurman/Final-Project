#include <ros.h>
#include <std_msgs/UInt16.h>

#define dirPin 3
#define stepPin 2
#define stepsPerRevolution 400
#define initLength 0

#define mainValveOnPin 4 //double check which is on which is off
#define mainValveOffPin 5
#define subValvePin 6 //double check this is indeed the subvalve pin

#define potLeftPin A1
#define potRightPin A2 //rearange if need be
#define vacSensorPin A0

float currentLength = initLength;
const float delTheta = 0.01570796; // [radians] -- converison: 0.9*pi/180  
const float pulleyRadius = 8.5; // [mm]
const float maxLength = 98; // [mm]
const float minLength = 0;
int servoLength = 0;

String VacuumOnMessage = "Vacuum is turned on"; // this is a string with information
String VacuumOffMessage = "Vacuum is turned off"; // this is a string with information

//creating a ROS Node on the Arduino 
ros::NodeHandle nh;

// Create 2 Publishers for the potentiometer data <--- consider making this 1 publisher after confirming they work
std_msgs::UInt16 potMsgL;
std_msgs::UInt16 potMsgR;

ros::Publisher potentiometerLeft("potentiometer_left", &potMsgL);

ros::Publisher potentiometerRight("potentiometer_right", &potMsgR);

//Create Publisher for the vacuum sensor data
std_msgs::UInt16 vacuumMsg;

ros::Publisher vacuumSensor("vacuumSensor", &vacuumMsg);

//Create Subscriber which listens for commands related to the valve system
//callback function once the toggle_valve topic recieves a message
void valveMsgCb( const std_msgs::UInt16& valveMsg){
  if(valveMsg.data == 0){
    mainValve_close();
  }
  // activate suction from both fingers
  else if(valveMsg.data == 1){ 
    subValve_open();
    mainValve_open();
  }
  // activate suction from only the left finger
  else if(valveMsg.data == 2){
    subValve_close();
    mainValve_open();
  }
  else{
    Serial.println("ERROR: unknown valve request published");
  }
}
ros::Subscriber<std_msgs::UInt16> toggle_valves("toggle_valves", &valveMsgCb );


//Create Subscriber for the Motor <-- NOTE: motorMsg.data should be in [mm]
void motorMsgCb( const std_msgs::UInt16& motorMsg){
  moveFingers(motorMsg.data);
}
ros::Subscriber<std_msgs::UInt16> toggle_dist("toggle_dist", &motorMsgCb );

void setup() {
  //initialize Node:
  nh.initNode();

  //set up subscribers and publishers
  nh.advertise(potentiometerLeft);
  nh.advertise(potentiometerRight);
  nh.advertise(vacuumSensor);
  nh.subscribe(toggle_valves);
  nh.subscribe(toggle_dist);
  
  // Declare pins as outputs and inputs, as well as their initial states:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, HIGH);
  digitalWrite(stepPin, LOW);

  pinMode(mainValveOnPin, OUTPUT);
  pinMode(mainValveOffPin, OUTPUT);
  digitalWrite(mainValveOnPin, HIGH);
  digitalWrite(mainValveOffPin, HIGH);

  pinMode(subValvePin, OUTPUT);
  digitalWrite(subValvePin, LOW); //initialize the subvalve to be closed

  pinMode(potLeftPin, INPUT);
  pinMode(potRightPin, INPUT);
  pinMode(vacSensorPin, INPUT);    
  

  //set Baud rate
  Serial.begin(57600);
//  Serial.setTimeout(500);
}

void loop() {
  potMsgL.data = analogRead(potLeftPin);
  potMsgR.data = analogRead(potRightPin);
  vacuumMsg.data = analogRead(vacSensorPin);
  

  potentiometerLeft.publish(&potMsgL);
  potentiometerRight.publish(&potMsgR);
  vacuumSensor.publish(&vacuumMsg);
  nh.spinOnce();
  delay(1);
}



void moveFingers(int L) {

  if ((L >= minLength) && (L <= maxLength)) {
    
    //to calculate the steps required, we make the following calculation:
    int steps = int(floor(0.5*abs(L-currentLength)/(pulleyRadius*delTheta)));

    if (L > currentLength){
      openFingers(steps);
    }
    else if (L < currentLength){
      closeFingers(steps);
    }
    else{
      Serial.println("The length between the fingers is already " + String(L) + "[mm].");
    }
    
    currentLength = L;
  }
  else{
    Serial.println("The number you inputed is either too high or too low. Try again...");
  }
  delay(1000);
  
}

void openFingers(int steps) {

  //sets the rotation of the motor to anticlockwise - opening gripper
  digitalWrite(dirPin, HIGH);
  
  //open the gripper by indicated number of steps
  for (int i = 0; i < steps; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000); // bigger number, slower rotation
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);
}

void closeFingers(int steps) {

  //sets the rotation of the motor to anticlockwise - opening gripper
  digitalWrite(dirPin, LOW);
  
  //open the gripper by indicated number of steps
  for (int i = 0; i < steps; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000); // bigger number, slower rotation
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);
}

void mainValve_open(){
  Serial.println(VacuumOnMessage);
  digitalWrite(mainValveOnPin, LOW);
  delay(1000);
  digitalWrite(mainValveOnPin, HIGH);
}

void mainValve_close(){
  Serial.println(VacuumOnMessage);
  digitalWrite(mainValveOffPin, LOW);
  delay(1000);
  digitalWrite(mainValveOffPin, HIGH);
}

void subValve_open(){
  digitalWrite(subValvePin, HIGH);
}

void subValve_close(){
  digitalWrite(subValvePin, LOW);
}
