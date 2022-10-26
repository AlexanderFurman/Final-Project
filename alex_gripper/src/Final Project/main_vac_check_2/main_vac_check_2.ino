int vacuumPinOpen = 4; // declareOpen pin10 as vacuumPinOpen
int vacuumPinClose = 5; // declare pin9 as vacuumPinClose
int subValvePin = 6; //subValve pin is 6
String vacuumState; 
String subValveState;
String VacuumOnMessage = "Vacuum is turned on"; // this is a string with information
String VacuumOffMessage = "Vacuum is turned off"; // this is a string with information
String SubValveOnMessage = "Sub-valve is turned on";
String SubValveOffMessage = "Sub-valve is turned off";

void setup()
{
  //set up the pins as output pins, and set them to HIGH to begin with - this is to do with the setup of the switches
  pinMode(vacuumPinOpen, OUTPUT);  
  digitalWrite(vacuumPinOpen, HIGH); 
  pinMode(vacuumPinClose, OUTPUT);  
  digitalWrite(vacuumPinClose, HIGH); 
  pinMode(subValvePin, OUTPUT);  
  digitalWrite(subValvePin, HIGH); //switched on??
  Serial.begin(9600);
}
void loop()
{
  Serial.println("Type ON to excite vacuum for 10 seconds"); //Prompt User for Input
  while (Serial.available() == 0) {
    // Wait for User to Input Data
  }
  vacuumState = Serial.parseInt(); //Read the data the user has input

  subValveState = "ON";
  
  if (vacuumState = "ON"){
    
    Serial.println(VacuumOnMessage);
    digitalWrite(vacuumPinOpen, LOW);
    delay(1000);
    digitalWrite(vacuumPinOpen, HIGH);

    if (subValveState == "ON"){
      digitalWrite(subValvePin, LOW);
      delay(5000);
      digitalWrite(subValvePin, HIGH);
    }
    
    

    delay(10000);
    
    Serial.println(VacuumOffMessage);
    digitalWrite(vacuumPinClose, LOW);
    delay(1000);
    digitalWrite(vacuumPinClose, HIGH);
  }// check if the configuration of the switches is right, i.e. if ON means switch to LOW or if I should switch to HIGH instead?
}
