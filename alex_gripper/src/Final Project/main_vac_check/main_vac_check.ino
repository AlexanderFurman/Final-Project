int vacuumPinOpen = 4; // declare pin4 as vacuumPinOpen
int vacuumPinClose = 5; // declare pin5 as vacuumPinClose
String vacuumState; 
String VacuumOnMessage = "Vacuum is turned on"; // this is a string with information
String VacuumOffMessage = "Vacuum is turned off"; // this is a string with information

void setup()
{
  //set up the pins as output pins, and set them to HIGH to begin with - this is to do with the setup of the switches
  pinMode(vacuumPinOpen, OUTPUT);  
  digitalWrite(vacuumPinOpen, HIGH); 
  pinMode(vacuumPinClose, OUTPUT);  
  digitalWrite(vacuumPinClose, HIGH); 
  Serial.begin(9600);
}
void loop()
{
//  Serial.println("Type ON or OFF"); //Prompt User for Input
//  while (Serial.available() == 0) {
//    // Wait for User to Input Data
//  }
//  vacuumState = Serial.parseInt(); //Read the data the user has input
//  if (vacuumState == "ON"){
//    
//    Serial.println(VacuumOnMessage);
//    digitalWrite(vacuumPinOpen, LOW);
//    delay(1000);
//    digitalWrite(vacuumPinOpen, HIGH);
//    
//    
//  }// check if the configuration of the switches is right, i.e. if ON means switch to LOW or if I should switch to HIGH instead?
//  else if (vacuumState == "OFF"){
//    Serial.println(VacuumOffMessage);
//    digitalWrite(vacuumPinClose, LOW);
//    delay(1000);
//    digitalWrite(vacuumPinClose, HIGH); //used to say vacuumPinOpen <-- check!!!!
//  }
//  else{
//    Serial.println("FUCK YOU");
//  }

    Serial.println(VacuumOnMessage);
    digitalWrite(vacuumPinOpen, LOW);
    delay(1000);
    digitalWrite(vacuumPinOpen, HIGH);

    delay(5000);

    Serial.println(VacuumOffMessage);
    digitalWrite(vacuumPinClose, LOW);
    delay(1000);
    digitalWrite(vacuumPinClose, HIGH); //used to say vacuumPinOpen <-- check!!!!

    delay(5000);
}
