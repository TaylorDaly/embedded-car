
#include <Wire.h>
#include <VL6180X.h>


#define trigPin 6
#define echo 7
#define RANGE 1

/* List of adresses for each sensor - after reset the address can be configured */
#define address0 0x22
#define address1 0x20                                             


/* These Arduino pins must be wired to the IO0 pin of VL6180x */
int enablePin0 = A0;
int enablePin1 = A1;

//Used for the sensor
long timer;                             //used to measure time between meausurements
long duration;                          //used to measure the time between pulse an echo
int USdistanceRead; 
int laserDist0;
int laserDist1;


/* Create a new instance for each sensor */
VL6180X sensor0;
VL6180X sensor1;


//*note on steering: Changing direction rapidly can cause unexpected effects. From a mechanical standpoint, going from forward
//to reverse rapidly could potentially damage a gear box.  From an electrical standpoint, it can cause large current and voltage spikes.  
//To resolve these issues, a motor needs to be taken from one direction to another with a small pause inbetween


void setup() {

  Serial.begin(9600);
  Wire.begin();

//US sensor
  pinMode(echo, INPUT);
  pinMode(trigPin, OUTPUT);
  
  // Reset all connected sensors
  pinMode(enablePin0,OUTPUT);
  pinMode(enablePin1,OUTPUT);

  
  digitalWrite(enablePin0, LOW);        //both pins must be set to low before they are set to high
  digitalWrite(enablePin1, LOW);



  // Sensor 0
  Serial.println("Start Sensor 0");
  digitalWrite(enablePin0, HIGH);
  delay(50);
  sensor0.init();
  sensor0.configureDefault();
  sensor0.setAddress(address0);
  Serial.println(sensor0.readReg(address0));                        // read I2C address
  sensor0.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor0.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor0.setTimeout(500);
  sensor0.stopContinuous();
  sensor0.setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
  delay(300);
  sensor0.startInterleavedContinuous(100);
  delay(100);
  
  // Sensor1
  Serial.println("Start Sensor 1");
  digitalWrite(enablePin1, HIGH);
  delay(50);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setAddress(address1);
  Serial.println(sensor1.readReg(address0),HEX);
  sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor1.setTimeout(500);
  sensor1.stopContinuous();
  sensor1.setScaling(RANGE);
  delay(300);
  sensor1.startInterleavedContinuous(100);
  delay(100);

  
  //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel B pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel B pin

}

void loop(){

   //lasers-----------------------------------
  //in mm
  laserDist0 = sensor0.readRangeContinuousMillimeters();
  laserDist1 = sensor1.readRangeContinuousMillimeters();

  //US distance reading in mm
  distance();

 
Serial.println(USdistanceRead);
Serial.println(laserDist0);
Serial.println(laserDist1);


/*
  
  //Motor A forward @ full speed
  digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 255);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(13, LOW);  //Establishes backward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 123);    //Spins the motor on Channel B at half speed

  
  delay(3000);

  
  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  digitalWrite(9, HIGH);  //Engage the Brake for Channel B


  delay(1000);
  
  
  //Motor A forward @ full speed
  digitalWrite(12, LOW);  //Establishes backward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 123);    //Spins the motor on Channel A at half speed
  
  //Motor B forward @ full speed
  digitalWrite(13, HIGH); //Establishes forward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 255);   //Spins the motor on Channel B at full speed
  
  
  delay(3000);
  
  
  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  digitalWrite(9, HIGH);  //Engage the Brake for Channel B
  */
  
  delay(1000);
  
}



/**
* US distance readings in mm
*/
void distance(){  

//US sensor ----------------------------------
  long pingTime=millis();
  
  digitalWrite(trigPin, HIGH);
  while((millis()-pingTime)>.02){
    //send out a 20 microsecond ping
  }
  digitalWrite(trigPin, LOW);
  if((millis() - pingTime)>.2){
    USdistanceRead = 48;
  }
  else{
  duration = pulseIn(echo, HIGH); 
  USdistanceRead = (duration *.0066929)*25.24;    //convert to mm
  }

 
}
