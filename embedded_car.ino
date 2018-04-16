#include <Wire.h>
#include <VL6180X.h>

// defines pins for ultrasonic sensor
#define trigPin 6
#define echoPin 7
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
int laserDist0;
int laserDist1;
int ultrasonicDist;

/* Create a new instance for each sensor */
VL6180X sensor0;
VL6180X sensor1;

//*note on steering: Changing direction rapidly can cause unexpected effects. From a mechanical standpoint, going from forward
//to reverse rapidly could potentially damage a gear box.  From an electrical standpoint, it can cause large current and voltage spikes.  
//To resolve these issues, a motor needs to be taken from one direction to another with a small pause inbetween


//Motor pins
int pinI1 = 8; //define I1 interface
int pinI2 = 11; //define I2 interface
int speedpinA = 9; //enable motor A
int pinI3 = 12; //define I3 interface
int pinI4 = 13; //define I4 interface
int speedpinB = 10; //enable motor B

int motorSpeed = 255; //define the speed of motor

//State logic
enum states {movingForward, turningLeft, turningRight, movingBackward, stopped} state;
enum states lastState = state;
enum states lastTurn = turningLeft;


void setup()
{
  Serial.begin(9600); // Starts the serial communication
  Wire.begin();

  //Ultrasonic sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

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
  Serial.println(sensor1.readReg(address1),HEX);
  sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor1.setTimeout(500);
  sensor1.stopContinuous();
  sensor1.setScaling(RANGE);
  delay(300);
  sensor1.startInterleavedContinuous(100);
  delay(100);

  //Set motor pins
  pinMode(pinI1, OUTPUT);
  pinMode(pinI2, OUTPUT);
  pinMode(speedpinA, OUTPUT);
  pinMode(pinI3, OUTPUT);
  pinMode(pinI4, OUTPUT);
  pinMode(speedpinB, OUTPUT);
  
  //Initial direction
  forward();
}

void loop()
{
  //get ultrasonic reading
  readUltrasonic();
  laserDist0 = sensor0.readRangeContinuousMillimeters();
  laserDist1 = sensor1.readRangeContinuousMillimeters();

  //if wall is within 25 cm
  if (ultrasonicDist <= 25)
  {
    if (lastTurn == turningRight) {
      left();
      //let turn left for .75 seconds (approx 90 degrees)
      delay(750);
    } else {
      right();
      delay(750);
    }
  } else {
    forward();
  }

  //print state changes
  if (state != lastState) {
    Serial.print("State: ");
    Serial.println(state);
  }
  //track previous turn
  if (state == turningLeft || state == turningRight) {
    lastTurn = state;
  }
  //track previous state
  lastState = state;
  delay(500);
  Serial.print("Ultrasonic Dist: ");
  Serial.println(ultrasonicDist);
  Serial.print("Laser Dist 0: ");
  Serial.println(laserDist0);
  Serial.print("Laser Dist 1: ");
  Serial.println(laserDist1);
}

void right()
{
  analogWrite(speedpinA, motorSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, motorSpeed);
  digitalWrite(pinI4, HIGH); //turn DC Motor B move clockwise
  digitalWrite(pinI3, LOW);
  digitalWrite(pinI2, LOW); //turn DC Motor A move anticlockwise
  digitalWrite(pinI1, HIGH);
  state = turningRight;
}
void left()
{
  analogWrite(speedpinA, motorSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, motorSpeed);
  digitalWrite(pinI4, LOW); //turn DC Motor B move anticlockwise
  digitalWrite(pinI3, HIGH);
  digitalWrite(pinI2, HIGH); //turn DC Motor A move clockwise
  digitalWrite(pinI1, LOW);
  state = turningLeft;
}
void forward()
{
  analogWrite(speedpinA, motorSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, motorSpeed);
  digitalWrite(pinI4, HIGH); //turn DC Motor B move clockwise
  digitalWrite(pinI3, LOW);
  digitalWrite(pinI2, HIGH); //turn DC Motor A move clockwise
  digitalWrite(pinI1, LOW);
  state = movingForward;
}
void backward()
{
  analogWrite(speedpinA, motorSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, motorSpeed);
  digitalWrite(pinI4, LOW); //turn DC Motor B move counter clockwise
  digitalWrite(pinI3, HIGH);
  digitalWrite(pinI2, LOW); //turn DC Motor A move counter clockwise
  digitalWrite(pinI1, HIGH);
  state = movingBackward;
}
void stop()
{
  digitalWrite(speedpinA, LOW); // disable the speed pin to stop the motor
  digitalWrite(speedpinB, LOW);
  state = stopped;
}

int readUltrasonic() {
  long pingTime=millis();
  
  digitalWrite(trigPin, HIGH);
  while((millis()-pingTime)>.02){
    //send out a 20 microsecond ping
  }
  digitalWrite(trigPin, LOW);
  if((millis() - pingTime)>.2){
    ultrasonicDist = 4000;
  }
  else{
  duration = pulseIn(echoPin, HIGH); 
  ultrasonicDist = (duration *.0066929)*25.24;    //convert to mm
  }

}

