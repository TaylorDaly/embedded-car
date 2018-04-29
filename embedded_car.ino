#include <Wire.h>
#include <VL6180X.h>

// defines pins for ultrasonic sensor
#define trigPin 6
#define echoPin 7
#define RANGE 1

/* List of adresses for each sensor - after reset the address can be configured */
#define address0 0x22
#define address1 0x20 

/* GPIO stands for general purpose input/output pin */
int enablePin0 = A0;
int enablePin1 = A1;


//Used for the sensor
long timer;                             //used to measure time between meausurements
long duration;                          //used to measure the time between pulse an echo
int laserDist0;
int laserDist1;
int ultrasonicDist;


/* Create a new instance for each sensor. Sensor 0 is on the left and sensor 1 is the right, sensor 0 has the 
GPIO pin wired to A0 and sensor1 has its SHDN pin wired to A1. The SDA and SCL of sensor 0 are connected to 
A4 and A5(The SDA and SCL pins). Both senors GND and VIN pins are connected to their own G and V pins.*/
VL6180X sensor0;
VL6180X sensor1;

/*Note on steering: Changing direction rapidly can cause unexpected effects. From a mechanical standpoint, going from forward
to reverse rapidly could potentially damage a gear box.  From an electrical standpoint, it can cause large current and voltage spikes.  
To resolve these issues, a motor needs to be taken from one direction to another with a small pause inbetween */

/*All measurements are returned in mm*/


//Motor pins
int pinI1 = 8; //define I1 interface
int pinI2 = 11; //define I2 interface
int speedpinA = 9; //enable motor A
int pinI3 = 12; //define I3 interface
int pinI4 = 13; //define I4 interface
int speedpinB = 10; //enable motor B


int motorSpeed = 200; //define the speed of motor
int turnSpeed = 150;
int correctionSpeed = 150;

//State logic
enum states {movingForward, turningLeft, correctLeft, turningRight, correctRight, movingBackward, stopped} state;
enum states lastState = state;
enum states lastTurn = turningLeft;


void setup()
{
  Serial.begin(9600); // Starts the serial communication
  Wire.begin();

  //Ultrasonic sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input


  // Reset all connected laser sensors
  pinMode(enablePin0,OUTPUT);
  pinMode(enablePin1,OUTPUT);

  //both pins must be set to low before they are set to high
  digitalWrite(enablePin0, LOW);        
  digitalWrite(enablePin1, LOW);

  // Sensor 0
  Serial.println("Start Sensor 0");
  digitalWrite(enablePin0, HIGH);
  delay(50);
  sensor0.init();
  sensor0.configureDefault();
  sensor0.setAddress(address0);
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
  Serial.println(sensor1.readReg(address1));
  sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor1.setTimeout(500);
  sensor1.stopContinuous();
  sensor1.setScaling(RANGE);
  delay(300);
  sensor1.startInterleavedContinuous(100);        //sets the rate at which readings are taken.
  delay(100);

  //Set motor pins
  pinMode(pinI1, OUTPUT);
  pinMode(pinI2, OUTPUT);
  pinMode(speedpinA, OUTPUT);
  pinMode(pinI3, OUTPUT);
  pinMode(pinI4, OUTPUT);
  pinMode(speedpinB, OUTPUT);
  
  //Initial direction
  stop();
  delay(10);
  forward();

}

void loop()
{
  //get ultrasonic reading
  readUltrasonic();
  
  //get laser readings 
  laserDist0 = sensor0.readRangeContinuousMillimeters();
  laserDist1 = sensor1.readRangeContinuousMillimeters();



//making a left turn on a 90 degree turn with no opening
//to the front and an opening to the left

  if(laserDist0 >= 200) //gonna have to measure the ultrasonic sensor so the turn will no hit the walls
  {
        stop();
        delay(30);
        readUltrasonic(); //reading the ultrasonic while in the loop
        left();
        delay(550);
        Serial.println("left");
        state=movingForward;
        forward();    
        delay(500);  
  }  

//making a right turn on a 90 degree turn with no opening
//to the front and an opening to the right

  if(ultrasonicDist <= 150 && laserDist1 >= 200) //gonna have to measure the ultrasonic sensor so the turn will no hit the walls
  {
        stop();
        delay(30);
        readUltrasonic();  //reading the ultrasonic while in the loop
        right();
        delay(550);
        Serial.println("right");
        state=movingForward;
        forward();
        delay(500);
  }  


// correcting the car when getting close to the wall on the left
// this only works if the course is 10 inches wide

//***** this is a work in progress since the correcting over adjusts the correction ****
  if(laserDist0 <= 80 && state == movingForward)
  {
      int temp = laserDist0; //should be less than 100mm
      stop();
      long checkCorrectTimeLeft = millis();
      do{
          if(millis() - checkCorrectTimeLeft > 1500){
            stop();
            delay(10);
            backward();
            delay(500);
            stop();
            delay(10);
            right();
            delay(500);
            break;
          }
          laserDist0 = sensor0.readRangeContinuousMillimeters(); //reading the laser sensor while in the loop 
          slightRight();
          delay(20);
          if(laserDist0 > temp) //once the laser reading is greater then the intial reading of getting close to the wall, exit do while loop
          {
            state=movingForward;
          }
      }while(state == correctRight);
      forward();
  }
  
// correcting the car when getting close to the wall on the right
// this only works if the course is 10 inches wide
  if(laserDist1 <= 80 && state == movingForward)
  {
      int temp = laserDist1;
      stop();
      //If car has been correcting too long it's probably stuck
      long checkCorrectTimeRight = millis();
      do{
          if(millis() - checkCorrectTimeRight > 1500){
            stop();
            delay(10);
            backward();
            delay(500);
            stop();
            delay(10);
            left();
            delay(450);
            break;
          }
          laserDist1 = sensor1.readRangeContinuousMillimeters(); //reading the laser sensor while in the loop
          slightLeft();
          delay(20);
          if(laserDist1 > temp) //once the laser reading is greater then the intial reading of getting close to the wall
          {
            state=movingForward;
          }
      }while(state == correctLeft);
      forward();
  }

  if(ultrasonicDist <75 && laserDist0 < 200 && laserDist1 <200){    //test function for when car is stuck at the end of a deadend hallway
    stop();
    delay(10);
    backward();
    delay(300);
    stop();
    right();
    delay(1000);
    stop();
    forward();
    state = movingForward;
  }


  
  //print state changes
  if (state != lastState) {
    Serial.print("State: ");
    Serial.println(state);
  }
//  //track previous turn
//  if (state == turningLeft || state == turningRight) {
//    lastTurn = state;
//  }
  //track previous state
  lastState = state;
  delay(300);
  Serial.print("Ultrasonic Dist: ");
  Serial.println(ultrasonicDist);
  Serial.print("Laser Dist 0: "); 
  Serial.println(laserDist0);
  Serial.print("Laser Dist 1: ");
  Serial.println(laserDist1); // it is dividing by two since sensor laser 1 is reading in data at double the values
}//end loop

void right()
{
  analogWrite(speedpinA, turnSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, turnSpeed);
  digitalWrite(pinI4, HIGH); //turn DC Motor B move clockwise
  digitalWrite(pinI3, LOW);
  digitalWrite(pinI2, LOW); //turn DC Motor A move anticlockwise
  digitalWrite(pinI1, HIGH);
  state = turningRight;
}
void slightRight()
{
  analogWrite(speedpinA, correctionSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, motorSpeed);
  digitalWrite(pinI4, HIGH); //turn DC Motor B move clockwise
  digitalWrite(pinI3, LOW);
  digitalWrite(pinI2, HIGH); //turn DC Motor A move clockwise
  digitalWrite(pinI1, LOW);
  Serial.println("correctingRight");
  state = correctRight;
}
void slightLeft()
{
  analogWrite(speedpinA, motorSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, correctionSpeed);
  digitalWrite(pinI4, HIGH); //turn DC Motor B move clockwise
  digitalWrite(pinI3, LOW);
  digitalWrite(pinI2, HIGH); //turn DC Motor A move clockwise
  digitalWrite(pinI1, LOW);
  Serial.println("correctingleft");
  state = correctLeft;
}
void left()
{
  analogWrite(speedpinA, turnSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, turnSpeed);
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
  Serial.println("forward");
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
  digitalWrite(pinI3, LOW);
  digitalWrite(pinI1, LOW);
  state = stopped;
}

int readUltrasonic() {
  long pingTime=millis();
  
  digitalWrite(trigPin, HIGH);
  while((millis()-pingTime)>.02){
    //send out a 20 microsecond ping
  }
  digitalWrite(trigPin, LOW);
//  if((millis() - pingTime)>.2){
//    ultrasonicDist = 4000;
//  }
//  else{
  duration = pulseIn(echoPin, HIGH); 
  ultrasonicDist = (duration *.0066929)*25.24;    //convert to mm
  // }
  if (ultrasonicDist > 4000){
    ultrasonicDist = 4000;
  }
}
