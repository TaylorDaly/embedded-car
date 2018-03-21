// defines pins for ultrasinic
const int trigPin = 6;
const int echoPin = 7;
long duration;
int distance;
int pinI1 = 8; //define I1 interface
int pinI2 = 11; //define I2 interface
int speedpinA = 9; //enable motor A
int pinI3 = 12; //define I3 interface
int pinI4 = 13; //define I4 interface
int speedpinB = 10; //enable motor B
int motorSpeed = 175; //define the speed of motor

void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  pinMode(pinI1, OUTPUT);
  pinMode(pinI2, OUTPUT);
  pinMode(speedpinA, OUTPUT);
  pinMode(pinI3, OUTPUT);
  pinMode(pinI4, OUTPUT);
  pinMode(speedpinB, OUTPUT);
  forward();
}

void loop()
{
  readUltrasonic();
  if (distance <= 25)
  {
    left();
    delay(1400);
  }//else //forward();
  forward();
  //  left();
  //  delay(2000);
  //  stop();
  //  right();
  //  delay(2000);
  //  stop();
  //  delay(2000);
  //  forward();
  //  delay(2000);
  //  stop();
  //  backward();
  //  delay(2000);
  //  stop();
}
void left()
{
  analogWrite(speedpinA, motorSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, motorSpeed);
  digitalWrite(pinI4, HIGH); //turn DC Motor B move clockwise
  digitalWrite(pinI3, LOW);
  digitalWrite(pinI2, LOW); //turn DC Motor A move anticlockwise
  digitalWrite(pinI1, HIGH);
}
void backward()
{
  analogWrite(speedpinA, motorSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, motorSpeed);
  digitalWrite(pinI4, LOW); //turn DC Motor B move anticlockwise
  digitalWrite(pinI3, HIGH);
  digitalWrite(pinI2, HIGH); //turn DC Motor A move clockwise
  digitalWrite(pinI1, LOW);
}
void forward()
{
  analogWrite(speedpinA, motorSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, motorSpeed);
  digitalWrite(pinI4, HIGH); //turn DC Motor B move clockwise
  digitalWrite(pinI3, LOW);
  digitalWrite(pinI2, HIGH); //turn DC Motor A move clockwise
  digitalWrite(pinI1, LOW);
}
void right()
{
  analogWrite(speedpinA, motorSpeed); //input a simulation value to set the speed
  analogWrite(speedpinB, motorSpeed);
  digitalWrite(pinI4, LOW); //turn DC Motor B move anticlockwise
  digitalWrite(pinI3, HIGH);
  digitalWrite(pinI2, LOW); //turn DC Motor A move clockwise
  digitalWrite(pinI1, HIGH);
}
void stop()
{
  digitalWrite(speedpinA, LOW); // disable the speed pin to stop the motor
  digitalWrite(speedpinB, LOW);
  //delay(1000);
}

int readUltrasonic() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
}

