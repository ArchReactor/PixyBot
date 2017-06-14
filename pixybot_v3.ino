// ArchReactor Pixybot v2.2
// Written by: Apollo W. Timbers
// 2017
//
// Code revision mark VII
//
// This code is a intigration of the Pixy vision system into a mobile platform to raise awerness of the maker space
// know as Arch Reactor.
// Sections of the code include following tracked objects, IR sensor object detecting, A PID control loop,
// A low battery warning/cuttoff and a button for pausing the drive servos so you can train the Pixy.
//
// Portions of this code are derived from Adafruit Industries Pixy Pet code.
// Portions of this code are derived from the Pixy CMUcam5 pantilt example code.
//

#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

#define X_CENTER 160L
#define Y_CENTER 100L
#define RCS_MIN_POS 0L
#define RCS_MAX_POS 1000L
#define RCS_CENTER_POS ((RCS_MAX_POS-RCS_MIN_POS)/2)
#define LEFT_CENTER 10
#define RIGHT_CENTER 8

// number of analog samples to take per reading
#define NUM_SAMPLES 10

// set pin numbers:
const int buttonPin = 2; // the number of the pushbutton pin
const int leftMotorPin = 9;
const int rightMotorPin = 10;
const int ledPin = 13; // the pin number of the LED pin
const int analogInput = A0; //battery voltage test
const int irSenseleft = A1; // Connect sensor to analog pin A0
const int irSenseright = A2; // Connect sensor to analog pin A0

int sum = 0; // sum of samples taken
unsigned char sample_count = 0; // current sample number
int ledgreen = 5; // the pin number of the LED pin
int ledred = 7; // the pin number of the LED pin
// variables will change:
int buttonState = 0; // variable for reading the pushbutton status
Servo leftServo; // Define the Servos
Servo rightServo;
unsigned int raw;
double vcc = 0;
double voltage; // calculated voltage
int distanceleft = 0; //IR
int distanceright = 0; //IR

class ServoLoop
{
  public:
    ServoLoop(int32_t pgain, int32_t dgain);

    void update(int32_t error);

    int32_t m_pos;
    int32_t m_prevError;
    int32_t m_pgain;
    int32_t m_dgain;
};

ServoLoop panLoop(350, 500);
ServoLoop tiltLoop(550, 700);

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  {
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>11;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos>RCS_MAX_POS)
    m_pos = RCS_MAX_POS;
    else if (m_pos<RCS_MIN_POS)
    m_pos = RCS_MIN_POS;
    //cprintf("%d %d %d\n", m_axis, m_pos, vel);
  }
  m_prevError = error;
}

Pixy pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  leftServo.attach(leftMotorPin); // servo on digital pin 9
  leftServo.writeMicroseconds(1500+LEFT_CENTER);
  rightServo.attach(rightMotorPin); // servo on digital pin 10
  rightServo.writeMicroseconds(1500+RIGHT_CENTER);
  
  pixy.init();
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(ledgreen, OUTPUT);
  pinMode(ledred, OUTPUT);
  digitalWrite(ledred, LOW);
  digitalWrite(ledgreen, LOW);
  //analogReference(INTERNAL);
  delay (450);
  pixy.getBlocks();
  panLoop.m_pos = 520;
  tiltLoop.m_pos = 840;
  pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

  Serial.print("Checking internal voltage...\n");
  readVcc(); // Check initial voltage
  Voltage();
  delay (50);
}

uint32_t hold = 0;
int32_t size = 400;
uint32_t lastBlockTime = 0;
//---------------------------------------
// Main loop - runs continuously after setup
//---------------------------------------
void loop()
{
  static unsigned long timer=millis(); // initialize timer variable and set to millisecond timer
  static unsigned long timerA=millis(); // initialize timer variable and set to millisecond timer
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  int32_t panError, tiltError;
  blocks = pixy.getBlocks();
  // If we have blocks in sight, track and follow them
  if (blocks)
  { 
    int trackedBlock = TrackBlock(blocks);
    FollowBlock(trackedBlock);
    lastBlockTime = millis();
    size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height;
    size -= size >> 4;
  }
  else if (millis() - lastBlockTime > 200)
  { 
    leftServo.writeMicroseconds(1500 + LEFT_CENTER); // Hold motors because of low power state
    rightServo.writeMicroseconds(1500 + RIGHT_CENTER);
    //leftServo.detach(); // servo on digital pin 9
    //rightServo.detach(); // servo on digital pin 10
    //delay (15);
    ScanForBlocks();
  }
  if(millis() - timer > 8000) // if at least 3000mS have passed
  {
    timer=millis(); // reset timer
    readVcc();
    Voltage();
  }
  if(millis()-timerA>100) //Wait 100 ms between each read for IR
  // According to datasheet time between each read
  // is -38ms +/- 10ms. Waiting 100 ms assures each
  // read is from a different sample
  {
    timerA=millis(); // reset timer
    Pausebutton();
    //irRead();
  }
  if (voltage >= 6){
    digitalWrite(ledgreen, HIGH);
  }
  else if ((voltage >= 5) && (voltage < 6))
  {
    digitalWrite(ledgreen, LOW);
    digitalWrite(ledred, HIGH);
  }
  else if (voltage < 5)
  {
    leftServo.writeMicroseconds(1500 + LEFT_CENTER); // Hold motors because of low power state
    rightServo.writeMicroseconds(1500 + RIGHT_CENTER);
    digitalWrite(ledgreen, LOW);
    digitalWrite(ledred, HIGH); // LED to warn of low battery
    delay (500);
    digitalWrite(ledred, LOW);
    delay (500);
  }
}

int oldX, oldY, oldSignature;

//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount)
{
  int trackedBlock = 0;
  long maxSize = 0;
  uint32_t hold = 0;

//  Serial.print("blocks =");
//  Serial.println(blockCount);

  for (int i = 0; i < blockCount; i++)
  {
    if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
    {
      long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
      if (newSize > maxSize)
      {
        trackedBlock = i;
        maxSize = newSize;
      }
    }
  }

  int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
  int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;

  panLoop.update(panError);
  tiltLoop.update(tiltError);

  pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);

  oldX = pixy.blocks[trackedBlock].x;
  oldY = pixy.blocks[trackedBlock].y;
  oldSignature = pixy.blocks[trackedBlock].signature;
  return trackedBlock;
}

void FollowBlock(int trackedBlock)
{
  int32_t followError = panLoop.m_pos-540;

  // Forward speed decreases as we approach the object (size is larger)
  // Size is the area of the object.
  // We keep a running average of the last 8.
  size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height;
  size -= size >> 4;
  
  // Forward speed decreases as we approach the object (size is larger)
  int forwardSpeed = constrain((size/400)-400, -400, 400); 

  int32_t differential = (followError + (followError * forwardSpeed))>>6;

  // Adjust the left and right speeds by the steering differential.
  int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
  int rightSpeed = constrain(forwardSpeed - differential, -400, 400);

  leftSpeed = map(leftSpeed,-400,400,1700,1300); // Map to servo output values
  rightSpeed = map(rightSpeed,-400,400,1300,1700); // Map to servo output values 

  if(leftSpeed < 1520 && leftSpeed > 1480) leftSpeed = 1500;
  if(rightSpeed < 1520 && rightSpeed > 1480) rightSpeed = 1500;
 
//  if(leftSpeed > 1400 && leftSpeed < 1500) leftSpeed = ((1500- leftSpeed)/2) + 1500;
//  if(rightSpeed < 1600 && rightSpeed > 1500) rightSpeed = ((rightSpeed - 1500)/2) + 1500;
/*
  Serial.print(size/400);
  Serial.print(",");
  Serial.print(panLoop.m_pos);
  Serial.print(",");
  Serial.print(tiltLoop.m_pos);
  Serial.print(" ");
  Serial.print(forwardSpeed);
  Serial.print(",");
  Serial.print(differential);
  Serial.print(" ");
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.println(rightSpeed);
*/
  // Update servos
  leftServo.writeMicroseconds(leftSpeed + LEFT_CENTER);
  rightServo.writeMicroseconds(rightSpeed + RIGHT_CENTER);
}

//---------------------------------------
// search for blocks
//---------------------------------------
void ScanForBlocks()
{
  static uint32_t lastMove = 0;
  static int tiltDir = -1;
  static int panDir = 1;
  
  if (millis() - lastMove > 30){
    panLoop.m_pos += panDir*15;
    if (panLoop.m_pos > RCS_MAX_POS - 50) {
      panLoop.m_pos = RCS_MAX_POS - 50;
      panDir = 0 - panDir;
      tiltLoop.m_pos += tiltDir*50;
    }
    if (tiltLoop.m_pos > 950) {
      tiltLoop.m_pos = 950;
      tiltDir = 0 - tiltDir;
    }
    if (panLoop.m_pos < RCS_MIN_POS + 50) {
      panLoop.m_pos = RCS_MIN_POS + 50;
      panDir = 0 - panDir;
      tiltLoop.m_pos += tiltDir*50;
    }
    if (tiltLoop.m_pos < 450) {
      tiltLoop.m_pos = 450;
      tiltDir = 0 - tiltDir;
    }
    
//    Serial.print(panLoop.m_pos); Serial.print(",");
//    Serial.print(tiltLoop.m_pos); Serial.println("");
    panLoop.m_prevError = 0x80000000L;
    tiltLoop.m_prevError = 0x80000000L;
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    lastMove = millis();
  }
}


// Take multiple readings, and average them out to reduce false readings
void irRead()
{
  Serial.println("IR start");
  int averagingleft = 0; // Holds value to average readings
  int averagingright = 0; // Holds value to average readings

  // Get a 3 samples of raw readings from IR sensors
  //for (int i=0; i<3; i++)
  {
    distanceleft = analogRead(irSenseleft);
    averagingleft = averagingleft + distanceleft;
    distanceright = analogRead(irSenseright);
    averagingright = averagingright + distanceright;
  }
  distanceleft = averagingleft / 5; // Average out readings
  //return(distanceleft); // Return value
  //Serial.println(distanceleft, DEC);
  distanceright = averagingright / 5; // Average out readings
  //return(distanceright); // Return value
  //Serial.println(distanceright, DEC);
  if (distanceleft < 100)
  {
    leftServo.writeMicroseconds(1500+LEFT_CENTER); //allstop
    rightServo.writeMicroseconds(1500+RIGHT_CENTER);
    delay(20);
    leftServo.writeMicroseconds(1660); //backup
    rightServo.writeMicroseconds(1460);
    delay(700);
    leftServo.writeMicroseconds(1460); //spin
    rightServo.writeMicroseconds(1460);
    delay(300);
  }
  else if (distanceright < 100)
  {
    leftServo.writeMicroseconds(1500+LEFT_CENTER); //allstop
    rightServo.writeMicroseconds(1500+RIGHT_CENTER);
    delay(20);
    leftServo.writeMicroseconds(1660); //backup
    rightServo.writeMicroseconds(1460);
    delay(700);
    leftServo.writeMicroseconds(1660); //spin
    rightServo.writeMicroseconds(1660);
    delay(300);
  }
}

void Voltage()
{
  Serial.print ("Battery Voltage = ");
  vcc = readVcc()/1000.0;
  raw = analogRead(analogInput);
  voltage = ((raw / 1023.0) * vcc) * 2;
  Serial.println(voltage, DEC);
  Serial.print ("VCC Voltage = ");
  Serial.println(vcc, DEC );
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1106400L / result; // Back-calculate AVcc in mV
  return result;
}

void Pausebutton()
{
  sample_count = 0;
  sum = 0;
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    leftServo.writeMicroseconds(1500+LEFT_CENTER); //allstop
    rightServo.writeMicroseconds(1500+RIGHT_CENTER);
    delay(18000);
  }
}
