//used libraries for LCD
#include <Wire.h>
#include "LiquidCrystal_I2C.h"
// setup of lcd
LiquidCrystal_I2C lcd(0x27, 20, 4); //LCD adress
// defines pins numbers
const int trigPinHand = 9; //hand position ultrasonic sensor Trig
const int echoPinHand = 10; //hand position ultrasonic sensor Echo
const int trigPinBall = 4; //ball position ultrasonic sensor Trig
const int echoPinBall = 7; //ball position ultrasonic sensor Echo
const int fanPin      = 3;// PWM output for motor control
const int tubeLightPin = 11; //PWM output for tube light control
const int led = 2; //output for the LED which represents the button state
const int button = 5; //input of the button state
// defines variables
unsigned long durationH;
double distanceH;
unsigned long durationB;
double distanceB;
double gas = 0;
double ref = 0; //position of hand
double ballPos = 0; //position of the ball in the tube
//Geometrical parameters of the system
const double upperGap = 1; //upper gap in cm
const double lowerGap = 10; //lower gap in cm
const double columnL = 38.5; //effective length of the tube in cm
#define PREV_REF 70;
double previousRef = PREV_REF; // the initial desired reference is PREV_REF [%] of the effective length of the tube
const double ballDiam = 4; //diameter of the ball in cm
const int maxWaitTime = (int) (columnL * 3.5 / 0.034); //maximal waiting time for the ultrasonic sensors to receive the reflected wave
//variables for the control loop
double controlP = 0;
double controlI = 0;
double controlD = 0;
double control = 0;
double err = 0;
double prevErr = 0;
double previousBallPos = 0; //the initial ball position, then updates
double prevControlI = 0; //initial condition for integrator
double prevBallPos = columnL + upperGap - ballDiam / 2; //used in control D
double storeHandPos[5]; // array to make average of hand position

//PID constants
const double samplingTime = 0.0625; //sampling time in seconds

const double intTimea = 0.4 * 2.0;                           
const double difTimea = 0.02 * 0.77; 
const double Kpa = 0.7 * 3;                            
const double Kia = Kpa / intTimea;
const double Kda = Kpa * difTimea;

const double intTimeb = 0.50 * 3.2;
const double difTimeb = 0.012 * 2.2; 
const double Kpb = 0.35 * 2.0; 
const double Kib = Kpb / intTimeb;
const double Kdb = Kpb * difTimeb;

const double intTimec = 0.5 * 3.2; 
const double difTimec = 0.014 * 1.1;
const double Kpc = 0.45 * 1.65;
const double Kic = Kpc / intTimec;
const double Kdc = Kpc * difTimec;
// defines variables to control the button state
int countButton = 0; //initial condition
int buttonState = LOW;
// variables to calculate duration
long t = 0;
long t_p = 0;
long duration = 0;
long t1 = 0;
long t2 = 0;
long t3 = 0;

//flags
int flag = 0; //used as a counter -> decides when to write to output
bool flagRefErr = 0; //true if there is an error reading reference
bool flagBallPosErr = 0; //true if there is an error reading ball position
int flagA = 0; //used as a counter for calculating average of references
int flagInstability=0;
char region = 'a'; //region 'a', 'b' or 'c' - division of the tube into three regions
//Limits for regions
const int limitAB = 18; //in % of the effective length of the tube
const int limitBC = 64; //in % of the effective length of the tube
//Lamp control limits
const int intensLowerLim = 10; //The lowest position of the ball that will produce difference in the LED light of the tube 
//(for all positions below, there would be no light)
const int intensUpperLim = 80; //The highest position of the ball that will produce difference in the LED light of the tube
//(for all positions above, there would be maximum light intensity)

void setup() {
  pinMode(trigPinBall, OUTPUT); // Sets the trigPin as an Output for Ball position sensor
  pinMode(echoPinBall, INPUT); // Sets the echoPin as an Input for Ball position sensor
  pinMode(trigPinHand, OUTPUT); // Sets the trigPin as an Output for Hand postion sensor
  pinMode(echoPinHand, INPUT); // Sets the echoPin as an Input for Hand position sensor
  pinMode(fanPin, OUTPUT); //Sets the fanPin to an output mode
  pinMode(led, OUTPUT); //Sets the led pin to an output mode (used for button state representation)
  pinMode (button, INPUT); //Sets the button pin to an input mode
  Serial.begin(9600); // Starts the serial communication
  //inital condition
  digitalWrite (led, LOW); //Turn off the button LED
  // initialize the LCD
  lcd.begin();
  lcd.backlight();
}



void loop() {
  //select the mode, ON or OFF, with the button
  buttonState = digitalRead(button);
  if (buttonState == LOW && countButton == 2) {

    countButton = 2; //update the indicator of the state

  }
  else if (buttonState == LOW && countButton == 0) {
    countButton = 0; //update the indicator of the state
  }
  else if (buttonState == LOW && countButton == 1) {

    if (flagInstability) 
    {
      delay(8);             //reset needed to let the ball fall down in case of instability risk
      flagInstability=0;
    }
    countButton = 2; //update the indicator of the state
    printInfoLCD();
    prevControlI = 0;
    previousRef = PREV_REF;
    previousBallPos = 0;
    flag = 0;
    flagA = 0;
    region = 'a';
  }
  else if (buttonState == HIGH && countButton == 1) {
    countButton = 1; //update the indicator of the state
  }
  else if (buttonState == HIGH && countButton == 0) {
    countButton = 1; //update the indicator of the state
  }
  else if (buttonState == HIGH && countButton == 2) {
    countButton = 3; //update the indicator of the state
  }
  else if (buttonState == LOW && countButton == 3) {
    countButton = 0; //update the indicator of the state
  }
  else if (buttonState == HIGH && countButton == 3) {
    countButton = 3; //update the indicator of the state
  }


  if (countButton == 2)
  {

    digitalWrite(led, HIGH);

    //____________HAND___________________
    digitalWrite(trigPinHand, LOW); // Clears the trigPin
    delayMicroseconds(2);
    digitalWrite(trigPinHand, HIGH);// Sets the trigPin on HIGH state for 10 micro seconds
    delayMicroseconds(10);
    digitalWrite(trigPinHand, LOW);
    durationH = pulseIn(echoPinHand, HIGH, maxWaitTime);// Reads the echoPin, returns the sound wave travel time in microseconds
    // Calculating the distance
    distanceH = durationH * 0.034 / 2;
    if (distanceH == 0) //Handling the error
    {
      flagRefErr = true;
      durationH = maxWaitTime;
    }
    digitalWrite(trigPinBall, LOW); //try this
    delay((int)(30 - durationH * 0.001));
    storeHandPos[flagA] = distanceH - lowerGap; //scales the value in the reference system



    //_____________BALL_________________
    digitalWrite(trigPinBall, LOW); // Clears the trigPin
    delayMicroseconds(2);
    digitalWrite(trigPinBall, HIGH);// Sets the trigPin on HIGH state for 10 microseconds
    delayMicroseconds(10);
    digitalWrite(trigPinBall, LOW);
    durationB = pulseIn(echoPinBall, HIGH, maxWaitTime);// Reads the echoPin, returns the sound wave travel time in microseconds
    // Calculating the distance
    distanceB = durationB * 0.034 / 2;
    
    if (distanceB == 0) //Handling the error
    {
      flagBallPosErr = true;
      durationB = maxWaitTime;
    }
    else flagBallPosErr = false;
    ballPos = (columnL + upperGap) - (distanceB + ballDiam / 2); // scales the ball position value in the reference system
    delay((int)(25 - durationB * 0.001));
    
    


    //scale the values from 0 to 100
    storeHandPos[flagA] = storeHandPos[flagA] * (100 / columnL);
    ballPos = ballPos * (100 / columnL);
    //avoid instability
    flagA = flagA + 1;
    if (flagA == 3)
    {
      ref = (storeHandPos[0] + storeHandPos[1] + storeHandPos[2]) / 3; //calculates the average
      if ((ref < 21) || (ref > 77) || flagRefErr) ref = previousRef;
      flagRefErr = false;
      previousRef = ref; //stores prevouos value of ref in memory for next block of 3 iterations
      flagA = 0; //resets the counter for calculating the average of reference
    }
    if ((ballPos < 0) || (ballPos > 100) || flagBallPosErr) ballPos = prevBallPos;

    
    
    //Anti wind-up solution for transitions between different subregions of the tube
    if (ballPos > limitAB && ballPos <= limitBC && previousBallPos <= limitAB) //transition from region A to B
    {
      prevControlI = control - Kpb * (ref - ballPos) - Kib * (ref - ballPos) * samplingTime - Kdb * (-ballPos + prevBallPos) / samplingTime; //bumpless control
      region = 'b';
    }
    else if (ballPos > limitBC && previousBallPos > limitAB && previousBallPos <= limitBC) //transition from region B to C
    {
      prevControlI = control - Kpc * (ref - ballPos) - Kic * (ref - ballPos) * samplingTime - Kdc * (-ballPos + prevBallPos) / samplingTime; //bumpless control
      region = 'c';
    }
    else if (ballPos > limitAB && ballPos <= limitBC && previousBallPos > limitBC) //transition from region C to B
    {
      prevControlI = control - Kpb * (ref - ballPos) - Kib * (ref - ballPos) * samplingTime - Kdb * (-ballPos + prevBallPos) / samplingTime; //bumpless control
      region = 'b';
    }
    else if (ballPos <= limitAB && previousBallPos > limitAB && previousBallPos <= limitBC) //transition from region B to A
    {
      prevControlI = control - Kpa * (ref - ballPos) - Kia * (ref - ballPos) * samplingTime - Kda * (-ballPos + prevBallPos) / samplingTime; //bumpless control
      region = 'a';
    }
    else if (ballPos > limitBC && previousBallPos <= limitAB) //transition from region A to C
    {
      prevControlI = control - Kpc*(ref-ballPos) - Kic*(ref-ballPos)*samplingTime - Kdc*(-ballPos+prevBallPos)/samplingTime; //bumpless control
      region = 'c';
    }

    else if (ballPos <= limitAB && previousBallPos > limitBC) //transition from region C to A
    {
      prevControlI = control - Kpa*(ref-ballPos) - Kia*(ref-ballPos)*samplingTime - Kda*(-ballPos+prevBallPos)/samplingTime; //bumpless control
      region = 'a';
    }

    PID(region); //calculates the control parameters for the current sampling interval
    //uses the PID to set the value of the power to the motor according to the position and speed
    if (ballPos>85) 
    {
      motorPower(fanPin,0);
      //Reset is needed to avoid the ball hit the sensor and remain stick to the sensor   
      flagInstability=1;
      countButton = 1; //updates the indicator of the state
    }
    else
    { 
      motorPower(fanPin, control); //Control voltage sent to motor
      ledPower(tubeLightPin, ballPos); //Control voltage sent to led light
      previousBallPos = ballPos; //stores prevouos value of ref in memory for next iteration
      flag = flag + 1;
    }
 Serial.println(ballPos);
//display values on serial monitor
    if (flag == 20) {
      t_p = t;
      t = millis();// used to get duratin of one loop
      duration = (t - t_p);
      t1=millis();
      updateInfoLCD(); //Updates the info to be shown on the LCD
      t2=millis();
      t3=t2-t1;
      flag = 0;
    }

    
  }
  else if (countButton == 0)
  {
    //System is OFF
    digitalWrite(led, LOW); //turns OFF the button LED
    motorPower(fanPin, 0); //turns OFF the motor
    ledPower(tubeLightPin, 0);

    //Initial state LCD display
    lcd.setCursor(0, 1);
    lcd.print("PRESS THE BUTTON TO");
    lcd.setCursor(0, 2);
    lcd.print("RUN THE SYSTEM.");
  }
  else if (countButton == 1)
  {
    digitalWrite(led, HIGH); //turns ON the button LED
    //Values are reset because a new state of the lamp starts
    previousRef = PREV_REF;
    flag = 0;
   
    //System is ON
  }
  else if (countButton == 3)
  {
    //System is turned OFF
    digitalWrite(led, LOW); //turns OFF the button LED
    lcd.clear(); //clears the text from the LCD screen
  }
  delay(5);

}

//**************************************************************************************************************************************************

double getDistance (int trigPin , int echoPin) {
  // this function get the distance and save the value as output the inputs are the pin of the  US sensor
  unsigned long duration = 0;
  double distance = 0;
  digitalWrite(trigPin, LOW); // Clears the trigPin
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);// Sets the trigPin on HIGH state for 10 micro seconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);// Reads the echoPin, returns the sound wave travel time in microseconds
  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance;
}


void motorPower (int motorPin , double power) {
  //Function that writes the values of the desired power on the pin of the motor.
  //The inputs are the pin to which the motor is attached and the desired power (number 0-100).


  double powerOut = (int)((power / 100) * 255); //gets the fraction of power in input, number between 0 and 1
  analogWrite(motorPin, powerOut); //gives the voltage to the motor, an integer 0-255, proportional to the fraction of the power in input, 20 mV each step

}
void ledPower (int ledPin , double intensity) {
  //Function that writes the values of the desired intensity on the pin of the tube LED light.
  //The inputs are the pin to wiich the led light is attached and the desired intensity (number 0-80).
  
  //Scaling the intensity
  if (intensity>intensUpperLim) intensity = intensUpperLim;
  else if (intensity<intensLowerLim) intensity = intensLowerLim;
  
  double powerOut = (int)(((intensity-intensLowerLim) / (intensUpperLim-intensLowerLim)) * 255); 
  //gets the fraction of intensity in input, number between 0 and 1
  analogWrite(ledPin, powerOut); //gives the voltage to the LED, an integer 0-255, 20 mV each unit
}




void PID (char region) {
  //This function calculates the control parameters of the feedback.
  //No inputs. Calculates and stores variables.
  double Kp, Ki, Kd;
  if (region == 'a') //controller parameters for the lower part of tube
  {
    Kp = Kpa;
    Ki = Kia;
    Kd = Kda;
  }
  else if (region == 'b') //controller parameters for the middle part of tube
  {
    Kp = Kpb;
    Ki = Kib;
    Kd = Kdb;
  }
  else  if (region == 'c') //controller parameters for the upper part of tube
  {
    Kp = Kpc;
    Ki = Kic;
    Kd = Kdc;
  }

  err = ref - ballPos; //current error
  //calculates the proportional part
  controlP = Kp * err;
  //calculates the integral part
  controlI = prevControlI + Ki * err * samplingTime; //integral-->area
  //calculates the derivative part
  controlD = Kd * (-ballPos + prevBallPos) / samplingTime; //neglecting the reference, because of infinitive derivatives on step changes

  //calculates the total control
  control = controlP + controlI + controlD;
  if (control > 100) {//Technically impossible to actuate, but necessary to saturate in program...
    control = 100;
    controlI = prevControlI; //...to prevent integrator wind-up
  }
  else if (control < 0) {//Technically impossible to actuate, but necessary to saturate in program...
    control = 0;
    controlI = prevControlI; //...to prevent integrator wind-up in opposite direction
  }
  prevControlI = controlI; //stores the integrator value for the next iteration
  prevErr = err; //stores the value of the error for the next iteration
}

void printInfoLCD()
{
  //Function used to print on the LCD screen all the characters containing the information about the current state of system:
  //Reference level, ball position and applied control, each 0-100 %
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SYSTEM INFORMATION:");
  lcd.setCursor(0, 1);
  lcd.print("handPos: ");
  lcd.print((double)ref, 1);
  lcd.print(" % ");
  lcd.setCursor(0, 2);
  lcd.print("ballPos: ");
  lcd.print((double)ballPos, 2);
  lcd.print(" % ");
  lcd.setCursor(0, 3);
  lcd.print("Control: ");
  lcd.print((double)control, 3);
  lcd.print(" % ");
}
void updateInfoLCD()
{
  //Function used to print on the LCD screen only the characters that determine the value of important current parameters of the system:
  //Reference level, ball position and applied control, each 0-100 %
  lcd.setCursor(9, 1);
  lcd.print((double)ref, 1);
  lcd.print(" % ");
  lcd.setCursor(9, 2);
  lcd.print((double)ballPos, 2);
  lcd.print(" % ");
  lcd.setCursor(9, 3);
  lcd.print((double)control, 3);
  lcd.print(" % ");
}
