/*************************************************************************************************************************************
  Wireless Axes Unity Script
  Written by Jim Smiley & Siddhant Tandon
  Immersive Analytics Laboratory - Faculty of Information Technology - Monash University - https://ialab.it.monash.edu/
  Version 1.0 - Last Modified JULY 28 2020
**************************************************************************************************************************************/

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

const int fwd2 = 15; ////////////////////////define hardware pin numbers  ////6 MAY!!!!!!!!!!!!!!!!!
const int rvs2 = 33;
const int fwd1 = 14;
const int rvs1 = 32;
//const int led = 21;        // use this for simplified led behaviour, can write high or low, otherwise batOut pwm controls led.
const int encoder1 = 13;
const int encoder2 = 27;
const int buttonSwitch = 26;
const int batOut = 21;
const int topSwitch = 4;
const int batVoltage = 35; //////////////////////////////////////
bool button1;
bool button2;
const int pwmFreq = 25000;
int encoderValue;
int interrupt_cnt = 0;
int times[2];
bool toggle;
int count = 0;
int sliderOne = 0;
int sliderTwo = 0;
int oldSliderOne = 0;
int oldSliderTwo = 0;
String output;
bool scrollOne = false;
bool scrollTwo = false;
int snapIncrement = 30;
const int numReadings = 10;  // number of reads to average
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;
int readings2[numReadings];      // the readings from the analog input
int readIndex2 = 0;              // the index of the current reading
int total2 = 0;                  // the running total
int average2 = 0;
int fader1Val = 0;
int fader2Val = 0;
int fader1Old = 0;
int fader2Old = 0;
int sliderOneOffset = 0;
int sliderTwoOffset = 0;
int scrollSpeed = 30;
int bumpPeriod = 2;
int blinkCount = 0;
int  bat;
int oldEncoder = 0;
int finalA;
int finalB;
int  sliderOneTarget = 0;
int sliderTwoTarget = 0;
int joyStickMode;
bool moveOne = false;
bool moveTwo = false;
int indexCount = 0;
char toArray[13];
int oneTargetCount = 0;
int towTargetCount = 0;
uint32_t sliderOneStart; // these will time out the slider to val function if it stalls next to correct value.
uint32_t sliderOneEnd;
uint32_t twoTargetCount;
uint32_t sliderTwoEnd;
uint32_t sliderTwoStart;
uint32_t sliderMoveTwoTime;
int difference = 40;
bool followModeOne = false;
bool followModeTwo = false;
int LEDVal = 0;
int followDist = 0;
int jsMode;
bool joystickMode = false;
bool joystickModeOne = false;
bool joystickModeTwo= false;
bool steppedMode = false; // changed from true
int steppedDistance = 20;

// Encoder Counting Function- counts encoder rotation
// Inputs: NONE
// Outputs: Modifies the encoderValue global variable
void IRAM_ATTR easyenc() // interrupt was causing bluetooth crash. Needs IRAM_ATTR to run in ram, and method placed up top.  This should be nice and clean if encoder board hardware is ok.... Sid, this might get ca
{ // also removed sendoutput();

  int inputB = digitalRead(27);

  if (inputB == LOW)
  {

    if (encoderValue < 127) {
      encoderValue++;
    }
  }
  else
  {
    if (encoderValue > -127 )
    {
      encoderValue--;
    }
  }

}

void setup() {
  Serial.begin(115200);  // for serial coms with cable only... Helpful for troubleshooting without requiring changing serial port to bluetooth. Make sure baud rate is set in the ide as well. just need Serial.print(""); or Serial.println("");
  SerialBT.begin("RedAxis2"); //name your axis here - is what comes up in bluetooth devices. Bt serial does not need a baud rate.
  ledcSetup (4, pwmFreq, 8); // sets up pwm
  for (int i = 0; i < 4; i++)
  {
    ledcSetup(i, pwmFreq, 8); // 0 - 255 duty cycle
  }
  ledcAttachPin(fwd1, 0);
  ledcAttachPin(rvs1, 1);
  ledcAttachPin(fwd2, 2);
  ledcAttachPin(rvs2, 3);
  ledcAttachPin(batOut, 4); //
  //pinMode(led, OUTPUT);  // more pin setups
  pinMode(buttonSwitch, INPUT_PULLUP);
  pinMode(topSwitch, INPUT_PULLUP);
  //digitalWrite(led, HIGH);
  analogReadResolution(12); //  can use highher analog read resolutions but values will get very shaky.

  pinMode(encoder1, INPUT_PULLUP);
  pinMode(encoder2, INPUT_PULLUP);
  pinMode(fwd1, OUTPUT);
  pinMode(rvs1, OUTPUT);
  pinMode(fwd2, OUTPUT);
  pinMode(rvs2, OUTPUT);
  digitalWrite(fwd1, LOW);
  digitalWrite(fwd2, LOW);
  digitalWrite(rvs1, LOW);
  digitalWrite(rvs2, LOW);


  attachInterrupt(digitalPinToInterrupt(encoder1), easyenc, RISING); // attaches interrupt on encoder.
  wiggle(); // Causes the sliders to move around when setup is complete, helps to know when sliders are ready for connecting with computer
  wiggle();


}

void loop() {
  if (SerialBT.available()) {
    GetSerialVals();
  }

  if (moveOne && moveTwo)  // snap to both
  {

    SliderToVal(3);

  }
  else if (moveOne)
  {

    SliderToVal(1);

  }
  else if (moveTwo)
  {

    SliderToVal(2);

  }

  ReadAndAverageInputs();
  readsliders();
  //batLevel();
  setLED(LEDVal);
  readButtons();
  if (encoderValue != oldEncoder) // removed from interrupt because killed BT
  {
    SendOutput();
    oldEncoder  = encoderValue;
  }


  if (followModeOne)
  {
    difference = followDist - 128; // + encoderValue;
    sliderOneTarget = finalB - difference;
    SliderToVal(1);
  }
  else if (followModeTwo)
  {
    difference = followDist - 128; // + encoderValue;
    sliderTwoTarget = finalA - difference;
    SliderToVal(2);
  }


  if ((analogRead(A3) / 4 == 0 || analogRead(A3) / 4 == 1023) && joystickModeOne ) // joystick mode only at ends
  {

    snapBack(1);

  }
  if ((analogRead(A2) / 4 == 0 || analogRead(A2) / 4 == 1023) && joystickModeTwo)

  {
    snapBack(2);
  }
}

// Function to read serial values coming into the device from Unity
void GetSerialVals() {

  char c = (SerialBT.read()); // SerialBtreads in on character at a time. We're going to assign each char into an array until the new line character
  //Serial.print(c);
  toArray[indexCount] = c;   // Store character in char array.
  indexCount++;
  if (c == '\n')
  {
    indexCount = 0;

    char buffer1[2];  //sliderOneTargetVal
    char buffer2[4]; //sliderTwoTargetVal


    buffer1[0] = toArray[0];
    buffer1[1] = '\0';               // needs for atoi to split
    buffer2[0] = toArray[1];
    buffer2[1] = toArray[2];
    buffer2[2] = toArray[3];
    buffer2[3] = '\0';


    int modeRead = atoi(buffer1);
    int valueRead = atoi(buffer2);

    // 0  - sliderOne snap. 1 sliderTwo snap. 2. follow mode. 0-256 sliderOne, 256 - 512 sliderTwo, > 512 followmodeOff.  3 joystickModeToggle. 4. stepped mode, range of 10-128 else false. 6, 7 haptic pulses, 8 led.

    if (modeRead == 0)  // slider one snap to.
    {
      if (valueRead < 256 && valueRead >= 0) // send greater from unity to ignore
      {
        sliderOneTarget = valueRead;
        moveOne = true;
        sliderOneStart = millis();
        // followMode = false;
        //  jsMode = 0;
        //moveTwo = false;
      }
    }
    else if (modeRead == 1)  //slider two snap to
    {
      if (valueRead < 256 && valueRead >= 0) // send greater from unity to ignore
      {
        sliderTwoTarget = valueRead;
        moveTwo = true;   //Do this in update loop and call a moveslider function. set false once hits target.
        sliderTwoStart = millis();
        //  followMode = false;
        //  jsMode = 0;
        //Serial.println("Slider 2 Move");
        //moveOne = false;
      }
    }
    else if (modeRead == 2) // sets follow mode
    {
      if (valueRead <= 256 && valueRead >= 0) // send greater from unity to ignore
      {
        followDist = valueRead;
        followModeOne = true;
        followModeTwo = false;
        joystickMode = false;
        joystickModeOne = false;
        joystickModeTwo = false;
      }
      else if (valueRead <= 512 && valueRead >= 257) // send greater from unity to ignore
      {
        followDist = valueRead - 256;
        followModeOne = false;
        followModeTwo = true;
        joystickMode = false;
        joystickModeOne = false;
        joystickModeTwo = false;
      }
      else
      {
        followModeOne = false;
        followModeTwo = false;
        ledcWrite(0, 0);
        ledcWrite(1, 0);
        ledcWrite(2, 0);
        ledcWrite(3, 0);
      }
    }
    else if (modeRead == 3 && valueRead == 1)
    {
      joystickModeOne = true;
     // joystickModeTwo = false;
      followModeOne = false;
      followModeTwo = false;
    }
    else if (modeRead == 3 && valueRead == 2)
    {
      //joystickModeOne = false;
      joystickModeTwo = true;
      followModeOne = false;
      followModeTwo = false;
    }
     else if (modeRead == 3 && valueRead == 3)
    {
      joystickModeOne = true;
      joystickModeTwo = true;
      followModeOne = false;
      followModeTwo = false;
    }
     else if (modeRead == 3 && valueRead == 4)
    {
      joystickModeOne = false;
      //joystickModeTwo = true;
      followModeOne = false;
      followModeTwo = false;
    }
    else if (modeRead == 3 && valueRead == 5)
    {
      //joystickModeOne = false;
      joystickModeTwo = false;
      followModeOne = false;
      followModeTwo = false;
    }
     else if (modeRead == 3 && valueRead == 6)
    {
      joystickModeOne = false;
      joystickModeTwo = false;
      followModeOne = false;
      followModeTwo = false;
    }
    else if (modeRead == 3 && valueRead == 0) // to do value 1 is on
    {
      joystickMode = false;

    }
    else if (modeRead == 4)
    {
      if (valueRead >= 10 && valueRead <= 128)
      {
        steppedMode = true;
        steppedDistance = valueRead;
      }
      else
      {
        steppedMode = false;
      }
      //  joystickMode = false;
      // followMode = false;
    }

    else if (modeRead == 6)  // haptic pulse sliderOne
    {
      if (valueRead > 0) bump(1, bumpPeriod , valueRead);
      valueRead = 0;
      //jsMode = 0;
    }
    else if (modeRead == 7) // haptic pulse sliderTwo
    {
      if (valueRead > 0) bump(2, bumpPeriod , valueRead);
      valueRead = 0;
      //jsMode = 0;
    }
    else if (modeRead == 8)
    {
      LEDVal = valueRead;
    }

  }
}

// Function to snap sliders to desired positions
// Inputs: SliderNo- which slider do you wish to toggle (1 for slider 1, 2 for slider 2, 3 for both)
//         sliderOneTarget/sliderTwoTarget - sets the desired slider position
//         finalA and finalB- analog read values informing of the positions of the sliders
//         sliderOneEnd/sliderOneStart/sliderTwoEnd/sliderTwoStart - timing varaibles to see how long motors have been active for
//         sliderOneCount/sliderTwoCount - How long the slider motors have been active
// Outputs: No variables- causes sliders to move
void SliderToVal(int sliderNo)
{
  int differenceScale = 15;

  if (sliderNo == 1 || sliderNo == 3)
  {
    if (sliderOneTarget < finalA )
    {
      int sliderDifference = finalA - sliderOneTarget;
      if (sliderDifference > 10)
      {
        ledcWrite(1, 0);
        ledcWrite(0, 255);
      }
      else
      {
        ledcWrite(1, 0);
        ledcWrite(0, sliderDifference * differenceScale + 70 );
      }
    }
    else if (sliderOneTarget > finalA )
    {
      int sliderDifference =  sliderOneTarget - finalA ;
      if (sliderDifference > 10)
      {
        ledcWrite(0, 0);
        ledcWrite(1, 255);
      }
      else
      {
        ledcWrite(0, 0);
        ledcWrite(1, sliderDifference * differenceScale + 70);
      }
    }
    else
    {
      sliderOneEnd = millis();
      oneTargetCount++;
      ledcWrite(1, 0);
      ledcWrite(0, 0 );
      if (oneTargetCount == 20 || sliderOneEnd - sliderOneStart > 500 )
      {
        moveOne = false;
        oneTargetCount = 0;
      }
    }
  }

  if (sliderNo == 2 || sliderNo == 3)
  {
    if (sliderTwoTarget < finalB )
    {
      int sliderDifference = finalB - sliderTwoTarget;
      if (sliderDifference > 10)
      {
        ledcWrite(2, 0);
        ledcWrite(3, 255);
      }
      else
      {
        ledcWrite(2, 0);  // might be wrong
        ledcWrite(3, sliderDifference * differenceScale + 70 );
      }
    }
    else if (sliderTwoTarget > finalB )
    {
      int sliderDifference =  sliderTwoTarget - finalB ;
      if (sliderDifference > 10)
      {
        ledcWrite(3, 0);  // might be wrong
        ledcWrite(2, 255);
      }
      else
      {
        ledcWrite(3, 0);  // might be wrong
        ledcWrite(2, sliderDifference * differenceScale + 70);
      }
    }
    else
    {
      sliderTwoEnd = millis();
      twoTargetCount++;
      ledcWrite(3, 0);
      ledcWrite(2, 0 );
      if (twoTargetCount == 20 || sliderTwoEnd - sliderTwoStart > 500 )
      {
        moveTwo = false;
        twoTargetCount = 0;
      }
    }

  }
}



// Causes the LED to flash, note LED out pin is on pin 4
void blinkLed()
{
  // blinkCount++;
  ledcWrite(4, 0 );
  if (blinkCount > 100000)
  {
    blinkCount == 0;
  }
  if (blinkCount % 10000  == 0)
  {
    ledcWrite(4, map(bat, 230, 273, 10, 255 ) );
  }
  if (blinkCount % 20000  == 0)
  {
    ledcWrite(4, 0 );
  }
}

// Function to read the button values
// Inputs: buttonSwitch (encoder button), topSwicth (top button)
// Function causes a status change to be sent to Unity if buttons change state
void readButtons()
{
  //Serial.println(digitalRead(buttonSwitch));
  //delay(10);
  if (digitalRead(buttonSwitch) != button1)
  {
    button1 = digitalRead(buttonSwitch);
    SendOutput();
  }
  if (digitalRead(topSwitch) == LOW)
  {
    button2 = 1;
    SendOutput();
    delay(200);
    button2 = 0;
    SendOutput();
  }
}

// Set Led brightness
// Input is val- the 0-255 LED desired brightness
void setLED(int val)
{
  ledcWrite(4, val);
}

void batLevel()   // writes led brightness based on battery charge. Also, power switch must be on to charge battery!
{
  bat = analogRead(batVoltage);
  if (button2) {
    ledcWrite(4, map(bat, 230, 273, 10, 255 ) );
  }
  else
  {
    blinkLed();
  }
}

// Reads the positions of the sliders, if we want stepped axes, this also causes small haptic pulses when we reach steps
// also sends position data back to Unity if the positions change
void readsliders()   //
{
  int ReadSliderOne = fader1Val;
  int ReadSliderTwo = fader2Val;
  if (ReadSliderOne != oldSliderOne && ReadSliderOne % 4 == 0 )
  {

    //bump(1, 2, 200);
    sliderOne = ReadSliderOne;
    oldSliderOne = sliderOne;
    if (!steppedMode)
    {
      SendOutput();
    }
    else
    {
      if (finalA % steppedDistance == 0 && finalA > 0 && finalA < 256) // stepped output but ignore ends.
      {
        bump(1, 4, 200);
        SendOutput();
      }
    }


  }
  if (ReadSliderTwo != oldSliderTwo && ReadSliderTwo % 4 == 0)
  {
    // bump(2, 2,200);
    sliderTwo = ReadSliderTwo;
    oldSliderTwo = sliderTwo;
    if (!steppedMode)
    {
      SendOutput();
    }
    else
    {
      if (finalB % steppedDistance == 0 && finalB > 0 && finalB < 256)
      {
        bump(2, 4, 200);
        SendOutput();
      }
    }

  }
}

void ReadAndAverageInputs() {
  total = total - readings[readIndex];
  total2 = total2 - readings2[readIndex];
  readings[readIndex] = (analogRead(A3) / 4) * -1 ;
  readings2[readIndex] = (analogRead(A2) / 4) * -1 ;
  total = total + readings[readIndex];
  total2 = total2 + readings2[readIndex];
  readIndex ++;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  fader1Val =  (total / numReadings);
  fader2Val = (total2 / numReadings);
  finalA = fader1Val / 4 + 255;
  finalB = fader2Val / 4 + 255;
  //Serial.println(fader1Val);
}

// Sends all relevants variables corresponding to positions of sliders/statues of buttons
// to Unity over bluetooth
void SendOutput()
{
  SerialBT.print(finalA);


  SerialBT.print (" ");


  SerialBT.print(finalB);


  SerialBT.print (" ");
  SerialBT.print(encoderValue);
  SerialBT.print (" ");
  SerialBT.print(button1);
  SerialBT.print (" ");
  SerialBT.print(button2);
  SerialBT.println("");
  delay(1);

}

// Thsi function is used in the joystick control mode. If the slider position
// is near the ends of the rnages, data is sent to unity, and the sliders give a tapping (scrolling)
// sensation
void snapBack(int slider)
{
  int delayPeriod = 30; // in ms. increase this delay if it is sending too fast in joystick mode
  if (slider == 1) {
    while ((analogRead(A3) / 4) < 10 && joystickModeOne)
    {
       if (SerialBT.available()) { // check if joystcik has turned off
    GetSerialVals();
  }
  if (!joystickModeOne)
  {
    digitalWrite(fwd1, LOW);
    digitalWrite(rvs1, LOW);
    break;
  }
     else
     { scrollOne = true;
      digitalWrite(fwd1, HIGH);
      digitalWrite(rvs1, LOW);
     bump(1, scrollSpeed, 245);
      finalA ++;
      SendOutput();
      delay(delayPeriod );
    }
    }

    while ((analogRead(A3) / 4) > 1013&& joystickModeOne)
    {
        if (SerialBT.available()) { // check if joystcik has turned off
    GetSerialVals();
  }
  if (!joystickModeOne)
  {
    digitalWrite(fwd1, LOW);
    digitalWrite(rvs1, LOW);
    break;
  }
 else
  {
      scrollOne = true;
      finalA --;
      digitalWrite(fwd1, LOW);
      digitalWrite(rvs1, HIGH);
     bump(3, scrollSpeed, 245);
      SendOutput();
      delay(delayPeriod );
  }
    }
    digitalWrite(fwd1, LOW);
    digitalWrite(rvs1, LOW);
    delay(300); // was 300
    scrollOne = false;
    sliderOneOffset = sliderOne;

  }
  else if (slider == 2)
  {
    while ((analogRead(A2) / 4) < 10 && joystickModeTwo) // was 200 and 800  //Upper graph limitg
    {
       if (SerialBT.available()) { // check if joystcik has turned off
    GetSerialVals();
  }
  if (!joystickModeTwo)
  {
    digitalWrite(fwd2,  LOW);
      digitalWrite(rvs2, LOW);
      //setLED(0);
      break;
  }
  else{
      scrollTwo = true;
      finalB ++;
      digitalWrite(fwd2, HIGH);
      digitalWrite(rvs2, LOW);
      bump(2, scrollSpeed, 245);
      SendOutput();
      delay(delayPeriod );
  }
    }
    while ((analogRead(A2) / 4) > 1013&& joystickModeTwo)
    {
      if (SerialBT.available()) { // check if joystcik has turned off
    GetSerialVals();
  }
  if (!joystickModeTwo)
  {
    digitalWrite(fwd2,  LOW);
      digitalWrite(rvs2, LOW);
      break;
  }
  else{
      scrollTwo = true;
      digitalWrite(fwd2, LOW);
      digitalWrite(rvs2, HIGH);
      finalB --;
      bump(4, scrollSpeed, 245);
      SendOutput();
      delay(delayPeriod );
  }
    }
    digitalWrite(fwd2, LOW);
    digitalWrite(rvs2, LOW);
    delay(300);
    scrollTwo = false;
    sliderTwoOffset = sliderTwo;


  }
}

// Function to provide haptic feedback
// Inputs: slider (which slider the feedback is to be provided on)
//          period (duration of the bump
//          pwm (how powerful the haptic bumps last for
void bump(int slider, int period, int pwm) // bumps for stepped mode.
{
  if (slider == 1 )
  {
    // digitalWrite(fwd1, HIGH);
    //digitalWrite(rvs1, LOW);
    ledcWrite(0, pwm);
    ledcWrite(1, 0);
    delay(period);
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    // digitalWrite (fwd1, LOW);
    //digitalWrite(rvs1, LOW);
  }
  else if (slider == 2)
  {
    //digitalWrite(fwd2, LOW);
    //digitalWrite(rvs2, HIGH);
    ledcWrite(2, 0);
    ledcWrite(3, pwm);
    delay(period);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    digitalWrite (fwd2, LOW);
    digitalWrite(rvs2, LOW);
  }
  else if (slider == 3) // backwards 1
  {
    //digitalWrite(fwd1, LOW);
    //digitalWrite(rvs1, HIGH);
    ledcWrite(0, 0);
    ledcWrite(1, pwm);
    delay(period);
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    digitalWrite (fwd1, LOW);
    digitalWrite(rvs1, LOW);
  }
  else if (slider == 4) // backwards 1
  {
    //digitalWrite(fwd2, HIGH);
    //digitalWrite(rvs2, LOW);
    ledcWrite(2, pwm);
    ledcWrite(3, 0);
    delay(period);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    digitalWrite (fwd2, LOW);
    digitalWrite(rvs2, LOW);
  }
}

// Function to wiggle sliders- used on bootup of the microcontroller
void wiggle()
{
  digitalWrite(fwd1, HIGH);
  digitalWrite(rvs1, LOW);
  digitalWrite(fwd2, HIGH);
  digitalWrite(rvs2, LOW);
  //  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(fwd1, LOW);
  digitalWrite(rvs1, HIGH);
  digitalWrite(fwd2, LOW);
  digitalWrite(rvs2, HIGH);
  //digitalWrite(led, HIGH);
  delay(300);
  digitalWrite(fwd1, LOW);
  digitalWrite(rvs1, LOW);
  digitalWrite(fwd2, LOW);
  digitalWrite(rvs2, LOW);
  // digitalWrite(led, HIGH);
}
