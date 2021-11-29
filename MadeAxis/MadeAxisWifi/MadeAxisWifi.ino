/*************************************************************************************************************************************
  Wireless Axes 
  Written by Jim Smiley & Siddhant Tandon
  Immersive Analytics Laboratory - Faculty of Information Technology - Monash University - https://ialab.it.monash.edu/
  Version 2.0 -   Wifi version 29/11/2021
 last modified 26/11/2021
 Needs the following libraries, and many thanks to :
 https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
 and https://github.com/CNMAT/OSC
**************************************************************************************************************************************/

/*
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

*/
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

char ssid[] = "";          // wifi network name - ESP32 does not support 5Ghz, 2.5 only.
char pass[] = "";    
// your network password
char ssid2[] = "";          //Alternative network - hold button down at turn on to connect to this instead
char pass2[] = "";  
/*
 * To minimise UDP latency - the Esp32 has to be on 2.5ghz network due to 5ghz unsupported, but try and keep other devices on 5ghz instead.
 * Address Hololens directly, does not seem to like universal broadcast  - or could be 5ghz issue also - you can hardcode the IP address, but suggest using a small client running on unity to forward OSC directly
 */

WiFiUDP Udp;                               
const IPAddress outIp (255,255,255,255);//255,255,255,255 is universal broadcast to all on network.
const unsigned int outPort = 9000;          // remote port to receive OSC
const unsigned int localPort = 9001;        // local port to listen for OSC packets

OSCErrorCode error;

String AxisName = "BlueAxis1"; // Name your MADEaxis
const int fwd2 = 15; ////////////////////////define hardware pin numbers  
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
long encoderValue;
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
long oldEncoder = 0;
int finalA;
int finalB;
int  sliderOneTarget = 0;
int sliderTwoTarget = 0;
//int joyStickMode;
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
//int jsMode;
//bool joystickMode = false;
bool joystickA = false;
bool joystickB= false;
bool steppedA = false; 
bool steppedB = false;
int steppedDistanceA = 20;
int steppedDistanceB = 20;
int slowedEncoder = 0; // encoder interrupt can bounce and trigger faster than update and sendOutput, so can jump. This increments only as fast as data can be sent
long idleTime1;
long idleTime2;//To get the twitches out of the slider between values if it is sitting still. Need to move slider by greater than 1 to reset, otherwise will ignore twitches of just 1
int idleTimeThreshold = 800; // time in ms for idle to de-twitch sliders
int joystickThreshold = 20;



// Encoder Counting Function- counts encoder rotation
// Inputs: NONE
// Outputs: Modifies the encoderValue global variable
void IRAM_ATTR easyenc() // interrupt was causing bluetooth crash. Needs IRAM_ATTR to run in ram, and method placed up top.  This should be nice and clean if encoder board hardware is ok....  
{ 
  
  int inputB = digitalRead(27);
  if (inputB == LOW)
  {
      encoderValue++;
  }
  else
  {
    
      encoderValue--;     

  }
  
}

void setup() {
  Serial.begin(115200); // for serial coms with cable only... Helpful for troubleshooting without requiring changing serial port to bluetooth. Make sure baud rate is set in the ide as well. just need Serial.print(""); or Serial.println("");
 // SerialBT.begin(axisName); //is what comes up in bluetooth devices. Bt serial does not need a baud rate.
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
setLED(255);
if (digitalRead(topSwitch) == HIGH)
{
  WiFi.begin(ssid, pass);
  Udp.begin(localPort);  
  wiggle();
  setLED(0);
}
else
{
  WiFi.begin(ssid2, pass2);
  Udp.begin(localPort); 
  wiggle();
  setLED(0);
  wiggle();
  setLED(255);
  wiggle();
  setLED(0);
}


}
void CheckSnaps()
{
   if (moveOne&&!followModeOne)
  {
    SliderToVal(1);
  }
   if (moveTwo && !followModeOne)
  {
    SliderToVal(2);
  }  
}

void loop() {


  

  ReadAndAverageInputs();
  //setLED(LEDVal);
  readButtons();
  UdpRead();
  CheckSnaps();
  CheckEncoder();
  if (joystickA||joystickB)
  {
  CheckJoystick();
  }

  if (followModeOne)
  {
    difference = followDist; //   
    sliderOneTarget = finalB - difference;// - difference;
   // Serial.println (sliderOneTarget);
    SliderToVal(1);
  }
   if (followModeTwo)
  {
    difference = followDist; // 
    sliderTwoTarget = finalA- difference;
    SliderToVal(2);
  }

/*
 
  */
}
void CheckJoystick()
{
   if ((analogRead(A3) / 4 == 0 || analogRead(A3) / 4 == 1023) && joystickA ) // joystick mode only at ends
  {

   snapBack(1);

  }
  if ((analogRead(A2) / 4 == 0 || analogRead(A2) / 4 == 1023) && joystickB)

  {
    snapBack(2);
  }
}
  
void UdpRead()
{
  String concatName = '/'+AxisName;  // Strings all need to be converted to c strings using c_str() to match osc addresses.
  String haptic = concatName + "/haptic";
  String snap = concatName + "/snap";
  String led = concatName + "/led";
  String followMode = concatName + "/followMode";
  String stepped = concatName + "/stepped";
  String joystick = concatName + "/joystick";
  OSCMessage msg(concatName.c_str());
  
  int size = Udp.parsePacket();
 
  if (size > 0) {
    while (size--) {
        
      msg.fill(Udp.read());
    }
    if (!msg.hasError() && msg.match(concatName.c_str())>0)
    {     
        ////Receive Haptic pulse
        if (msg.fullMatch(haptic.c_str())) // In your application send ("/AxisName/haptic") (not literally"AxisName", whatever you have set AxisName as)and int slider(1 or 2), int period (suggest 3 - 5), int value(0 - 255)
        {
            bump(msg.getInt(0), msg.getInt(1), msg.getInt(2));
        }

        // receive snapto
         if (msg.fullMatch(snap.c_str()))  // slider, slider target
        {          
          SnapReceive(msg.getInt(0), msg.getInt(1));
        } 
        /////receive steppedMode
          if (msg.fullMatch(stepped.c_str()))  // slider, stepped distance
        {          
          SteppedReceive(msg.getInt(0), msg.getInt(1));
        } 

          /////receive steppedMode
          if (msg.fullMatch(joystick.c_str()))   
        { 
          JoystickReceive(msg.getInt(0), msg.getInt(1)); // slider, 0 = off, anything else is on.
        } 

          if (msg.fullMatch(followMode.c_str()))  
        { 
          FollowReceive(msg.getInt(0), msg.getInt(1)); // slider, follow distance. Send 255 to turn off.
        } 
         if (msg.fullMatch(led.c_str()))  // ledValue
        { 
          setLED(msg.getInt(0)); 
     
        } 
       
      }
    } 
  }
void FollowReceive(int slider, int distance)  
/* Cheat in your application if using follow mode, by ignoring the actual value being sent by the following slider, and just locking it to the required offset from the value. 
 *  Both sliders Can be set to follow mode simultaneously, but it must be the same distance!And they will be twitchy. Potential upgrade is to add capacitive sensing to each knob to identify which one is being manipulated.
 */
{
  if (slider ==1)
  {
    if (distance ==255)
    {
      followModeOne = false;
    }
    else
    {
      
    followModeOne = true;
   // followModeTwo = false;
    followDist = distance;
    }
  }
   if (slider ==2)
  {
    if (distance ==255)
    {
      followModeTwo = false;
    }
    else
    {
      
   // followModeOne = false;
    followModeTwo = true;
    followDist = distance;
    }
  }
  
}

void JoystickReceive(int slider, int active)
{
  bool setActive = true;  // had issues mixing bool and integer
  if (active ==0)
  {   
    setActive = false;
  }
  if (slider ==1)
  {
    joystickA = setActive;
 
  }
  else if (slider ==2)
  {
    joystickB = setActive;
  }
  
}

void SnapReceive(int slider, int target)  // Just sets values, as SnaptoVal is called by update while slider does not equal the target value 
{
  //Serial.println("receivedSnap");
  if (slider ==1)
  {
    sliderOneTarget = target;
    moveOne = true;
    sliderOneStart = millis();
  }
  else if (slider ==2)
  {
    sliderTwoTarget = target;
    moveTwo = true;   
    sliderTwoStart = millis();
  }
    
}
  void SteppedReceive(int slider, int distance)  // send distance 0 to turn off step
  {
    if (slider ==1)
    {
      if (distance ==0)
      {
        steppedA = false;
      }
      else
      {
        steppedA = true;
        steppedDistanceA = distance;
      
      }
    }
    if (slider ==2)
    {
      if (distance ==0)
      {
        steppedB = false;
      }
      else
      {
       steppedB = true;
        steppedDistanceB = distance;
      }
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

void CheckEncoder()  // see if encoder interrupt has changed the encoder. Need to keep interrupts as simple as possible, so everything done here instead
{  
  if (encoderValue > oldEncoder) 
  {
    slowedEncoder ++;
    SendOutput();   
  }
  else if (encoderValue < oldEncoder)
  {
    slowedEncoder --;
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
   // blinkLed();
  }
}



void ReadAndAverageInputs() {
  bool isIdle1 = false;
  bool isIdle2 = false;
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

  int dif1 = finalA - oldSliderOne;
  int dif2 = finalB - oldSliderTwo;
  dif1 = abs(dif1);  // abs gets absolute value - best to avoid maths in abs function, so looks weird
  dif2 = abs (dif2);
  if (dif1>=2)
  {
    idleTime1 = millis(); //resets idle time
  }
   if (dif2>=2)
  {
    idleTime2 = millis();
  }
  if (millis() - idleTime1 > idleTimeThreshold) 
  {
    isIdle1 = true;
  }

  
  if (millis() - idleTime2 > idleTimeThreshold)
  {
    isIdle2 = true;
  }
  
  if (finalA != oldSliderOne && !moveOne && !isIdle1) 
  {
    //idleTime = millis();
   
       
    
    if (!steppedA) 
    {
    SendOutput();
    
    }
    else
    {
       if (finalA % steppedDistanceA == 0 )//&& finalA > 0 && finalA < 256) // stepped output but ignore ends //oscillates at ends?.
      {
        bump(1, 4, 200);
        SendOutput();
       
      }
      
    } 

  }
   if (finalB != oldSliderTwo && !moveTwo && !isIdle2) 
  {
    
    if (!steppedB) 
    {
    SendOutput();
    }
    else
    {
       if (finalB % steppedDistanceB == 0 )//&& finalA > 0 && finalA < 256) // stepped output but ignore ends //oscillates at ends?.
      {
        bump(2, 4, 200);
        SendOutput();
      }
    } 
   }

  
}
void snapBack(int slider)  /// Joystick mode - snap back from ends
{
  int delayPeriod = 30; // in ms. increase this delay if it is sending too fast in joystick mode
  if (slider == 1) {
    while ((analogRead(A3) / 4) < 10 && joystickA)
    {           
    
     ledcWrite(0, 255);
      ledcWrite(1, 0);
      delay(scrollSpeed);
      ledcWrite(0, 0);
     ledcWrite(1, 0);
      finalA ++;
      SendOutput();
      delay(delayPeriod );
      digitalWrite(fwd1, LOW);
      digitalWrite(rvs1, LOW);    
    }
    

    while ((analogRead(A3) / 4) > 1013&& joystickA)
    {
     // scrollOne = true;
      finalA --;
      
      ledcWrite(0, 0);
    ledcWrite(1, 255);
    delay(scrollSpeed);
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    digitalWrite (fwd1, LOW);
    digitalWrite(rvs1, LOW);
    
      SendOutput();
      delay(delayPeriod );
    }
    
    digitalWrite(fwd1, LOW);
    digitalWrite(rvs1, LOW);
    delay(300); // was 300
    //scrollOne = false;
    //sliderOneOffset = sliderOne;

  }
  else if (slider == 2)
  {
    while ((analogRead(A2) / 4) < 10 && joystickB) // was 200 and 800  //Upper graph limitg
    {    
  
    //
      finalB ++;
      
     // bump(2, scrollSpeed, 245);
     ledcWrite(2, 0);
    ledcWrite(3, 255);
    delay(scrollSpeed);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    digitalWrite (fwd2, LOW);
    digitalWrite(rvs2, LOW);
      SendOutput();
      delay(delayPeriod );  
    }
    digitalWrite(fwd2,  LOW);
    digitalWrite(rvs2, LOW);
    while ((analogRead(A2) / 4) > 1013&& joystickB)
    {
     // scrollTwo = true;
     
      finalB --;
      //bump(4, scrollSpeed, 245);
        ledcWrite(2,255);
    ledcWrite(3, 0);
    delay(scrollSpeed);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    digitalWrite (fwd2, LOW);
    digitalWrite(rvs2, LOW);
      SendOutput();
      delay(delayPeriod );  
    }
    digitalWrite(fwd2, LOW);
    digitalWrite(rvs2, LOW);
    delay(300);
    //scrollTwo = false;
   // sliderTwoOffset = sliderTwo;


  }
}



void SendOutput()
{
   String concatName = '/'+AxisName;
    OSCMessage msg(concatName.c_str());
      msg.add(finalA);
      msg.add(finalB);
      msg.add(slowedEncoder);
      msg.add(button1);
      msg.add(button2);
     

   Udp.beginPacket(outIp, outPort);
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
      oldSliderTwo =  finalB;
       oldSliderOne = finalA; 
       oldEncoder  = encoderValue;
      //delay(50);
      delay(8);
  
       

}


// Function to provide haptic feedback
// Inputs: slider (which slider the feedback is to be provided on)
//          period (duration of the bump
//          pwm (how powerful the haptic bumps last for
void bump(int slider, int period, int pwm) // bumps for stepped mode.
{
  if (slider == 1 )
  {    
    ledcWrite(0, pwm);
    ledcWrite(1, 0);
    delay(period);
    ledcWrite(0, 0);
    ledcWrite(1, pwm);
    delay(period);
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    
  }
  else if (slider == 2)
  {
    ledcWrite(2, 0);
    ledcWrite(3, pwm);
    delay(period);
    ledcWrite(2, pwm);
    ledcWrite(3, 0);
    delay(period);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
   }
 
  
}


// Function to snap sliders to desired positions
// Inputs: SliderNo- which slider do you wish to toggle (1 for slider 1, 2 for slider 2, 3 for both)
//         sliderOneTarget/sliderTwoTarget - sets the desired slider position
//         finalA and finalB- analog read values informing of the positions of the sliders
//         sliderOneEnd/sliderOneStart/sliderTwoEnd/sliderTwoStart - timing varaibles to see how long motors have been active for - not currently used todo: timeout function so motor is released if stalled just off the value.
//         sliderOneCount/sliderTwoCount - How long the slider motors have been active
// Outputs: No variables- causes sliders to move
// turns the motor pwm down down the closer it gets. Can be tricky to avoid self oscillation depending on slider friction.. balance difference scale and the offset for pwm to fade out.
// Cheat in your application if using follow mode, by ignoring the actual value being sent by the following slider, and just locking it to the required offset from the value.
void SliderToVal(int sliderNo)
{
  int differenceScale = 4;
 
  if (sliderNo == 1 || sliderNo == 3)
  {
    int quickerRead = ((analogRead(A3) / 4) * -1)/4 +255 ;  // Faster but dirtier than the filtered slider value in order to avoid oscillation
    //Serial.println(quickerRead);
    if (sliderOneTarget < quickerRead )
    {
      int sliderDifference = quickerRead - sliderOneTarget;
      if (sliderDifference > 30)
      {
        ledcWrite(1, 0);
        ledcWrite(0, 255);
      }
     
      else if (sliderDifference > 2)
      {
        ledcWrite(1, 0);
        ledcWrite(0, sliderDifference * differenceScale + 138 ); 
        /*// turns the motor pwm down down the closer it gets. Can be tricky to avoid self oscillation vs getting to the right point depending on slider friction.. 
         *balance difference scale and the offset for pwm to fade out.
         */
  
      }
       else // has equalled
    {
      sliderTwoEnd = millis();
      twoTargetCount++;
      ledcWrite(1, 0);
      ledcWrite(0, 0 );
      
    }
    }
    else if (sliderOneTarget > quickerRead)
    {
      int sliderDifference =  sliderOneTarget - quickerRead ;
      if (sliderDifference > 30)
      {
        ledcWrite(0, 0);
        ledcWrite(1, 255);
      }
     
      else if (sliderDifference > 2)
      {
        ledcWrite(0, 0);
        ledcWrite(1, sliderDifference * differenceScale +138);
    
      }
      else
      {
         sliderOneEnd = millis();
    // Serial.print("hit target");
    //  Serial.println(oneTargetCount);
      oneTargetCount++;
      ledcWrite(1, 0);
      ledcWrite(0, 0 );
      }
    }
    else
    {
     
      if (oneTargetCount == 20 || sliderOneEnd - sliderOneStart > 500 ) // kills oscillation when snapping, not so easy in follow mode
      {
        moveOne = false;
        oneTargetCount = 0;
      }
    }
  }

  if (sliderNo == 2 || sliderNo == 3)
  {
    int quickerRead = ((analogRead(A2) / 4) * -1)/4 +255 ;
    if (sliderTwoTarget < quickerRead )
    {
      int sliderDifference = quickerRead - sliderTwoTarget;
      if (sliderDifference > 30)
      {
        ledcWrite(2, 0);
        ledcWrite(3, 255);
      }
     else if (sliderDifference > 2)
      {
        ledcWrite(2, 0);  
        ledcWrite(3, sliderDifference * differenceScale + 138 );
      }
      else
      {
         sliderTwoEnd = millis();
      twoTargetCount++;
      ledcWrite(3, 0);
      ledcWrite(2, 0 );
        
      }
    }
    else if (sliderTwoTarget > quickerRead )
    {
      int sliderDifference =  sliderTwoTarget - finalB ;
      if (sliderDifference > 30)
      {
        ledcWrite(3, 0);  
        ledcWrite(2, 255);
      }
      else if (sliderDifference > 2)
      {
        ledcWrite(3, 0);  
        ledcWrite(2, sliderDifference * differenceScale + 138);
      }
    }
    else // has equalled
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

// Function to read serial values coming into the device from Unity
/*
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
// Reads the positions of the sliders, if we want stepped axes, this also causes small haptic pulses when we reach steps
// also sends position data back to Unity if the positions change
void readsliders()   //
{
  int ReadSliderOne = fader1Val;
  int ReadSliderTwo = fader2Val;
  if (ReadSliderOne != oldSliderOne && ReadSliderOne % 4 == 0 )
  {
    sliderOne = ReadSliderOne;
    oldSliderOne = sliderOne;
    if (!steppedMode && !moveOne)
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

*/
