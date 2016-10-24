
// Arduino Code for the Motor Speed Control Lab


// Standard
#ifdef ARDUINO
  #include <Print.h>
#else   // Photon
  #include "application.h"      // Should not be needed if file .ino or Arduino
  SYSTEM_THREAD(ENABLED);       // Make sure heat system code always run regardless of network status
#endif
#include "USB_Viewer.h"

// Test features
extern  const int   verbose         = 2;    // Debug, as much as you can tolerate (2)

// Likely User Modified Variables ******************************
unsigned long deltaT = 20000; // time between samples (usecs) 1000->50000


// End Likely User Modified Variables***************************


// Global
USB_Viewer* viewer;               // View plots at PC end of USB
boolean       switchFlag;
unsigned long scaleT      = 1;
int           monitorPin  = 8;  // for checking time sync

// For arduino and host interface, NO NEED TO MODIFY!
unsigned long transferDt = 20000; // usecs between host updates
int           angleSensorPin = A0;
int           pwmVoltagePin = A1;
int           motorVoltagePin = A2;
int           motorOutputPin = 9;  // Do not change this!!

// Arduino-specific DAC values.
int dacMax = 255; // Arduino dac is eight bits.
int adcMax = 1024;  //Arduino uses a 10 bit dac, 10th power of 2 = 1024
int adcCenter = 512; // 1024/2 = 512




// Variables to reset every time loop restarts.
int loop_counter;
int numSkip;      // Number of loops to skip between host updates


// Setup sets up pwm frequency (30khz), initializes past values.
int loopPin = 13;
boolean loopFlag;

void setup() {
  // Set up inputs
  pinMode(motorVoltagePin,  INPUT);
  pinMode(pwmVoltagePin,    INPUT);
  pinMode(angleSensorPin,   INPUT);

  // Set up PWM motor output pin, affects delay() function
  // Delay argument will be in 1/64 milliseconds.
  pinMode(motorOutputPin,OUTPUT);
  digitalWrite(motorOutputPin,LOW);
#ifdef ARDUINO
  setPwmFrequency(motorOutputPin, 1); // Set pwm to approx 30khz.
#endif

  // Set for fastest serial transfer.
  Serial.begin(230400);

  loop_counter  = 0;  // Initialize counter for status transfer
  switchFlag    = true;
  // Frequency Monitor
  pinMode(monitorPin, OUTPUT);
  digitalWrite(monitorPin,LOW);
  if (motorOutputPin == 5 || motorOutputPin == 6)
  {
    scaleT = 64;
  }
  else
  {
    scaleT = 1;
  }

  numSkip = max(int((transferDt+100)/deltaT),1);  // Number of loops between host transfers.


  String config_message_7_bytes = "&A~Desired~5&C&S~K_P~P~0~10~0.05&S~Direct~O~0~260~0.01&S~Desired~A~0~260~1&T~MotorSpeed~U1~0~260&T~ArmAngle~S1~-150~150&T~MotorCmd~U1~0~260&H~2&";
  String config_message = config_message_7_bytes; //for 7 bytes set the print
  viewer  = new USB_Viewer(config_message, deltaT, scaleT);

  // Diagnostic
  pinMode(loopPin,OUTPUT);
  loopFlag = false;

}


// Main code, runs repeatedly
void loop() {

  // Check if first time.
  viewer->startup();

  // Make sure loop start is deltaT microsecs since last start
  int headroom = viewer->timeSync();

  // Output square wave on monitor pin
  digitalWrite(monitorPin, switchFlag);
  switchFlag = !switchFlag;

  // Invert monitor
  //digitalWrite(loopPin,loopFlag);
  //loopFlag = !loopFlag;

  // User modifiable code between stars!!
  /***********************************************/
  // Measure the voltage across the motor and the voltage at the drive transistor.
  // Read each voltage three times and sum, and divide later. This averaging reduces noise.
  int VmotorADC = analogRead(motorVoltagePin);
  int VpwmADC   = analogRead(pwmVoltagePin);
  int VangleADC = analogRead(angleSensorPin) - adcCenter;

  // Compute the measured backEMF, which is proportional to motor speed, by
  // offsetting the motor voltage with the voltage across the 1/2 ohm resistor.
  // Convert to an integer between 0 -> 256 by dividing by six.
  float motorSpd = 0.5*float(VmotorADC - (VpwmADC - VmotorADC));
  float armAngle = 0.25*float(VangleADC);

  // Compute the error, which is the difference between desired and measured speed.
  float error = viewer->desired() - motorSpd;

  // Compute and limit the motor output command.
  int motorCmd  = int(viewer->Kp() * error + viewer->direct());
  motorCmd      = min(max(motorCmd, 0), dacMax);
  analogWrite(motorOutputPin,motorCmd);

  /***********************************************/

  //check for new parameter values
  viewer->serialEvent();
  if(viewer->stringComplete()) viewer->processString();

  // Transfer to host occasionally
  if (loop_counter % numSkip == 0) {
    loop_counter = 0;
    viewer->printStatus(int(motorSpd), int(armAngle), motorCmd, headroom);
  }

  // Increment the loop counter.
  loop_counter++;

}


#ifdef ARDUINO
void setPwmFrequency(int pin, int divisor)
{
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10)
  {
    switch(divisor)
    {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6)
    {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else
    {
        TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if(pin == 3 || pin == 11)
  {
    switch(divisor)
    {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
#endif
