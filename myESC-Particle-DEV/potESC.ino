/*
Controlling a servo position using a potentiometer (variable resistor)
by Dave Gutz

    Connections:
      ESC ------------------- Photon
        BLK ------------------- GND
        WHT ------------------- PWM Output (A4)
      ESC----------------- 3-wire DC Servomotor (stepper)
        Any three to any three
      POT---------------------Photon
        VHI ------------------VIN
        VLO ------------------GND
        WIPE -----------------Analog In A4
      Hardware Platform: Particle Photon
        ESC:
        Power Supply:
        Potentiometer:
        Motor:

  Reaquirements:
  Prime:
  1.  Manually sweep ESC command from min to max using pot.
  2.  Limit ESC command for safety using configurable parameters.
  3.  Initialize the ESC, which is built to accept min-max input on startup.
  Secondary:
  1.  USB status

  Tasks TODO:
  1.

  Revision history:
    31-Aug-2016   DA Gutz   Created

  Distributed as-is; no warranty is given.
*/


//Sometimes useful for debugging
//#pragma SPARK_NO_PREPROCESSOR
//
// Standard
#include "application.h"
SYSTEM_THREAD(ENABLED);      // Make sure heat system code always run regardless of network status
//
// Test features usually commented
//
// Disable flags if needed.  Usually commented
// #define DISABLE
//#define BARE_PHOTON                       // Run bare photon for testing.  Bare photon without this goes dark or hangs trying to write to I2C
//
// Usually defined
// #define USUALLY
//
// Constants always defined
// #define CONSTANT
#define PWM_PIN          A4                 // PWM output (A4)
#define POT_PIN          A0                 // Potentiometer input pin on Photon (A0)
//
// Dependent includes.   Easier to debug code if remove unused include files
//#include "math.h"

// Global variables
Servo               myservo;  // create servo object to control a servo
int                 throttle        = 0;    // Pot value, 0-179 degrees
int                 potValue        = 2872; // Dial raw value, 0-4096
int                 verbose         = 3;  // Debugging Serial.print as much as you can tolerate.  0=none

void setup()
{
  WiFi.disconnect();
  Serial.begin(9600);
  myservo.attach(PWM_PIN);  // attaches the servo.  Only supported on pins that have PWM
  pinMode(POT_PIN, INPUT);
  delay(10000);
  if (verbose>1) Serial.printf("\nCalibrating ESC...");
  while (throttle<179)
  {
    throttle = min(179, throttle+10);
    myservo.write(throttle);
    delay(20);
  }
  while (throttle>0)
  {
    throttle = max(0, throttle-10);
    myservo.write(throttle);
    delay(20);
  }
  if (verbose>1) Serial.printf("Done.\n");
  Serial.printf("To flash code to this device, push and hold both Photon buttons, release RESET until purple observed, then release.\n");

  WiFi.off();
  delay(5000);
}

void loop() {
// Interrogate pot; run fast for good tactile feedback
// my pot puts out 0- 4096 observed using Tinker
#ifndef BARE_PHOTON
  potValue    = analogRead(POT_PIN);
#endif
throttle = map(potValue, 0, 4096, 0, 179);
if (verbose>1) Serial.printf("Throttle=%ld\n", throttle);
myservo.write(throttle);                // sets the servo position according to the scaled value
delay(15);                              // waits for the servo to get there
}
