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
#include "application.h"      // Should not be needed if file .ino
SYSTEM_THREAD(ENABLED);       // Make sure heat system code always run regardless of network status
#include "math.h"
#include "analyzer.h"
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
#define CONTROL_DELAY    1000UL             // Control law wait (), micros
#define FR_DELAY         5000000UL          // Time to start FR, micros
#define LED_PIN          D7                 // Status LED
#define PUBLISH_DELAY    40000UL            // Time between cloud updates (), micros
#define READ_DELAY       1000UL             // Sensor read wait (1000, 100 for stress test), micros
#ifndef BARE_PHOTON
  #define FILTER_DELAY   1000UL              // In range of tau/4 - tau/3  * 1000, micros
#else
  #define FILTER_DELAY   1000UL              // In range of tau/4 - tau/3  * 1000, micros
#endif
//
// Dependent includes.   Easier to debug code if remove unused include files
#include "myFilters.h"

// Global variables
LagTustin*          throttleFilter1;         // Exponential rate lag filter
LagTustin*          throttleFilter2;         // Exponential rate lag filter
FR_Analyzer*        analyzer;                // Frequency response analyzer
Servo               myservo;  // create servo object to control a servo
int                 numTimeouts     = 0;    // Number of Particle.connect() needed to unfreeze
int                 potValue        = 2872; // Dial raw value, 0-4096
const  double       tau             = 0.1;  // Filter time constant, sec
double              throttle        = 0;    // Pot value, 0-179 degrees
double              throttle1       = 0;    // Pot value, 0-179 degrees
double              throttle_filt   = 0;    // Pot value, 0-179 degrees
double              updateTime      = 0.0;  // Control law update time, sec
int                 verbose         = 2;    // Debugging Serial.print as much as you can tolerate.  0=none
double              fn[2]           = {0, 0}; // Functions to analyze
const int           ix[1]           = {0};  // Indeces of fn to excitations
const int           iy[1]           = {1};  // Indeces of fn to responses
void setup()
{
  WiFi.disconnect();
  Serial.begin(9600);
  myservo.attach(PWM_PIN);  // attaches the servo.  Only supported on pins that have PWM
  pinMode(POT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  // Lag filter
  throttleFilter1  = new LagTustin(float(CONTROL_DELAY)/1000000.0, tau, -0.1, 0.1);
  throttleFilter2  = new LagTustin(float(CONTROL_DELAY)/1000000.0, tau, -0.1, 0.1);
  // analyzer
  analyzer         = new FR_Analyzer(0, 2, 0.1, 0.05, 1, double(FILTER_DELAY/1e6), fn, ix, iy, 2, 1);
  analyzer->publish();   Serial.printf("\n");
  delay(1000);
  if (verbose>1) Serial.printf("\nCalibrating ESC...");
  while (throttle<179)
  {
    throttle = min(179, throttle+10);
    myservo.write(throttle);
    delay(20);
  }
  delay(1000);
  while (throttle>0)
  {
    throttle = max(0, throttle-10);
    myservo.write(throttle);
    delay(20);
  }
  if (verbose>1) Serial.printf("Done.\n");
  Serial.printf("To flash code to this device, push and hold both Photon buttons, release RESET until purple observed, then release.\n");

  WiFi.off();
  delay(1000);
}

void loop() {
  bool                    control;            // Control sequence, T/F
  bool                    filter;             // Filter for temperature, T/F
  bool                    publish;            // Publish, T/F
  bool                    read;               // Read, T/F
  bool                    checkPot;           // Display to LED, T/F
  bool                    frequencyResponse;  // Begin frequencyResponse, T/F
  unsigned long           now = micros();     // Keep track of time
  static unsigned long    lastControl  = 0UL; // Last control law time, micros
  static unsigned long    lastFilter   = 0UL; // Last filter time, micros
  static unsigned long    lastPublish  = 0UL; // Last publish time, micros
  static unsigned long    lastRead     = 0UL; // Last read time, micros
  static unsigned long    lastFR       = 0UL; // Last frequencyResponse, micros
  static int              RESET        = 1;   // Dynamic reset
  static double           tFilter;            // Modeled temp, F
  static double           exciter      = 1;   // Frequency response excitation, fraction

  // Executive
  publish   = ((now-lastPublish) >= PUBLISH_DELAY);
  if ( publish ) lastPublish  = now;

  read    = ((now-lastRead) >= READ_DELAY || RESET>0) && !publish;
  if ( read     ) lastRead      = now;

  filter    = ((now-lastFilter)>=FILTER_DELAY) || RESET>0;
  if ( filter )
  {
    tFilter     = float(now-lastFilter)/1000000.0;
    if ( verbose > 3 ) Serial.printf("Filter update=%7.3f\n", tFilter);
    lastFilter    = now;
  }
  frequencyResponse = ((now-lastFR) >= FR_DELAY && !analyzer->complete() );
  if ( frequencyResponse )
  {
    digitalWrite(LED_PIN,  1);
  }
  else
  {
    digitalWrite(LED_PIN,  0);
  }

  checkPot   = read;

  // Interrogate pot; run fast for good tactile feedback
  // my pot puts out 0- 4096 observed using Tinker
  if ( checkPot && RESET<1 )
  {
#ifndef BARE_PHOTON
    potValue    = analogRead(POT_PIN);
#endif
    throttle = map(potValue, 0, 4096, 0, 179);
  }

  if ( filter )
  {
    if ( verbose>4 ) Serial.printf("FILTER\n");
    throttle_filt     = (int)throttleFilter1->calculate(throttle,  RESET, tFilter);
    if ( frequencyResponse ) exciter = analyzer->calculate();
    fn[0]   = throttle_filt*exciter;
    throttle1         = (int)throttleFilter2->calculate(fn[0], RESET, tFilter);
    fn[1]   = throttle1;
    if ( verbose > 4) Serial.printf("throttle=%f, RESET=%d, tFilter=%f a=%f b=%f rate=%f state=%f filt=%f\n", throttle, RESET, tFilter, throttleFilter2->a(), throttleFilter2->b(), throttleFilter2->rate(), throttleFilter2->state(), throttle_filt);
    RESET = 0;
  }


  unsigned long deltaT = now - lastControl;
  control = (deltaT >= CONTROL_DELAY);
  if ( control  )
  {
    updateTime    = float(deltaT)/1000000.0 + float(numTimeouts)/100000.0;
    lastControl   = now;
    myservo.write(throttle_filt*exciter);                // sets the servo position according to the scaled value
  }


  if ( publish )
  {
    if (verbose>1) Serial.printf("ThrotFlt=%4.2f, throt1=%4.2f, exc=%6.4f, omega=%4.2f, T=%7.5f, ",
    throttle_filt, throttle1, exciter, analyzer->omega(), updateTime);
    if( !analyzer->complete() )
    {
      analyzer->publish();
    }
    Serial.printf("\n");
  }
}
