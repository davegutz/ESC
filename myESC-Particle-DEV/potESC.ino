/*
Controlling a servo position using a potentiometer (variable resistor)
by Dave Gutz

    Connections:
      ESC ------------------- Photon
        BLK ------------------- GND
        WHT ------------------- PWM Output (A4)
      ESC----------------- 3-wire DC Servomotor (stepper)
        Any three to any three
      EMF-------------------Photon
        V5/V10----------------Analog In A2
        GND-------------------GND
      POT---------------------Photon
        VHI ------------------VIN
        VLO ------------------GND
        WIPE -----------------Analog In A0
      Hardware Platform:
        Microcontroller:  Particle Photon
        ESC:Hitec Energy Rotor 18A, 2-4S LiPo
        Power Supply:  BINZET AC 100-240V to DC 12V 10A
        Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
        Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
        F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)

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
    13-Sep-2016   DA Gutz   Initial frequencyResponse

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
#define EMF_PIN          A2                 // Fan speed back-emf input pin on Photon (A2)
#define FR_DELAY         40000000UL         // Time to start FR, micros
#define LED_PIN          D7                 // Status LED
#define CLOCK_TCK        8UL                // Clock tick resolution, micros
#define PUBLISH_DELAY    40000UL            // Time between cloud updates (), micros
#define CONTROL_DELAY    1000UL             // Control law wait (), micros
#define AEMF             -1.9058            // Curve fit to LM2907 circuit, %Nf/volt^2
#define BEMF             16.345             // Curve fit to LM2907 circuit, %Nf/volts
#define CEMF             -2.14              // Curve fit to LM2907 circuit, %Nf
#define AMDL             -0.0028            // Curve fit to fan, %/deg^2
#define BMDL             0.8952             // Curve fit to fan, %/deg
const double CMDL             =-38.0;              // Curve fit to fan, %

//
// Dependent includes.   Easier to debug code if remove unused include files
#include "myFilters.h"

// Global variables
LagTustin*          throttleFilter;           // Exponential lag filter
LagTustin*          modelFilter1;             // Exponential lag filter
LagTustin*          modelFilter2;             // Exponential lag filter
FRAnalyzer*         analyzer;                 // Frequency response analyzer
Servo               myservo;  // create servo object to control a servo
int                 numTimeouts     = 0;      // Number of Particle.connect() needed to unfreeze
int                 potValue        = 1500;   // Dial raw value, 0-4096
int                 emfValue        = 1000;   // Dial raw value, 0-4096
double              model1          = 0;      // Model lag 1 output, %Nf
double              model2          = 0;      // Model lag 2 output, %Nf
double              pcnf            = 0;      // Fan speed, % of 57600 rpm
const  double       tau             = 0.10;   // Input noise filter time constant, sec
const  double       tau1            = 0.35;   // Model time constant, sec
const  double       tau2            = 0.10;   // Model time constant, sec
double              throttle        = 0;      // Pot value, 0-179 degrees
double              throttle_filt   = 0;      // Pot value, 0-179 degrees
double              updateTime      = 0.0;    // Control law update time, sec
double              vemf            = 0;      // Converted sensed back emf LM2907 circuit measure, volts
int                 verbose         = 2;      // Debugging Serial.print as much as you can tolerate.  0=none
double              fn[3]           = {0, 0, 0}; // Functions to analyze
const int           ix[2]           = {0, 0}; // Indeces of fn to excitations
const int           iy[2]           = {1, 2}; // Indeces of fn to responses
void setup()
{
  WiFi.disconnect();
  Serial.begin(9600);
  myservo.attach(PWM_PIN);  // attaches the servo.  Only supported on pins that have PWM
  pinMode(POT_PIN, INPUT);
  pinMode(EMF_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Lag filter
  throttleFilter  = new LagTustin(float(CONTROL_DELAY)/1000000.0, tau, -0.1, 0.1);
  modelFilter1    = new LagTustin(float(CONTROL_DELAY)/1000000.0, tau1, -0.1, 0.1);
  modelFilter2    = new LagTustin(float(CONTROL_DELAY)/1000000.0, tau2, -0.1, 0.1);
  analyzer        = new FRAnalyzer(-1, 2, 0.1,    2,    3, 1/tau1, double(CONTROL_DELAY/1e6), fn, ix, iy, 3, 2);
  //                               on ox   do minCy iniCy  wSlow

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
  analyzer->publish();   Serial.printf("\n");

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
  static unsigned long    start        = 0UL; // Time to start looping, micros
  double                  elapsedTime;        // elapsed time, micros
  static unsigned long    lastControl  = 0UL; // Last control law time, micros
  static unsigned long    lastFilter   = 0UL; // Last filter time, micros
  static unsigned long    lastPublish  = 0UL; // Last publish time, micros
  static unsigned long    lastRead     = 0UL; // Last read time, micros
  static unsigned long    lastFR       = 0UL; // Last frequencyResponse, micros
  static int              RESET        = 1;   // Dynamic reset
  static double           tFilter;            // Modeled temp, F
  static double           exciter      = 0;   // Frequency response excitation, fraction

  // Executive
  if ( start == 0UL ) start = now;
  elapsedTime  = double(now - start)*1e-6;

  publish   = ((now-lastPublish) >= PUBLISH_DELAY);
  if ( publish ) lastPublish  = now;

  read    = ((now-lastRead) >= CONTROL_DELAY-CLOCK_TCK/2 || RESET>0) && !publish;
  if ( read     ) lastRead      = now;

  filter    = ((now-lastFilter)>=CONTROL_DELAY-CLOCK_TCK/2 ) || RESET>0;
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
    emfValue    = analogRead(EMF_PIN);
#endif
//    throttle = map(potValue, 0, 4096, 0, 179);
    throttle = double(potValue)/4096.0*179.0;
//    vemf     = map(emfValue, 0, 4096, 0, 3.3);
    vemf     = double(emfValue)/4096.0*3.3;
    pcnf     = max(AEMF*vemf*vemf + BEMF*vemf + CEMF, 0.0);
    fn[2]    = pcnf;
  }


  if ( filter )
  {
    if ( verbose>4 ) Serial.printf("FILTER\n");
    throttle_filt  = throttleFilter->calculate(throttle,  RESET, tFilter);
    fn[0]          = throttle_filt*(1+exciter/10);
    double modPcnf = (AMDL*fn[0] + BMDL)*fn[0] + CMDL;
    model1         = modelFilter1->calculate(modPcnf, RESET, tFilter);
    model2         = modelFilter1->calculate(model1,  RESET, tFilter);
    if ( frequencyResponse ) exciter = analyzer->calculate();
    fn[1]          = model2;
    RESET          = 0;
  }


  unsigned long deltaT = now - lastControl;
  control = (deltaT >= CONTROL_DELAY-CLOCK_TCK/2 );
  if ( control  )
  {
    updateTime    = float(deltaT)/1000000.0 + float(numTimeouts)/100000.0;
    lastControl   = now;
    myservo.write(fn[0]);  // sets the servo position according to the scaled value
}


  if ( publish )
  {
    if (verbose>1) Serial.printf("elapsedTime=%10.6f, exc=%6.4f, servo=%4.2f, model=%4.2f, pcnf=%4.2f, T=%8.6f, ",
    elapsedTime, exciter, fn[0], fn[1], fn[2], updateTime);
    if( !analyzer->complete() )
    {
      analyzer->publish();
    }
    Serial.printf("\n");
  }
}
