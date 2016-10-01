/*
Controlling a servo position using a potentiometer (variable resistor)
by Dave Gutz

Connections for Photon:
  ESC ------------------- Photon
    BLK ------------------- GND
    WHT ------------------- PWM Output (A4)
  ESC----------------- 3-wire DC Servomotor (stepper)
    Any three to any three
  EMF F2V-----------------Photon
    V5/V10----------------Analog In A2
    GND-------------------GND
  POT---------------------Photon
    VHI ------------------VIN
    VLO ------------------GND
    WIPE -----------------Analog In A0
  LED to indicate frequency response----Digital Output (D7)
  Hardware Platform:
    Microcontroller:  Particle Photon
    ESC:Hitec Energy Rotor 18A, 2-4S LiPo
    Power Supply:  BINZET AC 100-240V to DC 12V 10A
    Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
    Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
    EMF F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)


Connections for Arduino:
  ESC ------------------- Arduino
    BLK ------------------- GND
    WHT ------------------- PWM Output (5)
  ESC----------------- 3-wire DC Servomotor (stepper)
    Any three to any three
  EMF F2V-----------------Arduino
    V5/V10----------------Analog In A2
    GND-------------------GND
  POT---------------------Photon
    VHI ------------------VIN
    VLO ------------------GND
    WIPE -----------------Analog In A0
  LED to indicate frequency response----Digital Output (7)
  JUMPER---------------4 to GND for Closed Loop
  Hardware Platform:
    Microcontroller:  Arduino Uno R3
    ESC:Hitec Energy Rotor 18A, 2-4S LiPo
    Power Supply:  BINZET AC 100-240V to DC 12V 10A
    Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
    Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
    EMF F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)

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
    13-Sep-2016   DA Gutz   Initial analyzing
    30-Sep-2016   DA Gutz   Arduino added

  Distributed as-is; no warranty is given.
*/


//Sometimes useful for debugging
//#pragma SPARK_NO_PREPROCESSOR
//

// Standard
#ifndef ARDUINO
  #include "application.h"      // Should not be needed if file .ino or Arduino
  SYSTEM_THREAD(ENABLED);       // Make sure heat system code always run regardless of network status
#else
  #include <Servo.h>
  #include <Print.h>
#endif
#include "math.h"
#include "analyzer.h"
//
// Test features usually commented
//
// Disable flags if needed.  Usually commented
// #define DISABLE
//#define BARE_PHOTON                       // Run bare photon for testing.  Bare photon without this goes dark or hangs trying to write to I2C

// Test features
extern  const int   verbose         = 2;    // Debug, as much as you can tolerate
const         bool  freqResp        = false;  // Perform frequency response test on boot

// Constants always defined
// #define CONSTANT
#ifndef ARDUINO  // Photon
  #define PWM_PIN          A4                 // PWM output (A4)
  #define POT_PIN          A0                 // Potentiometer input pin on Photon (A0)
  #define EMF_PIN          A2                 // Fan speed back-emf input pin on Photon (A2)
  #define LED_PIN          D7                 // Status LED
  #define CL_PIN           D0                 // 3.3v to close loop (D0)
  #define FR_DELAY         40000000UL         // Time to start FR, micros
  #define CLOCK_TCK        8UL                // Clock tick resolution, micros
  #define PUBLISH_DELAY    40000UL            // Time between cloud updates (), micros
  #define CONTROL_DELAY    1000UL             // Control law wait (), micros
  #define INSCALE          4096.0             // Input full range from OS
#else
  #define PWM_PIN          5                  // PWM output (PD5)
  #define POT_PIN          A0                 // Potentiometer input pin on Photon (PC0)
  #define EMF_PIN          A2                 // Fan speed back-emf input pin on Photon (PC2)
  #define LED_PIN          7                  // Status LED (OD7)
  #define CL_PIN           4                  // GND to close loop (D4 to GND)
  #define FR_DELAY         40000000UL         // Time to start FR, micros
  #define CLOCK_TCK        16UL               // Clock tick resolution, micros
  #define PUBLISH_DELAY    60000UL            // Time between cloud updates (), micros
  #define CONTROL_DELAY    15000UL            // Control law wait (), micros
  #define INSCALE          1023.0             // Input full range from OS
#endif

//
// Dependent includes.   Easier to debug code if remove unused include files
#include "myFilters.h"

// Global variables
#ifdef ARDUINO
  char buffer[256];
#endif
LagTustin*          throttleFilter;           // Tustin lag filter
LeadLagTustin*      modelFilterE;             // Tustin lead lag filter esc
LagTustin*          modelFilterG;             // Tustin lag filter gas gen
LagTustin*          modelFilterF;             // Tustin lag filter fan
LagTustin*          modelFilterV;             // Tustin lag filter F2V sensor
FRAnalyzer*         analyzer;                 // Frequency response analyzer
Servo               myservo;  // create servo object to control a servo
const double        AEMF            =-1.9058*3.125; // Curve fit to LM2907 circuit, %Nf/volt^2
const double        BEMF            = 16.345*3.125; // Curve fit to LM2907 circuit, %Nf/volts
const double        CEMF            =-2.14*3.125;   // Curve fit to LM2907 circuit, %Nf
const double        AMDL            =-0.0028*3.125; // Curve fit to fan, %/deg^2
const double        BMDL            = 0.8952*3.125; // Curve fit to fan, %/deg
const double        CMDL            =-38.0*3.125;   // Curve fit to fan, %
int                 potValue        = 1500;   // Dial raw value, 0-4096
int                 emfValue        = 1000;   // Dial raw value, 0-4096
const double        thtlMax         = 115;    // Maximum throttle to prevent shutdown due to small charger, deg
const double        thtlMin         = 0;      // Minimum throttle, deg
#ifndef ARDUINO
const double        Ki              = 11.15/3.125;  // Int gain, deg/s/%Nf
const double        Kp              = 3.0/3.125;    // Prop gain, deg/%Nf
const double        KiM             = 11.15/3.125;  // Int gain, deg/s/%Nf
const double        KpM             = 3.0/3.125;    // Prop gain, deg/%Nf
#else
const double        Ki              = 10.10/3.125;  // Int gain, deg/s/%Nf
const double        Kp              = 2.61/3.125;   // Prop gain, deg/%Nf
const double        KiM             = 9.69/3.125;   // Int gain, deg/s/%Nf
const double        KpM             = 2.52/3.125;   // Prop gain, deg/%Nf
#endif
double              modelE          = 0;      // Model ESC output, %Ng
double              modelG          = 0;      // Model Gas Generator output, %Ng
double              modelF          = 0;      // Model Fan, %Nf
double              modelFS         = 0;      // Model Fan Sensed, %Nf
double              pcnf            = 0;      // Fan speed, % of 57600 rpm
const  double       tau             = 0.10;   // Input noise filter time constant, sec
const  double       tldE            = 0.05;   // Model ESC lead time constant, sec
const  double       tauE            = 0.01;   // Model ESC lag time constant, sec
const  double       tauV            = 0.07;   // Model F2V time constant, sec
const  double       tauG            = 0.13;   // Model Gas Generator time constant, sec
const  double       tauF            = 0.13;   // Model Fan time constant, sec
double              throttle        = 0;      // Pot value, 0-179 degrees
double              throttle_filt   = 0;      // Pot value, 0-179 degrees
double              updateTime      = 0.0;    // Control law update time, sec
double              vemf            = 0;      // Converted sensed back emf LM2907 circuit measure, volts
double              fn[3]           = {0, 0, 0}; // Functions to analyze
const int           ix[2]           = {0, 0}; // Indeces of fn to excitations
const int           iy[2]           = {1, 2}; // Indeces of fn to responses


void setup()
{
#ifndef ARDUINO
  WiFi.disconnect();
#endif
  Serial.begin(230400);
  myservo.attach(PWM_PIN);  // attaches the servo.  Only supported on pins that have PWM
  pinMode(POT_PIN, INPUT);
  pinMode(EMF_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CL_PIN,  INPUT);

  // Lag filter
  double T        = float(CONTROL_DELAY)/1000000.0;
  throttleFilter  = new LagTustin(    T, tau,  -0.1, 0.1);
  modelFilterE    = new LeadLagTustin(T, tldE, tauE,-0.1, 0.1);
  modelFilterG    = new LagTustin(    T, tauG, -0.1, 0.1);
  modelFilterF    = new LagTustin(    T, tauF, -0.1, 0.1);
  modelFilterV    = new LagTustin(    T, tauV, -0.1, 0.1);
  analyzer        = new FRAnalyzer(0, 3, 0.1,    2,    6, 1/tauG, double(CONTROL_DELAY/1e6), fn, ix, iy, 3, 2);
  //                               on ox   do minCy iniCy  wSlow
  delay(1000);
#ifndef ARDUINO
  if (verbose>1) Serial.printf("\nCalibrating ESC...");
#else
  if (verbose>1) sprintf(buffer,"\nCalibrating ESC...");
  Serial.print(buffer);
#endif
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
#ifndef ARDUINO
  if (verbose>1) Serial.printf("Done.\n");
  Serial.printf("To flash code to this device, push and hold both Photon buttons, release RESET until purple observed, then release.\n");
#else
  if (verbose>1) Serial.println("Done.");
  Serial.println("To flash code to this device, use Arduino IDE - Ctrl-R, Ctrl-U.");
#endif
  analyzer->publish();
#ifndef ARDUINO
  Serial.printf("\n");
#else
  Serial.println("");
#endif

#ifndef ARDUINO
  WiFi.off();
#endif

  delay(1000);
}

void loop() {
  bool                    closingLoop = false;// Closing loop by pin cmd, T/F
  bool                    control;            // Control sequence, T/F
  bool                    filter;             // Filter for temperature, T/F
  bool                    publish;            // Publish, T/F
  bool                    reading;            // Read, T/F
  bool                    ttl;                // TTL model processing, T/F
  bool                    checkPot;           // Display to LED, T/F
  bool                    analyzing;          // Begin analyzing, T/F
  unsigned long           now = micros();     // Keep track of time
  static unsigned long    start        = 0UL; // Time to start looping, micros
  double                  elapsedTime;        // elapsed time, micros
  static double           intState     = 0;   // PI control integrate state, deg
  static double           intStateM    = 0;   // PI control integrate state for model, deg
  static unsigned long    lastControl  = 0UL; // Last control law time, micros
  static unsigned long    lastFilter   = 0UL; // Last filter time, micros
  static unsigned long    lastPublish  = 0UL; // Last publish time, micros
  static unsigned long    lastRead     = 0UL; // Last read time, micros
  static unsigned long    lastTTL      = 0UL; // Last TTL time, micros
  static unsigned long    lastFR       = 0UL; // Last analyzing, micros
  static double           modPcng      = 0;   // Modeled pcng ref after esc ttl delay, %Nf
  static double           pcnfRef      = 0;   // Fan speed closed loop reference, %Nf
  static int              RESET        = 1;   // Dynamic reset
  static double           tFilter;            // Actual update time, s
  static double           exciter      = 0;   // Frequency response excitation, fraction
  static double           throttleCL, throttleCLM, e, eM;

  // Executive
  if ( start == 0UL ) start = now;
  elapsedTime  = double(now - start)*1e-6;

  closingLoop =   digitalRead(CL_PIN) == LOW;

  publish   = ((now-lastPublish) >= PUBLISH_DELAY-CLOCK_TCK/2 );
  if ( publish ) {
    lastPublish  = now;
  }

  reading    = ((now-lastRead) >= CONTROL_DELAY-CLOCK_TCK/2 || RESET>0);
  if ( reading     ) lastRead      = now;

  ttl     = ((now-lastTTL) >= CONTROL_DELAY-CLOCK_TCK/2 || RESET>0);
  if ( ttl      ) lastTTL      = now;

  filter    = ((now-lastFilter)>=CONTROL_DELAY-CLOCK_TCK/2 ) || RESET>0;
  if ( filter )
  {
    tFilter     = float(now-lastFilter)/1000000.0;
    lastFilter    = now;
  }
  if ( freqResp )
    analyzing = ( (now-lastFR) >= FR_DELAY && !analyzer->complete() );
  else
    analyzing = false;

  // Report that running Freq Resp
  if ( analyzing )
  {
    digitalWrite(LED_PIN,  1);
  }
  else
  {
    digitalWrite(LED_PIN,  0);
  }

  // Interrogate pot; run fast for good tactile feedback
  // my pot puts out 0- 4096 observed using Tinker
  checkPot   = reading;
  if ( checkPot && RESET<1 )
  {
#ifndef BARE_PHOTON
    potValue = analogRead(POT_PIN);
    emfValue = analogRead(EMF_PIN);
    vemf     = double(emfValue)/INSCALE*3.3;
    pcnf     = fmax(AEMF*vemf*vemf + BEMF*vemf + CEMF, 0.0);
#else
    pcnf     = modelF;
#endif
    throttle = fmin(double(potValue)/INSCALE*thtlMax, thtlMax);
    fn[2]    = pcnf;
  }

  // Control law
  if ( filter )
  {
    throttle_filt = throttleFilter->calculate(throttle,  RESET);
    pcnfRef       = fmax(fmin(throttle_filt/thtlMax*30*3.125-1.0*3.125, 27.5*3.125), 0.);
    // Control law
    e    = pcnfRef - pcnf;
    eM   = pcnfRef - modelFS;
    if ( !closingLoop ) intState = throttle_filt;
    intState    = fmax(fmin(intState  + Ki*e*updateTime,   thtlMax*1.26), -thtlMax*1.26);
    throttleCL  = fmax(fmin(intState  + fmax(fmin(Kp*e,    thtlMax*1.26), -thtlMax*1.26), thtlMax), thtlMin);
    intStateM   = fmax(fmin(intStateM + KiM*eM*updateTime, thtlMax*1.26), -thtlMax*1.26);
    throttleCLM = fmax(fmin(intStateM + fmax(fmin(KpM*eM,  thtlMax*1.26), -thtlMax*1.26), thtlMax), thtlMin);
    if ( freqResp )
    {
      fn[0]     = throttle_filt*(1+exciter/20);
    }
    else
    {
      if ( closingLoop )
      {
        fn[0]     = throttleCL;
      }
      else
      {
        fn[0]     = throttle_filt;
      }
    }
  }

  // DC model value
  if ( freqResp )
  {
    if ( ttl ) modPcng = fmax((AMDL*fn[0] + BMDL)*fn[0] + CMDL, 0.0);
  }
  else
  {
    if ( ttl ) modPcng = fmax((AMDL*throttleCLM + BMDL)*throttleCLM + CMDL, 0.0);
  }

  // Model
  if ( filter )
  {
    modelE      = modelFilterE->calculate(modPcng, RESET);
    modelG      = modelFilterG->calculate(modelE,  RESET);
    modelF      = modelFilterF->calculate(modelG,  RESET);
    modelFS     = modelFilterV->calculate(modelF,  RESET);
    if ( analyzing ) exciter = analyzer->calculate();  // use previous exciter for everything
    fn[1]          = modelF;
    RESET          = 0;
  }


  // DAC
  unsigned long deltaT = now - lastControl;
  control = (deltaT >= CONTROL_DELAY-CLOCK_TCK/2 );
  if ( control  )
  {
    updateTime    = float(deltaT)/1000000.0;
    lastControl   = now;
    myservo.write(fn[0]);  // sets the servo position according to the scaled value
  }

  if ( publish )
  {
    if ( freqResp )
    {
#ifndef ARDUINO
      if (verbose>1) Serial.printf("tim=%10.6f, ref=%6.4f, exc=%6.4f, ser=%4.2f, mod=%4.2f, nf=%4.2f, T=%8.6f, ",
        elapsedTime, pcnfRef, exciter, fn[0], fn[1], fn[2], updateTime);
#else
      if (verbose>1) sprintf(buffer, "t=%s, ref=%s, exc=%s, ser=%s, mod=%s, nf=%s, T=%s,",
        String(elapsedTime,6).c_str(), String(pcnfRef).c_str(), String(exciter).c_str(), String(fn[0]).c_str(),
        String(fn[1]).c_str(), String(fn[2]).c_str(), String(updateTime,6).c_str());
      Serial.print(buffer);
#endif
      if( !analyzer->complete() )
      {
        analyzer->publish();
      }
#ifdef ARDUINO
      Serial.println("");
#endif
    }  // freqResp
    else
    {
#ifndef ARDUINO
      if (verbose>1) Serial.printf("tim=%10.6f, cl=%ld, ref=%6.4f, nf=%4.2f, e=%4.2f, s=%4.2f, ser=%4.2f, :::: ref=%4.2f, nfM=%4.2f, eM=%4.2f, sM=%4.2f, serM=%4.2f, ttl=%ld, modPcng=%4.2f,\n",
        elapsedTime, closingLoop, pcnfRef, pcnf, e, intState, throttleCL, pcnfRef, modelF, eM, intStateM, throttleCLM, ttl, modPcng);
#else
      if (verbose>1) sprintf(buffer, "t=%s, cl=%s, ref=%s, nf=%s, e=%s, s=%s, ser=%s, :::: ref=%s, nfM=%s, eM=%s, sM=%s, serM=%s, ttl=%s, modPcng=%s, T=%s\n",
        String(elapsedTime,6).c_str(), String(closingLoop).c_str(), String(pcnfRef).c_str(), String(pcnf).c_str(), String(e).c_str(),
        String(intState).c_str(), String(throttleCL).c_str(), String(pcnfRef).c_str(), String(modelF).c_str(),
        String(eM).c_str(), String(intStateM).c_str(), String(throttleCLM).c_str(), String(ttl).c_str(),
        String(modPcng).c_str(), String(updateTime,6).c_str());
        Serial.print(buffer);
#endif
    }
  }  // publish
}
