//Sometimes useful for debugging
//#pragma SPARK_NO_PREPROCESSOR
//

// Standard
#ifdef ARDUINO
  #include <Servo.h>
  #include <Arduino.h>  // Used instead of Print.h - breaks Serial
#else   // Photon
  #include "application.h"      // Should not be needed if file ino or Arduino
  SYSTEM_THREAD(ENABLED);       // Make sure heat system code always run regardless of network status
#endif
#include "math.h"
#include "analyzer.h"
#include "myTables.h"
/*
Controlling a servo position using a potentiometer (variable resistor)
by Dave Gutz

Connections for Photon:
  ESC ------------------- Photon
    BLK ------------------- GND
    WHT ------------------- PWM Output (A4)
  ESC----------------- 3-wire DC Servomotor (stepper)
    Any three to any three
  DPST switch-------------Photon
    GND-------------------D4
  DPST switch
    HI--------------------3.3V
    LO--------------------10K to GND
  Push button-------------Photon
      R---------------------D2
    Push button
      HI--------------------3.3V
      LO--------------------10K to GND
  F2V---------------------Photon
    V5/V10----------------Analog In A2
    GND-------------------GND
  POT---------------------Photon
    VHI ------------------3.3v
    VLO ------------------GND
    WIPE -----------------Analog In A0
  LED to indicate frequency response----Digital Output (D7)
  Hardware Platform:
    Microcontroller:  Particle Photon
    ESC:Hitec Energy Rotor 18A, 2-4S LiPo
    Power Supply:  BINZET AC 100-240V to DC 12V 10A
    Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
    Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
    F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)


Connections for Arduino:
  ESC ------------------- Arduino
    BLK ------------------- GND
    WHT ------------------- PWM Output (5)
  ESC----------------- 3-wire DC Servomotor (stepper)
    Any three to any three
  F2V-----------------Arduino
    V5/V10----------------Analog In A2
    GND-------------------GND
  DPST switch-------------Arduino
    CTR-------------------4
  DPST switch
    HI--------------------3.3V
    LO--------------------10K to GND
  Push button-------------Arduino
    R---------------------2
  Push button
    HI--------------------3.3V
    LO--------------------10K to GND
  POT---------------------Arduino
    VHI ------------------5V
    VLO ------------------GND
    WIPE -----------------Analog In A0
  BUTTON ----------------Ardunio  D2
    see https://www.arduino.cc/en/Tutorial/Button
  LED to indicate frequency response----Digital Output (7)
  JUMPER---------------4 to GND for Closed Loop
  Hardware Platform:
    Microcontroller:  Arduino Uno R3
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
  4.  Switch from open to closed loop and vice versa with small, <5 deg throttle
      bump, in response to switch change.
  5.  Perform frequency response analysis in responseto button push.
  6.  Update closed loop algorithms, sense inputs, and produce outputs at rate
      at least as fast as 0.015 second update rate.
  Secondary:
  1.  Repeated Pushbutton toggles frequency response in progress.   When
      restarting, it begins completely fresh.
  2.  For non-Arduino, may use a software Pushbutton - send string on serial.

  Tasks TODO:
  1.  Investigate use of zener version of F2V chip.
  2.  Add table lookup for control schedules.

  Revision history:
    31-Aug-2016   DA Gutz   Created
    13-Sep-2016   DA Gutz   Initial analyzing
    30-Sep-2016   DA Gutz   Arduino added
    10-Oct-2016   DA Gutz   First frequency response completion.

  Distributed as-is; no warranty is given.
*/


// Test features usually commented
//
// Disable flags if needed.  Usually commented
// #define DISABLE
//#define BARE_MICROPROCESSOR                       // Run bare photon for testing.  Bare photon without this goes dark or hangs trying to write to I2C

// Test features
extern  const int   verbose         = 2;    // Debug, as much as you can tolerate (2)
bool                freqResp        = false;  // Perform frequency response test

// Constants always defined
// #define CONSTANT
#ifdef ARDUINO
  #define BUTTON_PIN       2                  // Button 3-way input momentary 3.3V, steady GND (D2)
  #define PWM_PIN          5                  // PWM output (PD5)
  #define POT_PIN          A0                 // Potentiometer input pin on Arduino (PC0)
  #define F2V_PIN          A2                 // Fan speed back-emf input pin on Arduino (PC2)
  #define CL_PIN           4                  // Closed loop 3-way switch 5V or GND (D4 to GND)
  #define FR_DELAY         4000000UL          // Time to start FR, micros
  #define CLOCK_TCK        16UL               // Clock tick resolution, micros
  #define PUBLISH_DELAY    60000UL            // Time between cloud updates (), micros
  #define CONTROL_DELAY    15000UL            // Control law wait (), micros
  #define INSCALE          1023.0             // Input full range from OS
#else   // Photon
  #define BUTTON_PIN       D2                 // Button 3-way input momentary 3.3V, steady GND (D2)
  #define PWM_PIN          A4                 // PWM output (A4)
  #define POT_PIN          A0                 // Potentiometer input pin on Photon (A0)
  #define F2V_PIN          A2                 // Fan speed back-emf input pin on Photon (A2)
  #define LED_PIN          D7                 // Status LED
  #define CL_PIN           D0                 // Closed loop 3-way switch 3.3V or GND  (D0)
  #define FR_DELAY         4000000UL          // Time to start FR, micros
  #define CLOCK_TCK        8UL                // Clock tick resolution, micros
  #define PUBLISH_DELAY    40000UL            // Time between cloud updates (), micros
  #define CONTROL_DELAY    10000UL            // Control law wait (), micros
  #define INSCALE          4096.0             // Input full range from OS
#endif

//
// Dependent includes.   Easier to debug code if remove unused include files
#include "myFilters.h"

// Global variables
char buffer[256];
LagTustin*          throttleFilter;           // Tustin lag filter
LeadLagTustin*      modelFilterE;             // Tustin lead lag filter esc
LeadLagTustin*      modelFilterG;             // Tustin lag filter gas gen
LeadLagTustin*      modelFilterF;             // Tustin lag filter fan
LeadLagTustin*      modelFilterV;             // Tustin lag filter F2V sensor
FRAnalyzer*         analyzer;                 // Frequency response analyzer
Servo               myservo;  // create servo object to control a servo
double              modelE          = 0;      // Model ESC output, %Ng
double              modelG          = 0;      // Model Gas Generator output, %Ng
double              modelF          = 0;      // Model Fan, %Nf
double              modelFS         = 0;      // Model Fan Sensed, %Nf
const  double       tau             = 0.10;   // Input noise filter time constant, sec
const  double       tldE            = 0.05;   // Model ESC lead time constant, sec
const  double       tauE            = 0.01;   // Model ESC lag time constant, sec
#ifdef ARDUINO
  const  double       tldV            = 0.0;  // Model F2V lead time constant, sec
  const  double       tauV            = 0.03; // Model F2V lag time constant, sec
#else
  const  double       tldV            = 0.0;  // Model F2V lead time constant, sec
  const  double       tauV            = 0.07; // Model F2V lag time constant, sec
#endif
const  double       tldG            = 0.0;    // Model Gas Generator lead time constant, sec
const  double       tauG            = 0.13;   // Model Gas Generator lag time constant, sec
const  double       tldF            = 0.0;    // Model Fan lead time constant, sec
const  double       tauF            = 0.13;   // Model Fan lag time constant, sec
const double        THTL_MAX        = 180;    // Maximum throttle to prevent shutdown due to small charger, deg
const double        THTL_MIN        = 0;      // Minimum throttle, deg
double              throttle        = -5;     // Servo value, 0-179 degrees
double              throttleM       = 0;      // Modeled servo value, 0-179 degrees
double              throttleL       = 0;      // Limited servo value, 0-179 degrees
double              throttleML      = 0;      // Limited modeled servo value, 0-179 degrees
double              updateTime      = 0.0;    // Control law update time, sec
const int           nsigFn          = 4;      // Length of fn
const int           ntfFn           = 2;      // Number of transfer  functions to calculate <= length(ix)
double              fn[4]           = {0, 0, 0, 0}; // Functions to analyze
const int           ix[2]           = {0, 0}; // Indeces of fn to excitations
const int           iy[2]           = {1, 2}; // Indeces of fn to responses
const double        freqRespScalar  = 20;     // Use 40 for +/-5 deg, 20 for +/-10 deg, etc
#ifdef ARDUINO
const double xKI[1] = {90};   // Int gain breakpoints, %Nf
const double yKI[1] = {3};    // Int gain, deg/s/%Nf
const double xKP[1] = {90};   // Prop gain breakpoints, %Nf
const double yKP[1] = {0.8};  // Prop gain, deg/%Nf
const double xKIM[1]= {90};   // Model int gain breakpoints, %Nf
const double yKIM[1] = {3.3}; // Model Int gain, deg/s/%Nf
const double xKPM[1]= {90};   // Model prop gain breakpoints, %Nf
const double yKPM[1] = {1};   // Prop gain, deg/%Nf
#else  // Photon
const double xKI[1] = {90};   // Int gain breakpoints, %Nf
const double yKI[1] = {3};    // Int gain, deg/s/%Nf
const double xKP[1] = {90};   // Prop gain breakpoints, %Nf
const double yKP[1] = {0.8};  // Prop gain, deg/%Nf
const double xKIM[1]= {90};   // Model int gain breakpoints, %Nf
const double yKIM[1] = {3.3}; // Model Int gain, deg/s/%Nf
const double xKPM[1]= {90};   // Model prop gain breakpoints, %Nf
const double yKPM[1] = {1};   // Prop gain, deg/%Nf
#endif
TableInterp1D *KI_T, *KP_T, *KIM_T, *KPM_T;

// Serial event stuff
#ifndef ARDUINO
  String inputString = "";         // a string to hold incoming data
  boolean stringComplete = false;  // whether the string is complete
#endif

void setup()
{
#ifndef ARDUINO
  WiFi.disconnect();
  pinMode(LED_PIN, OUTPUT);
#endif
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(230400);
  myservo.attach(PWM_PIN);  // attaches the servo.  Only supported on pins that have PWM
  pinMode(POT_PIN, INPUT);
  pinMode(F2V_PIN, INPUT);
  pinMode(CL_PIN,  INPUT);

  // Lag filter
  double T        = float(CONTROL_DELAY)/1000000.0;
  throttleFilter  = new LagTustin(    T, tau,  -0.1, 0.1);
  modelFilterE    = new LeadLagTustin(T, tldE, tauE,-0.1, 0.1);
  modelFilterG    = new LeadLagTustin(T, tldG, tauG, -0.1, 0.1);
  modelFilterF    = new LeadLagTustin(T, tldF, tauF, -0.1, 0.1);
  modelFilterV    = new LeadLagTustin(T, tldV, tauV, -0.1, 0.1);
  analyzer        = new FRAnalyzer(-0.8, 2.2, 0.1,    2,    6,     1/tauG, double(CONTROL_DELAY/1e6), ix, iy, nsigFn, ntfFn, "t,ref,exc,thr,mod,nf,T");
  //analyzer        = new FRAnalyzer(1.0, 1.3, 0.1,    2,    6,     1/tauG, double(CONTROL_DELAY/1e6), ix, iy, nsigFn, ntfFn, "t,ref,exc,thr,mod,nf,T");
  //                               wmin  wmax dw      minCy iniCy  wSlow
 // 2.2 is Nyquist for T=.020

  myservo.write(throttle);

  // Header for analyzer
  analyzer->publish();
  Serial.println("");

#ifndef ARDUINO
  WiFi.off();
#endif

  if (verbose>1)
  {
    sprintf(buffer, "time,cl,pcnfref,pcnf,err,state,thr,pcnfrefM,pcnfM,errM,stateM,thrM,modPcng,T\n");
    Serial.print(buffer);
  }

  // Serial Event
#ifndef ARDUINO
  inputString.reserve(200);   // Reserve 200 bytes for inputString
#endif

  delay(100);

  KI_T    = new TableInterp1D(sizeof(xKI)/sizeof(double),  xKI,  yKI);
  KP_T    = new TableInterp1D(sizeof(xKP)/sizeof(double),  xKP,  yKP);
  KIM_T   = new TableInterp1D(sizeof(xKIM)/sizeof(double), xKIM, yKIM);
  KPM_T   = new TableInterp1D(sizeof(xKPM)/sizeof(double), xKPM, yKPM);

}



void loop() {
  int                     buttonState = 0;    // Pushbutton
  static bool             closingLoop = false;// Closing loop by pin cmd, T/F
  bool                    control;            // Control sequence, T/F
  bool                    publish;            // Publish, T/F
  bool                    analyzing;          // Begin analyzing, T/F
  unsigned long           now = micros();     // Keep track of time
  static unsigned long    start        = 0UL; // Time to start looping, micros
  double                  elapsedTime;        // elapsed time, micros
  static double           intState     = 0;   // PI control integrate state, deg
  static double           intStateM    = 0;   // PI control integrate state for model, deg
  static unsigned long    lastControl  = 0UL; // Last control law time, micros
  static unsigned long    lastPublish  = 0UL; // Last publish time, micros
  static unsigned long    lastButton   = 0UL; // Last button push time, micros
  static unsigned long    lastFR       = 0UL; // Last analyzing, micros
  static double           modPcng      = 0;   // Modeled pcng ref after esc ttl delay, %Nf
  static double           pcnfRef      = 0;   // Fan speed closed loop reference, %Nf
  static int              RESET        = 1;   // Dynamic reset
  const  double           RESEThold    = 5;   // RESET hold, s
  static double           exciter      = 0;   // Frequency response excitation, fraction
  static double           throttleCL, throttleCLM, e, eM;
  // See calibration20160913.xlsx for these hardware conversion derivations
  #ifdef ARDUINO
  const double            POT_MAX      = 5.0; // Maximum POT value, vdc
  const double            POT_MIN      = 0;   // Minimum POT value, vdc
  const double            F2V_MAX      = 5.0; // Maximum F2V value, vdc
  const double            F2V_MIN      = 0;   // Minimum F2V value, vdc
  const double            P_V4_NF[3]   = {0, 7227,-476};  // Coeff V4(v) to NF(rpm)
  const double            P_TH_NG[2]   = {0, 121};        // Coeff throttle(deg) to NG(rpm)
  const double            P_P_PNF[2]   = {-17.72,18.94};  // Coeff pot(v) to PCNF(%)
  #else  // Photon
  const double            POT_MAX      = 3.3; // Maximum POT value, vdc
  const double            POT_MIN      = 0;   // Minimum POT value, vdc
  const double            F2V_MAX      = 5.0; // Maximum F2V value, vdc
  const double            F2V_MIN      = 0;   // Minimum F2V value, vdc
  const double            P_V4_NF[3]   = {0, 7448,-435};  // Coeff V4(v) to NF(rpm)
  const double            P_TH_NG[2]   = {-0, 0};    // Coeff throttle(deg) to NG(rpm)
  const double            P_P_PNF[2]   = {-17.72, 28.7};  // Coeff pot(v) to PCNF(%)
  #endif
  const double            P_NG_NF[2]   = {-3926, 0.9420}; // Coeff NG(rpm) to NF(rpm)
  const double            P_NF_NG[2]   = {4180,  1.0602}; // Coeff NF(rpm) to NG(rpm)
  const double            RPM_P        = 222;             // (rpm/%)
////////////////////////////////////////////////////////////////////////////////////
  const double            SCMAXI       = 1*float(CONTROL_DELAY)/1000000.0;    // Maximum allowable step change reset, deg/update
  const double            SCMAX        = 240*float(CONTROL_DELAY)/1000000.0;     // Maximum allowable step change, deg/update
  #ifdef ARDUINO
    double           Ki              = 10.10/3.125;  // Int gain, deg/s/%Nf
    double           Kp              = 2.61/3.125;   // Prop gain, deg/%Nf
    double           KiM             = 9.69/3.125;   // Int gain, deg/s/%Nf
    double           KpM             = 2.52/3.125;   // Prop gain, deg/%Nf
  #else  // Photon
    double           Ki              = 11.15/3.125;  // Int gain, deg/s/%Nf
    double           Kp              = 3.0/3.125;    // Prop gain, deg/%Nf
    double           KiM             = 11.15/3.125;  // Int gain, deg/s/%Nf
    double           KpM             = 3.0/3.125;    // Prop gain, deg/%Nf
  #endif
  static double       pcnf            = 0;      // Fan speed, %
  static double       vf2v            = 0;      // Converted sensed back emf LM2907 circuit measure, volts
  static double       vpot_filt       = 0;      // Pot value, volts
  static double       vpot            = 0;      // Pot value, volts
  static int          f2vValue        = INSCALE/4;   // Dial raw value
  static int          potValue        = 0;      // Dial raw value

  // Executive
  if ( start == 0UL ) start = now;
  elapsedTime  = double(now - start)*1e-6;
#ifdef BARE_MICROPROCESSOR
  #ifdef ARDUINO
    closingLoop = true;
  #endif
#else
  closingLoop = (digitalRead(CL_PIN) == HIGH);
#endif
  buttonState = digitalRead(BUTTON_PIN);
  if ( buttonState == HIGH && (now-lastButton>2000000UL ) )
  {
    lastButton = now;
    analyzer->complete(freqResp);  // reset if doing freqResp
#ifndef BARE_MICROPROCESSOR
    freqResp = !freqResp;
#endif
  }
  publish   = ((now-lastPublish) >= PUBLISH_DELAY-CLOCK_TCK/2 );
  if ( publish )
  {
    lastPublish  = now;
  }
  unsigned long deltaT = now - lastControl;
  control = (deltaT >= CONTROL_DELAY-CLOCK_TCK/2 );
  if ( control )
  {
    lastControl   = now;
  }
  if ( freqResp )
  {
    analyzing = ( (now-lastFR) >= FR_DELAY && !analyzer->complete() );
  }
  else
  {
    analyzing = false;
  }

#ifndef ARDUINO
  // Report that running Freq Resp
  if ( analyzing )
  {
    digitalWrite(LED_PIN,  1);
  }
  else
  {
    digitalWrite(LED_PIN,  0);
  }

  // Serial event  (terminate Send String data with 0A using CoolTerm)
  if ( stringComplete )
  {
    String doFR = "f\n";
    if ( inputString == doFR ) {
       analyzer->complete(freqResp);  // reset if doing freqResp
       freqResp = !freqResp;
    }
    String doCL = "c\n";
    if ( inputString == doCL ) {
       closingLoop = !closingLoop;
    }
    inputString = "";
    stringComplete  = false;
  }
#endif

  // Interrogate inputs
  if ( control )
  {
#ifdef BARE_MICROPROCESSOR
    pcnf      = modelFS;
#else
    potValue  = analogRead(POT_PIN);
    f2vValue  = analogRead(F2V_PIN);
    vf2v      = double(f2vValue)/INSCALE*F2V_MAX;
    pcnf      = fmin(fmax(P_V4_NF[0] + vf2v*(P_V4_NF[1] + vf2v*P_V4_NF[2])/RPM_P, 0.0), 100);
#endif
    vpot      = fmin(fmax(double(potValue)/INSCALE*POT_MAX, POT_MIN), POT_MAX);
    fn[2]     = pcnf;
  }

  // Control law
  if ( control )
  {
    if ( !freqResp ) vpot_filt   = throttleFilter->calculate(vpot,  RESET);  // Freeze pot for FR
    pcnfRef     = P_P_PNF[0]  + vpot_filt*P_P_PNF[1];
    if ( closingLoop && freqResp ) pcnfRef *= (1+exciter/freqRespScalar);
    if ( RESET )
    {
      double throttleRPM  = P_NF_NG[0] + pcnfRef*RPM_P*P_NF_NG[1];      // RPM Ng
      intState  = intStateM = (throttleRPM-P_TH_NG[0])/P_TH_NG[1]; // deg throttle
      pcnf      = pcnfRef;
      modelFS   = pcnfRef;
    }
    e           = pcnfRef - pcnf;
    eM          = pcnfRef - modelFS;
    if ( !closingLoop ) intState = throttle;
    /*
    Ki  = tab1(pcnf, xKI,   yKI,  sizeof(xKI)/sizeof(double));
    Kp  = tab1(pcnf, xKP,   yKP,  sizeof(xKP)/sizeof(double));
    KiM = tab1(pcnf, xKIM,  yKIM, sizeof(xKIM)/sizeof(double));
    KpM = tab1(pcnf, xKPM,  yKPM, sizeof(xKPM)/sizeof(double));
    */
    Ki  = KI_T->interp(pcnf);
    Kp  = KP_T->interp(pcnf);
    KiM = KIM_T->interp(pcnf);
    KpM = KPM_T->interp(pcnf);

    if ( verbose > 2 )
    {
      sprintf(buffer, "%s,%s,\n", String(Ki).c_str(), String(Kp).c_str()); Serial.print(buffer);
      sprintf(buffer, "%s,%s,\n", String(KiM).c_str(), String(KpM).c_str()); Serial.print(buffer);
    }
    intState    = fmax(fmin(intState  + Ki*e*updateTime,   THTL_MAX*1.26), -THTL_MAX*1.26);
    throttleCL  = fmax(fmin(intState  + fmax(fmin(Kp*e,    THTL_MAX*1.26), -THTL_MAX*1.26), THTL_MAX), THTL_MIN);
    intStateM   = fmax(fmin(intStateM + KiM*eM*updateTime, THTL_MAX*1.26), -THTL_MAX*1.26);
    throttleCLM = fmax(fmin(intStateM + fmax(fmin(KpM*eM,  THTL_MAX*1.26), -THTL_MAX*1.26), THTL_MAX), THTL_MIN);
    if ( closingLoop )
    {
      double  throttleU   = throttleCL;
      double throttleMU   = throttleCLM;
     // Apply rate limits as needed
      if ( RESET )
      {
        throttleL  = fmax(fmin(throttleU,   throttleL+SCMAXI),   throttleL-SCMAXI);
        throttleML = fmax(fmin(throttleMU,  throttleML+SCMAXI),  throttleML-SCMAXI);
      }
      else
      {
        throttleL  = fmax(fmin(throttleU,   throttleL+SCMAX),   throttleL-SCMAX);
        throttleML = fmax(fmin(throttleMU,  throttleML+SCMAX),  throttleML-SCMAX);
      }
      throttle  = throttleL;
      throttleM = throttleML;
    }
    else  // open loop
    {
      double throttleRPM    = (P_NF_NG[0] + pcnfRef*RPM_P*P_NF_NG[1]);          // RPM Ng
      double throttleU      = (throttleRPM-P_TH_NG[0])/P_TH_NG[1] * (1+exciter/freqRespScalar);  // deg throttle
      //throttleU = throttleRPM*0.0083;
      // Apply rate limits as needed
      if ( RESET )
      {
        throttleL = throttleML = fmax(fmin(throttleU, throttleL+SCMAX), throttleL-SCMAX);
      }
      else
      {
        throttleL   = throttleU;
      }
      if ( freqResp ) throttle = throttleM  = throttleU* (1+exciter/freqRespScalar);
      else            throttle  = throttleM = throttleL;
    } // open loop

    // Final throttle limits
    throttle = fmax(fmin(throttle, THTL_MAX), THTL_MIN);
    fn[0] = throttle;
  }

  // Model
  if ( control )
  {
    if ( RESET )
      modPcng   = (P_NF_NG[0] + pcnfRef*RPM_P*P_NF_NG[1])/RPM_P;
    else
      modPcng   = fmax((P_TH_NG[0] + P_TH_NG[1]*double(int(throttleM))) / RPM_P, 0.0);
    modelE      = modelFilterE->calculate(modPcng, RESET);
    modelG      = modelFilterG->calculate(modelE,  RESET);
    modelF      = modelFilterF->calculate((P_NG_NF[0] + modelG*RPM_P*P_NG_NF[1])/RPM_P,  RESET);
    modelFS     = modelFilterV->calculate(modelF,  RESET);
    if ( analyzing ) exciter = analyzer->calculate(fn, nsigFn);  // use previous exciter for everything
    fn[1]       = modelFS;
    fn[3]       = pcnfRef;
    if ( elapsedTime>RESEThold ) RESET = 0;
  }


  // DAC
  if ( control  )
  {
    updateTime    = float(deltaT)/1000000.0;
    myservo.write(throttle);  // sets the servo position according to the scaled value
  }

  if ( publish )
  {
    if ( freqResp )
    {
      sprintf(buffer, "%s,%s,%s,%s,%s,%s,%s,",
        String(elapsedTime,6).c_str(), String(pcnfRef).c_str(),
        String(exciter).c_str(), String(throttle).c_str(),
        String(modelFS).c_str(), String(pcnf).c_str(),
        String(updateTime,6).c_str());
      Serial.print(buffer);
      if( !analyzer->complete() )
      {
        analyzer->publish();
      }
      Serial.println("");
    }  // freqResp
    else
    {
      if (verbose>1)
      {
        sprintf_P(buffer, PSTR("%s,%s,  "), String(elapsedTime,6).c_str(), String(closingLoop).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,%s,  %s,%s,%s,  "), String(pcnfRef).c_str(), String(pcnf).c_str(),
        String(e).c_str(), String(intState).c_str(), String(throttle).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,%s,  %s,%s,%s,  "), String(pcnfRef).c_str(), String(modelFS).c_str(),
        String(eM).c_str(), String(intStateM).c_str(), String(throttleM).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,%s,\n"), String(modPcng).c_str(), String(updateTime,6).c_str());
        Serial.print(buffer);
      }
    }
  }  // publish
  if ( analyzer->complete() ) freqResp = false;
}


/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
#ifndef ARDUINO
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
#endif
