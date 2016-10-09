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
  POT---------------------Arduino
    VHI ------------------VIN
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
    EMF F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)

  Reaquirements:
  Prime:
  1.  Manually sweep ESC command from min to max using pot.
  2.  Limit ESC command for safety using configurable parameters.
  3.  Initialize the ESC, which is built to accept min-max input on startup.
  Secondary:
  1.  USB status

  Tasks TODO:
  1.  Use the SerialEvent feature to read
      a.  f for frequency response:  freeze throttle, no rate limits, sweep, unfreeze
      b.  s for square wave response:  bias on pcnfref if closed, throttle if open
  2.  Capture closed loop frequency response, add pcnfref to array
  3.  Verify freq response again
  4.  Check against model and tune again
  5.  Mitigate integrator windup

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
#ifdef ARDUINO
  #include <Servo.h>
  #include <Print.h>
#else   // Photon
  #include "application.h"      // Should not be needed if file .ino or Arduino
  SYSTEM_THREAD(ENABLED);       // Make sure heat system code always run regardless of network status
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
extern  const int   verbose         = 2;    // Debug, as much as you can tolerate (2)
bool                freqResp        = false;  // Perform frequency response test

// Constants always defined
// #define CONSTANT
#ifdef ARDUINO
  #define BUTTON_PIN       2                  // Button 3-way input momentary 5V, steady GND (D2)
  #define PWM_PIN          5                  // PWM output (PD5)
  #define POT_PIN          A0                 // Potentiometer input pin on Photon (PC0)
  #define EMF_PIN          A2                 // Fan speed back-emf input pin on Photon (PC2)
  #define CL_PIN           4                  // Closed loop 3-way switch 5V or GND (D4 to GND)
  #define FR_DELAY         4000000UL          // Time to start FR, micros
  #define CLOCK_TCK        16UL               // Clock tick resolution, micros
  #define PUBLISH_DELAY    60000UL            // Time between cloud updates (), micros
  #define CONTROL_DELAY    15000UL            // Control law wait (), micros
  #define INSCALE          1023.0             // Input full range from OS
#else   // Photon
  #define PWM_PIN          A4                 // PWM output (A4)
  #define POT_PIN          A0                 // Potentiometer input pin on Photon (A0)
  #define EMF_PIN          A2                 // Fan speed back-emf input pin on Photon (A2)
  #define LED_PIN          D7                 // Status LED
  #define CL_PIN           D0                 // Closed loop 3-way switch 5V or GND  (D0)
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
LagTustin*          modelFilterG;             // Tustin lag filter gas gen
LagTustin*          modelFilterF;             // Tustin lag filter fan
LagTustin*          modelFilterV;             // Tustin lag filter F2V sensor
FRAnalyzer*         analyzer;                 // Frequency response analyzer
Servo               myservo;  // create servo object to control a servo
double              modelE          = 0;      // Model ESC output, %Ng
double              modelG          = 0;      // Model Gas Generator output, %Ng
double              modelF          = 0;      // Model Fan, %Nf
double              modelFS         = 0;      // Model Fan Sensed, %Nf
const  double       tau             = 0.10;   // Input noise filter time constant, sec
const  double       tldE            = 0.05;   // Model ESC lead time constant, sec
const  double       tauE            = 0.01;   // Model ESC lag time constant, sec
const  double       tauV            = 0.07;   // Model F2V time constant, sec
const  double       tauG            = 0.13;   // Model Gas Generator time constant, sec
const  double       tauF            = 0.13;   // Model Fan time constant, sec
double              throttle        = 0;      // Servo value, 0-179 degrees
double              throttleM       = 0;      // Modeled servo value, 0-179 degrees
double              throttleL       = 0;      // Limited servo value, 0-179 degrees
double              throttleML      = 0;      // Limited modeled servo value, 0-179 degrees
double              updateTime      = 0.0;    // Control law update time, sec
double              fn[4]           = {0, 0, 0, 0}; // Functions to analyze
const int           ix[2]           = {0, 0}; // Indeces of fn to excitations
const int           iy[2]           = {1, 2}; // Indeces of fn to responses

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
#else
  pinMode(BUTTON_PIN, INPUT);
#endif
  Serial.begin(230400);
  myservo.attach(PWM_PIN);  // attaches the servo.  Only supported on pins that have PWM
  pinMode(POT_PIN, INPUT);
  pinMode(EMF_PIN, INPUT);
  pinMode(CL_PIN,  INPUT);

  // Lag filter
  double T        = float(CONTROL_DELAY)/1000000.0;
  throttleFilter  = new LagTustin(    T, tau,  -0.1, 0.1);
  modelFilterE    = new LeadLagTustin(T, tldE, tauE,-0.1, 0.1);
  modelFilterG    = new LagTustin(    T, tauG, -0.1, 0.1);
  modelFilterF    = new LagTustin(    T, tauF, -0.1, 0.1);
  modelFilterV    = new LagTustin(    T, tauV, -0.1, 0.1);
  // analyzer        = new FRAnalyzer(-0.8, 2.3, 0.1,    2,    6,     1/tauG, double(CONTROL_DELAY/1e6), fn, ix, iy, 4, 4);
  //analyzer        = new FRAnalyzer(-0.8, 2.3, 0.1,    2,    6,     1/tauG, double(CONTROL_DELAY/1e6), fn, ix, iy, 4, 2);
  //analyzer        = new FRAnalyzer(1, 1.3, 0.1,    2,    6,     1/tauG, double(CONTROL_DELAY/1e6), fn, ix, iy, 4, 2, "t,ref,exc,thr,mod,nf,T");
  analyzer        = new FRAnalyzer(-0.8, 2.3, 0.1,    2,    6,     1/tauG, double(CONTROL_DELAY/1e6), fn, ix, iy, 4, 2, "t,ref,exc,thr,mod,nf,T");
 //                               wmin  wmax dw      minCy iniCy  wSlow
 // 2.3 is Nyquist for T=.015
  if (verbose>1) sprintf(buffer,"\nCalibrating ESC...");
  Serial.print(buffer);
  throttle = -10;
  while (throttle<179)
  {
    throttle = min(179, throttle+2);
    myservo.write(throttle);
    delay(20);
  }
  delay(1000);
  while (throttle>0)
  {
    throttle = max(0, throttle-2);
    myservo.write(throttle);
    delay(20);
  }  if (verbose>1) Serial.println("Done.");
  analyzer->publish();
  Serial.println("");

#ifndef ARDUINO
  WiFi.off();
#endif

  if (verbose>1)
  {
    sprintf(buffer, "time,cl,pcnfref,pcnf,err,state,thr,pcnfrefM,pcnfM,errM,stateM,thrM,modPcng,T\n");
    Serial.print(buffer);Serial.flush();
  }

  // Serial Event
#ifndef ARDUINO
  inputString.reserve(200);   // Reserve 200 bytes for inputString
#endif

  delay(1000);
}



void loop() {
#ifdef ARDUINO
  int                     buttonState = 0;    // Pushbutton
#endif
  bool                    closingLoop = false;// Closing loop by pin cmd, T/F
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
  const double            P_V4_NF[3]   = {0, 7734,-587}; // Coeff V4(v) to NF(rpm)
  const double            P_LT_NG[2]   = {-74859,20204}; // Coeff log(throttle(deg)) to NG(rpm)
  const double            P_F_PNG[2]   = {0,     4189};  // Coeff F2V(v) to NG(rpm)
  const double            P_F_NG[2]    = {0,     20};    // Coeff F2V(v) to PCNG(%)
  const double            P_F_PNF[2]   = {-5000, 4239};  // Coeff F2V(v) to NF(rpm)
  const double            P_F_NF[2]    = {-24,   20};    // Coeff F2V(v) to PCNF(%)
  const double            P_P_PNF[2]   = {0,     16};    // Coeff pot(v) to PCNF(%)
  const double            P_P_THTL[2]  = {40,    15};    // Coeff pot(v) to throttle(deg)
  const double            P_NG_NF[2]   = {-4432, 0.9849}; // Coeff NG(rpm) to NF(rpm)
  const double            P_NF_NG[2]   = {4586,  1.007};  // Coeff NF(rpm) to NG(rpm)
  const double            F2V_MAX      = 3.3; // Maximum F2V value, vdc
  const double            F2V_MIN      = 0;   // Minimum F2V value, vdc
  const double            POT_MAX      = 5;   // Maximum POT value, vdc
  const double            POT_MIN      = 0;   // Minimum POT value, vdc
  const double            THTL_MAX     = 115;    // Maximum throttle to prevent shutdown due to small charger, deg
  const double            THTL_MIN     = 40;     // Minimum throttle, deg
  const double            RPM_P        = 209.455; // (rpm/%)
  const double            SCMAXI       = 30*float(CONTROL_DELAY)/1000000.0;    // Maximum allowable step change in reset, deg/update
  const double            SCMAX        = 240*float(CONTROL_DELAY)/1000000.0;   // Maximum allowable step change in reset, deg/update
  #ifdef ARDUINO
    const double      Ki              = 10.10/3.125;  // Int gain, deg/s/%Nf
    const double      Kp              = 2.61/3.125;   // Prop gain, deg/%Nf
    const double      KiM             = 9.69/3.125;   // Int gain, deg/s/%Nf
    const double      KpM             = 2.52/3.125;   // Prop gain, deg/%Nf
  #else  // Photon
    const double      Ki              = 11.15/3.125;  // Int gain, deg/s/%Nf
    const double      Kp              = 3.0/3.125;    // Prop gain, deg/%Nf
    const double      KiM             = 11.15/3.125;  // Int gain, deg/s/%Nf
    const double      KpM             = 3.0/3.125;    // Prop gain, deg/%Nf
  #endif
  static double       pcnf            = 0;      // Fan speed, % of 57600 rpm
  static double       vemf            = 0;      // Converted sensed back emf LM2907 circuit measure, volts
  static double       vpot_filt       = 0;      // Pot value, 0-179 degrees
  static double       vpot            = 0;      // Pot value, 0-179 degrees
  static int          emfValue        = INSCALE/4;   // Dial raw value, 0-4096
  static int          potValue        = INSCALE/3;   // Dial raw value, 0-4096

  // Executive
  if ( start == 0UL ) start = now;
  elapsedTime  = double(now - start)*1e-6;
#ifdef BARE_PHOTON
  closingLoop = true;
#else
  closingLoop = (digitalRead(CL_PIN) == HIGH);
#endif
#ifdef ARDUINO
  buttonState = digitalRead(BUTTON_PIN);
  if ( buttonState == HIGH && (now-lastButton>2000000UL ) )
  {
    lastButton = now;
    analyzer->complete(freqResp);  // reset if doing freqResp
    freqResp = !freqResp;
  }
#endif
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

  // Report that running Freq Resp
#ifndef ARDUINO
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
       Serial.print(inputString);
       Serial.print(doFR);
       Serial.println(":");
       analyzer->complete(freqResp);  // reset if doing freqResp
       freqResp = !freqResp;
    }
    inputString = "";
    stringComplete  = false;
  }
#endif

  // Interrogate inputs
  if ( control )
  {
#ifdef BARE_PHOTON
    pcnf      = modelFS;
#else
    potValue  = analogRead(POT_PIN);
    emfValue  = analogRead(EMF_PIN);
    vemf      = double(emfValue)/INSCALE*F2V_MAX;
    pcnf      = fmin(fmax(P_V4_NF[0] + vemf*(P_V4_NF[1] + vemf*P_V4_NF[2])/RPM_P, 0.0), 100);
#endif
    vpot      = fmin(fmax(double(potValue)/INSCALE*POT_MAX, POT_MIN), POT_MAX);
    fn[2]     = pcnf;
  }

  // Control law
  if ( control )
  {
    vpot_filt   = throttleFilter->calculate(vpot,  RESET);
    if ( closingLoop )
    {
      pcnfRef     = P_P_PNF[0]  + vpot_filt*P_P_PNF[1];
      if ( freqResp ) pcnfRef *= (1+exciter/20);
    }
    else              pcnfRef     = P_P_THTL[0] + vpot_filt*P_P_THTL[1];
    if ( RESET )
    {
      double throttleRPM  = P_NF_NG[0] + pcnfRef*RPM_P*P_NF_NG[1];      // RPM Ng
      intState  = intStateM = exp((throttleRPM-P_LT_NG[0])/P_LT_NG[1]); // deg throttle
      pcnf      = pcnfRef;
      modelFS   = pcnfRef;
    }
    e           = pcnfRef - pcnf;
    eM          = pcnfRef - modelFS;
    if ( !closingLoop ) intState = vpot_filt;
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
    else
    {
      double throttleRPM    = (P_NF_NG[0] + pcnfRef*RPM_P*P_NF_NG[1]);          // RPM Ng
      double throttleU      = exp((throttleRPM-P_LT_NG[0])/P_LT_NG[1]) * (1+exciter/20);  // deg throttle
      // Apply rate limits as needed
      if ( RESET )
      {
        throttleL = throttleML = fmax(fmin(throttleU, throttleL+SCMAX), throttleL-SCMAX);
      }
      else
      {
        throttleL   = throttleU;
      }
      if ( freqResp ) throttle = throttleM  = throttleU* (1+exciter/20);
      else            throttle  = throttleM = throttleL;
    }
    fn[0] = throttle;
  }

  // Model
  if ( control )
  {
    if ( RESET )
      modPcng   = (P_NF_NG[0] + pcnfRef*RPM_P*P_NF_NG[1])/RPM_P;
    else
      modPcng   = fmax((P_LT_NG[0] + P_LT_NG[1]*log(throttleM)) / RPM_P, 0.0);
    modelE      = modelFilterE->calculate(modPcng, RESET);
    modelG      = modelFilterG->calculate(modelE,  RESET);
    modelF      = modelFilterF->calculate((P_NG_NF[0] + modelG*RPM_P*P_NG_NF[1])/RPM_P,  RESET);
    modelFS     = modelFilterV->calculate(modelF,  RESET);
    if ( analyzing ) exciter = analyzer->calculate();  // use previous exciter for everything
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
      Serial.println("");Serial.flush();
    }  // freqResp
    else
    {
      if (verbose>1) sprintf(buffer, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n",
        String(elapsedTime,6).c_str(), String(closingLoop).c_str(),
        String(pcnfRef).c_str(), String(pcnf).c_str(), String(e).c_str(),
        String(intState).c_str(), String(throttle).c_str(),
        String(pcnfRef).c_str(), String(modelF).c_str(),
        String(eM).c_str(), String(intStateM).c_str(), String(throttleM).c_str(),
        String(modPcng).c_str(), String(updateTime,6).c_str());
      Serial.print(buffer);Serial.flush();
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
