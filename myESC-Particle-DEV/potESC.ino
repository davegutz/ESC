//Sometimes useful for debugging
//#pragma SPARK_NO_PREPROCESSOR
//

// Standard
#ifdef ARDUINO
#include <Servo.h>
#include <Arduino.h>     // Used instead of Print.h - breaks Serial
#else                    // Photon
#include "application.h" // Should not be needed if file ino or Arduino
SYSTEM_THREAD(ENABLED); // Make sure code always run regardless of network status
#endif
#include "myAnalyzer.h"
#include "myTables.h"
#include "math.h"

// Test features
extern int verbose = 1; // Debug, as much as you can tolerate (1)
extern bool bareOrTest = false;       // Fake inputs and sensors for test purposes (false)
bool beSquare = false;         // Do step instead of freqResp (false)
double stepVal = 6;           // Step input, %nf.  Try to make same as freqRespAdder (6)

/*
Controlling a servo position using a potentiometer (variable resistor)
by Dave Gutz

Connections for Photon:
  ESC ------------------- Photon
    BLK ------------------- GND
    WHT ------------------- PWM Output (A4)
  ESC----------------- 3-wire DC Servomotor
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
    ESC:Turnigy Plush 25A, 2-4S LiPo
    Power Supply:  BINZET AC 100-240V to DC 12V 10A
    Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
    Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
    F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)


Connections for Arduino:
  ESC ------------------- Arduino
    BLK ------------------- GND
    WHT ------------------- PWM Output (5)
  ESC----------------- 3-wire DC Servomotor
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
    ESC:Turnigy Plush 25A, 2-4S LiPo
    Power Supply:  BINZET AC 100-240V to DC 12V 10A
    Potentiometer:  Mouser 314-1410F-10K-3  10K Linear Pot
    Motor:  Hobby King 50mm Alloy EDF 4800 Kv (3s Version)
    F2V:  Texas Instruments 926-LM2907N/NOPB (14 pin, no Zener)

  Reaquirements:
  Prime:
  1.  Inputs:  fan speed, pushbutton, slide switch, potentiometer
  2.  Outputs: ESC command using TTL/PWM servo control library
  3.  Manually sweep ESC command from min to max using pot.
  4.  Limit ESC command for safety using configurable parameters.
  5.  Turn on to minimum ESC input level.  Calibration of the ESC is done elsewhere.
  6.  Switch from open to closed loop and vice versa with small, <5 deg throttle
      bump, in response to switch change.
  7.  Perform frequency response analysis in response to button push.
  8.  Update closed loop algorithms, sense inputs, and produce outputs at rate
      at least as fast as 0.015 second update rate.
  Secondary:
  1.  Repeated Pushbutton toggles frequency response in progress.   When
      restarting, it begins completely fresh.
  2.  For non-Arduino, may also use a software Pushbutton - send string on serial.
  3.  Embed a plant model for dry code checkouts.
  4.  Filter pot noise.

  Tasks TODO:
  1.  Photon frequency response with button

  Revision history:
    31-Aug-2016   DA Gutz   Created
    13-Sep-2016   DA Gutz   Initial analyzing
    30-Sep-2016   DA Gutz   Arduino added
    10-Oct-2016   DA Gutz   First frequency response completion
    30-Oct-2016   DA Gutz   Gain scheduling
    16-Nov-2016   DA Gutz   Retune again again

  Distributed as-is; no warranty is given.
*/

// Test features usually commented
//

// Disable flags if needed.  Usually commented
//#define DISTURB_CONTROL                       // Use disturbance rejection gains in CLAW
//#define USE_FIXED_LL                          // Use the fixed lead lag compensator

// Constants always defined
// #define CONSTANT
#ifdef ARDUINO
#define BUTTON_PIN 2                                           // Button 3-way input momentary 3.3V, steady GND (D2)
#define PWM_PIN 5                                              // PWM output (PD5)
#define POT_PIN A0                                             // Potentiometer input pin on Arduino (PC0)
#define F2V_PIN A2                                             // Fan speed back-emf input pin on Arduino (PC2)
#define CL_PIN 4                                               // Closed loop 3-way switch 5V or GND (D4 to GND)
#define CLOCK_TCK 16UL                                         // Clock tick resolution, micros
#define INSCALE 1023.0                                         // Input full range from OS
const double vpotHalfDB = 0.05;                                // Half deadband sliding deadband filter, volts
const double POT_MAX = 5.0;                                    // Maximum POT value, vdc
const double F2V_MAX = 5.0;                                    // Maximum F2V value, vdc
const double POT_BIA = 0.36 + vpotHalfDB;                      // Pot adder, vdc
const double POT_SCL = (4.6 - vpotHalfDB - POT_BIA) / POT_MAX; // Pot scalar, vdc
#else                                                          // Photon
#define BUTTON_PIN D2                                          // Button 3-way input momentary 3.3V, steady GND (D2)
#define PWM_PIN A4                                             // PWM output (A4)
#define POT_PIN A0                                             // Potentiometer input pin on Photon (A0)
#define F2V_PIN A2                                             // Fan speed back-emf input pin on Photon (A2)
#define CL_PIN D0                                              // Closed loop 3-way switch 3.3V or GND  (D0)
#define CLOCK_TCK 8UL                                          // Clock tick resolution, micros
#define INSCALE 4096.0                                         // Input full range from OS
const double vpotHalfDB = 0.0;                    // Half deadband sliding deadband filter, volts
const double POT_MAX = 3.3;                       // Maximum POT value, vdc
const double F2V_MAX = 3.45;                      // Maximum F2V value, vdc
const double POT_BIA = 0.0;                       // Pot adder, vdc
const double POT_SCL = (3.3 - POT_BIA) / POT_MAX; // Pot scalar, vdc
#endif
#define PUBLISH_DELAY 150000UL // Time between cloud updates (), micros
#define CONTROL_DELAY 15000UL // Control law wait (), micros
#define FR_DELAY 4000000UL    // Time to start FR, micros
const double F2V_MIN = 0.0;   // Minimum F2V value, vdc
const double POT_MIN = 0;     // Minimum POT value, vdc
const double DENS_SI = 1.225; // Air density, kg/m^3

// Test
bool freqResp = false;             // Perform frequency response test status
const int nsigFn = 4;              // Length of fn
const int ntfFn = 2;               // Number of transfer  functions to calculate <= length(ix)
double fn[4] = {0, 0, 0, 0};       // Functions to analyze
const int ix[2] = {0, 0};          // Indeces of fn to excitations
const int iy[2] = {1, 2};          // Indeces of fn to responses
const double freqRespScalar = 1e8; // Use 40 for +/-3 deg, 20 for +/-6 deg, 13 for +/-10 at 50% Nf
const double freqRespAdder = 6;    // +/- deg

//
// Dependent includes
#include "myCLAW.h"
#include "myFilters.h"

// Global variables
double throttle = -5; // Servo value, 0-179 degrees
char buffer[256];
LagTustin *throttleFilter; // Tustin lag noise filter
FRAnalyzer *analyzer;      // Frequency response analyzer
Servo myservo;             // create servo object to control dc motor
ControlLaw *CLAW;          // Control Law

#ifndef ARDUINO
String inputString = "";        // a string to hold incoming data
boolean stringComplete = false; // whether the string is complete
#endif

void setup()
{
#ifndef ARDUINO
  WiFi.disconnect();
#endif
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(230400);
  myservo.attach(PWM_PIN, 1000, 2000); // attaches the servo.  Only supported on pins that have PWM
  pinMode(POT_PIN, INPUT);
  pinMode(F2V_PIN, INPUT);
  pinMode(CL_PIN, INPUT);

  // Lag filter
  double T = float(CONTROL_DELAY) / 1000000.0;
  throttleFilter = new LagTustin(T, tau, -0.1, 0.1);

  //                        wmin    wmax  dw    minCy numCySc  iniCy  wSlow
  //  analyzer = new FRAnalyzer(-0.8,   2.1,  0.1,  2,    1.0,     6,     1 / tauG,
  analyzer = new FRAnalyzer(-0.8,   1.4,  0.1,  2,  1.0, 6, 1 / tauG,
                            double(CONTROL_DELAY / 1e6), ix, iy, nsigFn, ntfFn, "t,ref,exc,thr,mod,nf,T"); // 15 ms any
                                                                                                           // 2.2 is Nyquist for T=.020
                                                                                                           // 1.4 is 25 r/s

  myservo.write(throttle);

// Serial headers used by plotting programs
// Header for analyzer.  TODO:  should be done in analyzer.cpp so done when needed
#ifndef ARDUINO
  delay(5000); // Allow time to start serial monitor for Photon.  Ardino auto-starts
#endif
  /*
    analyzer->publish();
    Serial.println("");
    */
  // Header for time data
  if (verbose > 0)
  {
    sprintf(buffer, "time,cl,vpot,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
    Serial.print(buffer);
  }

#ifndef ARDUINO
  // Serial Event to allow switches to be passed in by user Serial transmit
  inputString.reserve(200); // Reserve 200 bytes for inputString Serial Event
#endif

  // Instatiate gain scheduling tables
  CLAW = new ControlLaw(T, DENS_SI);

#ifdef ARDUINO
  delay(100);
#else
  delay(1000);
  WiFi.off();
  delay(1000);
#endif
}

void loop()
{
  int buttonState = 0;                    // Pushbutton
  static bool closingLoop = false;        // Closing loop by pin cmd, T/F
  static bool stepping = false;           // Step by Photon send String
  bool control;                           // Control sequence, T/F
  bool publish;                           // Publish, T/F
  bool analyzing;                         // Begin analyzing, T/F
  unsigned long now = micros();           // Keep track of time
  static unsigned long start = 0UL;       // Time to start looping, micros
  double elapsedTime;                     // elapsed time, micros
  static double updateTime = 0.0;         // Control law update time, sec
  static unsigned long lastControl = 0UL; // Last control law time, micros
  static unsigned long lastPublish = 0UL; // Last publish time, micros
  static unsigned long lastButton = 0UL;  // Last button push time, micros
  static unsigned long lastFR = 0UL;      // Last analyzing, micros
  static int RESET = 1;                   // Dynamic reset
  const double RESEThold = 5;             // RESET hold, s
  static double exciter = 0;              // Frequency response excitation, fraction
                                          ////////////////////////////////////////////////////////////////////////////////////
  static double vf2v = 0;                 // Converted sensed back emf LM2907 circuit measure, volts
  static double vpot_filt = 0;            // Pot value, volts
  static double vpotDead = 0;             // Sliding deadband value, volts
  static double vpot = 0;                 // Pot value, volts
  static int f2vValue = INSCALE / 4;      // Dial raw value
  static int potValue = INSCALE / 3;      // Dial raw value

  // Executive
  if (start == 0UL) start = now;
  elapsedTime = double(now - start) * 1e-6;
  if (bareOrTest)
  {
#ifdef ARDUINO
    closingLoop = true;
#endif
  }
  else
  {
    closingLoop = (digitalRead(CL_PIN) == HIGH);
  }
  buttonState = digitalRead(BUTTON_PIN);
#ifdef ARDUINO
  if (buttonState == HIGH && (now - lastButton > 2000000UL))
  {
    lastButton = now;
    analyzer->complete(freqResp); // reset if doing freqResp
    freqResp = !freqResp;
    if (beSquare)
    {
      freqResp = false;
      stepping = true;
      stepVal = -stepVal;
    }
  }
#endif
  publish = ((now - lastPublish) >= PUBLISH_DELAY - CLOCK_TCK / 2);
  if (publish)
  {
    lastPublish = now;
  }
  unsigned long deltaTick = now - lastControl;
  control = (deltaTick >= CONTROL_DELAY - CLOCK_TCK / 2);
  if (control)
  {
    updateTime = float(deltaTick) / 1000000.0;
    lastControl = now;
  }
  if (freqResp)
    analyzing = ((now - lastFR) >= FR_DELAY && !analyzer->complete());
  else
    analyzing = false;

#ifndef ARDUINO
  // Serial event  (terminate Send String data with 0A using CoolTerm)
  if (stringComplete)
  {
    String doFR = "f\n";
    if (inputString == doFR)
    {
      analyzer->complete(freqResp); // reset if doing freqResp
      freqResp = !freqResp;
    }
    String doBareOrTest = "t\n";
    if (inputString == doBareOrTest)
    {
      bareOrTest = !bareOrTest;
    }
    String doCL = "c\n";
    if (inputString == doCL)
    {
      closingLoop = !closingLoop;
    }
    String doStep = "s\n";
    if (inputString == doStep)
    {
      stepping = true;
      stepVal = -stepVal;
    }
    if (inputString.charAt(0) == 'v')
    {
      int vcheck = atoi(inputString.substring(1));
      if (vcheck>=0 && vcheck<10) verbose = vcheck;
    }
    inputString = "";
    stringComplete = false;
  }
#endif

  // Interrogate inputs
  if (control)
  {
    if (!bareOrTest)
    {
      potValue = analogRead(POT_PIN);
      f2vValue = analogRead(F2V_PIN);
    }
    vf2v = double(f2vValue) / INSCALE * F2V_MAX;
    vpot = fmin(fmax((double(potValue) / INSCALE * POT_MAX - POT_BIA) / POT_SCL, POT_MIN), POT_MAX);
  }

  // Control law
  if (control)
  {
    vpotDead = fmax(fmin(vpotDead, vpot + vpotHalfDB), vpot - vpotHalfDB);
    if (!freqResp)
      vpot_filt = throttleFilter->calculate(vpotDead, RESET); // Freeze pot for FR
    double potThrottle = vpot_filt * THTL_MAX / POT_MAX;      // deg
    double dNdT = P_LT_NG[1] / fmax(potThrottle, 1) / RPM_P;  // Rate normalizer, %Ng/deg
    potThrottle += stepping * stepVal / dNdT;
    throttle = CLAW->calculate(RESET, updateTime, closingLoop, analyzing, freqResp, exciter, freqRespScalar, freqRespAdder, potThrottle, vf2v);
    if (elapsedTime > RESEThold)
      RESET = 0;
  }

  // Commands to Hardware
  if (control)
  {
    myservo.write(throttle); // sets the servo position according to the scaled value
  }

  // Calculate frequency response
  if (control)
  {
    fn[0] = throttle;
    fn[1] = CLAW->modelTS();
    fn[2] = CLAW->pcnt();
    fn[3] = CLAW->pcntRef();
    if (analyzing)
      exciter = analyzer->calculate(fn, nsigFn); // use previous exciter for everything
  }

  // Publish results to serial bus
  if (publish)
  {
    if (freqResp)
    {
      if (verbose > 1 || (beSquare && verbose>0) )
      {
        sprintf(buffer, "%s,%s,%s,%s,%s,%s,%s,",
                String(elapsedTime, 6).c_str(), String(CLAW->pcntRef()).c_str(),
                String(exciter).c_str(), String(throttle).c_str(),
                String(CLAW->modelTS()).c_str(), String(CLAW->pcnt()).c_str(),
                String(updateTime, 6).c_str());
        Serial.print(buffer);
        if (!analyzer->complete())
        {
          analyzer->publish();
        }
        Serial.println("");
      }
    } // freqResp
    else
    {
    sprintf(buffer, "time,cl,vpot,  pcntref,pcntSense,pcntSenseM,  err,state,thr, modPcng,T\n");
      if (verbose > 0)
      {
        sprintf_P(buffer, PSTR("%s,"), String(elapsedTime, 6).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s, "), String(closingLoop).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,  "), String(vpot).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,"), String(CLAW->pcntRef()).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,"), String(CLAW->pcnt()).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,  "), String(CLAW->modelTS()).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,"), String(CLAW->e()).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,"), String(CLAW->intState()).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,  "), String(throttle).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,"), String(CLAW->modelG()).c_str());
        Serial.print(buffer);
        sprintf_P(buffer, PSTR("%s,\n"), String(updateTime, 6).c_str());
        Serial.print(buffer);
      }
    }
  } // publish
  if (analyzer->complete())
    freqResp = false;
}

#ifndef ARDUINO
/*
  Special handler that uses built-in callback.
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
 */
void serialEvent()
{
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n')
    {
      stringComplete = true;
    }
  }
}
#endif
