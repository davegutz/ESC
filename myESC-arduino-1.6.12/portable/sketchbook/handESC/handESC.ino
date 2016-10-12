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

//
// Test features usually commented
//
// Disable flags if needed.  Usually commented
// #define DISABLE
//#define BARE_PHOTON                       // Run bare photon for testing.  Bare photon without this goes dark or hangs trying to write to I2C

// Test features
extern  const int   verbose         = 2;    // Debug, as much as you can tolerate (2)

// Constants always defined
// #define CONSTANT
#ifdef ARDUINO
  #define BUTTON_PIN       2                  // Button 3-way input momentary 3.3V, steady GND (D2)
  #define PWM_PIN          5                  // PWM output (PD5)
  #define POT_PIN          A0                 // Potentiometer input pin on Arduino (PC0)
  #define CLOCK_TCK        16UL               // Clock tick resolution, micros
  #define PUBLISH_DELAY    60000UL            // Time between cloud updates (), micros
  #define CONTROL_DELAY    15000UL            // Control law wait (), micros
  #define INSCALE          1023.0             // Input full range from OS
#else   // Photon
  #define BUTTON_PIN       D2                  // Button 3-way input momentary 3.3V, steady GND (D2)
  #define PWM_PIN          A4                 // PWM output (A4)
  #define POT_PIN          A0                 // Potentiometer input pin on Photon (A0)
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

// Global variables
char buffer[256];
Servo               myservo;  // create servo object to control a servo
double              throttleL       = 0;      // Limited servo value, 0-179 degrees
double              throttle        = -5;     // Servo value, 0-179 degrees
double              updateTime      = 0.0;    // Control law update time, sec

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

  delay(1000);

#ifndef ARDUINO
  WiFi.off();
#endif
  delay(100);
  sprintf(buffer, "elapsedTime,vpot,throttleU,throttle,updateTime\n");Serial.print(buffer);
}



void loop() {
#ifdef ARDUINO
  int                     buttonState = 0;    // Pushbutton
#endif
  bool                    control;            // Control sequence, T/F
  bool                    publish;            // Publish, T/F
  unsigned long           now = micros();     // Keep track of time
  static unsigned long    start        = 0UL; // Time to start looping, micros
  double                  elapsedTime;        // elapsed time, micros
  static unsigned long    lastControl  = 0UL; // Last control law time, micros
  static unsigned long    lastPublish  = 0UL; // Last publish time, micros
  static unsigned long    lastButton   = 0UL; // Last button push time, micros
  static int              RESET        = 1;   // Dynamic reset
  const  double           RESEThold    = 5;   // RESET hold, s
  // See calibration20160913.xlsx for these hardware conversion derivations
  #ifdef ARDUINO
  const double            POT_MAX      = 5.0; // Maximum POT value, vdc
  const double            POT_MIN      = 0;   // Minimum POT value, vdc
  const double            THTL_MAX     = 130;    // Maximum throttle to prevent shutdown due to small charger, deg
  const double            THTL_MIN     = 75;     // Minimum throttle, deg
  #else  // Photon
  const double            POT_MAX      = 3.3; // Maximum POT value, vdc
  const double            POT_MIN      = 0;   // Minimum POT value, vdc
  const double            THTL_MAX     = 115;    // Maximum throttle to prevent shutdown due to small charger, deg
  const double            THTL_MIN     = 60;     // Minimum throttle, deg
  #endif
////////////////////////////////////////////////////////////////////////////////////
  const double            SCMAXI       = 1*float(CONTROL_DELAY)/1000000.0;    // Maximum allowable step change reset, deg/update
  const double            SCMAX        = 240*float(CONTROL_DELAY)/1000000.0;     // Maximum allowable step change, deg/update
  static double       vpot            = 0;      // Pot value, volts
  static int          potValue        = 0;   // Dial raw value

  // Executive
  if ( start == 0UL ) start = now;
  elapsedTime  = double(now - start)*1e-6;
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

  // Interrogate inputs
  if ( control )
  {
    potValue  = analogRead(POT_PIN);
    vpot      = fmin(fmax(double(potValue)/INSCALE*POT_MAX, POT_MIN), POT_MAX);
  }
  double throttleU = (vpot-POT_MIN)/(POT_MAX-POT_MIN)*(THTL_MAX-THTL_MIN)+THTL_MIN;
  // Control law
  if ( control )
  {
   // Apply rate limits as needed
    if ( RESET )
    {
      throttleL  = fmax(fmin(throttleU,   throttleL+SCMAXI),   throttleL-SCMAXI);
    }
    else
    {
      throttleL  = fmax(fmin(throttleU,   throttleL+SCMAX),   throttleL-SCMAX);
    }
    throttle  = fmin(fmax(throttleL, THTL_MIN), THTL_MAX);
}

  if ( control )
  {
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
      sprintf(buffer, "%s,%s,%s,%s,%s,\n",
        String(elapsedTime,6).c_str(), String(vpot).c_str(),
        String(throttleU).c_str(), String(throttle).c_str(),
        String(updateTime,6).c_str());
      Serial.print(F(buffer));
  }  // publish
}
