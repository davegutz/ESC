/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

//#include <Servo.h>
#define POT_PIN          A2                 // Potentiometer input pin on Photon (A2)

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(D0);  // attaches the servo on pin D0 to the servo object
  // Only supported on pins that have PWM
  pinMode(POT_PIN,    INPUT);
}

void loop() {
// Interrogate pot; run fast for good tactile feedback
// my pot puts out 2872 - 4088 observed using following
#ifndef BARE_PHOTON
  potValue    = 4095 - analogRead(POT_PIN);
#else
  potValue    = 2872;
#endif
potDmd      = roundf((float(potValue)-2872)/(4088-2872)*26+47);

  myservo.write(potDmd);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}
