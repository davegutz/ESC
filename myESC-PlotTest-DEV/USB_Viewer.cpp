// Standard
#ifdef ARDUINO
  #include <Arduino.h> //needed for Serial.println
#else
  #include "application.h"      // Should not be needed if file .ino or Arduino
#endif
#include "USB_Viewer.h"
extern const int verbose;

// Class USB_Viewer
USB_Viewer::USB_Viewer()
: config_message_(""), deltaT_(20000), first_time_(true), inputString_(""), scaleT_(1),
  stringComplete_(false){}
USB_Viewer::USB_Viewer(const String msg, const unsigned long deltaT, const unsigned long scaleT)
: config_message_(msg), deltaT_(deltaT), first_time_(true), inputString_(""),  scaleT_(scaleT),
  stringComplete_(false)
{
  // Initialization for data transfer from host.
  inputString_.reserve(10);  // Reserve space for rcvd data
  stringComplete_ = false;

  // Initialize inputs from host.
  desired_ = 0;
  Kp_      = 0;
  direct_  = 0;//direct term in motor command

  // Initialize timeSync
  timeSyncInit_();
}

// Creates a signed byte number from val, without using 0 and 255
byte USB_Viewer::signedByte_(int val)
{
  return byte(min(max(val+128,1),254));
}

// Packs up a 5-byte status package for host.
void USB_Viewer::printStatus(int mSpd, int error, int mCmd, int hdrm)
{
  char *d;
  d = buf_;
  // Start Byte.
  *d = byte(0);
  d++;
  *d = (uint8_t) min(max(mSpd,0),255);
  d++;
  *d = (int8_t) min(max(error,-128),127);
  d++;
  *d = (uint8_t) min(max(mCmd,0),255);
  d++;
  memcpy(d,&hdrm,2);
  d+=2;
  // Stop byte (255 is not unique).
  *d = byte(255);

  // Write the buffer to the host
#ifdef ARDUINO
  Serial.write(buf_, 7);//for 7 byte message.
#else
  Serial.write(buf_);
#endif
}


void USB_Viewer::serialEvent()
{
  if (! first_time_)
  {
    while (Serial.available())
    {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      inputString_ += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if (inChar == '\n')
      {
        stringComplete_ = true;
        break;
      }
    }
  }
}



void USB_Viewer::processString()
{
  char St = inputString_.charAt(0);
  inputString_.remove(0,1);
  float val = inputString_.toFloat();
  switch (St)
  {
    case 'P':
      Kp_ = val;
      break;
    case 'O':
      direct_ = val;
      break;
    case 'A':
      desired_ = val;
      break;
    case '~':
      first_time_ = true;
      break;
    default:
    break;
  }
  inputString_ = "";
  stringComplete_ = false;
}


void USB_Viewer::timeSyncInit_()
{
  headroom_ts_  = deltaT_;
  starttime_    = micros();
}


// Wait until it has been deltaT since this was last called.
// Returns headroom, delay in units of 5us.
int USB_Viewer::timeSync()
{
  unsigned long delayMuS = max((deltaT_ - (micros()-starttime_))/scaleT_, 1);
  if (delayMuS > 5000)
  {
    delay(delayMuS / 1000);
    delayMicroseconds(delayMuS % 1000);
  }
  else
  {
    delayMicroseconds(delayMuS); // Wait to start exactly deltaT after last start
  }

  headroom_ts_  = min(headroom_ts_,delayMuS);  // min loop headroom should be > 0!!
  starttime_    = micros(); // Note time loop begins


  return min(headroom_ts_, 32767); // Truncate headroom_ts_ to an int.
}

bool USB_Viewer::init_gui_()
{
  delay(1);
  if(Serial.available())
  {
    while (Serial.available())
    {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      inputString_ += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if (inChar == '\n')
      {
        stringComplete_ = true;
        break;
      }
    }
    if (inputString_=="SET\n")
    {
      first_time_ = false;
      return true;//gui initialized
    }
  }
  return false;
}


void USB_Viewer::startup()
{
  if (first_time_)
  {
    Serial.println(config_message_);
    while (first_time_)
    {  //wait here until complete/happy
      bool is_ok = init_gui_();
      if (is_ok)
      {
        delay(2000);
        timeSyncInit_(); //right before loop proper starts
      }
    }
  }
}
