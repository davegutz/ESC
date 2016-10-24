#ifndef __USB_VIEWER_H__
#define __USB_VIEWER_H__

class USB_Viewer
{
public:
  USB_Viewer();
  USB_Viewer(const String msg, const unsigned long deltaT, const unsigned long scaleT);
  ~USB_Viewer(){};
  // operators
  // functions
  void      printStatus(int mSpd, int error, int mCmd, int hdrm);
  void      processString(void);
  void      serialEvent(void);
  void      startup(void);
  int       timeSync(void);
  boolean   stringComplete(){return stringComplete_;};
  int       direct(){return direct_;};
  float     Kp(){return Kp_;};
  float     desired(){return desired_;};
private:
  byte      signedByte_(int val);
  void      timeSyncInit_(void);
  bool      init_gui_(void);
  char            buf_[8];          // five bytes to hold data for host.
  String          config_message_;
  unsigned long   deltaT_;
  float           desired_;         // Desired value (speed or angle) from host.
  int             direct_;          // Direct motor command from host.
  bool            first_time_;
  unsigned  long  headroom_ts_;
  String          inputString_;     //holds received serial data.
  float           Kp_;              // Feedback gains (proportional)
  unsigned  long  scaleT_;
  unsigned  long  starttime_;
  boolean         stringComplete_;  // String received flag
 };

#endif

