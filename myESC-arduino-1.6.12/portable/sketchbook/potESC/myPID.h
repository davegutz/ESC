#ifndef _myPID_h
#define _myPID_h

// Control Ref Step Fixed Lead 12/01/2016
static const double tldF    = 0.15;                                           // Lead, s
static const double tlgF    = 0.03;                                           // Lag, s
static const double yLG[6]  = {3.6,   3.6,    3.75,   4.05,   5.,     5.};    // Loop gain, r/s
static const double yTLD[6] = {0.420, 0.420,  0.240,  0.125,  0.090,  0.090}; // Lead, s

#endif
