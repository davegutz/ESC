#ifndef _myCLAW_h
#define _myCLAW_h

#include "myTables.h"
#include "myFilters.h"

/* Control11/2/2016
static const    double    xALL[6]     = {0,     16,  	35,   	50,   	79,   	100};   // Gain breakpoints, %Nf
static const    double    yLG[6]      = {2.5, 	2.5, 	3.0, 	  3.1, 	  2.8,	  2.8};   // Loop gain, r/s
static const    double    yTLD[6]     = {0.36,  0.36, 0.36,  	0.36, 	0.36, 	0.36};  // Lead, s
*/

// Control 11/4/2016
static const    double    xALL[6]     = {0,     9,     24,     45,     74,    100,};      // Gain breakpoints, %Nf
static const    double    yLG[6]      = {3.4,   3.4,   3.5,    4.3,    5.6,   5.6};       // Loop gain, r/s
static const    double    yTLD[6]     = {0.488, 0.488, 0.310,  0.200,  0.141, 0.141 };    // Lead, s

// Plant
static const    double          tau         = 0.10; // Input noise filter time constant, sec
static const    double          tldV        = 0.0;  // Model F2V lead time constant, sec
static const    double          tauF2V      = 0.1;  // Model F2V lag time constant, sec
static const    double          tldG        = 0.0;  // Model Gas Generator lead time constant, sec
static const    double          tauG        = 0.1;  // Model Gas Generator lag time constant, sec
static const    double          tldF        = 0.0;  // Model Fan lead time constant, sec
static const    double          J           = 3.496e-8;// Fan inertia, (rpm/s)/(ft-lbf)
static const    double          NG_MAX   	  = 100;  // Maximum trim, %Ng
static const    double          NG_MIN      = 0;    // Minimum trim, %Ng
static const    double          THTL_MAX    = 180;  // Maximum throttle to prevent shutdown due to small charger, deg
static const    double          THTL_MIN    = 0;    // Minimum throttle, deg
static const    double 	        RATE_MAX 	  = 240;  // Maximum throttle change rate, deg/sec to avoid lockout
// See calibration<date>.xlsx for these hardware conversion derivations
static const    double          P_LNT_TAU[2]= {0.698, -0.1573};   // Coeff Nt(%) to TauT(s) 
static const    double          P_V4_NF[3]  = {0, 14098, -171};   // Coeff V4(v) to NF(rpm)
static const    double          P_LT_NG[2]  = {-28327,  14190};   // Coeff throttle(deg) to NG(rpm)
static const    double          P_NG_NF[2]  = {-10231,  1.0237};  // Coeff NG(rpm) to NF(rpm)
static const    double          P_NF_NG[2]  = {10154,   0.9683};  // Coeff NF(rpm) to NG(rpm)
static const    double          P_N_Q[3]    = {0,  1.75e-7,  1.154e-11};  // Load line N(rpm) to Q(ft-lbf)
static const    double          RPM_P       = 461;  // (rpm/%)

class ControlLaw
{
public:
  ControlLaw();
  ControlLaw(const double T);
  ~ControlLaw(){};
  // operators
  // functions
  double  calculate(const int RESET, const double updateTime, const boolean closingLoop,
    const boolean analyzing, const boolean freqResp, const double exciter, const double freqRespScalar,
    const double freqRespAdder, const double potThrottle, const double vf2v);
  double  e(void){return(e_);};
  double  eM(void){return(eM_);};
  double  intState(void){return(intState_);};
  double  intStateM(void){return(intStateM_);};
  double  modelFS(void){return(modelFS_);};
  double  modelG(void){return(modelG_);};
  double  pcnf(void){return(pcnf_);};
  double  pcnfRef(void){return(pcnfRef_);};
  double  throttleM(void){return(throttleM_);};
private:
  LeadLagTustin *modelFilterG_; // Tustin lag model gas gen
  LeadLagTustin *modelFilterF_; // Tustin lag model fan
  LeadLagTustin *modelFilterV_; // Tustin lag model F2V sensor
  TableInterp1D *LG_T_;       // Gain schedule lead time constant, s
  TableInterp1D *TLD_T_;      // Gain schedule loop gain, r/s
  double        e_;           // Closed loop error, %Nf
  double        eM_;          // Closed loop model error, %Nf
  double        intState_;    // PI control integrate state, deg
  double        intStateM_;   // PI control integrate state for model, deg
  double        modelG_;      // Model Gas Generator output, %Ng
  double        modelF_;      // Model Fan, %Nf
  double        modelFS_;     // Model Fan Sensed, %Nf
  double        modPcng_;     // Modeled pcng ref after esc ttl delay, %Nf
  double        pcnf_;        // Fan speed, %
  double        pcnfRef_;     // Fan speed closed loop reference, %Nf
  double        throttleM_;   // Modeled servo value, 0-179 degrees
  double        throttleL_;   // Limited servo value, 0-179 degrees
  double        throttleML_;  // Limited modeled servo value, 0-179 degrees
  double        throttleCL_;  // Closed loop throttle output, deg
  double        throttleCLM_; // Closed loop model throttle output, deg
};



#endif
