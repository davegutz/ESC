#ifndef _myCLAW_h
#define _myCLAW_h

#include "myTables.h"
#include "myFilters.h"

/* Control11/2/2016
static const    double    xALL[6]     = {0,     16,  	35,   	50,   	79,   	100};   // Gain breakpoints, %Nt
static const    double    yLG[6]      = {2.5, 	2.5, 	3.0, 	  3.1, 	  2.8,	  2.8};   // Loop gain, r/s
static const    double    yTLD[6]     = {0.36,  0.36, 0.36,  	0.36, 	0.36, 	0.36};  // Lead, s
*/

/* Control 11/4/2016
static const    double    xALL[6]     = {0,     9,     24,     45,     74,    100,};      // Gain breakpoints, %Nt
static const    double    yLG[6]      = {3.4,   3.4,   3.5,    4.3,    5.6,   5.6};       // Loop gain, r/s
static const    double    yTLD[6]     = {0.488, 0.488, 0.310,  0.200,  0.141, 0.141 };    // Lead, s
*/

// Control Ref Step 11/15/2016
#ifndef DISTURB_CONTROL
  static const    double    xALL[6]     = {0.,	  16.,	  25.,    47.5,	  62.,    80.};     // Gain breakpoints, %Nt
  static const    double    yLG[6]      = {2.5,   2.5,    2.8,    3.3,    4.,     4.8};     // Loop gain, r/s
  static const    double    yTLD[6]     = {0.500, 0.500,  0.320,  0.230,  0.175,  0.175};   // Lead, s
#else
  static const    double    xALL[6]     = {0.,	  16.,	  25.,    47.5,	  62.,    80.};     // Gain breakpoints, %Nt
  static const    double    yLG[6]      = {3.8,   3.8,    3.4,    3.5,    4.1,    5.5};     // Loop gain, r/s
  static const    double    yTLD[6]     = {0.488, 0.488,  0.340,  0.255,  0.210,  0.210};   // Lead, s
#endif

// Plant
static const    double          tau         = 0.10; // Input noise filter time constant, sec
static const    double          tldV        = 0.0;  // Model F2V lead time constant, sec
static const    double          tauF2V      = 0.1;  // Model F2V lag time constant, sec
static const    double          tldG        = 0.0;  // Model Gas Generator lead time constant, sec
static const    double          tauG        = 0.1;  // Model Gas Generator lag time constant, sec
static const    double          tldT        = 0.0;  // Model Turbine lead time constant, sec
static const    double          J           = 3.496e-8;// Turbine inertia, (rpm/s)/(ft-lbf)
static const    double          NG_MAX   	  = 100;  // Maximum trim, %Ng
static const    double          NG_MIN      = 0;    // Minimum trim, %Ng
static const    double          THTL_MAX    = 180;  // Maximum throttle to prevent shutdown due to small charger, deg
static const    double          THTL_MIN    = 0;    // Minimum throttle, deg
static const    double 	        RATE_MAX 	  = 240;  // Maximum throttle change rate, deg/sec to avoid lockout
// See calibration<date>.xlsx for these hardware conversion derivations
static const    double          P_LNT_TAU[2]= {0.698, -0.1573};   // Coeff Nt(%) to TauT(s)
static const    double          P_V4_NT[3]  = {0, 14098, -171};   // Coeff V4(v) to NT(rpm)
static const    double          P_LT_NG[2]  = {-28327,  14190};   // Coeff throttle(deg) to NG(rpm)
static const    double          P_NG_NT[2]  = {-10231,  1.0237};  // Coeff NG(rpm) to NT(rpm)
static const    double          P_NT_NG[2]  = {10154,   0.9683};  // Coeff NT(rpm) to NG(rpm)
static const    double          P_N_Q[3]    = {0,  1.75e-7,  1.154e-11};  // Load line N(rpm) to Q(ft-lbf)
static const    double          RPM_P       = 461;  // (rpm/%)
static const    double          FG_SI       = 4.425;    // Thrust at rated speed, N
static const    double          AREA_SI     = 0.002121; // Flow area, m^2
static const    double          DCPDL       = -.770;    // dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const    double          D_SI        = 0.055;    // Turbine dia, m
static const    double          LAMBDA      = 2.64;     // Turbine tip speed ratio to air velocity, dimensionless
static const    double          DELTAV      = 4;        // Air velocity turbine first moves, m/s
static const    double          NM_2_FTLBF  = 0.738;    // ft-lbf/N-m conversion

// Control Law Class
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
    const double freqRespAdder, const double potThrottle, const double vf2v, const double DENS_SI);
  double  e(void)         { return( e_          );};
  double  eM(void)        { return( eM_         );};
  double  intState(void)  { return( intState_   );};
  double  intStateM(void) { return( intStateM_  );};
  double  modelFS(void)   { return( modelTS_    );};
  double  modelG(void)    { return( modelG_     );};
  double  pcnt(void)      { return( pcnt_       );};
  double  pcntRef(void)   { return( pcntRef_    );};
  double  throttleM(void) { return( throttleM_  );};
private:
  LeadLagTustin *modelFilterG_; // Tustin lag model gas gen
  LeadLagTustin *modelFilterT_; // Tustin lag model turbine
  LeadLagTustin *modelFilterV_; // Tustin lag model F2V sensor
  TableInterp1D *LG_T_;       // Gain schedule lead time constant, s
  TableInterp1D *TLD_T_;      // Gain schedule loop gain, r/s
  double        e_;           // Closed loop error, %Nt
  double        eM_;          // Closed loop model error, %Nt
  double        intState_;    // PI control integrate state, deg
  double        intStateM_;   // PI control integrate state for model, deg
  double        modelG_;      // Model Gas Generator output, %Ng
  double        modelT_;      // Model Turbine, %Nt
  double        modelTS_;     // Model Turbine Sensed, %Nt
  double        modPcng_;     // Modeled pcng ref after esc ttl delay, %Nt
  double        pcnt_;        // Turbine speed, %
  double        pcntRef_;     // Turbine speed closed loop reference, %Nt
  double        throttleM_;   // Modeled servo value, 0-179 degrees
  double        throttleL_;   // Limited servo value, 0-179 degrees
  double        throttleML_;  // Limited modeled servo value, 0-179 degrees
  double        throttleCL_;  // Closed loop throttle output, deg
  double        throttleCLM_; // Closed loop model throttle output, deg
};

#endif
