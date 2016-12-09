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

#ifndef DISTURB_CONTROL
#ifndef USE_FIXED_LL
// Control Ref Step 11/19/2016
static const double xALL[6] = {0.,    16.,    25.,    47.5,   62.,    80.};   // Gain breakpoints, %Nt
static const double yLG[6]  = {2.4,   2.4,    2.7,    3.2,    3.75,   4.4};   // Loop gain, r/s
static const double yTLD[6] = {0.475, 0.475,  0.325,  0.225,  0.200,  0.200}; // Lead, s
#else
// Control Ref Step Fixed Lead 12/01/2016
static const double tldF    = 0.15;                                           // Lead, s
static const double tlgF    = 0.03;                                           // Lag, s
static const double xALL[6] = {0.,    16.,    25.,    47.5,   62.,    80.};   // Gain breakpoints, %Nt
static const double yLG[6]  = {3.6,   3.6,    3.75,   4.05,   5.,     5.};    // Loop gain, r/s
static const double yTLD[6] = {0.420, 0.420,  0.240,  0.125,  0.090,  0.090}; // Lead, s
#endif
#else
// Control Ref Step 11/15/2016
static const double xALL[6] = {0., 16., 25., 47.5, 62., 80.}; // Gain breakpoints, %Nt
static const double yLG[6]  = {3.8, 3.8, 3.4, 3.5, 4.1, 5.5};  // Loop gain, r/s
static const double yTLD[6] = {0.488, 0.488, 0.340, 0.255, 0.210, 0.210}; // Lead, s
#endif

// Plant
static const double tau = 0.10;     // Input noise filter time constant, sec
static const double tldV = 0.0;     // Model F2V lead time constant, sec
static const double tauF2V = 0.1;   // Model F2V lag time constant, sec
static const double tldG = 0.0;     // Model Gas Generator lead time constant, sec
static const double tauG = 0.1;     // Model Gas Generator lag time constant, sec
static const double tldT = 0.0;     // Model Turbine lead time constant, sec
static const double NG_MAX = 100;   // Maximum trim, %Ng
static const double NG_MIN = 0;     // Minimum trim, %Ng
static const double RATE_MAX = 240; // Maximum throttle change rate, deg/sec to avoid lockout

// See calibration<date>.xlsx for these hardware conversion derivations
static const double RPM_P = 461;                   // (rpm/%)
static const double THTL_MAX = 180; // Maximum throttle to prevent shutdown due to small charger, deg
static const double THTL_MIN = 0;   // Minimum throttle, deg
static const double AREA_SI = 0.002121;            // Flow area, m^2
static const double NM_2_FTLBF = 0.738;            // ft-lbf/N-m conversion
static const double J = 3.743e-8;   // Turbine inertia, (rpm/s)/(ft-lbf)
static const double D_SI = 0.055;                  // Turbine dia, m
static const double FG_SI = 4.425;                 // Thrust at rated speed, N

/*
// CalPhotonTurnigy
static const double P_V4_NT[3] = {0, 14543, -351}; // Coeff V4(v) to NT(rpm)
static const double P_LT_NG[2] = {-28327, 14190};  // Coeff throttle(deg) to NG(rpm)
static const double P_NG_NT[2] = {-8712, 0.9826};  // Coeff NG(rpm) to NT(rpm)
static const double P_NT_NG[2] = {8893, 1.0165};   // Coeff NT(rpm) to NG(rpm)
static const double DCPDL = -1.447;                // dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const double LAMBDA = 2.64;                 // Turbine tip speed ratio to air velocity, dimensionless
static const double DELTAV = 7;                    // Air velocity turbine first moves, m/s
*/

// Ard1_Turn1_ESC1_G1b_T1a
static const double P_V4_NT[3] = {0, 14315, -1137};// Coeff V4(v) to NT(rpm)
static const double P_LT_NG[2] = {-20773, 12971};  // Coeff throttle(deg) to NG(rpm)
static const double P_NG_NT[2] = {-7294, 0.9911};  // Coeff NG(rpm) to NT(rpm)
static const double P_NT_NG[2] = {7426, 1.0053};   // Coeff NT(rpm) to NG(rpm)
static const double DCPDL = -0.942;                // TODO dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const double LAMBDA = 3.03;                 // TODO Turbine tip speed ratio to air velocity, dimensionless
static const double DELTAV = 5;                    // TODO Air velocity turbine first moves, m/s


// Control Law Class
class ControlLaw
{
public:
  ControlLaw();
  ControlLaw(const double T, const double DENS_SI);
  ~ControlLaw(){};
  // operators
  // functions
  double calculate(const int RESET, const double updateTime, const boolean closingLoop,
                   const boolean analyzing, const boolean freqResp, const double exciter, const double freqRespScalar,
                   const double freqRespAdder, const double potThrottle, const double vf2v);
  double e(void) { return (e_); };
  double intState(void) { return (intState_); };
  double modelTS(void) { return (modelTS_); };
  double modelG(void) { return (modelG_); };
  double p(void) { return (p_); };
  double pcnt(void) { return (pcnt_); };
  double pcntRef(void) { return (pcntRef_); };
private:
  double throttleLims(const int RESET, const double updateTime, const boolean closingLoop,
                  const boolean freqResp, const double exciter, const double freqRespScalar,
                  const double freqRespAdder, const double potThrottle);
  void model(const double throttle, const int RESET, const double updateTime);
  LeadLagExp *modelFilterG_; // Exponential lag model gas gen
  LeadLagExp *modelFilterT_; // Exponential lag model turbine
  LeadLagExp *modelFilterV_; // Exponential lag model F2V sensor
  TableInterp1Dclip *LG_T_;  // Gain schedule lead time constant, s
  TableInterp1Dclip *TLD_T_; // Gain schedule loop gain, r/s
  double DENS_SI_;           // Air density, kg/m^3
  double dQ_;                // Precalculated coefficient, N-m/rpm/(m/s)
  double e_;                 // Closed loop error, %Nt
  double intState_;          // PI control integrate state, deg
  double modelG_;            // Model Gas Generator output, %Ng
  double modelT_;            // Model Turbine, %Nt
  double modelTS_;           // Model Turbine Sensed, %Nt
  double modPcng_;           // Modeled pcng ref after esc ttl delay, %Nt
  double p_;                 // Prop path, %Nt
  double pcnt_;              // Turbine speed, %
  double pcntRef_;           // Turbine speed closed loop reference, %Nt
  double throttle_;          // Final value sent to hardware and model, deg
  double throttleL_;         // Limited servo value, 0-179 degrees
  double throttleCL_;        // Closed loop throttle output, deg
#ifdef USE_FIXED_LL
  LeadLagExp *clawFixedL_;   // Exponential control fixed lead lag
#endif
};

#endif
