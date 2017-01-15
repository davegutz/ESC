#ifndef _myCLAW_h
#define _myCLAW_h

#include "myTables.h"
#include "myFilters.h"
#define CTYPE 1   // 0=P+I, 1=I, 2=PID
#define KIT   3   // -1=Photon, 0-4 = Arduino


#if   CTYPE==0  // P+I
#include "myPI.h"
#elif CTYPE==1  // I
#include "myI.h"
#elif CTYPE==2  // PID
#include "myPID.h"
#else
#error "Unknown CTYPE="
#endif


// Plant
static const double tau = 0.10;     // Input noise filter time constant, sec
static const double tldV = 0.0;     // Model F2V lead time constant, sec
static const double tauF2V = 0.1;   // Model F2V lag time constant, sec
static const double tldG = 0.0;     // Model Gas Generator lead time constant, sec
static const double tauG = 0.1;     // Model Gas Generator lag time constant, sec
static const double tldT = 0.0;     // Model Turbine lead time constant, sec
static const double NG_MIN = 0;     // Minimum trim, %Ng
static const double RATE_MAX = 240; // Maximum throttle change rate, deg/sec to avoid lockout
// See calibration<date>.xlsx for these hardware conversion derivations
static const double RPM_P = 461;                   // (rpm/%)
static const double THTL_MIN = 0;   // Minimum throttle, deg
static const double AREA_SI = 0.002121;            // Flow area, m^2
static const double NM_2_FTLBF = 0.738;            // ft-lbf/N-m conversion
static const double J = 3.743e-8;   // Turbine inertia, (rpm/s)/(ft-lbf)
static const double D_SI = 0.055;                  // Turbine dia, m
static const double FG_SI = 4.425;                 // Thrust at rated speed, N
static const double THTL_MAX = 180; // Maximum throttle, deg
static const double NG_MAX = 100;   // Maximum trim, %Ng

static const double P_LTALL_NG[2] = {-25454, 13062};  // Common coeff throttle(deg) to NG(rpm)
static const double P_NGALL_NT[2] = {-7208, 1.0000};  // Coeff NG(rpm) to NT(rpm)
static const double P_NTALL_NG[2] = {-7208, 1.0000};  // Coeff NG(rpm) to NT(rpm)
#if KIT==-1
// CalPhotonTurnigy 12/24/2016
static const double xALL[6] = {0.,    16.,    25.,    47.5,   62.,    80.};   // Gain breakpoints, %Nt
static const double P_V4_NT[3] = {0, 28857, 296};  // Coeff V4(v) to NT(rpm)
//static const double P_LT_NG[2] = {-35968, 15569};  // Coeff throttle(deg) to NG(rpm)
static const double P_NG_NT[2] = {-4629, 0.9778};  // Coeff NG(rpm) to NT(rpm)
static const double P_NT_NG[2] = {4826,  1.0194};  // Coeff NT(rpm) to NG(rpm)
static const double DCPDL = -1.765;                // dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const double LAMBDA = 3.17;                 // Turbine tip speed ratio to air velocity, dimensionless
static const double DELTAV = 5.0;                  // Air velocity turbine first moves, m/s
#elif KIT==0
// Ard1_Turn0_ESC0_G0b_T0a 1/4/2017
static const double xALL[6] = {0.,    0.01,  27.2,    47.4,   75.1,    80.};   // Gain breakpoints, %Nt
static const double P_V4_NT[3] = {0, 13952,   27};  // Coeff V4(v) to NT(rpm)
//static const double P_LT_NG[2] = {-19172, 11411}; // Coeff throttle(deg) to NG(rpm)
static const double P_NG_NT[2] = {-8053, 0.9475};  // Coeff NG(rpm) to NT(rpm)
static const double P_NT_NG[2] = {8510,  1.0550};  // Coeff NT(rpm) to NG(rpm)
static const double DCPDL = -0.418;                // TODO dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const double LAMBDA = 2.54;                 // TODO Turbine tip speed ratio to air velocity, dimensionless
static const double DELTAV = 1.0;                  // TODO Air velocity turbine first moves, m/s
#elif KIT==1
// Ard1_Turn1x_ESC1_G1b_T1a
static const double xALL[6] = {0.,    21.7,  37.3,    50.5,   64.1,    80.};   // Gain breakpoints, %Nt
static const double P_V4_NT[3] = {0, 12894, 288};  // Coeff V4(v) to NT(rpm)
//static const double P_LT_NG[2] = {-19172, 11411};  // Coeff throttle(deg) to NG(rpm)
static const double P_NG_NT[2] = {-5795, 0.9776};  // Coeff NG(rpm) to NT(rpm)
static const double P_NT_NG[2] = {5943,  1.0222};  // Coeff NT(rpm) to NG(rpm)
static const double DCPDL = -0.789;                // dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const double LAMBDA = 3.16;                 // Turbine tip speed ratio to air velocity, dimensionless
static const double DELTAV = 1.0;                  // Air velocity turbine first moves, m/s
#elif KIT==2
// Ard2_Turn2_ESC2_G2b_T2a
static const double xALL[6] = {0.,    22.0,  36.5,    49.1,   67.7,    80.};   // Gain breakpoints, %Nt
//static const double P_V4_NT[3] = {0, 15929, -1553}; // Coeff V4(v) to NT(rpm)
static const double P_V4_NT[3] = {0, 15501, -1097}; // Coeff V4(v) to NT(rpm)   r2_ct1_ol_atten_20170113.xlsx  add attenuator.  cross plot v4, theo Nt
//static const double P_LT_NG[2] = {-21339, 12143};   // Coeff throttle(deg) to NG(rpm)
static const double P_NG_NT[2] = {-5553, 0.9312};   // Coeff NG(rpm) to NT(rpm)
static const double P_NT_NG[2] = {5986, 1.0728};    // Coeff NT(rpm) to NG(rpm)
static const double DCPDL = -0.927;                 // dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const double LAMBDA = 2.99;                  // Turbine tip speed ratio to air velocity, dimensionless
static const double DELTAV = 4;                     // Air velocity turbine first moves, m/s
#elif KIT==3
// Ard3_Turn3_ESC3_G3b_T3a
static const double xALL[6] = {0.,    21.6,  37.0,    51.0,   70.8,    80.};   // Gain breakpoints, %Nt
static const double P_V4_NT[3] = {0, 13130,   111};// Coeff V4(v) to NT(rpm)
//static const double P_LT_NG[2] = {-23777, 12519};// Coeff throttle(deg) to NG(rpm)
static const double P_NG_NT[2] = {-6115, 0.9573};  // Coeff NG(rpm) to NT(rpm)
static const double P_NT_NG[2] = {6404, 1.0438};   // Coeff NT(rpm) to NG(rpm)
static const double DCPDL = -0.697;                // dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const double LAMBDA = 2.95;                 // Turbine tip speed ratio to air velocity, dimensionless
static const double DELTAV = 3.0;                  // Air velocity turbine first moves, m/s
#elif KIT==4
// Ard4_Turn4_ESC4_G4b_T4a
static const double xALL[6] = {0.,    21.6,  37.5,    51.3,   67.9,    80.};   // Gain breakpoints, %Nt
static const double P_V4_NT[3] = {0, 12801, 29};    // Coeff V4(v) to NT(rpm)
//static const double P_LT_NG[2] = {-31258, 14180};   // Coeff throttle(deg) to NG(rpm)
static const double P_NG_NT[2] = {-7440, 1.0080};   // Coeff NG(rpm) to NT(rpm)
static const double P_NT_NG[2] = {7464,  0.9878};   // Coeff NT(rpm) to NG(rpm)
static const double DCPDL = -0.975;                 // dCpdLambda, dimensionless.  Cp is power coefficient and Lambda is speed tip ratio
static const double LAMBDA = 3.13;                  // Turbine tip speed ratio to air velocity, dimensionless
static const double DELTAV = 4.0;                   // Air velocity turbine first moves, m/s
#endif

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
                   const boolean analyzing, const boolean freqResp, const boolean vectoring, 
                   const double exciter, const double freqRespScalar,
                   const double freqRespAdder, const double potThrottle, const double vf2v);
  double e(void) { return (e_); };
  double intState(void) { return (intState_); };
  double Ki(void) {return (Ki_); };
  double Kp(void) {return (Kp_); };
  double modelTS(void) { return (modelTS_); };
  double modelG(void) { return (modelG_); };
  double p(void) { return (p_); };
  double pcnt(void) { return (pcnt_); };
  double pcntRef(void) { return (pcntRef_); };
private:
  double throttleLims(const int RESET, const double updateTime, const boolean closingLoop,
                  const boolean freqResp, const boolean vectoring, const double exciter, const double freqRespScalar,
                  const double freqRespAdder, const double potThrottle, const double ngmin);
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
  double Ki_;                // Integral gain, r/s
  double Kp_;                // Proportional gain, rad
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
