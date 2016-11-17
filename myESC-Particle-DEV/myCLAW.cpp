
// Standard
#ifdef ARDUINO
  #include <Arduino.h> //needed for Serial.println
#else
  #include "application.h"      // Should not be needed if file .ino or Arduino
#endif

#include "myCLAW.h"
#include "math.h"


// Global variables
extern const int verbose;
extern char buffer[256];

//Class ControlLaw
ControlLaw::ControlLaw()
: intState_(0), intStateM_(0), modelG_(0), modelT_(0), modelTS_(0), modPcng_(0),
  pcnt_(0), pcntRef_(0), throttleM_(0), throttleL_(0), throttleML_(0)
{
  LG_T_   = new TableInterp1D(sizeof(xALL)/sizeof(double), xALL, yLG);
  TLD_T_  = new TableInterp1D(sizeof(xALL)/sizeof(double), xALL, yTLD);
  modelFilterG_   = new LeadLagTustin(0, tldG, tauG,  -1e6, 1e6);
  modelFilterT_   = new LeadLagTustin(0, tldT, 1.00,  -1e6, 1e6);
  modelFilterV_   = new LeadLagTustin(0, tldV, tauF2V,-1e6, 1e6);
}
ControlLaw::ControlLaw(const double T)
: intState_(0), intStateM_(0), modelG_(0), modelT_(0), modelTS_(0), modPcng_(0),
  pcnt_(0), pcntRef_(0), throttleM_(0), throttleL_(0), throttleML_(0)
{
  LG_T_   = new TableInterp1D(sizeof(xALL)/sizeof(double), xALL, yLG);
  TLD_T_  = new TableInterp1D(sizeof(xALL)/sizeof(double), xALL, yTLD);
  modelFilterG_   = new LeadLagTustin(T, tldG, tauG,  -1e6, 1e6);
  modelFilterT_   = new LeadLagTustin(T, tldT, 1.00,  -1e6, 1e6);
  modelFilterV_   = new LeadLagTustin(T, tldV, tauF2V,-1e6, 1e6);
}


// Control Law
double ControlLaw::calculate(const int RESET, const double updateTime, const boolean closingLoop,
  const boolean analyzing, const boolean freqResp, const double exciter, const double freqRespScalar,
  const double freqRespAdder, const double potThrottle, const double vf2v, const double DENS_SI)
{
  double    tld, lg, tldm, lgm; // Gain schedule lookup table outputs
  double    dQt_dNt;            // Load line for inertia calculation, ft-lbf/rpm
  double    tauT;               // Model Fan lag time constant, sec
  double    throttle;           // Return value throttle command, deg

  // Inputs
#ifdef BARE_MICROPROCESSOR
  pcnt_     = modelTS_;
#else
  pcnt_     = fmin(fmax(P_V4_NT[0] + vf2v*(P_V4_NT[1] + vf2v*P_V4_NT[2])/RPM_P, 0.0), 100);
#endif
  double throttleRPM  = fmax(P_LT_NG[0] + P_LT_NG[1]*log(fmax(potThrottle, 1)), 0); // Ng demand at throttle, rpm
  pcntRef_            = (P_NG_NT[0] + P_NG_NT[1]*throttleRPM) / RPM_P;
  if ( closingLoop && freqResp )
  {
    pcntRef_  *= (1+exciter/freqRespScalar);
    pcntRef_  += exciter*freqRespAdder;
  }

  // Initialization
  if ( RESET )
  {
    intState_ = intStateM_ = potThrottle; // deg throttle
    pcnt_     = pcntRef_;
    modelTS_  = pcntRef_;
  }

  // Control Law
  e_  = pcntRef_ - pcnt_;
  eM_ = pcntRef_ - modelTS_;
  double Ki  	  = LG_T_->interp(pcnt_);        // r/s
  double Kp  	  = TLD_T_->interp(pcnt_) * Ki;  // %Ng/%Nf=1
  double KiM    = Ki;
  double KpM  	= Kp;
  double dNdT   = P_LT_NG[1] / fmax(potThrottle, 1) / RPM_P;   // Rate normalizer, %Ng/deg
  double riMax  = RATE_MAX*dNdT;
  // Hardware
  intState_     = fmax(fmin(intState_  + updateTime*fmax(fmin(Ki*e_,  0.5*riMax), -0.5*riMax), 	NG_MAX), -NG_MAX);
  double pcngCL = fmax(fmin(intState_  + fmax(fmin(Kp*e_,      		    NG_MAX),    -NG_MAX), 		NG_MAX), NG_MIN);
  throttleCL_   = exp((pcngCL*RPM_P   - P_LT_NG[0])/P_LT_NG[1]);
  // Model
  intStateM_    = fmax(fmin(intStateM_ + updateTime*fmax(fmin(KiM*eM_,0.5*riMax), -0.5*riMax), 	NG_MAX), -NG_MAX);
  double pcngCLM  = fmax(fmin(intStateM_ + fmax(fmin(KpM*eM_,  			  NG_MAX), 	  -NG_MAX), 		NG_MAX), NG_MIN);
  throttleCLM_  = exp((pcngCLM*RPM_P  - P_LT_NG[0])/P_LT_NG[1]);

  // Rate Limits
  double stepChangeMaxInit  = 1*updateTime;         // Maximum allowable step change reset, deg/update
  double stepChangeMax      = RATE_MAX*updateTime;  // Maximum allowable step change, deg/update
  if ( closingLoop )
  {
    double  throttleU   = throttleCL_;
    double throttleMU   = throttleCLM_;
    // Apply rate limits as needed
    if ( RESET )
    {
      throttleL_  = fmax(fmin(throttleU,   throttleL_+stepChangeMaxInit),   throttleL_-stepChangeMaxInit);
      throttleML_ = fmax(fmin(throttleMU,  throttleML_+stepChangeMaxInit),  throttleML_-stepChangeMaxInit);
  }
  else
  {
    throttleL_  = fmax(fmin(throttleU,   throttleL_+stepChangeMax),   throttleL_-stepChangeMax);
    throttleML_ = fmax(fmin(throttleMU,  throttleML_+stepChangeMax),  throttleML_-stepChangeMax);
  }
  throttle  = throttleL_;
  throttleM_ = throttleML_;
}
else  // open loop
{
  double throttleU      = potThrottle * (1+exciter/freqRespScalar) + exciter*freqRespAdder;  // deg throttle

  // Apply rate limits as needed
  if ( RESET )
  {
    throttleL_ = throttleML_ = fmax(fmin(throttleU, throttleL_+stepChangeMax), throttleL_-stepChangeMax);
  }
  else
  {
    throttleL_   = throttleU;
  }
  if ( freqResp ) throttle  = throttleM_  = throttleU;
  else            throttle  = throttleM_  = throttleL_;
  } // open loop

  // Final throttle limits
  throttle = fmax(fmin(throttle, THTL_MAX), THTL_MIN);

  // Model
  if ( RESET )
    modPcng_  = (P_NT_NG[0] + pcntRef_*RPM_P*P_NT_NG[1])/RPM_P;
  else
    modPcng_  = fmax((P_LT_NG[0] + P_LT_NG[1]*log(double(int(throttleM_)))) / RPM_P, 0.0);
  double Fg_SI  = FG_SI * pow(modelG_/100, 3);           // N
  double Vw_SI  = sqrt(Fg_SI / AREA_SI / DENS_SI);    // m/s
  //dQt_dNt     = P_N_Q[1] + P_N_Q[2]*2*(modelT_*RPM_P); // Uses past value OK
  dQt_dNt     = DCPDL * DENS_SI * D_SI*D_SI * AREA_SI * 3.1415926 / 240 * (Vw_SI-DELTAV) / LAMBDA * NM_2_FTLBF;
  dQt_dNt     = fmin(dQt_dNt, -1e-16);
  //tauT        = fmin( J / fmax( dQt_dNt, 1e-32), 0.2);
  //tauT        = fmin(fmax(P_LNT_TAU[0] + P_LNT_TAU[1]*log(fmax(modelT_, 1e-8)), 0.02), 0.5);
  tauT        = fmin(fmax(-J / dQt_dNt, 0.02), 0.5);
  modelG_     = modelFilterG_->calculate(modPcng_,  RESET);
  modelT_     = modelFilterT_->calculate((P_NG_NT[0] + modelG_*RPM_P*P_NG_NT[1])/RPM_P,  RESET,
                              updateTime, tauT, tldT);
  modelTS_    = modelFilterV_->calculate(modelT_,  RESET);
/*
  if ( verbose > 2 )
  {
    sprintf(buffer, "T: %s,%s,", String(modelT_).c_str(),String(dQt_dNt,16).c_str()); Serial.print(buffer);
    sprintf(buffer, "%s,%s,", String(tauT).c_str(), String(tldT).c_str()); Serial.print(buffer);
    sprintf(buffer, "%s,%s,\n", String(P_N_Q[1],10).c_str(), String(P_N_Q[2],10).c_str()); Serial.print(buffer);
  }
  */

  if ( !closingLoop ) intState_ = throttle;
  return( throttle );
}
