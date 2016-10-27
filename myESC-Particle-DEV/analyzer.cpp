// Standard
#ifdef ARDUINO
  #include <Arduino.h> //needed for Serial.println
#else
  #include "application.h"      // Should not be needed if file .ino or Arduino
#endif
#include "analyzer.h"
#include "math.h"

// Global variables
extern char buffer[256];

//Class FRAnalyzer

FRAnalyzer::FRAnalyzer(const double omegaLogMin, const double omegaLogMax,
  const double deltaOmegaLog, const int minCycles, const double numCycleScalar, const int numInitCycles,
  const double wSlow, const double T, const int ix[],
  const int iy[], const int nsig, const int ntf, const String inHeader, const boolean square)
{
  aint_           = 0;
  complete_       = false;
  cosOmT_         = 0;
  deltaOmegaLog_  = deltaOmegaLog;
  excite_         = 0;
  iOmega_         = 0;
  iResults_       = 0UL;
  iTargetOmega_   = 0;
  iTargetResults_ =0U;
  minCycles_      = minCycles;
  nsig_           = nsig;
  ntf_            = ntf;
  numCycles_      = 0;
  numCycleScalar_ = numCycleScalar;
  omega_          = 0;
  omegaLog_       = omegaLogMin;
  omegaLogMax_    = omegaLogMax;
  omegaLogMin_    = omegaLogMin;
  sinOmT_         = 0;
  square_         = square;
  T_              = T;
  timeAtOmega_    = 0;
  timeTargetOmega_= 0;
  timeTotalSweep_ = 0;
  Tlog_           = log10(T);
  wSlow_          = wSlow;
  inHeader_       = inHeader;

  // Initialize arrays
  a1_         = new double[nsig_];
  b1_         = new double[nsig_];
  ix_         = new unsigned int[ntf_]; for (int i=0; i<ntf_; i++) ix_[i] = ix[i];
  iy_         = new unsigned int[ntf_]; for (int i=0; i<ntf_; i++) iy_[i] = iy[i];
  sig_        = new double[nsig_];
  sigGain_    = new double[nsig_];
  sigPhas_    = new double[nsig_];
  transGain_  = new double[ntf_];
  transPhas_  = new double[ntf_];

  // Calculate run time and number of points
  iTargetResults_  = (unsigned int)((omegaLogMax_ - omegaLogMin_) / deltaOmegaLog_ + .5) + 1;
  int iResults = -2;
  while( iResults < (int)iTargetResults_ )
  {
    switch ( iResults )
    {
      case -2:
        initializeSET_();
        break;
      case -1:
        initializeINI_();
        break;
      default:
        initializeRUN_();
    }
    timeTotalSweep_   += iTargetOmega_ * T_;
    iResults++;
  }
  initializeSET_();
  sprintf(buffer, "Number points = %s, timeTotalSweep_=%s\n", String(iTargetResults_).c_str(), String(timeTotalSweep_).c_str());
  Serial.print(buffer);
  sprintf(buffer, "%s,Mode,omegaLog,numCycles,omega,iOmega,iTargetOmega,timeAtOmega,timeTargetOmega,iResults,targetResults,\n", String(inHeader_).c_str());
  Serial.print(buffer);
  frMode_     = WAI;
}

// Restart frequency response
void    FRAnalyzer::complete(const bool set)
{
  frMode_           = WAI;
  iResults_         = 0;
  iOmega_           = 0;
  timeAtOmega_      = 0;
  omegaLog_         = omegaLogMin_;
  timeAtOmega_      = 0;
  timeTargetOmega_  = 0;
  complete_         = false;
  initializeSET_();
};

// Initialize for settling (SET)
void FRAnalyzer::initializeSET_(void)
{
  tau_            = 1./wSlow_;
  iTargetOmega_   = int(tau_/T_)*8;
  iOmega_         = 0;
  omega_          = 0;
}

// Initialize for transient settle (INI)
void FRAnalyzer::initializeINI_(void)
{
  double aint;
  modf(omegaLog_, &aint);
  numCycles_       = fmax(minCycles_, int(omega_*320.0*T_*numCycleScalar_)); // at least 1300 poiints
  omega_            = properOmega_(T_,  numCycles_, omegaLog_, &iTargetOmega_);
  iTargetOmega_     += iTargetOmega_/numCycles_/4;  // Add 1/4 cycle to start @ -1
  iOmega_           = 0;
  timeTargetOmega_  = iTargetOmega_*T_;
}

// Initialize for frequency point (RUN)
void FRAnalyzer::initializeRUN_(void)
{
  double aint;
  modf(omegaLog_, &aint);
  numCycles_       = fmax(minCycles_, int(omega_*320.0*T_*numCycleScalar_));  // at least 1300 points
  omega_           = properOmega_(T_,  numCycles_, omegaLog_, &iTargetOmega_);
  iOmega_  		     = 0;
  timeTargetOmega_ = iTargetOmega_*T_;

  // Initialize integrators
  for(int isig = 0; isig < nsig_; isig++)
  {
    a1_[isig]    = 0.;
    b1_[isig]    = 0.;
  }

}


// Calculate frequency and iterations for exact transition precision between frequency points
double FRAnalyzer::properOmega_(const double updateTime, const int numCycles, const double omegaLog, unsigned long *iTargetOmega)
{
  *iTargetOmega 	= (unsigned long)(2. * pi * numCycles / updateTime / pow(10, omegaLog) + .5);
  return ( 2. * pi * numCycles / updateTime / (float)(*iTargetOmega) );
}


// Calculation executive
double FRAnalyzer::calculate(const double *sig, const int nsig)
{
  for (int isig = 0; isig<nsig_; isig++)
  {
    sig_[isig] = sig[isig];
  }
  switch ( frMode_ )
  {
    case WAI:
    case SET:
      frMode_   = SET;
      iOmega_++;
      if ( iOmega_ < iTargetOmega_ )
      {
        excite_ = calculateSET_();
      }
      else
      {
        frMode_ = INI;
        omegaLog_ = omegaLogMin_ + (float)(iResults_) * deltaOmegaLog_;
        initializeINI_();
        excite_ = -1;
      }
      break;
    case INI:
      iOmega_++;
      if ( iOmega_ < iTargetOmega_ )
      {
        excite_ = calculateINI_();
      }
      else
      {
        frMode_ = RUN;
        omegaLog_ = omegaLogMin_ + (float)(iResults_) * deltaOmegaLog_;
        initializeRUN_();
        excite_ = 0;
      }
      break;
    case RUN:
      iOmega_++;
      if ( iOmega_ <= iTargetOmega_ )
      {
        excite_ = calculateRUN_();
      }
      else
      {
        if ( iResults_ >= iTargetResults_ )
        {
          complete_ = true;
          frMode_   = CPT;
          excite_   = 0.0;
        }
        else
        {
          // New frequency
          // Adjust for integral number of time steps
          omegaLog_ = omegaLogMin_ + (float)(iResults_) * deltaOmegaLog_;
          initializeRUN_();
          excite_ = 0;
        }
      }
      break;
    default:
      excite_ = 0;
  }
  if ( complete_ ) frMode_ = CPT;
  return(excite_);
}

// Calculate SET excitation
double FRAnalyzer::calculateSET_(void)
{
  timeAtOmega_  = iOmega_ * T_;
  double xNormal = fmax(-1, excite_ - T_/tau_/4);
  return (xNormal);
}

// Calculate INI excitation, begins at bottom of sine wave
double FRAnalyzer::calculateINI_(void)
{
  double  xNormal;
  timeAtOmega_  = iOmega_ * T_;
  xNormal       = sin(omega_*(timeAtOmega_-2*pi/omega_/4));
  if ( square )
  {
    if      ( xNormal>0 ) xNormal = 1;
    else if ( xNormal<0 ) xNormal = -1;
    else                  xNormal = 0;
  }
  return(xNormal);
}

// Calculate RUN excitation
double FRAnalyzer::calculateRUN_(void)
{
  double  xNormal;
  // Update Fourier Series integrals.
  runIntegrate_();
  timeAtOmega_  = iOmega_ * T_;
  xNormal       = sin(omega_*timeAtOmega_);
  return(xNormal);
}


// Calculate RUN excitation
double FRAnalyzer::runIntegrate_(void)
{
  double coswtn  = cos(omega_ * timeAtOmega_);
  double sinwtn  = sin(omega_ * timeAtOmega_);
  for(int isig = 0; isig < nsig_; isig++)
  {
    a1_[isig]    += sig_[isig] * coswtn;
    b1_[isig]    += sig_[isig] * sinwtn;
  }
  // Finish up Fourier Series integrals.
  if ( iOmega_ >= iTargetOmega_ )
  {
    iResults_++;
    for(int isig = 0; isig < nsig_; isig++)
    {
      a1_[isig]    *= T_ * omega_ / pi / numCycles_;
      b1_[isig]    *= T_ * omega_ / pi / numCycles_;
    }
    // Compute signal mag and phase.
    for(int isig = 0; isig < nsig_; isig++)
    {
      sigGain_[isig]    = 20. * log10( sqrt(a1_[isig]*a1_[isig] + b1_[isig]*b1_[isig]) );
      sigPhas_[isig]    = 180. / pi * atan2(a1_[isig], b1_[isig]);
    }
    // Compute transfer function mag and phase and print results.
    sprintf(buffer, "fr,%s,", String(omega_).c_str());
    Serial.print(buffer);
    for(int itf = 0; itf < ntf_; itf++)
    {
      transGain_[itf]     = sigGain_[iy_[itf]]  - sigGain_[ix_[itf]];
      transPhas_[itf]     = sigPhas_[iy_[itf]]  - sigPhas_[ix_[itf]];
      sprintf(buffer, "%s,%s,", String(transGain_[itf]).c_str(), String(transPhas_[itf]).c_str());
      Serial.print(buffer);
    }
    Serial.println("");
  }
}

// Display results
void FRAnalyzer::publish()
{
  sprintf(buffer, "%s,",    String(frMode_).c_str());         Serial.print(buffer);
  sprintf(buffer, "%s,",    String(omegaLog_).c_str());       Serial.print(buffer);
  sprintf(buffer, "%s,",    String(numCycles_).c_str());      Serial.print(buffer);
  sprintf(buffer, "%s,",    String(float(omega_)).c_str());   Serial.print(buffer);
  sprintf(buffer, "%s/",    String(iOmega_).c_str());         Serial.print(buffer);
  sprintf(buffer, "%s,",    String(iTargetOmega_).c_str());   Serial.print(buffer);
  sprintf(buffer, "%s/",    String(timeAtOmega_).c_str());    Serial.print(buffer);
  sprintf(buffer, "%s,",    String(timeTargetOmega_).c_str());Serial.print(buffer);
  sprintf(buffer, "%s/",    String(iResults_).c_str());       Serial.print(buffer);
  sprintf(buffer, "%s,",    String(iTargetResults_).c_str()); Serial.print(buffer);
/*  sprintf_P(buffer, PSTR("%s/%s,%s/%s,%s/%s,"),
    String(iOmega_).c_str(),      String(iTargetOmega_).c_str(),
    String(timeAtOmega_).c_str(), String(timeTargetOmega_).c_str(),
    String(iResults_).c_str(),    String(iTargetResults_).c_str());
  Serial.print(buffer);
*/
}

