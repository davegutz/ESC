#include "application.h"
#include "analyzer.h"
#include "math.h"

//Class FRAnalyzer

FRAnalyzer::FRAnalyzer(const int omegaLogMin, const int omegaLogMax, const double deltaOmegaLog, const int minCycles,
  const int numInitCycles, const double wSlow, const double T, const double sig[], const int ix[], const int iy[], const int nsig, const int ntf)
  : aint_(0), complete_(false), cosOmT_(0), deltaOmegaLog_(deltaOmegaLog), excite_(0), iOmega_(0), iResults_(0UL),
  iTargetOmega_(0), iTargetResults_(0UL), minCycles_(minCycles), nsig_(nsig), ntf_(ntf), numCycles_(0),
  omega_(0), omegaLog_(0), omegaLogMax_(omegaLogMax), omegaLogMin_(omegaLogMin), sig_(sig), sinOmT_(0), T_(T),
  timeAtOmega_(0), timeTargetOmega_(0), timeTotalSweep_(0), Tlog_(log10(T)), wSlow_(wSlow)
{

  // Initialize arrays
  a1_         = new double[nsig_];
  b1_         = new double[nsig_];
  ix_         = new unsigned int[ntf_]; for (int i=0; i<ntf_; i++) ix_[i] = ix[i];
  iy_         = new unsigned int[ntf_]; for (int i=0; i<ntf_; i++) iy_[i] = iy[i];
  sigGain_    = new double[nsig_];
  sigPhas_    = new double[nsig_];
  transGain_  = new double[ntf_];
  transPhas_  = new double[ntf_];

  // Calculate run time and number of points
  iTargetResults_  = (int)((omegaLogMax_ - omegaLogMin_) / deltaOmegaLog_ + .5) + 1;
  for(int iResults = -2; iResults < iTargetResults_; iResults++){
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
  }
  initializeSET_();
  frMode_     = WAI;
  Serial.printf("Number points = %ld, timeTotalSweep_=%5.2f", iTargetResults_, timeTotalSweep_);
}

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
  numCycles_        = fmin(fmax(minCycles_ + aint, 1), 10);
  omega_            = properOmega_(T_,  numCycles_, omegaLog_, &iTargetOmega_);
  iTargetOmega_     += iTargetOmega_/numCycles_/4;  // Add 1/4 cycle to start @ -0.5
  iOmega_           = 0;
  timeTargetOmega_  = iTargetOmega_*T_;
}

// Initialize for frequency point (RUN)
void FRAnalyzer::initializeRUN_(void)
{
  double aint;
  modf(omegaLog_, &aint);
  numCycles_       = fmin(fmax(minCycles_ + aint, 1), 10);
  omega_           = properOmega_(T_,  numCycles_, omegaLog_, &iTargetOmega_);
  iOmega_  		     = 0;
  timeTargetOmega_ = iTargetOmega_*T_;

  // Initialize integrators
  for(int isig = 0; isig < nsig_; isig++)
  {
    a1_[isig]    = .5 * sig_[isig];
    b1_[isig]    = 0.;
  }
}

// Calculate frequency and iterations for exact transition precision between frequency points
double FRAnalyzer::properOmega_(const double updateTime, const unsigned int numCycles, const double omegaLog, unsigned long *iTargetOmega)
{
  *iTargetOmega 	= (unsigned long)(2. * pi * numCycles / updateTime / pow(10, omegaLog) + .5);
  return ( 2. * pi * numCycles / updateTime / (float)(*iTargetOmega) );
}


// Calculation executive
double FRAnalyzer::calculate()
{
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
        excite_ = -0.5;
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
  double xNormal = fmax(-0.5, excite_ - 0.5*T_/tau_/4);
  return (xNormal);
}

// Calculate INI excitation, begins at bottom of sine wave
double FRAnalyzer::calculateINI_(void)
{
  double  xNormal;
  timeAtOmega_  = iOmega_ * T_;
  xNormal       = 0.5*sin(omega_*(timeAtOmega_-2*pi/omega_/4));
  return(xNormal);
}

// Calculate RUN excitation
double FRAnalyzer::calculateRUN_(void)
{
  double  xNormal;
  // Update Fourier Series integrals.
  runIntegrate_();
  timeAtOmega_  = iOmega_ * T_;
  xNormal       = 0.5*sin(omega_*timeAtOmega_);
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
      a1_[isig]    = (a1_[isig] - .5 * sig_[isig]) * T_ *
                          omega_ / pi / numCycles_;
      b1_[isig]    = b1_[isig] * T_ * omega_ / pi / numCycles_;
    }
    // Compute signal mag and phase.
    for(int isig = 0; isig < nsig_; isig++)
    {
      sigGain_[isig]    = 20. * log10(sqrt(a1_[isig]*a1_[isig] + b1_[isig]*b1_[isig]));
      sigPhas_[isig]    = 180. / pi * atan2(a1_[isig], b1_[isig]);
    }
      // Compute transfer function mag and phase and print results.
      Serial.printf("% #10.4g", omega_);
      for(int itf = 0; itf < ntf_; itf++)
      {
        transGain_[itf]     = sigGain_[iy_[itf]]  - sigGain_[ix_[itf]];
        transPhas_[itf]     = sigPhas_[iy_[itf]]  - sigPhas_[ix_[itf]];
        Serial.printf(" % #10.4g % #10.4g", transGain_[itf], transPhas_[itf]);
      }
      Serial.printf("\n");
  }
}



void FRAnalyzer::publish()
{
  Serial.printf("M=%ld, omegaLog=%5.3f, numCycles=%u, omega=%5.3f, iOmega=%u/%u, timeAtOmega=%5.3f/%5.3f, iResults=%u/%u",
  frMode_, omegaLog_, numCycles_, omega_, iOmega_, iTargetOmega_, timeAtOmega_, timeTargetOmega_, iResults_, iTargetResults_);
}
