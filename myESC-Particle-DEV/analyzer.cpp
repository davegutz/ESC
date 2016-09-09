#include "application.h"
#include "analyzer.h"
#include "math.h"

FR_Analyzer::FR_Analyzer(const int omegaLogMin, const int omegaLogMax, const double deltaOmegaLog, const double fractionalInputHalfSine,
  const int numInitCycles, const double T, const double sig[], const int ix[], const int iy[], const int nsig, const int ntf)
  : aint_(0), complete_(false), cosOmT_(0), deltaOmegaLog_(deltaOmegaLog), fractionalInputHalfSine_(fractionalInputHalfSine),
  iOmega_(0), iResults_(0UL), iTargetOmega_(0), iTargetResults_(0UL), nsig_(nsig), ntf_(ntf), numInitCycles_(0),
  omega_(0), omegaLog_(0), omegaLogMax_(omegaLogMax), omegaLogMin_(omegaLogMin), sig_(sig), sinOmT_(0), T_(T), timeAtOmega_(0),
  timeTargetOmega_(0), timeTotalSweep_(0), Tlog_(log10(T))
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

  // Calculate run time
  omegaLog_     	= omegaLogMin_;
  numInitCycles_  = .25 + (float)(numInitCycles_);
  iTargetOmega_  	= (int)(2. * pi * numInitCycles_ / T_ / pow(10, omegaLog_) + .5);
  omega_   		    = 2. * pi * numInitCycles_ / T_ / (float)(iTargetOmega_);
  iTargetResults_ = (int)((omegaLogMax_ - omegaLogMin_) / deltaOmegaLog_ + .5) + 1;
  for(iResults_ = 0; iResults_ < iTargetResults_; iResults_++){
      omegaLog_         = omegaLogMin_ + (float)(iResults_) * deltaOmegaLog_;
      modf((omegaLog_ + Tlog_), &aint_);
      numInitCycles_    = fmin(fmax((4. + aint_), 1.), 4.);
      iTargetOmega_     += (int)(2. * pi * numInitCycles_ / T_ / pow(10, omegaLog_) + .5);
      timeTotalSweep_   += iTargetOmega_ * T_;
  }
  omegaLog_     	= omegaLogMin_;
  modf((omegaLog_ + Tlog_), &aint_);
  numInitCycles_    = fmin(fmax((4. + aint_), 1.), 4.);
  iTargetOmega_  	= (int)(2. * pi * numInitCycles_ / T_ / pow(10, omegaLog_) + .5);
  omega_   		    = 2. * pi * numInitCycles_ / T_ / (float)(iTargetOmega_);
  iOmega_  		    = -iTargetOmega_;
  iResults_       = 0;
  Serial.printf("timeTotalSweep_=%5.2f", timeTotalSweep_);
  //xNormal   = 1. + fractionalInputHalfSine_ * (1. + sin(omega_ * timeAtOmega_));
}




double FR_Analyzer::calculate()
{
  double  xNormal = 0;
  int     isig    = 0;
  int     itf     = 0;
  timeAtOmega_  = iOmega_ * T_;
	if(iResults_ > iTargetResults_)
	{
		xNormal   = 1. + fractionalInputHalfSine_;
    complete_ = true;
		return(xNormal);
  }

  // Update Fourier Series integrals.
  if(timeTargetOmega_ != 0)
  {
    double coswtn  = cos(omega_ * timeAtOmega_);
    double sinwtn  = sin(omega_ * timeAtOmega_);
    for(isig = 0; isig < nsig_; isig++)
    {
      a1_[isig]    += sig_[isig] * coswtn;
      b1_[isig]    += sig_[isig] * sinwtn;
    }
  }
  if(iOmega_ < iTargetOmega_)
  {
    iOmega_++;
  }
  else
  {
    if(timeTargetOmega_ != 0)
    {
      // Finish up Fourier Series integrals.
      for(isig = 0; isig < nsig_; isig++)
      {
          a1_[isig]    = (a1_[isig] - .5 * sig_[isig]) * T_ *
                              omega_ / pi / numInitCycles_;
          b1_[isig]    = b1_[isig] * T_ * omega_ / pi / numInitCycles_;
      }
      // Compute signal mag and phase.
      for(isig = 0; isig < nsig_; isig++)
      {
        sigGain_[isig]    = 20. * log10(sqrt(a1_[isig]*a1_[isig] + b1_[isig]*b1_[isig]));
        sigPhas_[isig]    = 180. / pi * atan2(a1_[isig], b1_[isig]);
      }
      // Compute transfer function mag and phase and print results.
      Serial.printf("% #10.4g", omega_);
      for(itf = 0; itf < ntf_; itf++)
      {
        transGain_[itf]     = sigGain_[iy_[itf]]  - sigGain_[ix_[itf]];
        transPhas_[itf]     = sigPhas_[iy_[itf]]  - sigPhas_[ix_[itf]];
        Serial.printf(" % #10.4g % #10.4g", transGain_[itf], transPhas_[itf]);
      }
      Serial.printf("\n");

      // Increment counter and check for end
      iResults_++;
      if(iResults_ > iTargetResults_)
      {
        complete_ = true;
        xNormal   = 1. + fractionalInputHalfSine_;
        return(xNormal);
      }
    }
    // New frequency
    // Adjust for integral number of time steps
    omegaLog_ = omegaLogMin_ + (float)(iResults_) * deltaOmegaLog_;
    modf((omegaLog_ + Tlog_), &aint_);
    numInitCycles_    = fmin(fmax((4. + aint_), 1.), 4.);
    iTargetOmega_     = (int)(2. * pi * numInitCycles_ / T_ / pow(10, omegaLog_) + .5);
    omega_   = 2. * pi * numInitCycles_ / T_ / (float)(iTargetOmega_);
    timeTargetOmega_   = iTargetOmega_ * T_;
    // Initialize integrators
    for(isig = 0; isig < nsig_; isig++)
    {
      a1_[isig]    = .5 * sig_[isig];
      b1_[isig]    = 0.;
    }
    iOmega_  = 1;
  }
  timeAtOmega_   = iOmega_ * T_;
  xNormal   = 1. + fractionalInputHalfSine_ * (1. + sin(omega_ * timeAtOmega_));
  return(xNormal);
}


void FR_Analyzer::publish()
{
  Serial.printf("omegaLog=%5.3f, aint=%5.3f, numInitCycles=%u, omega=%5.3f, iOmega=%u/%u, timeAtOmega=%5.3f/%5.3f, iResults=%u/%u",
  omegaLog_, aint_, numInitCycles_, omega_, iOmega_, iTargetOmega_, timeAtOmega_, timeTargetOmega_, iResults_, iTargetResults_);
}
