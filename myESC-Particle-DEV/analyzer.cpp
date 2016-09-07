#include "application.h"
#include "analyzer.h"
#include "math.h"

FR_Analyzer::FR_Analyzer(const int omegaLogMin, const int omegaLogMax, const double deltaOmegaLog, const double fractionalInputHalfSine,
  const int numInitCycles, const double T, const double sig[], const int ix[], const int iy[], const int nsig, const int ntf)
  : aint_(0), deltaOmegaLog_(deltaOmegaLog), fractionalInputHalfSine_(fractionalInputHalfSine), cosOmT_(0),
  iTime_(0), iTimMax_(0), maxIt_(0UL), nsig_(nsig), ntf_(ntf), numIt_(0UL), numInitCycles_(0), numAtFreq_(0), omega_(0), omegaLog_(0),
  omegaLogMax_(omegaLogMax), omegaLogMin_(omegaLogMin), sig_(sig), sinOmT_(0), T_(T), Tlog_(log10(T)), totalTime_(0),
  timeAtFreqMax_(0), timeOmega_(0)
  {

  // Initialize arrays
  a1_         = new double[nsig_];
  b1_         = new double[nsig_];
  ix_         = new int[ntf_]; for (int i=0; i<ntf_; i++) ix_[i] = ix[i];
  iy_         = new int[ntf_]; for (int i=0; i<ntf_; i++) iy_[i] = iy[i];
  sigGain_    = new double[nsig_];
  sigPhas_    = new double[nsig_];
  transGain_  = new double[ntf_];
  transPhas_  = new double[ntf_];

  // Calculate run time
  omegaLog_     	= omegaLogMin_;
  numInitCycles_  = .25 + (float)(numInitCycles_);
  numAtFreq_    	= (int)(2. * pi * numInitCycles_ / T_ / pow(10, omegaLog_) + .5);
  omega_   		    = 2. * pi * numInitCycles_ / T_ / (float)(numAtFreq_);
  iTime_  		    = -numAtFreq_;
  maxIt_          = (int)((omegaLogMax_ - omegaLogMin_) / deltaOmegaLog_ + .5) + 1;
  iTimMax_        = numAtFreq_;
  for(numIt_ = 0; numIt_ < maxIt_; numIt_++){
      omegaLog_         = omegaLogMin_ + (float)(numIt_) * deltaOmegaLog_;
      modf((omegaLog_ + Tlog_), &aint_);
      numInitCycles_    = fmin(fmax((4. + aint_), 1.), 4.);
      numAtFreq_        = (int)(2. * pi * numInitCycles_ / T_ / pow(10, omegaLog_) + .5);
      iTimMax_          += numAtFreq_;
  }
  totalTime_  = iTimMax_ * T_;
  numIt_      = 0;
  //xNormal   = 1. + fractionalInputHalfSine_ * (1. + sin(omega_ * timeOmega_));
}

double FR_Analyzer::calculate()
{
  double  xNormal = 0;
  int     isig    = 0;
  int     itf     = 0;
  timeOmega_  = iTime_ * T_;
	if(numIt_ > maxIt_)
	{
		xNormal   = 1. + fractionalInputHalfSine_;
		return(xNormal);
  }

  // Update Fourier Series integrals.
  if(timeAtFreqMax_ != 0)
  {
    double coswtn  = cos(omega_ * timeOmega_);
    double sinwtn  = sin(omega_ * timeOmega_);
    for(isig = 0; isig < nsig_; isig++)
    {
      a1_[isig]    += sig_[isig] * coswtn;
      b1_[isig]    += sig_[isig] * sinwtn;
    }
  }
  if(iTime_ < timeAtFreqMax_)
  {
    iTime_++;
  }
  else
  {
    if(timeAtFreqMax_ != 0)
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

      // Increment counter and check for end.
      numIt_++;
      if(numIt_ > maxIt_)
      {
        xNormal   = 1. + fractionalInputHalfSine_;
        return(xNormal);
      }
    }
  }
  // Adjust frequency so that there are an integral number of time steps.
  omegaLog_ = omegaLogMin_ + (float)(numIt_) * deltaOmegaLog_;
  modf((omegaLog_ + Tlog_), &aint_);
  numInitCycles_    = fmin(fmax((4. + aint_), 1.), 4.);
  timeAtFreqMax_  = (int)(2. * pi * numInitCycles_ / T_ / pow(10, omegaLog_) + .5);
  omega_   = 2. * pi * numInitCycles_ / T_ / (float)(timeAtFreqMax_);
  for(isig = 0; isig < nsig_; isig++)
  {
    a1_[isig]    = .5 * sig_[isig];
    b1_[isig]    = 0.;
  }
  iTime_  = 1;
  timeOmega_   = iTime_ * T_;
  xNormal   = 1. + fractionalInputHalfSine_ * (1. + sin(omega_ * timeOmega_));
  return(xNormal);
}
