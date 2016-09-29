/***************************************************
  A simple dynamic filter library

  Class code for embedded application.

  07-Jan-2015   Dave Gutz   Created
 ****************************************************/
#include "myFilters.h"
#include "math.h"
#ifndef ARDUINO
  #include "application.h"      // Should not be needed if file .ino or Arduino
#endif
extern const int verbose;

// class DiscreteFilter
// constructors
DiscreteFilter::DiscreteFilter()
    : max_(1e32), min_(-1e32), rate_(0.0), T_(1.0), tau_(0.0){}
DiscreteFilter::DiscreteFilter(const double T, const double tau, const double min, const double max)
    : max_(max),  min_(min),   rate_(0.0), T_(T), tau_(tau){}
DiscreteFilter::~DiscreteFilter(){}
// operators
// functions
double DiscreteFilter::calculate(double input, int RESET)
{
  if (RESET>0)
  {
    rate_   = 0.0;
  }
  return(rate_);
}
void DiscreteFilter::rateState(double in){}
void DiscreteFilter::assignCoeff(double tau){}
double DiscreteFilter::state(void){return(0);}

// Tustin rate-lag rate calculator, non-pre-warped, no limits, fixed update rate
// constructors
RateLagTustin::RateLagTustin() : DiscreteFilter(){}
RateLagTustin::RateLagTustin(const double T, const double tau, const double min, const double max)
: DiscreteFilter(T, tau, min, max)
{
  RateLagTustin::assignCoeff(tau);
}
//RateLagTustin::RateLagTustin(const RateLagTustin & RLT)
//: DiscreteFilter(RLT.T_, RLT.tau_, RLT.min_, RLT.max_){}
RateLagTustin::~RateLagTustin(){}
// operators
// functions
double RateLagTustin::calculate(double in, int RESET)
{
  if (RESET>0)
  {
    state_ = in;
  }
  RateLagTustin::rateState(in);
  return(rate_);
}
void RateLagTustin::rateState(double in)
{
  rate_   =  fmax(fmin(a_*(in - state_), max_), min_);
  state_  = in*(1.0-b_) + state_*b_;
}
void RateLagTustin::assignCoeff(double tau)
{
  a_  = 2.0 / (2.0*tau_ + T_);
  b_  = (2.0*tau_ - T_)/(2.0*tau_ + T_);
}
double RateLagTustin::state(void){return(state_);};







// Tustin lead-lag alculator, non-pre-warped, no limits, fixed update rate
// constructors
LeadLagTustin::LeadLagTustin() : DiscreteFilter(){}
LeadLagTustin::LeadLagTustin(const double T, const double tld, const double tau, const double min, const double max)
: DiscreteFilter(T, tau, min, max)
{
  LeadLagTustin::assignCoeff(tld, tau);
}
//LeadLagTustin::LeadLagTustin(const LeadLagTustin & RLT)
//: DiscreteFilter(RLT.T_, RLT.tau_, RLT.min_, RLT.max_){}
LeadLagTustin::~LeadLagTustin(){}
// operators
// functions
double LeadLagTustin::calculate(double in, int RESET)
{
  if (RESET>0)
  {
    state_ = in;
  }
  LeadLagTustin::rateState(in);
  return(rate_);
}
void LeadLagTustin::rateState(double in)
{
  rate_   =  fmax(fmin(a_*(in - state_), max_), min_);
  state_  = in*(1.0-b_) + state_*b_;
}
void LeadLagTustin::assignCoeff(double tld, double tau)
{
  tld_  = tld;
  a_    = 2.0 / (2.0*tau_ + T_);
  b_    = (2.0*tau_ - T_)/(2.0*tau_ + T_);
}
double LeadLagTustin::state(void){return(state_);};






// Exponential rate-lag rate calculator, non-pre-warped, no limits, fixed update rate
// constructors
RateLagExp::RateLagExp() : DiscreteFilter(){}
RateLagExp::RateLagExp(const double T, const double tau, const double min, const double max)
: DiscreteFilter(T, tau, min, max){
  RateLagExp::assignCoeff(tau);
}
//RateLagExp::RateLagExp(const RateLagExp & RLT)
//: DiscreteFilter(RLT.T_, RLT.tau_, RLT.min_, RLT.max_){}
RateLagExp::~RateLagExp(){}
// operators
// functions
double RateLagExp::calculate(double in, int RESET)
{
  if (RESET>0)
  {
    lstate_ = in;
    rstate_ = in;
  }
  RateLagExp::rateState(in);
  return(rate_);
}
double RateLagExp::calculate(double in, int RESET, const double T)
{
  if (RESET>0)
  {
    lstate_ = in;
    rstate_ = in;
  }
  RateLagExp::rateState(in, T);
  return(rate_);
}
void RateLagExp::rateState(double in)
{
  rate_    =  fmax(fmin(c_*(a_*rstate_ + b_*in - lstate_), max_), min_);
  rstate_  =  in;
  lstate_  += T_*rate_;
}
void RateLagExp::rateState(double in, const double T)
{
  rate_    =  fmax(fmin(c_*(a_*rstate_ + b_*in - lstate_), max_), min_);
  rstate_  =  in;
  lstate_  += T*rate_;
}
void RateLagExp::assignCoeff(double tau)
{
  double eTt = exp(-T_/tau_);
  a_   = tau_/T_ - eTt/(1-eTt);
  b_   = 1.0/(1-eTt) - tau_/T_;
  c_   = (1.0-eTt)/T_;
}
double RateLagExp::state(void){return(lstate_);};



// Tustin lag calculator, non-pre-warped, no limits, fixed update rate
// constructors
LagTustin::LagTustin() : DiscreteFilter(){}
LagTustin::LagTustin(const double T, const double tau, const double min, const double max)
: DiscreteFilter(T, tau, min, max)
{
  LagTustin::assignCoeff(tau);
}
//LagTustin::LagTustin(const LagTustin & RLT)
//: DiscreteFilter(RLT.T_, RLT.tau_, RLT.min_, RLT.max_){}
LagTustin::~LagTustin(){}
// operators
// functions
double LagTustin::calculate(double in, int RESET)
{
  if (RESET>0)
  {
    state_ = in;
  }
  LagTustin::calcState(in);
  return(state_);
}
double LagTustin::calculate(double in, int RESET, const double T)
{
  if (RESET>0)
  {
    state_ = in;
  }
  LagTustin::calcState(in, T);
  return(state_);
}
void LagTustin::calcState(double in)
{
  rate_   =  fmax(fmin(a_*(in - state_), max_), min_);
  state_  = in*(1.0-b_) + state_*b_;
}
void LagTustin::calcState(double in, const double T)
{
  a_  = 2.0 / (2.0*tau_ + T);
  b_  = (2.0*tau_ - T)/(2.0*tau_ + T);
  rate_   =  fmax(fmin(a_*(in - state_), max_), min_);
  state_  = in*(1.0-b_) + state_*b_;
}
void LagTustin::assignCoeff(double tau)
{
  a_  = 2.0 / (2.0*tau_ + T_);
  b_  = (2.0*tau_ - T_)/(2.0*tau_ + T_);
}
double LagTustin::state(void){return(state_);};
