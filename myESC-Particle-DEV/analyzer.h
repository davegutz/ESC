#ifndef _analyzer_h
#define _analyzer_h

static  const   double  pi  = 3.14159265358979323846264338327950288419716939937510;

class FR_Analyzer
{
public:
  FR_Analyzer();
  FR_Analyzer(const int omegaLogMin, const int omegaLogMax, const double deltaOmegaLog, const double fractionalInputHalfSine,
    const int numInitCycles, const double T, const double sig[], const int ix[], const int iy[], const int nsig, const int ntf);
  ~FR_Analyzer(){};
  // operators
  // functions
  double  calculate(void);
  double  omega(void){return(omega_);};
  bool    complete(void){return(complete_);};
private:
  double 	      *a1_; 			    // Fourier series coefficient
  double 	      aint_;			    // Mantissa buffer for modf call
  double 	      *b1_;			      // Fourier series coefficient
  double 	      cosOmT_; 		    // Memory of intermediate calculation
  bool          complete_;      // If done, T/F
  double        deltaOmegaLog_; // Delta log10(frequency, r/s) in sweep
  double        fractionalInputHalfSine_;   // Half amplitude sine wave excitation, fraction of steady state value
  int 		      iOmega_;			  // Time step at frequency
  unsigned long iResults_; 	    // Current number of results
  int		        iTargetOmega_;  // Time step target at frequency
  unsigned long iTargetResults_;// Target result limit
  int           *ix_;           // Indeces of inputs to TFs
  int           *iy_;           // Indeces of outputs to TFs, corresponding ot ix_
  unsigned int  nsig_;          // Number of signals derived [from] sig[] size
  unsigned int  ntf_;           // Number of transfer functions derived from ix[] size
  int		        numAtOmega_; 	  // Number of iterations at frequency
  double 	      numInitCycles_; // Number of cycles for initialization
  double 	      omega_; 		    // Excitation frequency, r/s
  double 	      omegaLogMax_;	  // Maximum log10(frequency, r/s), of sweep
  double 	      omegaLogMin_;	  // Minimum log10(frequency, r/s), of sweep
  double 	      omegaLog_; 		  // Excitation frequency log10
  const double  *sig_;          // Signal pointers from sig input
  double 	      *sigGain_;  	  // Signal gains, dB
  double 	      *sigPhas_;  	  // Signal phase, deg
  double 	      sinOmT_; 		    // Memory of intermediate calculation
  double 	      T_; 			      // Update time, sec
  double 	      Tlog_; 			    // Log10 of update time
  double        timeTotalSweep_; // Total expected time to complete sweep, sec
  int 		      timeTargetFreq_;// Target time at frequency, sec
  double 	      timeAtOmega_; 	// Time at frequency point, sec
  double 	      *transGain_;   	// Transfer function gains, dB
  double	      *transPhas_; 	  // Transfer function phase, deg
};

#endif
