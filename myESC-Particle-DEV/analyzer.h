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
private:
  double 	      *a1_; 			    // Fourier series coefficient
  double 	      aint_;			    // Mantissa buffer for modf call
  double 	      *b1_;			      // Fourier series coefficient
  double 	      cosOmT_; 		    // Memory of intermediate calculation
  double        deltaOmegaLog_; // Delta log10(frequency, r/s) in sweep
  double        fractionalInputHalfSine_;   // Half amplitude sine wave excitation, fraction of steady state value
  int 		      iTime_;			    // Time counter
  int		        iTimMax_; 		  // Max time at frequency counter
  int           *ix_;           // Indeces of inputs to TFs
  int           *iy_;           // Indeces of outputs to TFs, corresponding ot ix_
  unsigned long maxIt_; 	      // Total iteration limit
  unsigned int  nsig_;          // Number of signals derived [from] sig[] size
  unsigned int  ntf_;           // Number of transfer functions derived from ix[] size
  unsigned long numIt_; 	      // Total number of iterations
  double 	      numInitCycles_; // Number of cycles for initialization
  int		        numAtFreq_; 	  // Number of time steps at frequency
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
  double        totalTime_;     // Total expected time to complete sweep, sec
  double 	      *transGain_;   	// Transfer function gains, dB
  int 		      timeAtFreqMax_;	// Maximum time at frequency, sec
  double 	      timeOmega_; 	  // Time into frequency point, sec
  double	      *transPhas_; 	  // Transfer function phase, deg
};

#endif
