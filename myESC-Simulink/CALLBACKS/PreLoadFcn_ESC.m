%% PreLoadFcn_ESC
% 18-Oct-2016       DA Gutz     Created
% Revisions

%% GE PROPRIETARY INFORMATION:
% The information contained in this document is GE proprietary
% information and is disclosed in confidence.  It is the property of GE and
% shall not be used, disclosed to others, reproduced, or
% exported without the express written consent of GE, including, but
% without limitation, it is not to be used in the creation, manufacture,
% development, or derivation of any repairs, modifications, spare parts, designs,
% or configuration changes
% or to obtain FAA or any other government or regulatory approval to do so.
% If consent is given for reproduction in whole or in part, this notice and t
% he notice set forth on each page of this document shall appear in
% any such reproduction in whole or in part.  

% Executive defaults
default('MOD.tFinal',   '0.6');
default('MOD.tStep',    '0.2');
default('MOD.DECIMATE', '4');
default('MOD.tInit',    '0.1');
default('MOD.tMinor',   '0.02'); % Standard ESC update time, s
default('MOD.verbose',  '1');
default('MOD.tuning',   '0');   % set to 1 to experiment with GEO values on successive runs.   Leave 0 to have model geometry defined fresh and clean each run
default('MOD.linearizing',  '0');
default('Z.ngrpm',      '32000');   % operating conditioni ~50% Nf 

% Input defaults
ESC_hardware;
initializing = 0;

clear temp
