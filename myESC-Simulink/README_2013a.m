%% RADME_2013a.m
%  18-Oct-2016  Create

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

setPath
open('ESC.slx');

% Run the data match
fr_OL_A_T_461_50_06_avg

% Open and design
open('frFanA_20161020aSIM.slx');
% Analysis - Control Design - Compensator Design
% Tune blocks Kp, KpM, Ki, KiM
% Design plot Bode
% Analysis plot CL Step
% Automated Tuning - Optimization Based Tuning - Optimize Compensators -
% Select all the gains - Design Requirements - new - Step response bound, NfRef to Nf, 5% OS, 0.25 s rise to
% 90%, 2% settle in 1 sec 
% repeat new for NfRefM to NfM
% Ki=12.155, Kp=3.5202, KiM=11.637, KpM=3.3094;
% GM=18 @ 13, PM=78 @ 2.1



% 
