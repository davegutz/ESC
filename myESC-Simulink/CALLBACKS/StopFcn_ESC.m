%% stopFcn_ESC
% 18-Oct-2016   DA Gutz         Created

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

warning off MATLAB:xlswrite:AddSheet

%% Adopted features
if MOD.verbose-MOD.linearizing>0, fprintf('%s/StopFcn:  writing output files...', bdroot); timestamper;end

% openScopes_ESC;

if ~MOD.linearizing
    if ~exist('lastFig', 'var'), lastFig=0; end
    if ~exist('swHcopy', 'var'), swHcopy=0; end
    if swHcopy, close all, lastFig=0; end
    try
%         [DAT, lastFig]= plot_ESC(bdroot, MOD.run, E, lastFig, swHcopy);
    catch ERR
       error('%s:  if linearizing, set MOD.linearizing=1; ', mfilename);
    end
end
% warning on Simulink:Engine:SolverMinStepSizeWarn

warning on MATLAB:xlswrite:AddSheet

%%% End %%%
