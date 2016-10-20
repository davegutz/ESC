function default(varS, valS)
% 15-Oct-2015   DA Gutz         Created

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

%%
try evalin('caller', sprintf('temp = %s;', varS));
catch ERR %#ok<NASGU>
    evalin('caller', sprintf('%s = %s;', varS, valS))
end
