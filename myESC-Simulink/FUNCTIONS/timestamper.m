function [ts] = timestamper()
% stamp the time in the output stream
% DA Gutz   14-Sep-10   10_MODEL_122:  streamlined scripts from GE3000
%
% Inputs
% none

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

c = clock;
cm10 = floor(c(5)/10); cm1 = c(5) - 10*cm10; cs10 = floor(c(6)/10); cs1 = c(6) - 10*cs10;
ts = sprintf('Date: %d/%d/%d   Time: %d:%d%d:%d%2.1f\n', c(1), c(2), c(3), c(4), cm10, cm1, cs10, cs1);
fprintf('%s', ts);
