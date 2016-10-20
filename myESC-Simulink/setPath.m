%% setPath  Administration
% 09-Jul-2015   DA Gutz     Created
% Revisions
% Inputs

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
% any such reproduction in whole or in part.  This technical data is considered ITAR
% and];or EAR controlled pursuant to 22 CFR Part 120-130 and 15 CFR Parts 730
% -774 respectively.  Transfer of this data by any means to a Non-US Person,
% whether in the United States or abroad, without the proper U.S. Government
% authorization (e.g., License, exemption, NLR, etc.), is strictly prohibited.
% This document contains trade secrets and confidential information exempt
% from disclosure under the FREEDOM OF INFORMATION ACT (FOIA) 5 USC § 552(b)(4).
%

%% Add path that has liraries and paramter files to keep parent directory less busy
if ~exist('./saves', 'dir'), mkdir('saves'), end
if ~exist('./figures', 'dir'), mkdir('figures'), end
% check path
try temp = MOD.modelRoot.Path;
    fprintf('MESSAGE(%s):  path and modelRoot already defined\n', mfilename);
    cd(MOD.modelRoot.Path)
catch ERR
    MOD.modelRoot.Path = pwd;
    addpath(MOD.modelRoot.Path, '-begin');
    addpath ([MOD.modelRoot.Path '\saves'], '-begin');
    addpath ([MOD.modelRoot.Path '\figures'], '-begin');
    addpath ([MOD.modelRoot.Path '\INPUTS'], '-begin');
    addpath ([MOD.modelRoot.Path '\FUNCTIONS'], '-begin');
    addpath ([MOD.modelRoot.Path '\LIBRARY'], '-begin');
    addpath ([MOD.modelRoot.Path '\CALLBACKS'], '-begin');
    addpath ([MOD.modelRoot.Path '\DATA'], '-begin');
end

