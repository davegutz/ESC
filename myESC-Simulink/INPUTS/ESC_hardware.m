%% ESC_hardware
% ESC circuitry parameters
% Outputs:
% E   Structure of engine
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

% Turnigy Plush 25A ESC
E.Tminor    = 0.020;    % Standard ESC
E.Ra        = 0.04;     % Ohms
E.La        = 4.2e-6;      % MicroHenries
E.Kv        = 4800;     % rpm/Volt
E.Kv_rps    = E.Kv*2*pi/60;     % rad/s / Volt
E.Kt        = 7.0/(E.Kv_rps);   % ft-lbf/Amp

% Gas generator
% Dr Mad Thrust HobbyKing 50mm EDF
E.G.J       = 4.5e-8;   % (rpm/s) / (ft-lbf)
E.G.B       = 0;        % ft-lbf / rpm^2
E.G.P_N_SHP = [0    -2.286e-6   1.811e-10]; % polynomial, ft-lbf, ft-lbf/rpm, ft-lbf/rpm^2
E.G.RPM_P   = 465;      % rpm/% to match 100% NG to 240 Watts

% Driven fan
% Dr Mad Thrust HobbyKing 50mm EDF
E.P_NG_NF   = [-10231  1.0237];     % poly for ss installed speed-speed NG to NF in rpm
E.F.J       = 4.5e-8;   % (rpm/s) / (ft-lbf)
E.F.B       = 0;        % ft-lbf / rpm^2
E.F.P_N_SHP = E.G.P_N_SHP;
E.F.RPM_P   = 465;      % rpm/% to match 100% NG to 240 Watts

% Interaction
E.P_N_SHPback = [7.97e-4     -5.356e-7   4.336e-11];  % polynomial, ft-lbf, ft-lbf/rpm, ft-lbf/rpm^2

% Throttle table
E.P_LT_NG       = [-18412   11951];         % observed ln(T)-->NG

