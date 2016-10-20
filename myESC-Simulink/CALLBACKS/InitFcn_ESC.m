%% InitFcn_ESC
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

if MOD.verbose-MOD.linearizing>0, fprintf('%s/InitFcn:  beginning init...', bdroot); timestamper;end


%% Initialization
clear ic

ic.E.G.nrpm     = Z.ngrpm;
ic.errqf        = 9999;
ic.nfrpm_max    = ic.E.G.nrpm;
ic.nfrpm_min    = 0;
ic.E.F.nrpm     = (ic.nfrpm_min + ic.nfrpm_max)/2;
ic.count        = 0;
ic.countMax     = 50;
while abs(ic.errqf) > 1e-8 && ic.count<ic.countMax,
    ic.count        = ic.count + 1;
    ic.trqgg        = (E.G.P_N_SHP(1)   + ic.E.G.nrpm*(E.G.P_N_SHP(2)   + ic.E.G.nrpm*E.G.P_N_SHP(3)))      / ic.E.G.nrpm   * 5252;
    ic.nbias        = ic.E.G.nrpm - (E.P_NG_NF(1) + E.P_NG_NF(2)*ic.E.G.nrpm);
    ic.nbiased      = ic.E.G.nrpm - ic.nbias;
    ic.trqgf        = (E.G.P_N_SHP(1)   + ic.nbiased*(E.G.P_N_SHP(2)    + ic.nbiased*E.G.P_N_SHP(3)))       / ic.nbiased    * 5252;
    ic.trqfgBack    = (E.P_N_SHPback(1) + ic.E.F.nrpm*(E.P_N_SHPback(2) + ic.E.F.nrpm*E.P_N_SHPback(3)))    / ic.E.F.nrpm   * 5252;
    ic.trqff        = (E.F.P_N_SHP(1)   + ic.E.F.nrpm*(E.F.P_N_SHP(2)   + ic.E.F.nrpm*E.F.P_N_SHP(3)))      / ic.E.F.nrpm   * 5252;
    ic.trqg         = E.G.B*ic.E.G.nrpm^2 + ic.trqgg + ic.trqfgBack;
    ic.backEMF      = ic.E.G.nrpm / E.Kv;
    ic.errqf        = (ic.trqgf - ic.trqff - E.F.B*ic.E.F.nrpm^2)/ic.trqgf;
    if ic.errqf>0
        ic.nfrpm_min    = ic.E.F.nrpm;
        ic.E.F.nrpm     = (ic.E.F.nrpm+ic.nfrpm_max)/2;
    else
        ic.nfrpm_max    = ic.E.F.nrpm;
        ic.E.F.nrpm     = (ic.E.F.nrpm+ic.nfrpm_min)/2;
    end
    if MOD.verbose-MOD.linearizing > 1
        fprintf('ESC(%ld):  nfrpm=%f/%f/%f/%f\n', ic.count, ic.nfrpm_min, ic.E.F.nrpm, ic.nfrpm_max, ic.errqf);
    end
end
clear ic.nbiased
if ic.count >= ic.countMax
    fprintf('\n*****WARNING(%s):  loop counter maximum.   nfrpm=%f/%f\n', mfilename, ic.E.F.nrpm, ic.errqf);
end
ic.dQgdNg       = 2*(ic.trqgg+E.G.B*ic.E.G.nrpm)/ic.E.G.nrpm;
ic.tauG         = E.G.J/ic.dQgdNg;
ic.dQfdNf       = 2*(ic.trqff+E.F.B*ic.E.F.nrpm)/ic.E.F.nrpm;
ic.tauF         = E.F.J/ic.dQfdNf;
ic.ia           = ic.trqg / E.Kt;
ic.verr         = ic.ia   * E.Ra;
ic.vcmd         = ic.verr + ic.backEMF;
ic.throttleRPM  = ic.vcmd * E.Kv;
ic.throttle     = exp((ic.throttleRPM-E.P_LT_NG(1))/E.P_LT_NG(2));

E   = OrderAllFields(E);
ic  = OrderAllFields(ic);
Z   = OrderAllFields(Z);
MOD = OrderAllFields(MOD);

if MOD.verbose-MOD.linearizing > 0, fprintf('%s/InitFcn:  complete init...', bdroot); timestamper; end
