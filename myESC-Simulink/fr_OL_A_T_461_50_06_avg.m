%% fr_OL_A_T_461_50_06_avg
% Average of 5 runs at 50 deg throttle (50), 100% rpm-46100 (461) and +/-6deg
% throttle (06) swing, open loop (OL) on Arduino (A) with Turnigy ESC (T)
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

% Match model to freq resp data
titl=sprintf('%s', mfilename);
% steady throttle setting where fr done
throttle = 76;
% throttle to Nf char
P_LT_NG     = [-28327, 14190];
P_NG_NF     = [-10231, 1.0237];
P_NG_Q      = [0 1.750e-7  1.154e-11];
N_MOD       = 461;

M.Ng        = P_LT_NG(1) + P_LT_NG(2)*log(throttle);
M.Nf        = P_NG_NF(1) + P_NG_NF(2)*M.Ng;
M.modGainx  = P_LT_NG(2) * P_NG_NF(2) / N_MOD / throttle;
M.Qg        = P_NG_Q(1)  + M.Ng*(P_NG_Q(2) + M.Ng*P_NG_Q(3));  % ft-lbf
M.dQgdNg    = M.Qg / 2 / M.Ng;  % ft-lbf / RPM
M.Qf        = P_NG_Q(1)  + M.Nf*(P_NG_Q(2) + M.Nf*P_NG_Q(3));  % ft-lbf
M.dQfdNf    = M.Qf / 2 / M.Nf;  % ft-lbf / RPM
M.J         = 4.5e-8;           %rpm/s / ft-lbf
M.TauG      = M.J / M.dQgdNg;   % s
M.TauF      = M.J / M.dQfdNf;   % s
M.La        = 4.2e-6;           % Henries
M.Ra        = 0.0399;           % Ohms
M.TauA      = M.La/M.Ra;        % s
M.Kv        = 4800;             % rpm / V
M.Kt        = 7.0/(M.Kv*2*pi/60); % ft-lbf/A
M.magGain   = M.Kt/M.Ra/M.Kv/M.dQgdNg;  % dimensionless
M.sysMagOL  = M.magGain*tf(1, [M.TauA 1])*tf(1, [M.TauG 1]);
M.sysMag    = M.sysMagOL / (1 + M.sysMagOL);

data=[...
0.16	-7.56	-2.96	-8.274	-2.558
0.2     -7.58	-3.91	-8.244	-3.064
0.25	-7.58	-4.92	-8.236	-4.28
0.32	-7.58	-6.18	-8.226	-5.672
0.4     -7.58	-7.78	-8.284	-7.562
0.5     -7.57	-9.77	-8.252	-9.662
0.63	-7.56	-12.26	-8.334	-12.058
0.79	-7.55	-15.39	-8.16	-15.016
1       -7.52	-19.2	-8.204	-18.72
1.26	-7.5	-23.95	-8.236	-23.844
1.58	-7.45	-29.51	-8.086	-28.994
1.99	-7.41	-36.47	-7.998	-36.32
2.51	-7.33	-44.15	-7.904	-43.89
3.16	-7.25	-53.22	-7.734	-52.576
3.98	-7.91	-64.41	-8.33	-63.404
5.01	-8.88	-87.17	-9.4	-86.394
6.31	-10.25	-102.55	-10.746	-101.028
7.95	-12.9	-99.49	-13.224	-96.648
10.01	-12.61	-116.89	-12.778	-114.318
12.58	-15.44	-152.83	-15.858	-149.864
15.86	-17.68	-139.12	-17.792	-134.662
19.95	-31.26	-212.4	-33.944	-200.242
25.11	-25.3	-196.2	-25.674	-185.63
31.64	-31.58	-225.91	-32.43	-209.708
39.83	-23.17	-433.65	-22.702	-437.928
50.11	-41.02	-329.09	-45.388	-288.246
63.08	-30.72	-376.01	-32.08	-373.374
79.49	-47.71	-429.89	-55.93	-354.602
99.97	-25.71	-403.09	-25.6	-400.784
126.05	-22.1	-463.33	-21.938	-461.736
158.49	-21.99	-439.28	-21.994	-440.624
];
D.w=data(:,1);
D.Mmodel=data(:,2)-3.7;
D.Pmodel=data(:,3);
D.Mfan=data(:,4);
D.Pfan=data(:,5);

% Simulink model
% Model path
modelPath = 'ESC/';
modelTopPath = 'ESC';
open('ESC.slx');
Z.ngrpm         = M.Ng;
MOD.linearizing = 1;
LIN.ioLin(1)    = linio([modelPath 'StepThrottle'], 1, 'in', 'off');
LIN.ioLin(2)    = linio([modelPath 'nf'], 1, 'out', 'off');
LIN.sys_thtl_nf = linearize(modelTopPath, LIN.ioLin);
[MR, P] = bode(LIN.sys_thtl_nf, D.w); 
LIN.Msys_thtl_nf = zeros(1,length(D.w));
LIN.Psys_thtl_nf = zeros(1,length(D.w));
LIN.sys_thtl_nf.Name='LIN.sys_thtl_nf';
for i=1:length(D.w), LIN.Msys_thtl_nf(i) = 20*log10(MR(i)); LIN.Psys_thtl_nf(i) = P(i); end
clear MR P

% Hand coded model
M.fit.tauF      = 0.13;     % square law aero damping
M.fit.tauG      = 0.13;     % square law aero damping
M.fit.tldE      = 0.05;     % placeholder
M.fit.tauE      = 0.01;     % ESC back emf dynamics
M.tdelay        = 0.02/2 + 0.015/2;     % ESC update time/2 =  + asynch Arduino update timeZOH
M.fit.tauF2V    = .03;
% R*C = 1e-6*1e5
M.sys_rotorG = tf(1, [M.fit.tauG 1]);
M.sys_rotorF = tf(1, [M.fit.tauF 1]);
M.sys_t_Nf=M.sys_rotorG*M.sys_rotorF*tf([M.fit.tldE 1],[M.fit.tauE 1], 'ioDelay', M.tdelay)*M.modGainx*tf(1,[M.fit.tauF2V 1]);
[MR, P] = bode(M.sys_t_Nf, D.w);
M.Msys_t_Nf = zeros(1,length(D.w));
M.Psys_t_Nf = zeros(1,length(D.w));
M.sys_t_Nf.Name='M.sys_t_Nf';
for i=1:length(D.w), M.Msys_t_Nf(i) = 20*log10(MR(i)); M.Psys_t_Nf(i) = P(i); end
clear MR P

figure
subplot(2,1,1)
semilogx(D.w, D.Mfan,  'g.')
hold on
% semilogx(D.w, D.Mmodel,  'ro')
semilogx(D.w, M.Msys_t_Nf,  'b-')
semilogx(D.w, LIN.Msys_thtl_nf,  'r--')
% semilogx(D.w, D.Mmodel,  'ro')
title(['Magnitude Limits for ' titl], 'Interpreter', 'none')
grid on
legend('FanData', 'MatHandModel', 'SimModel', 'location', 'southwest')
xlabel('Frequency (r/s)')
ylabel('Servo->Fan Speed Gain (dB)')
V = axis; V(1) = V(1); V(2) = V(2)/10; axis(V);
subplot(2,1,2)
semilogx(D.w, D.Pfan,  'g.')
hold on
semilogx(D.w, M.Psys_t_Nf,  'b-')
semilogx(D.w, LIN.Psys_thtl_nf,  'r--')
% semilogx(D.w, D.Pmodel,  'ro')
title(['Phase Limits for ' titl], 'Interpreter', 'none')
grid on
legend('FanData', 'MatHandModel', 'SimModel', 'location', 'southwest')
xlabel('Frequency, r/s')
ylabel('Phase (degrees)')
V = axis; V(1) = V(1); V(2) = V(2)/10; V(3)=-360;axis(V);




return

% sysid using freq
D.MRfan = 10.^(D.Mfan/20);
zfr = D.MRfan.*exp(1i*D.Pfan*pi/180);
Ts = 0.001;
gfr = idfrd(zfr, D.w, Ts);
figure
bode(gfr), legend('gfr')
m1      = oe(gfr,[2 2 1]);      % Discrete-time Output error (transfer function) model
ms      = ssest(gfr);           % Continuous-time state-space model with default choice of order
mproc   = procest(gfr,'P2UDZ'); % 2nd-order, continuous-time model with underdamped poles
mproc3  = procest(gfr,'P3D'); % 3rd-order, continuous-time model with underdamped poles
compare(gfr,m1,ms,mproc,M.sys_t_Nf)
L = findobj(gcf,'type','legend');
L.Location = 'southwest'; % move legend to non-overlapping location
grid('on')

