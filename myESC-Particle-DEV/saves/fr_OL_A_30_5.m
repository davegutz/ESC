% Match model to freq resp data
titl=sprintf('%s', mfilename);
% steady throttle setting where fr done
throttle = 94;
% throttle to Nf char
P_LT_NG     = [-119175 28507];
P_NG_NF     = [-3926 0.9420];
N_MOD       = 222;

M.Ng    = P_LT_NG(1) + P_LT_NG(2)*log(throttle);
M.Nf    = (P_NG_NF(1) + P_NG_NF(2)*M.Ng)/N_MOD;
M.modGainx = P_LT_NG(2)*P_NG_NF(2)/N_MOD/throttle;

data=[...
0.16	2.1	-2.35	1.4	-3.43
0.2	2.15	-2.56	1.47	-3.15
0.25	2.15	-3.21	1.58	-4.98
0.32	2.15	-4.02	1.45	-5.6
0.4	2.15	-5.05	1.44	-6.87
0.5	2.14	-6.34	1.48	-9.54
0.63	2.12	-7.96	1.46	-11.79
0.79	2.09	-10	1.34	-14.63
1	2.05	-12.47	1.35	-18.39
1.26	1.99	-15.59	1.14	-24.23
1.58	1.86	-19.38	1.1	-27.93
1.99	1.67	-23.99	0.72	-34.88
2.51	1.39	-29.83	0.18	-43.21
3.16	0.98	-36.08	-0.31	-52.77
3.98	0.33	-49.8	-1.34	-68.26
5.01	0.61	-70.1	-1.34	-91.77
6.31	-1.05	-87.77	-3.39	-113.17
7.95	-4.93	-96.45	-8.33	-120.06
10.01	-5.44	-107.3	-9.06	-130.23
12.58	-7.51	-138.71	-11.18	-166.32
15.86	-10.78	-136.73	-15.99	-154.06
19.95	-18.5	-176.56	-26.31	-226.75
25.11	-18.1	-185.76	-23.96	-207.05
31.64	-23.59	-205.21	-31.53	-234.95
39.83	-26.33	-452.93	-23.19	-428.68
50.11	-35.28	-247.75	-48	-315.42
63.08	-34.72	-349.95	-32.67	-378.56
79.49	-45.81	-280.86	-51.41	-190.83
99.97	-27.38	-394.02	-26.68	-398
126.05	-23.07	-454.37	-22.2	-452.1
158.49	-23.35	-433.44	-22.68	-434.06
199.35	-23.35	-431.76	-23	-433.62
];
D.w=data(:,1);
D.Mmodel=data(:,2);
D.Pmodel=data(:,3);
D.Mfan=data(:,4);
D.Pfan=data(:,5);

M.temf      = 0.18;     % square law aero damping
M.tldesc    = 0.05;     % placeholder
M.tlgesc    = 0.01;     % ESC back emf dynamics
M.tdelay    = 0.02/2 + 0.015/2;     % ESC update time/2 =  + asynch Arduino update timeZOH
M.tsense    = .03;       % R*C = 1e-6*1e5
M.sys_rotor = tf(1, [M.temf 1]);
M.sys_t_Nf=M.sys_rotor*M.sys_rotor*tf([M.tldesc 1],[M.tlgesc 1], 'ioDelay', M.tdelay)*M.modGainx*tf(1,[M.tsense 1]);
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
% semilogx(D.w, D.Mmodel,  'ro')
title(['Magnitude Limits for ' titl], 'Interpreter', 'none')
grid on
legend('Fan', 'MatModel')
xlabel('Frequency (r/s)')
ylabel('Servo->Fan Speed Gain (dB)')
V = axis; V(1) = V(1); V(2) = V(2)/10; axis(V);
subplot(2,1,2)
semilogx(D.w, D.Pfan,  'g.')
hold on
semilogx(D.w, M.Psys_t_Nf,  'b-')
% semilogx(D.w, D.Pmodel,  'ro')
title(['Phase Limits for ' titl], 'Interpreter', 'none')
grid on
legend('Fan', 'MatModel')
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

