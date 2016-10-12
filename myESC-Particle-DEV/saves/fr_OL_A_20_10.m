% Match model to freq resp data
titl=sprintf('%s', mfilename);
% steady throttle setting where fr done
throttle = 90;
% throttle to Nf char
P_LT_NG     = [-119175 28507];
P_NG_NF     = [-3926 0.9420];
N_MOD       = 222;

M.Ng    = P_LT_NG(1) + P_LT_NG(2)*log(throttle);
M.Nf    = (P_NG_NF(1) + P_NG_NF(2)*M.Ng)/N_MOD;
M.modGainx = P_LT_NG(2)*P_NG_NF(2)/N_MOD/throttle;

data=[...
0.16	2.57	-2.51	5.47	-8.48
0.2	2.6	-2.93	5.73	-12.57
0.25	2.59	-3.68	5.74	-13.88
0.32	2.59	-4.63	5.62	-14.92
0.4	2.59	-5.83	5.12	-15.02
0.5	2.57	-7.32	5.51	-20.96
0.63	2.55	-9.19	5.31	-23.82
0.79	2.52	-11.53	5.23	-28.82
1	2.47	-14.47	4.63	-32.91
1.26	2.4	-18.13	4.36	-39.21
1.58	2.26	-22.44	3.71	-43.95
1.99	2.08	-28.01	2.88	-50.32
2.51	1.74	-34.72	1.92	-57.34
3.16	1.29	-42.76	0.92	-68.65
3.98	0.57	-55.89	-0.42	-84.43
5.01	0.18	-73.01	-1.37	-105.77
6.31	-1.37	-90.01	-3.73	-124.91
7.95	-4.26	-103.78	-7.68	-138.92
10.01	-5.74	-118.12	-9.86	-153.87
12.58	-8.23	-141.9	-13.36	-181.8
15.86	-11.63	-151.31	-18.86	-186.83
19.95	-16.85	-178.45	-24.63	-232.04
25.11	-19.32	-192.39	-28.08	-234.49
31.64	-24.26	-210.62	-34.58	-257.81
39.83	-34.85	-526.75	-31.49	-419.35
50.11	-34.87	-248.38	-43.64	-346.12
63.08	-40.33	-311.13	-38.47	-356.67
79.49	-45.43	-281.39	-46.3	-380.83
99.97	-35.49	-390.53	-33.08	-397.23
126.05	-32.09	-459.97	-30.2	-462.58
158.49	-31.71	-438.56	-30.04	-441.31
199.35	-31.61	-437.04	-30.21	-435.79
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

