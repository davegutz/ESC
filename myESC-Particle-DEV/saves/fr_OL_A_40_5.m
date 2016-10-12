% Match model to freq resp data
titl=sprintf('%s', mfilename);
% steady throttle setting where fr done
throttle = 104;
% throttle to Nf char
P_LT_NG     = [-119175 28507];
P_NG_NF     = [-3926 0.9420];
N_MOD       = 222;

M.Ng    = P_LT_NG(1) + P_LT_NG(2)*log(throttle);
M.Nf    = (P_NG_NF(1) + P_NG_NF(2)*M.Ng)/N_MOD;
M.modGainx = P_LT_NG(2)*P_NG_NF(2)/N_MOD/throttle;

data=[...
0.16	1.25	-2.39	0.15	-2.43
0.2	1.3	-2.63	0.2	-2.6
0.25	1.3	-3.33	0.29	-3.96
0.32	1.3	-4.16	0.15	-4.42
0.4	1.3	-5.26	0.18	-6.28
0.5	1.29	-6.6	0.06	-7.36
0.63	1.27	-8.29	0.07	-9.56
0.79	1.26	-10.38	0.05	-11.75
1	1.22	-13.02	0.17	-14.44
1.26	1.17	-16.25	0.06	-18.5
1.58	1.06	-20.11	-0.1	-23.02
1.99	0.92	-24.92	-0.28	-28.59
2.51	0.66	-30.73	-0.6	-34.21
3.16	0.32	-37.19	-0.92	-42.3
3.98	-0.3	-50.55	-1.84	-55.6
5.01	-0.13	-71.56	-1.71	-77.32
6.31	-1.77	-88.67	-3.61	-95.47
7.95	-5.54	-94.85	-7.63	-98.64
10.01	-5.8	-105.91	-8.12	-107.81
12.58	-7.96	-138.58	-10.41	-144.41
15.86	-10.97	-133.1	-13.84	-130.4
19.95	-19.95	-176.25	-29.65	-159.23
25.11	-18.23	176.71	-23.42	-173.43
31.64	-23.86	157.47	-30.14	-160.48
39.83	-23.35	-85.16	-20.57	-75.22
50.11	-36.13	112.33	-37.75	-125.86
63.08	-32.62	2.13	-34.05	-35.02
79.49	-46.72	79.49	-35.26	-137.45
99.97	-25.24	-34.82	-26.28	-39.18
126.05	-20.93	-94.57	-20.9	-95.74
158.49	-21.24	-73.67	-22.08	-74.3
199.35	-21.25	-71.94	-21.27	-68.69
];
D.w=data(:,1);
D.Mmodel=data(:,2);
D.Pmodel=data(:,3);
D.Mfan=data(:,4);
D.Pfan=data(:,5);

M.temf      = 0.14;     % square law aero damping
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

