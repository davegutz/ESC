% Match model to freq resp data
titl=sprintf('%s', mfilename);
% steady throttle setting where fr done
throttle = 115;
% throttle to Nf char
P_LT_NG     = [-119175 28507];
P_NG_NF     = [-3926 0.9420];
N_MOD       = 222;

M.Ng    = P_LT_NG(1) + P_LT_NG(2)*log(throttle);
M.Nf    = (P_NG_NF(1) + P_NG_NF(2)*M.Ng)/N_MOD;
M.modGainx = P_LT_NG(2)*P_NG_NF(2)/N_MOD/throttle;

data=[...
0.16	0.23	-2.58	-0.87	-1.81
0.2	0.25	-3.06	-0.74	-2.14
0.25	0.25	-3.85	-0.71	-3.07
0.32	0.24	-4.84	-0.77	-4.27
0.4	0.24	-6.1	-0.84	-5.38
0.5	0.23	-7.67	-0.8	-7.1
0.63	0.21	-9.62	-0.87	-8.81
0.79	0.19	-12.09	-0.76	-11.19
1	0.16	-15.13	-0.77	-14.3
1.26	0.1	-18.86	-0.81	-17.66
1.58	-0.01	-23.54	-0.89	-22.4
1.99	-0.14	-29.27	-1.02	-27.61
2.51	-0.41	-36.12	-1.27	-34.7
3.16	-0.75	-44.45	-1.49	-42.66
3.98	-1.44	-57.04	-2.14	-53.93
5.01	-2	-75.11	-2.54	-72.07
6.31	-3.51	-91.47	-3.9	-87.45
7.95	-6.29	-101.08	-6.39	-98.65
10.01	-7.34	-115.57	-7.36	-113.72
12.58	-9.97	-141.93	-9.93	-139.73
15.86	-12.96	-144.42	-13.1	-147.08
19.95	-20.05	-175.78	-20.13	-180.78
25.11	-20.51	-187.55	-20.83	-193.55
31.64	-25.74	-206.04	-26.76	-220.05
39.83	-28.51	-460.27	-29.23	-449.67
50.11	-37.21	-248.06	-41.36	-284.86
63.08	-36.65	-349.46	-35.39	-365.91
79.49	-47.79	-281.41	-53.37	-293.36
99.97	-29.15	-396.53	-28.92	-398.29
126.05	-25.5	-461.1	-25.23	-460.69
158.49	-25.39	-439.91	-25.59	-442.74
199.35	-25.36	-438.15	-25.33	-438.66
];
D.w=data(:,1);
D.Mmodel=data(:,2);
D.Pmodel=data(:,3);
D.Mfan=data(:,4);
D.Pfan=data(:,5);

M.temf      = 0.12;     % square law aero damping
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

