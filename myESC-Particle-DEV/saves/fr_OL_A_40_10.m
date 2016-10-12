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
0.16	1.19	-2.54	0.3	-3.11
0.2	1.21	-3.01	0.15	-3.63
0.25	1.21	-3.79	0.43	-4.12
0.32	1.21	-4.76	0.46	-5.25
0.4	1.2	-5.99	0.51	-6.56
0.5	1.2	-7.53	0.51	-8.53
0.63	1.18	-9.45	0.41	-10.46
0.79	1.15	-11.84	0.42	-12.78
1	1.12	-14.87	0.34	-16.08
1.26	1.04	-18.59	0.24	-20.12
1.58	0.93	-23.09	0.03	-24.71
1.99	0.76	-28.75	-0.17	-31.21
2.51	0.48	-35.57	-0.53	-38.26
3.16	0.08	-43.75	-0.95	-47.05
3.98	-0.61	-56.69	-1.78	-59.71
5.01	-1.11	-74.21	-2.55	-77.81
6.31	-2.64	-90.78	-4.23	-94.31
7.95	-5.45	-102.36	-7.44	-104.1
10.01	-6.69	-116.65	-8.88	-116.32
12.58	-9.26	-141.97	-12.19	-142.44
15.86	-12.4	-147.16	-15.63	-137.99
19.95	-18.74	-177.72	-25.09	-158.5
25.11	-20.01	170.66	-25.65	-172.35
31.64	-25.13	152.2	-32.56	-175.92
39.83	-31.14	-112.58	-28.25	-81.59
50.11	-36.26	111.91	-43	40.56
63.08	-38.05	20.24	-36.86	-12.26
79.49	-46.81	78.55	-49.56	6.6
99.97	-30.85	-35.4	-30.57	-42.52
126.05	-27.22	-100.9	-26.53	-101.41
158.49	-27.08	-79.69	-26.72	-81.2
199.35	-27.03	-77.96	-26.42	-77.05
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

