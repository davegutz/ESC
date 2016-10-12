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
0.16	2.05	-2.52	1.8	-3.69
0.2	2.07	-2.97	1.84	-4.44
0.25	2.07	-3.72	1.85	-5.71
0.32	2.07	-4.67	1.85	-6.73
0.4	2.06	-5.9	1.74	-8.62
0.5	2.06	-7.4	1.79	-10.76
0.63	2.04	-9.29	1.81	-13.44
0.79	2	-11.65	1.69	-17.01
1	1.96	-14.61	1.59	-21.21
1.26	1.88	-18.25	1.44	-26.24
1.58	1.76	-22.78	1.17	-32.92
1.99	1.57	-28.29	0.8	-39.94
2.51	1.27	-34.87	0.25	-49.7
3.16	0.83	-43.18	-0.47	-59.94
3.98	0.13	-56.15	-1.5	-75.19
5.01	-0.31	-73.62	-2.37	-96.87
6.31	-1.84	-90.3	-4.32	-115.66
7.95	-4.68	-103.17	-7.92	-127.92
10.01	-6.1	-117.75	-9.77	-143.51
12.58	-8.61	-142.04	-12.65	-172.39
15.86	-11.9	-149.64	-17.37	-174.15
19.95	-17.61	-177.77	-24.12	-226.03
25.11	-19.56	-191.16	-26.54	-221.78
31.64	-24.59	-209.64	-31.67	-232.67
39.83	-34.04	-497.38	-29.41	-429.8
50.11	-35.4	-247.69	-41.95	-320.93
63.08	-39.37	-325.56	-37.52	-369.27
79.49	-45.89	-281.75	-57.74	-265.25
99.97	-33.18	-393.41	-31.99	-398.51
126.05	-29.59	-460.5	-28.95	-458.54
158.49	-29.36	-439.18	-28.17	-442.77
199.35	-29.29	-437.56	-28.38	-440.53
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

