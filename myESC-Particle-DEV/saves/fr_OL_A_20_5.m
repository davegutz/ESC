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
0.16	2.67	-2.32	4.18	-4.28
0.2	2.73	-2.48	4.21	-4.61
0.25	2.72	-3.12	4.24	-5.42
0.32	2.72	-3.92	4.24	-6.83
0.4	2.72	-4.92	4.24	-8.53
0.5	2.7	-6.18	4.25	-10.88
0.63	2.7	-7.75	4.2	-13.76
0.79	2.66	-9.7	4.17	-17.41
1	2.61	-12.06	3.97	-21.55
1.26	2.54	-15.17	3.78	-27.3
1.58	2.39	-18.96	3.46	-33.7
1.99	2.18	-23.38	3.06	-40.97
2.51	1.85	-28.73	2.31	-49.76
3.16	1.37	-35.35	1.37	-60.79
3.98	0.73	-49.31	0.2	-78.01
5.01	1.1	-69.26	-0.04	-101.24
6.31	-0.58	-87.1	-2.33	-124.38
7.95	-4.47	-97.72	-7.67	-132.84
10.01	-5.22	-108.39	-9.12	-143.39
12.58	-7.21	-138.72	-11.8	-178.8
15.86	-10.72	-139.64	-18.75	-173.03
19.95	-17.59	-174.59	-26.75	-240.53
25.11	-18.08	-187.93	-25.98	-215.23
31.64	-23.39	-207.26	-35.27	-246.25
39.83	-29.95	-466.39	-25.51	-425.18
50.11	-34.72	-246.26	-48.07	-316.16
63.08	-36.88	-338.94	-35.94	-374.94
79.49	-45	-282.46	-46.3	-288.34
99.97	-29.93	-392.85	-29.39	-402.9
126.05	-25.58	-454.07	-23.68	-457.85
158.49	-25.81	-433.15	-25.05	-433.29
199.35	-25.78	-431.46	-24.57	-430.14
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

