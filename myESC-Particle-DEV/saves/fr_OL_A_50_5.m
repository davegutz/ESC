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
0.16	0.18	-2.45	-0.26	-0.92
0.2	0.22	-2.78	-0.16	-2
0.25	0.22	-3.48	-0.03	-2.46
0.32	0.22	-4.36	-0.15	-2.62
0.4	0.22	-5.49	-0.09	-4.2
0.5	0.22	-6.89	-0.09	-6.04
0.63	0.2	-8.64	-0.19	-7.37
0.79	0.19	-10.84	-0.15	-9.56
1	0.16	-13.51	-0.21	-11.96
1.26	0.13	-16.84	-0.27	-14.88
1.58	0.04	-20.86	-0.32	-18.24
1.99	-0.05	-25.88	-0.46	-23.24
2.51	-0.25	-31.72	-0.63	-28.74
3.16	-0.53	-38.37	-0.8	-33.38
3.98	-1.15	-51.45	-1.25	-47.05
5.01	-1.06	-72.91	-1.2	-66.17
6.31	-2.69	-89.84	-2.96	-81.91
7.95	-6.37	-93.01	-6.36	-85.73
10.01	-6.37	-104.57	-6.3	-96.41
12.58	-8.6	-138.72	-8.63	-127.67
15.86	-11.31	-129.35	-11.28	-120.63
19.95	-21.75	-169.1	-22.76	-140.01
25.11	-18.51	-180.7	-19.33	-159.02
31.64	-24.26	-200.01	-26.73	-165.51
39.83	-21.34	-441.16	-18.75	-438.59
50.11	-37.19	-246.79	-34.83	-485.03
63.08	-31.05	-362.67	-33.41	-402.19
79.49	-47.73	-281.24	-37.06	-510.23
99.97	-23.81	-395.52	-24.56	-403.72
126.05	-19.46	-454.68	-19.23	-463.28
158.49	-19.79	-433.8	-20.22	-433.57
199.35	-19.82	-432.07	-20.01	-429.8
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

