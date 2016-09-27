% Match model to freq resp data frFan_20160913a
titl='frFan_20160926a';
% steady throttle setting where fr done
throttle = 80.3;
% throttle to Nf char
M.AMDL = -0.0028;
M.BMDL = 0.8952;
M.CMDL = -2.14;

data=[...
1	-7.66	-14.64	-12.43	-52.64
1.259	-7.71	-18.32	-13.42	-62.75
1.585	-7.801	-22.69	-14.67	-71.1
1.995	-7.95	-28.45	-16.23	-82.22
2.512	-8.177	-35.71	-17.92	-92.17
3.162	-8.498	-43.77	-19.71	-101.4
3.981	-9.036	-54.85	-21.75	-111.2
5.012	-9.831	-68.58	-24.03	-125.2
6.31	-10.99	-83.65	-26.78	-135.6
7.943	-12.69	-99.65	-30.34	-143.2
10	-14.65	-117.2	-32.65	-151.6
12.59	-17.39	-136.7	-35.85	-170.8
15.85	-20.65	-152.2	-40.19	-166.3
19.95	-25.12	-171.6	-46.92	-204.1
25.12	-29	-186.8	-47.15	-202.8
31.62	-34.01	-200.8	-52.91	-213.3
39.81	-41.07	-196	-52.38	-447.84
50.12	-45.43	-226.5	-63.67	-325.12
63.1	-53.43	-248.9	-60.33	-374.41
79.44	-55.21	-223	-70.28	-529.1
99.99	-54.12	-410.08	-50.86	-412.32
125.9	-50.43	-443.49	-50.09	-442.61
158.5	-50.33	-474	-50.02	-472.1
199.5	-50.54	-424.31	-50.97	-424.66
251.2	-61.43	-529	-62.62	-508.3
316.2	-49.58	-457.64	-50.28	-456.12
398.1	-98.19	-255.7	-71.74	-187.8
501.1	-52.71	-496.5	-53.38	-497.1
631	-59.69	-377.91	-57.72	-383.65
794.4	-52.63	-496.2	-51.95	-492.6
1000	-66.4	-367.766	-65.77	-398.86
];
M.Nf = (M.AMDL*throttle+M.BMDL)*throttle+M.CMDL;
M.modGainx = 2*M.AMDL*throttle+M.BMDL;
D.w=data(:,1);
D.Mmodel=data(:,2);
D.Pmodel=data(:,3);
D.Mfan=data(:,4);
D.Pfan=data(:,5);

M.temf      = 0.13;     % square law aero damping
M.tldesc    = 0.05;     % placeholder
M.tlgesc    = 0.01;     % ESC back emf dynamics
M.tdelay    = 0.01;     % ESC update time/2 = ZOH
M.tsense    = .7;       % R*C = 1e-6*1e5
M.sys_rotor = tf(1, [M.temf 1]);
M.sys_t_Nf=M.sys_rotor*M.sys_rotor*tf([M.tldesc 1],[M.tlgesc 1], 'ioDelay', M.tdelay)*M.modGainx*tf(1,[M.tsense 1]);
[MR, P] = bode(M.sys_t_Nf, D.w);
M.Msys_t_Nf = zeros(1,length(D.w));
M.Psys_t_Nf = zeros(1,length(D.w));
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

