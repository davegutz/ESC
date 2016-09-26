% Match model to freq resp data frFan_20160913a
titl='frFan_20160915a';
% steady throttle setting where fr done
throttle = 80.3;
% throttle to Nf char
M.AMDL = -0.0028;
M.BMDL = 0.8952;
M.CMDL = -2.14;

data=[...
1	-7.129	-15.78	-7.78	-17.28
1.259	-7.186	-19.46	-7.85	-22.77
1.585	-7.302	-24.39	-7.987	-25.43
1.995	-7.468	-30.69	-8.504	-32.59
2.512	-7.733	-37.65	-8.514	-40.03
3.162	-8.119	-47.34	-9.347	-50
3.981	-8.79	-59.18	-9.908	-61.62
5.012	-9.666	-73.38	-10.89	-76.7
6.31	-10.91	-88.51	-12.08	-91.63
7.943	-12.77	-104	-14.06	-105.7
10	-15.01	-122.7	-16.17	-124.3
12.59	-17.88	-141.7	-18.84	-143.5
15.85	-21.37	-157.2	-21.92	-158.8
19.95	-25.83	-175.4	-25.88	-176.6
25.12	-29.77	-196.5	-29.83	-189.7
31.62	-35	-218.4	-34.89	-203.8
39.81	-42.31	-225.9	-41.26	-197.4
50.12	-46.5	-258.4	-45.02	-229.6
63.1	-54.23	-297.77	-49.85	-250.9
79.44	-55.89	-303.59	-58.62	-223.7
99.99	-54.41	-395.08	-52.76	-409.08
125.9	-51.07	-442.99	-49.81	-445.21
158.5	-50.89	-464.3	-50.54	-473.6
199.5	-51.1	-426.09	-51.44	-424.24
251.2	-61.98	-517.1	-61.28	-528.9
316.2	-50.17	-449.32	-50.17	-457.38
398.1	-96.51	-258.5	-75.23	-225.5
501.1	-53.25	-500.1	-53.9	-496.7
631	-60.14	-366.254	-61.53	-377.4
794.4	-53.22	-498.7	-53.45	-496.2
1000	-67.1	-372.82	-67.09	-368.179
];
M.Nf = (M.AMDL*throttle+M.BMDL)*throttle+M.CMDL;
M.modGainx = 2*M.AMDL*throttle+M.BMDL;
D.w=data(:,1);
D.Mmodel=data(:,2);
D.Pmodel=data(:,3);
D.Mfan=data(:,4);
D.Pfan=data(:,5);

M.temf      = 0.11;     % square law aero damping
M.tldesc    = 0.0;      % placeholder
M.tlgesc    = 0.05;     % ESC back emf dynamics
M.tdelay    = 0.01;     % ESC update time / 2 = ZOH
M.sys_rotor = tf(1, [M.temf 1]);
M.sys_t_Nf=M.sys_rotor*M.sys_rotor*tf(1,[M.tldesc 1])*tf(1,[M.tlgesc 1], 'ioDelay', M.tdelay)*M.modGainx;
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
V = axis; V(1) = V(1)/2; V(2) = V(2)*10; axis(V);
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
V = axis; V(1) = V(1)/2; V(2) = V(2)*10; axis(V);

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

