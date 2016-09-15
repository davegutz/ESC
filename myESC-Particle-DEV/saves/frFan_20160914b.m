% Match model to freq resp data frFan_20160913a
titl='frFan_20160914b';
% steady throttle setting where fr done
throttle = 80.3;
% throttle to Nf char
M.AMDL = -0.0028;
M.BMDL = 0.8952;
M.CMDL = -2.14;

data=[...
1	-7.466	-18.65	-7.76	-17.36
1.259	-7.692	-22.9	-8.114	-21.42
1.585	-8.058	-27.92	-7.95	-27.17
1.995	-8.595	-33.84	-8.292	-32.94
2.512	-9.267	-39.67	-8.885	-40.46
3.162	-10.19	-46.96	-8.959	-51.53
3.981	-11.27	-53.3	-9.807	-62.7
5.012	-12.79	-61.87	-10.88	-80.58
6.311	-14.25	-65.9	-12.29	-91.73
7.943	-15.76	-70.22	-13.8	-108.1
10	-17.88	-74.68	-16.62	-125.3
12.59	-19.71	-76.62	-19.13	-140.4
15.85	-21.7	-79.68	-22.37	-161.8
19.95	-23.7	-81.49	-26.26	-179.7
25.12	-25.45	-82.38	-31.07	-391.07
31.63	-27.56	-82.19	-37.01	-397.01
39.81	-29.46	-82.45	-40.67	-400.67
50.13	-31.27	-80.64	-45.61	-405.61
63.09	-33.01	-80.86	-51.9	-411.9
79.45	-34.48	-75.92	-45.66	-405.66
100	-37.41	-88.12	-54.99	-414.99
125.9	-37.58	-87.28	-51.12	-411.12
158.5	-38.46	-86.65	-48.57	-408.57
199.5	-39.1	-85.53	-46.63	-406.63
251.2	-39.15	-84.85	-44.07	-404.07
316.2	-39.36	-76.44	-43.48	-403.48
398.1	-38.13	-79.13	-41.2	-401.2
501.1	-37.34	-74.41	-39.46	-399.46
630.8	-35.88	-72.86	-36.65	-396.65
794.2	-34.19	-72.79	-34.94	-394.94
1000	-48.87	-91.81	-52.72	-412.72
];
M.Nf = (M.AMDL*throttle+M.BMDL)*throttle+M.CMDL;
M.modGainx = 2*M.AMDL*throttle+M.BMDL;
D.w=data(:,1);
D.Mmodel=data(:,2);
D.Pmodel=data(:,3);
D.Mfan=data(:,4);
D.Pfan=data(:,5);

M.temf = 0.12;
M.tldesc = 0.01;
M.tlgesc = 0.05;
M.sys_rotor = tf(1, [M.temf 1]);
M.sys_t_Nf=M.sys_rotor*M.sys_rotor*tf(1,[M.tldesc 1])*tf(1,[M.tlgesc 1], 'ioDelay', 0.01)*M.modGainx;
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
compare(gfr,m1,ms,mproc)
L = findobj(gcf,'type','legend');
L.Location = 'southwest'; % move legend to non-overlapping location

