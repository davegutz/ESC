% Match model to freq resp data frFan_20160913a
titl='frFan_20160913a';
% steady throttle setting where fr done
throttle = 80.3;
% throttle to Nf char
M.AMDL = -0.0028;
M.BMDL = 0.8952;
M.CMDL = -2.14;
data=[...
0.1	-7.014	-1.931	-7.475	-2.241
0.1259	-7.016	-2.414	-7.468	-1.471
0.1585	-7.021	-3.035	-7.204	-3.428
0.1995	-7.026	-3.825	-7.369	-4.177
0.2512	-7.032	-4.772	-7.595	-4.595
0.3162	-7.052	-6.035	-7.621	-4.798
0.3981	-7.082	-7.572	-7.584	-6.041
0.5012	-7.124	-9.461	-7.502	-8.212
0.631	-7.196	-11.87	-7.641	-12.57
0.7943	-7.3	-14.75	-7.576	-13.89
1	-7.466	-18.24	-7.559	-17.11
1.259	-7.72	-22.66	-7.745	-21.39
1.585	-8.087	-27.72	-7.761	-24.71
1.995	-8.578	-33.08	-8.154	-33.21
2.512	-9.344	-39.68	-8.427	-40.58
3.162	-10.22	-46.45	-8.988	-50.85
3.98	-11.28	-52.3	-9.719	-58.74
5.013	-12.58	-58.95	-10.58	-75.22
6.308	-14.02	-64.39	-11.79	-89.79
7.943	-15.69	-69.84	-13.62	-106.2
10	-17.5	-73.79	-15.83	-121.2
12.59	-19.4	-79.96	-18.3	-143.8
15.85	-21.04	-84.57	-21.29	-159.2
19.95	-22.41	-87.74	-24.17	-170.8
25.13	-23.77	-92.65	-27.73	-177.2
31.63	-24.7	-98.37	-30.74	-181.9
39.85	-25.39	-104.2	-32.79	-173.5
50.13	-25.4	-108.7	-31.47	-130.7
63.04	-24.85	-113.3	-28.7	-129
79.53	-23.97	-118.3	-27.38	-128
100.1	-24.78	-119.2	-27.46	-127.9
];
M.Nf = (M.AMDL*throttle+M.BMDL)*throttle+M.CMDL;
M.modGainx = 2*M.AMDL*throttle+M.BMDL;
D.w=data(:,1);
D.Mmodel=data(:,2);
D.Pmodel=data(:,3);
D.Mfan=data(:,4);
D.Pfan=data(:,5);

M.sys_rotor = tf([.04 1], [0.16 1]);
M.sys_t_Nf=M.sys_rotor*M.sys_rotor*tf([0.080 1],[0.05 1])*tf([0.080 1],[0.05 1], 'ioDelay', 0.04)*M.modGainx;
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

