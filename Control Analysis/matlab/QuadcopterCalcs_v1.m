clear 
% close all

%% System Dynamics
% Mass info from solidworks (g*mm^2)
% syms Ix Iy
% Without blade guards
% mass_adj = 550/455.60;
% Ix = 2203385.34 * (1/1000) * (1/(1000^2)) * mass_adj;
% Iy = 2273769.07 * (1/1000) * (1/(1000^2)) * mass_adj;
% Iz = 3437944.37

% With blade guards
% mass_adj = 586.8/491.6;
% Ix = 2623253.85 * (1/1000) * (1/(1000^2)) * mass_adj;
% Iy = 2693637.60 * (1/1000) * (1/(1000^2)) * mass_adj; % Along the battery
% % Iz = 3437944.37;

% In coordinate system with top plane parallel with propeller planes
mass_real = 586.8/1000; % kg
mass_adj = mass_real/(491.6/1000);
Ix = 3115393.09 * (1/1000) * (1/(1000^2)) * mass_adj;
Iy = 3045011.97 * (1/1000) * (1/(1000^2)) * mass_adj;
deltaZ = 29.29/1000; % Offset between propeller plane and center of mass
% Iz = 4263587.01;

% Matlab is 1-indexed, matrix access is (row, col)
g0 = 9.81; % m/s^2
A = zeros(4);
A(3,1) = 1;
A(4,2) = 1;
A(1,3) = -g0*mass_adj*deltaZ/Ix; % Valid for small angles
A(2,4) = -g0*mass_adj*deltaZ/Iy; % Valid for small angles

r = 0.1116;
f_to_m = [r 0; 0 r]*4; % the delta F is provided by 4 propellers
B = [1/Ix 0; 0 1/Iy; 0 0; 0 0] * f_to_m;

C = [0 0 1 0; 0 0 0 1];
D = zeros(2);

%% Control input vs. Error
% B1^2 + B2^2 = 1; B1=B2
b0 = sqrt(1/2);
u_max = 3.5; % 3.5 N max thrust
rho = 1.0; % TODO - test it!
R = rho*[b0^2/u_max^2 0; 0 b0^2/u_max^2]; % weight on control inputs

% a1^2 + a2^2 + a3^2 + a4^2 = 1; a3=a4
alf12_sq = 1/10; % weight on the angular rates
alf34_sq = 4/10; % weight on the angles
ang_max = (pi/8);% 22,5 deg
rate_max = ang_max; 
Q = [alf12_sq/rate_max^2 0 0 0;
    0 alf12_sq/rate_max^2 0 0;
    0 0 alf34_sq/ang_max^2 0;
    0 0 0 alf34_sq/ang_max^2];
% p = 1;
% Q = p*(C')*C; % weight on state errors

% syms p11 p12 p13 p14 p21 p22 p23 p24 p31 p32 p33 p34 p41 p42 p43 p44
% syms p11 p13 p22 p24 p31 p33 p42 p44
% p12 = 0; p14 = 0; p21 = 0; p23 = 0; p32 = 0; p34 = 0; p41=0; p43=0;
% 
% 
% P = [p11 p12 p13 p14; p21 p22 p23 p24;
%     p31 p32 p33 p34; p41 p42 p43 p44];
% rhs = transpose(A)*P + P*A + Q - P*B*(R^-1)*transpose(B)*P;
% 
% % sols = solve(zeros(4) == rhs, [p11 p12 p13 p14 p21 p22 p23 p24 p31 p32 p33 p34 p41 p42 p43 p44]);
% % sols = solve(zeros(4) == rhs, [p11 p13 p22 p24 p31 p33 p41 p42 p43 p44], 'Real', true, 'IgnoreAnalyticConstraints', true); %, 'PrincipalValue', true)
% 
% sols = vpasolve(zeros(4) == rhs, [p11 p13 p22 p24 p31 p33 p42 p44]);
% % sols = vpasolve(zeros(4) == rhs, [p11 p13 p22 p24]);
% 
% k_lqr = R^-1*transpose(B)*P;
% 
% for i = 1:length(sols.p11)
%     tmp = subs(k_lqr,[p11 p13 p22 p24 p31 p33 p42 p44], ...
%         [sols.p11(i) sols.p13(i) sols.p22(i) sols.p24(i) ...
%          sols.p31(i) sols.p33(i) sols.p42(i) sols.p44(i)]);
%     if tmp >=0
%         k_lqr_subs = tmp
%     end
% end

%% Calculate LQR values
[k_lqr, P, e] = lqr(A,B,Q,R); %, Nbar
k_lqr

%% Precompensation Factor
Nbar = rscale(A,B,C,D,k_lqr)
% Nbar = (-C*((A-B*k_lqr)^-1)*B)^1;

A_closedloop = A - double(B*k_lqr);
B_closedloop = B*Nbar;
C_closedloop = C;
D_closedloop = D;
closedLoopSys = ss(A_closedloop, B_closedloop, C_closedloop, D_closedloop);

%% Nonlinear simulation
% time       = 0:0.01:5;
% x0         = [0 0 0 0];
% odeOptions = odeset('RelTol',1e-6,'AbsTol',[1e-8 1e-8]);
% u          = sin(time);
% [t,y]      = ode45(@system,time,x0,odeOptions,u);


%% Plotting
figure(1)
step_size = 0.2;
step(step_size*closedLoopSys); ylim([0,0.3]); % 0.2 radian (11 degree) step
% [y,t,x] = 
figure(2)
impulse(0.2*closedLoopSys)

%% SISO for pidTuner
bSISO = B(:,1);
cSISO = C(1,:);
dSISO = D(1,1);
openLoopSysSISO = ss(A, bSISO, cSISO, dSISO);
pidTuner(openLoopSysSISO,'pdf');

bClosedLoopSISO = B_closedloop(:,1);
cClosedLoopSISO = C_closedloop(1,:);
dClosedLoopSISO = D_closedloop(1,1);
closedLoopSysSISO = ss(A_closedloop, bClosedLoopSISO, cClosedLoopSISO, dClosedLoopSISO);

% for j=1:size(x,1)
%     u(j,:,:) = Nbar*step_size-k_lqr*squeeze(x(j,:,:));
% end
