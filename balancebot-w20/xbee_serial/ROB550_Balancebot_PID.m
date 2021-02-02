% Balancebot TF Model and PID Control design
clc
close all
clear 

%% constants
DT      = .01;          % 100hz controller loop
m_w     = .103;         % mass of one wheel in Kg MEASURED
m_b     = 1.1;          % balancebot body mass without wheels MEASURED
R       = .042;          % radius of wheel in m MEASURED
L       = .098;         % center of wheel to Center of mass MEASURED
I_r     = 0.00606935;        % Inertia of body about center (not wheel axis) Kg*m^2 (TO BE DETERMINED)
g       = 9.81;         % gravity m/s^2
R_gb    = 20.4;         % gearbox ratio
tau_s   = 0.28275%0.50;         % Motor output stall Torque @ V_nominal 
w_free  = 38.65;%50;           % Motor output free run speed @ V_nominal
V_n     = 12.0;         % motor nominal drive voltage
I_gb    = 100.0*10^-5;  % inertial of motor armature and gearbox 

% add inertia of wheels modeled as disks and times two for both sides
I_w = 2 * (I_gb+(m_w*R^2)/2);



%% inner loop plant including motor dynamics
a1 = I_w + (m_b + m_w)*R^2;
a2 = m_b * R * L;
a3 = I_r + m_b*L^2;
a4 = m_b * g * L;

% motor model equation used: t = e*u - f*w
b1 = 2 * tau_s; % stall torque of two motors
b2  = b1 / (w_free);   % constant provides zero torque @ free run

numG1 = [-b1*(a1+a2), 0];
denG1 = [(a1*a3 - a2^2), b2*(a1+a3+2*a2), -a1*a4, -a4*b2];
% Make TF monic 
numG1 = (1/denG1(1))*numG1; 
denG1 = (1/denG1(1))*denG1;
%make the TF
G1 = tf(numG1,denG1)
disp('G1 poles')
roots(denG1)
disp('G1 zeros')
roots(numG1)

%% outer loop plant
numG2 = [-(a2+a3), 0, a4];
denG2 = [a1+a2, 0,0];
% make monic
numG2 = (1/denG2(1))*numG2;
denG2 = (1/denG2(1))*denG2;
% make the TF
G2=tf(numG2,denG2)

%% Inner Loop Controller
% 
tr1 = 0.05; %desired rise time of inner loop
wc1 = 1.8/tr1; %rule of thumb crossover freq
gain = 1.0;
Kp1 = -3.0;
Ki1 = -14.0;
Kd1 = -0.18;
D1 = gain*pid(Kp1, Ki1, Kd1, 1/wc1);
OL1 = minreal(D1*G1);
CL1 = feedback(OL1,1);
figure
step(CL1)

%% Outer loop position controller
tr2 = 0.35; % desired rise time of outer loop
wc2 = 1.8/tr2;
Kp2 = 0.03;
Ki2 = 0.016;
Kd2 = 0.008;
D2 = pid(Kp2, Ki2, Kd2, 1/wc2)
OL2 = minreal(D2*CL1*G2);
CL2 = feedback(OL2,1);
figure
step(CL2)

%% find discrete controller for inner loop
D1z = c2d(D1,DT,'tustin');
tf(D1z)

%% find discrete controller for outer loop
D2z = c2d(D2,DT,'tustin');
tf(D2z)
