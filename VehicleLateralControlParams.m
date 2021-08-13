%% Reference
% [1] "Stability Analysis using Quadratic Constraints for Systems with
%       Neural Network Controllers" by He Yin, Peter Seiler and Murat Arcak Fellow, IEEE


%% Plant
% Lateral Vehicle Control example [1]
%   Note:
%           longituditional dynamics is jus constant velocity
%   States:
%           e       (lateral error)
%           edot    (lateral velocity)
%           phi     (heading error)
%           phi     (heading angular velocity)
%           s       (path position)
%   Inputes:
%           u       (steering)
%   Disturbance:
%           c       (road curvature)
%   Uncertainty:
%           w       (input sector norm-bounded uncertainty)

% Dimentions
Dyn.Nx = 6;
Dyn.Nu = 1;

% coefficient
m = 1.67e3;     % mass (kg)
Iz = 2.1e3;     % moment of inertia (kg/m^2)
a = 0.99;       % C.G. to front axle (m)
b = 1.7;        % C.G. to rear axle (m)
Caf = -1.232e5; % front cornering stiffness (N/rad)
Car = -1.042e5; % rear cornering stiffness (N/rad)
U = 28;         % longitudinal velocity (m/s)

% State-space matrices
A = zeros(Dyn.Nx);
A(1,2) = 1;
A(2,2) = (Caf+Car)/(m*U);
A(2,3) = -(Caf+Car)/m;
A(2,4) = (a*Caf-b*Car)/(m*U);
A(3,4) = 1;
A(4,2) = (a*Caf-b*Car)/(Iz*U);
A(4,3) = -(a*Caf-b*Car)/Iz;
A(4,4) = (a^2*Caf+b^2*Car)/(Iz*U);
A(5,6) = 1; 

Bu = [0; -Caf/m; 0; -a*Caf/Iz; 0; 0];
% Bc = [0; (a*Caf-b*Car)/m-U^2; 0; (a^2*Caf+b^2*Car)/Iz; 0; 0];     %disturbance
C = eye(Dyn.Nx);
D = zeros(Dyn.Nx,Dyn.Nu);
Dyn.G = ss(A,Bu,C,D);

% [additivie-input uncertainty]
%   beta:   the uncertainty level
%   Delta:  the actual uncertainty  
Dyn.beta = 0.5;         % uncertainty level
% Dyn.beta = 0;

% Dyn.Delta = Dyn.beta*(2*rand-1);    % sampled an uncertianty
Dyn.Delta = Dyn.beta;               % set uncertainty to the level of bound
Dyn.Delta = -Dyn.beta;
Dyn.Delta = -0.9;
Dyn.Delta = 0;
Dyn.Delta = -1;                     % worst-case

%% Control Design
% [Nominal controller] 
% Design LQR to stabilize the plant
R = 5;
Q = diag([10,1,1/30,1]);

[K,P] = lqr(A(1:4,1:4),Bu(1:4),Q,R);
Dyn.K = [K zeros(1,2)];

% [CBF safety filter]
% Default value for safety filter design parameter
% eta(1) for OBSTACLE
SafetySpec.eta = 30;

%% Safety constraints
% [Note]
%   keep simple first. The vehicle is modeled as a circle of typical car width
%   in order to fit the US roadway standard

% [Check] US road standard
%       1. https://www.236safetyimprovements.org/roadway-sections
% [Pick a car's geometry]

w_vehicle = 1.5;
% w_land = 3.35;
% w_barrier = 5;

% Obastacle at the center of both lanes
SafetySpec.e_obs = 0;
SafetySpec.s_obs = 0;
SafetySpec.r_obs = w_vehicle;

h = @(x) (x(:,1)-SafetySpec.e_obs).^2+(x(:,5)-SafetySpec.s_obs).^2 - (2*SafetySpec.r_obs)^2;


%% [Simulatino setting]
s0 = 20;
x0 = [2; 0; 0; 0; -s0; U];     % Initial conditions
Ref = [0; 0; 0; 0; s0; U];       % Drive the states to origin

% Simulation time span
tf_upperbound = 2*s0/U;
% tf_upperbound = 0.5;
% tf_upperbound = 5;

% tf_upperbound = 3*tf_upperbound;