function [x,u,t] = SimulationFcn(Dyn,SafetySpec,filterCase,tspan,x0,Ref)
% ode option
odeOpt = odeset('Events',@hOriginCaptureEvent,'RelTol',1e-6,'AbsTol',1e-6);

% augmented system [x;u]
x0_aug = [x0;0];

% run simulatino based on filter case
switch filterCase
    case 0  % open-loop simulation
        [~,t,x] = initial(Dyn.G,x0,tspan(2));
        x_aug = [x, t*0];
        
    case 1  % nominal LQR controller
        [t,x_aug] = ode45(@(t,x) dyn_CL_LQR(t,x,Dyn,Ref,SafetySpec),tspan,x0_aug,odeOpt);
        
    case 2  % nominal ECBF-QP
        [t,x_aug] = ode45(@(t,x) dyn_CL_ECBF(t,x,Dyn,Ref,SafetySpec),tspan,x0_aug,odeOpt);
        
    case 3  % robust ECBF-SOCP
        [t,x_aug] = ode45(@(t,x) dyn_CL_RECBF(t,x,Dyn,Ref,SafetySpec),tspan,x0_aug,odeOpt);
        
    otherwise
        error(['[filterCase] ', num2str(filterCase),'not implemented!']);
end

x = x_aug(:,1:Dyn.Nx);
u = x_aug(:,Dyn.Nx+1);
end

%% Baseline LQR
% closed-loop by LQR controller
function xdot_aug = dyn_CL_LQR(t,x_aug,Dyn,Ref,SafetySpec)
x = x_aug(1:Dyn.Nx);

e = x - Ref;
u = -Dyn.K*e;

if Dyn.Delta ~= -1
    xdot = Dyn.G.A*x + Dyn.G.B*(1+Dyn.Delta)*u;
else
    [~, ~, ~, LgLf_h] = compute_ECBF(Dyn,SafetySpec,x);    
    w_wc = -Dyn.beta*norm(u)*sign(LgLf_h);
    xdot = Dyn.G.A*x + Dyn.G.B*(u + w_wc);
end

xdot_aug = [xdot; u];
end

%% ECBF-QP
function xdot_aug = dyn_CL_ECBF(t,x_aug,Dyn,Ref,SafetySpec)
x = x_aug(1:Dyn.Nx);

% nominal control command
e = x - Ref;
u0 = -Dyn.K*e;

% [nominal CBF - QP formulation]
% minimize (u-u0)^2
% s.t.     Lfr_h(x) + LgLfrn1_h(x)*u >= - Kalpha*eta(x),
%           where eta(x) = [h; hdot; hd2; ...; hdrn-1]
[h, hdot, Lf2_h, LgLf_h] = compute_ECBF(Dyn,SafetySpec,x);

H_QP = eye(Dyn.Nu);
f_QP = -u0;

K_alpha = [SafetySpec.eta^2, SafetySpec.eta*2];
hstack = [h; hdot];

% A'*x <= b
A_QP = -LgLf_h;
b_QP = Lf2_h + K_alpha*hstack;

options = optimoptions('quadprog','Display','off','Diagnostics','off');
u_CBF = quadprog(H_QP,f_QP,A_QP,b_QP,[],[],[],[],[],options);

if Dyn.Delta ~= -1
    xdot = Dyn.G.A*x + Dyn.G.B*(1+Dyn.Delta)*u_CBF;
else
    w_wc = -Dyn.beta*norm(u_CBF)*sign(LgLf_h);
    xdot = Dyn.G.A*x + Dyn.G.B*(u_CBF + w_wc);
end

xdot_aug = [xdot; u_CBF];
end

%% RECBF-SOCP
function xdot_aug = dyn_CL_RECBF(t,x_aug,Dyn,Ref,SafetySpec)
x = x_aug(1:Dyn.Nx);

% nominal control command
e = x - Ref;
u0 = -Dyn.K*e;

% [Robust CBF - SOCP formulation]
[h, hdot, Lf2_h, LgLf_h] = compute_ECBF(Dyn,SafetySpec,x);

% decision variable [u; t]
% objective function f'*x
f = [-u0' 1];

% SOC: ||Ax-b|| <= d'*x - gamma
%   >> secondordercone(A,b,d,gamma)
Znu1 = zeros(Dyn.Nu,1);

% minimum norm square of u
A2 = blkdiag(eye(Dyn.Nu), 1/sqrt(2));
b2 = [Znu1; 1/sqrt(2)];
d2 = [Znu1; 1/sqrt(2)];
gamma2 = -1/sqrt(2);

% [Robust CBF constraints]
scaling = Dyn.beta*norm(LgLf_h);
K_alpha = [SafetySpec.eta^2, SafetySpec.eta*2];
hstack = [h; hdot];

solver = {'augmented','normal','prodchol','schur'};
options = optimoptions('coneprog','MaxIterations',100,'LinearSolver',solver{3},'Display','off');

if scaling ~= 0
    sq_scaling = sqrt(scaling);
    
    A1 = [eye(Dyn.Nu) Znu1; Znu1' 0]*sq_scaling;
    b1 = [Znu1; 0]*sq_scaling;
    d1 = [LgLf_h'; 0]/sq_scaling;
    gamma1 = -(Lf2_h+K_alpha*hstack)/sq_scaling;
    
    socConstraints(1) = secondordercone(A1,b1,d1,gamma1);
    socConstraints(2) = secondordercone(A2,b2,d2,gamma2);
    
    [xopt,~,flag,~] = coneprog(f,socConstraints,[],[],[],[],[],[],options);
else
    d1 = [LgLf_h'; 0];
    gamma1 = -(Lf2_h+K_alpha*hstack);
    
    socConstraints = secondordercone(A2,b2,d2,gamma2);
    
    [xopt,~,flag,~] = coneprog(f,socConstraints,-d1',-gamma1,[],[],[],[],options);
end

switch flag
    case {-2,-3,-10}
        save('Failed_data');
        disp(['SOCP failed with flag: ',num2str(flag)]);
        % disp([t,x']);
        u_RCBF = 0;
        
    case {0,1}
        u_RCBF = xopt(1:Dyn.Nu);
        
    case {-7}
        u_RCBF = xopt(1:Dyn.Nu);
        % disp('SOCP is feasible but having small search direction');
end

% forward dynamics
if Dyn.Delta ~= -1
    xdot = Dyn.G.A*x + Dyn.G.B*(1+Dyn.Delta)*u_RCBF;
else
    w_wc = -Dyn.beta*norm(u_RCBF)*sign(LgLf_h);
    xdot = Dyn.G.A*x + Dyn.G.B*(u_RCBF + w_wc);
end

xdot_aug = [xdot; u_RCBF];
end

%% ECBF function
function [h, hdot, Lf2_h, LgLf_h] = compute_ECBF(Dyn,SafetySpec,x)
h = (x(1)-SafetySpec.e_obs)^2+(x(5)-SafetySpec.s_obs)^2 - (2*SafetySpec.r_obs)^2;
Dh = 2*[x(1)-SafetySpec.e_obs; 0;0;0; x(5)-SafetySpec.s_obs; 0];
hdot = Dh'*(Dyn.G.A*x);
Dhdot = 2*[x(2); x(1)-SafetySpec.e_obs; 0;0; x(6); x(5)-SafetySpec.s_obs];

Lf2_h = Dhdot'*(Dyn.G.A*x);
LgLf_h = Dhdot'*Dyn.G.B;
end

%% ODE options
function [value,isterminal,direction] = hOriginCaptureEvent(t,x) %#ok<INUSL>
%% hOriginCaptureEvent Function
% This function defines capture event to stop the simulation

% We want to bring the states to origin
FinalPosition = 20;
FinalVelocity = 0;
value(1)      = (x(1)-FinalPosition)^2 + (x(2)-FinalVelocity)^2 - sqrt(eps);
isterminal(1) = 1;   % we want to end integration when collision occurs
direction(1)  = 0;   % The zero should only ever be approached from above
end