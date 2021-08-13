%% Load System Params
VehicleLateralControlParams;
Dyn.Delta = -1;
tspan = [0,tf_upperbound];

%% LQR Simulation
filterCase = 1; 
[xLQR,uLQR,tLQR] = SimulationFcn(Dyn,SafetySpec,filterCase,tspan,x0,Ref);

%% Nominal ECBF-QP
filterCase = 2; 
[xECBF,uECBF,tECBF] = SimulationFcn(Dyn,SafetySpec,filterCase,tspan,x0,Ref);

%% Robust ECBF-SOCP
filterCase = 3;
[xRECBF,uRECBF,tRECBF] = SimulationFcn(Dyn,SafetySpec,filterCase,tspan,x0,Ref);

%% Save Data
save(mfilename);