%% Load System Params
VehicleLateralControlParams;
Dyn.Delta = 0;
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

%% Robust ECBF-SOCP with Different Uncertainty Level
thetaList = 0.2:0.2:0.8;
Nunc = length(thetaList);
xRECBFAll = cell(Nunc,1);
uRECBFAll = cell(Nunc,1);
tRECBFAll = cell(Nunc,1);    
for i = 1:Nunc
    disp(i);
    Dyn.beta = thetaList(i);    
    [xRECBFAll{i},uRECBFAll{i},tRECBFAll{i}] = SimulationFcn(Dyn,SafetySpec,filterCase,tspan,x0,Ref);
end

%% Save Data
save(mfilename);