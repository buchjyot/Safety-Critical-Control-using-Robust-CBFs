%% Figure 1
load('SimWithUncertainPlant.mat');
figure;

hold on;grid on;box on;

% Reference trajectories
p1 = plot([x0(5),Ref(5)],[Ref(1),Ref(1)],':k','LineWidth',2.5);

% LQR trajectories
p2 = plot(xLQR(:,5),xLQR(:,1),'b','LineWidth',2.5);
plot(xLQR(1,5),xLQR(1,1),'bo','MarkerFaceColor','b','LineWidth',2.5);

% Unsafe location: circle of radius rT around obstacle
theta = linspace(-pi,pi,1000);
c1 = SafetySpec.s_obs;
c2 = SafetySpec.e_obs;
Xc = c1 + 2*SafetySpec.r_obs*cos(theta);
Yc = c2 + 2*SafetySpec.r_obs*sin(theta);
plot(c1,c2,'ro','MarkerFaceColor','r','LineWidth',2.5);
p3 = patch(Xc,Yc,'r','LineWidth',2.5);
p3.EdgeColor = 'r';
alpha(0.1);

% % CBF trajectories
p4 = plot(xECBF(:,5),xECBF(:,1),'-.g','LineWidth',2.5);

% Robust CBF trajectories
p5 = plot(xRECBF(:,5),xRECBF(:,1),'m--','Linewidth',2.5);

xlabel('s','FontSize',14);
ylabel('e','FontSize',14);

axis equal
xlim([-s0, s0]);
ylim([-5.5,5.5]);

hl = legend([p2 p4 p5],'LQR','ECBF','RECBF','FontSize',12,'Orientation','horizontal');
set(hl,'location','southeast','fontsize',20);

%% Figure 2
figure;
subplot(1,2,1);

grid on;box on;hold on;
plot(tLQR,uLQR,'b','LineWidth',2);
plot(tECBF,uECBF,'-.g','LineWidth',2);
plot(tRECBF,uRECBF,'m--','Linewidth',2);

xlim(tspan);
xlabel('Time (sec)'); ylabel('u(t)');

% Plot CBFLOGS.h
subplot(1,2,2); cla

grid on;box on;hold on;
plot(tLQR,h(xLQR),'b','linewidth',2);
plot(tECBF,h(xECBF),'-.g','LineWidth',2);
plot(tRECBF,h(xRECBF),'m--','LineWidth',2);

ylim([-10,10]);xlim([0.55,0.9]);
xlabel('Time (sec)'); ylabel('h(x(t))');

%% Figure 3
load('SimWithNominalPlant.mat');
figure;

h = cell(Nunc,1);
legend_text = cell(Nunc,1);

% RECBF trajectories
for i = 1:Nunc
    h{i} = plot(xRECBFAll{i}(:,5),xRECBFAll{i}(:,1)); hold on
    set(h{i},'linewidth',1.5);    
    legend_text{i} = ['$\theta = $', num2str(thetaList(i))];
end

% LQR trajectories
hLQR = plot(xLQR(:,5),xLQR(:,1),'b');
set(hLQR,'linewidth',1.5);

% Unsafe location: circle of radius rT around obstacle
% obstacle
theta = linspace(-pi,pi,1000);
c1 = SafetySpec.s_obs;
c2 = SafetySpec.e_obs;
Xc = c1 + 2*SafetySpec.r_obs*cos(theta);
Yc = c2 + 2*SafetySpec.r_obs*sin(theta);
plot(c1,c2,'ro','MarkerFaceColor','r','LineWidth',2);
p3 = patch(Xc,Yc,'r','LineWidth',2);
p3.EdgeColor = 'r';
alpha(0.1);

xlabel('s','FontSize',14);
ylabel('e','FontSize',14);

axis equal
xlim([-6, 6]);
ylim([0,3.5]);

grid on; box on;

hl = legend([flip([h{:}]),hLQR,p3],[flip(legend_text);'LQR';'Unsafe Region'],'Location','northoutside','Orientation','Horizontal');
set(hl,'location','northoutside','fontsize',20,'interpreter','latex');