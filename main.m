clear all;
close all;
clc;

max_speed = 0;
%% Targets Ground Truth Setup
add_folders_to_path;
digits(20)

%Discrete Time Step
DT = 1.0;
%Number of Steps
horizon_steps=5;
N = 3600;

% ------------------------------
% Generate Targets' Ground Truth
% ------------------------------
numOfTargets=4;
tgt = zeros(numOfTargets,N,2);
load('CastawayDrift_4sources_1hr_06_10_22.mat')

for targetIndex=1:numOfTargets
    tgt(targetIndex,:,1) = squeeze(ballsPos(targetIndex,1,1:N))';
    tgt(targetIndex,:,2) = squeeze(ballsPos(targetIndex,2,1:N))';
end

% P process covariance matrix
P = zeros(N,numOfTargets,4,4);
q = zeros(numOfTargets,4,1);
for targetIndex=1:numOfTargets
   P(1,targetIndex,:,:) = eye(4,4)*5000;
end

%% Agent Init Setup
%field of view for the agent
FoVh = 69;
FoVv = 54;

% --------------------
% AGENT INITIAL POSITION
% --------------------
lowestAlt = 1;
agentX = [10 10 25];%[25, 40, lowestAlt*25];%[-96, -140, lowestAlt*25];%
radarX = agentX;
radarRadius = 0.1;

ux1=0;uy1=0;uz1=0;ax1=0;ay1=0;az1=0;

%Variables regarding the sensing model of the agent
mu= [0 0];
Pd_min = 0.25;
ratio = 0.6;
s1 = 2500;
s2 = s1*ratio;
SigmaInit = [s1 0; 0 s2];
Sigma(1,:,:) = SigmaInit;

%% setup init
nextStepIndex=1;
w(1:3,1) = agentX;
z1 = zeros(numOfTargets+1, horizon_steps-nextStepIndex,3,N);
z1(1,1,1:3,1) = agentX;
for targetIndex=2:numOfTargets+1
    z1(targetIndex,1,1:3,1) = [measureRadar(tgt(targetIndex-1,1,:));0];
end

p(:,:,1)= eye(4);
numOfParticles = 5000;
particles = zeros(numOfTargets,4,numOfParticles,1);
searching = zeros(1,length(tgt));
objValue = ones(1,length(tgt));

filt = trackingPF.empty(0,numOfTargets);
inView = false;
inViewVision = false;
inViewRadar = false;
agentStartDelay = 1;
notFoundCounter = 0;
notFoundTrigger = agentStartDelay + 5;
targetAquired = false;
offIndxInit = 1;
offIndx=offIndxInit;
offIndxUpdt=10;
searchTargetCounter=1;
cl2follow = 1;
detectHistory = zeros(length(tgt)-1, numOfTargets,2);
Fmax = zeros(length(tgt)-1,1);
objCovariance = zeros(numOfTargets,1);
ux = zeros(length(tgt)-1,1); uy = zeros(length(tgt)-1,1); uz = zeros(length(tgt)-1,1);

%start = tic;

realPlot = realTimePlot([],[], [], [], [], [], [], [], 'init');
pauseTime = 0.05;
isExperiment=false;
if(isExperiment)
    pauseTime = 0.01;
    rosnode = rosHandle([],[],[],[],'kios_mavich','http://10.42.0.1:11311','init');%'http://172.20.81.93:11311'   'http://172.20.243.33:11311'  'http://172.20.229.149:11311'
    previousAgentCartesianPos = [agentX(1),agentX(2)];
    previousAgentDdPos = [double(vpa(rosnode.pose.latitude)), double(vpa(rosnode.pose.longitude))];
end

%% main
for n=2:length(tgt)-1
    timing = tic;
    
    %calculate the FoV polygone and the sensing model
    [F,Sigma,x_coor,y_coor] = caclucatePd(n, agentX, FoVh, FoVv, s1, s2, mu, Pd_min, Sigma);

    %check wheather the target is within the FoV of either sensor prefering
    %the onboard sensor
    for targetIndex=1:numOfTargets
        [targetInFoV,targetOnFoV] = inpolygon(tgt(targetIndex,n,1), tgt(targetIndex,n,2), x_coor, y_coor) ;
        if(targetInFoV)
            index = ceil([sqrt((tgt(targetIndex,n,1)-x_coor(1))^2+(tgt(targetIndex,n,2)-y_coor(1))^2), sqrt((tgt(targetIndex,n,1)-x_coor(4))^2+(tgt(targetIndex,n,2)-y_coor(4))^2)]);
            inView(targetIndex) = binornd(1,F(index(1),index(2)));
            inViewVision(targetIndex) = true;
            inViewRadar(targetIndex) = false;
            targetAquired=true;
        elseif( (sqrt((tgt(targetIndex,n,1)-radarX(1))^2+(tgt(targetIndex,n,2)-radarX(2))^2)<=radarRadius) || (n<4) )
            inView(targetIndex) = true;
            inViewVision(targetIndex) = false;
            inViewRadar(targetIndex) = true;
            if(n==2)
                measureN = measureRadar(tgt(targetIndex,n,:)); measureN_1 = measureRadar(tgt(targetIndex,n-1,:));
                targetV = measureN-measureN_1./DT;
                x(targetIndex,1:4) = [measureN;targetV];
            end
        else
            targetAquired=true;
            inView(targetIndex) = false;
            inViewVision(targetIndex) = false;
            inViewRadar(targetIndex) = false;
        end
        detectHistory(n,targetIndex,1) = targetInFoV;
        detectHistory(n,targetIndex,2) = inView(targetIndex); 
    end

    %when we dont have a measurement
    if (any(inView) == false)
        %update the estimate without measurement
        y = zeros(numOfTargets,3,1);
        nonMeasureVector = zeros(numOfTargets,1);
        
        [filt,x,P(n,:,:,:)] = multiParticleFilterSimpleNLCustom(filt,DT,numOfParticles,x,squeeze(P(n-1,:,:,:)),y, nonMeasureVector);
        notFoundCounter = notFoundCounter+1;
    %when we have a measurement
    else
        if(mod(n,10)==0)
            disp(['n: ',num2str(n)]);
        end
        %set measure vector based on the sensor that has the measurement
        %and update the prediction
        y = zeros(numOfTargets,3);
        
        for targetIndex=1:numOfTargets
            measure(targetIndex)=false;
            if(inViewVision(targetIndex))
                y(targetIndex,:) = [measureVision(tgt(targetIndex,n,:),agentX(3));0];
                measure(targetIndex)=true;
            elseif(inViewRadar(targetIndex))
                y(targetIndex,:) = [measureRadar(tgt(targetIndex,n,:));0];
                measure(targetIndex)=true;
            end 
        end
        [filt,x,P(n,:,:,:)] = multiParticleFilterSimpleNLCustom(filt,DT,numOfParticles,x,squeeze(P(n,:,:,:)),y, measure);
    end
    
    if(~isempty(filt))
        for targetIndex=1:numOfTargets
            particles(targetIndex,:,:,n) = filt(1,targetIndex).Particles;
            q(targetIndex,:,n) = x(1:4);
        end
    end
    
    if(targetAquired)
        for covIndex=1:numOfTargets
            objCovariance(covIndex) = P(n,covIndex,1,1);
            if objCovariance(covIndex) >= 32e3
                fprintf('Target Covariance Great: P(1,1)=%.3f ,Target#=%d, Detected:%s\n', objCovariance(covIndex), covIndex, mat2str(measure(covIndex)));
%                 objCovariance(covIndex)=0;
            end
        end
        %follow the new waypoint in case we dont have a measurement
        
        agentX0 = [agentX(1),ux1,agentX(2),uy1,agentX(3),uz1];
        targetX0 = zeros(numOfTargets,4);
        for targetIndex=1:numOfTargets
            targetX0(targetIndex,:) = [x(targetIndex,1), x(targetIndex,3), x(targetIndex,2), x(targetIndex,4)];
        end
        
        [xx,yy,zz,ux1,uy1,uz1,ax1,ay1,az1,xxt,yyt, objValue(n)] = MPC_wSensingModelMulti_CovObj_Kalman(agentX0,targetX0,horizon_steps,DT,lowestAlt,FoVh, FoVv, objCovariance);
        nextStepIndex=1;
        
        
        %update vectors to be displayed later
        agentX = [xx(nextStepIndex),yy(nextStepIndex),zz(nextStepIndex)];
        agentXfuture = zeros(horizon_steps-nextStepIndex, 3);
        for futureIndex=1:horizon_steps-nextStepIndex
            agentXfuture(futureIndex,:) = [xx(futureIndex+nextStepIndex), yy(futureIndex+nextStepIndex), zz(futureIndex+nextStepIndex)];
        end
        
        targetXfuture = zeros(numOfTargets,horizon_steps-nextStepIndex, 3);
        for targetIndex=1:numOfTargets
            for futureIndex=1:horizon_steps-nextStepIndex
                targetXfuture(targetIndex,futureIndex,:) = [xxt(targetIndex,futureIndex+nextStepIndex), yyt(targetIndex,futureIndex+nextStepIndex),0];
            end
        end
        
        w(1:3,n) = agentX;
        z1(1,:,1:3,n) = agentXfuture;
        for targetIndex=2:numOfTargets+1
            z1(targetIndex,:,1:3,n) = targetXfuture(targetIndex-1,:,:);
        end
    end
    w(1:3,n) = agentX;

    [realPlot] = realTimePlot(realPlot,w, x, n, x_coor, y_coor, true, pauseTime, 'plot');%tgt(:,n,:)
    if ~ishghandle(realPlot.figure)
        close all;
        return
    end
    
    if (n>3) && (isExperiment)
%         [ux1, uy1, uz1]
%         [ax1,ay1,az1]

        rosnode = rosHandle(uy1, ux1, uz1,rosnode,[],[],'pub');
        pauseTime = 0.1-toc(timing);
        if(pauseTime>0)
            pause(pauseTime);
        end
        currentAgentDdPos = [double(vpa(rosnode.pose.latitude)), double(vpa(rosnode.pose.longitude))];
        [dy, dx, uy1, ux1] = displacement(previousAgentDdPos, currentAgentDdPos, double(vpa(rosnode.pose.velocity)), double(vpa(rosnode.pose.heading)));
        if(max_speed<max(ux1,uy1))
            max_speed=max(ux1,uy1);
            fprintf("\nmax speed: %.4f\n",max(ux1,uy1))
        end
%         ux1=0.1; uy1=0.1;
%         uz1=0;
        
        previousAgentCartesianPos = previousAgentCartesianPos + [dx,dy];
        previousAgentDdPos = currentAgentDdPos;
        agentX(1:2) = previousAgentCartesianPos;

        if(round(double(vpa(rosnode.pose.altitude)))>=25)
            agentX(3) = round(double(vpa(rosnode.pose.altitude)));
        else
            agentX(3) = zz(2)
            uz1=0;
        end
    end
    %toc(timing)
    Fmax(n) = max(F(:));
    ux(n)=ax1; uy(n)=ay1; uz(n)=az1;
end
%toc(start)

if(isExperiment)
    rosnode = rosHandle([],[],[],[],[],[],'stop');
end

%% 3D Result Plot

title = ['Multi-Castaway Tracking using MPC(time ',num2str(n),')_',num2str(numOfTargets),'targets_',num2str(Ns),'sources_', datestr(now,'dd_mm_yy_HH_MM')];

saveData = false;
if(saveData)
    dataFileName = [title,'.mat'];%nextname(title,'01','.mat');
    save(dataFileName, 'Sigma', 'searching', 'title', 'w', 'FoVh', 'FoVv', 'P', 'radarX', 'radarRadius', 'horizon_steps', 'detectHistory', 'Fmax', 'objValue', 'n', 'ux', 'uy', 'uz', 'numOfTargets')
end

%% Results Figure
sub_fig_plot = figure('units','normalized','outerposition',[-0.003645833333333,0.02962962962963,1.003645833333333,0.969444444444444]);

subplot(2,1,1)
yyaxis right
h = plot(w(3,1:end));
grid on;
hold on;
h.LineWidth = 2;
% legend({'$z^a_{k}$'},'Interpreter','latex', 'fontSize', 15, 'location', 'Southeast')
ylabel('Altitude (m)', 'fontSize',20);
xlabel('Timestep', 'fontSize',20)

% subplot(3,1,3)
yyaxis left
g = plot(Fmax(1:end));
grid on;
hold on;
g.LineWidth = 2;
% legend({'$Pd_{k}$','$z^a_{k}$'},'Interpreter','latex', 'fontSize', 15, 'location', 'Southeast')
ylabel('Probability (%)', 'fontSize',20);
xlabel('Timestep', 'fontSize',20)

subplot(2,1,2)
yyaxis right
[~, NumTar] = size(detectHistory(:,:,1));
targetsInView = zeros(size(detectHistory(:,1,1)));
for i=1:NumTar
    targetsInView = targetsInView + detectHistory(:,i,1);
end
g = plot(targetsInView);
yticks(1:1:NumTar)
grid on;
hold on;
g.LineWidth = 2;
% legend({'$Num of Targets$'},'Interpreter','latex', 'fontSize', 15, 'location', 'Southeast')
ylabel('Number of Targets ', 'fontSize',20);
xlabel('Timestep', 'fontSize',20)

for i=1:length(w)
   X=([-w(3,i)*tand(FoVh/2), w(3,i)*tand(FoVh/2), w(3,i)*tand(FoVh/2), -w(3,i)*tand(FoVh/2), -w(3,i)*tand(FoVh/2)]);
   Y=([-w(3,i)*tand(FoVv/2), -w(3,i)*tand(FoVv/2), w(3,i)*tand(FoVv/2), w(3,i)*tand(FoVv/2), -w(3,i)*tand(FoVv/2)]); 
   [~, volume(i)] = boundary(X(:), Y(:), 1);
end
yyaxis left
targetsInView = detectHistory(:,1,1) + detectHistory(:,2,1);
[~, NumTar] = size(detectHistory(:,:,1));
g = plot(volume);
grid on;
hold on;
g.LineWidth = 2;
% legend({'$Num of Targets$'},'Interpreter','latex', 'fontSize', 15, 'location', 'Southeast')
ylabel('FoV Volume (m^2) ', 'fontSize',20);
xlabel('Timestep', 'fontSize',20)


%plot objective value
figure();
op = plot(smoothdata(objValue, 'movmedian',10));
grid on;
hold on
op.LineWidth=2;
ylabel('Objective Value', 'fontsize', 25)
xlabel('Timestep', 'fontSize',25)
xlim([0 n])
