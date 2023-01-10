%Receding Horizon NMPC
%Single agent
%Multi-target
%Andreas Anastasiou
%October 2022

function [xx,yy,zz,ux1,uy1,uz1,ax1,ay1,az1,xxt,yyt, objValue] = MPC_wSensingModelMulti_CovObj_Kalman(agentX0,targetX0,N,DT,~,FoVx,FoVy,P)

%initial state vector of agent
%agentX0 = [x0 vx0 ax0 y0 vy0 ay0 z0 vz0 az0]'
%initial state vector of target
%targetX0 = [x0 vx0 y0 vy0 z0 vz0]'

%% Testing Variables
% clear all; clc;
% N=3;
% DT=1;
% agentX0 = [0 0  0 0  80 1];
% targetX0 = [1 2  3 4   5 6 ; 8 9  10 11   12 13];
% P = [7;14];
% zMin = 0.1;
% FoVx=60;
% FoVy=40;

%% Formulation Init
start = tic;
%DT = 1; % Dt:sampling time [s]
vmax = 15.0; vmin = -vmax; % Maximum velocities the drone can reach horizontally(m/s)
vmaxZ = 4.0; vminZ = -vmaxZ; % Maximum velocities the drone can reach vertically(m/s)
fmax = 3.0; fmin = -fmax; % Maximum force the drone can reach(m/s^2)
smoothingFactor=1.5; % Maximum difference between two consecutive forces 
worldMin = -5000; %minimum world dimension
worldMax = 5000; %maximum world dimension
worldMaxZ = 500; %maximum agent height
worldMinZ = 25; %minimum agent height
M=worldMax;%M technique
[NumTar, ~] =size(targetX0); % Number of targets to visit
tan_fov_x = tand(FoVx/2); % Field of View constant (Horizontal)
tan_fov_y = tand(FoVy/2); % Field of View constant (Vertical)
sigma = 0.4; % Measurement Noise std
qNoise = 1; % State Variance

if(worldMinZ-1>agentX0(5))
    disp(['Agent cannot be lower than the world boundaries! worldMinZ:', num2str(worldMinZ), ' agentZ: ', num2str(agentX0(5))])
    return
end

%% Construction of variables names
% tic
% Agent States
names = {};
for i=1:N
    [~,j]=size(names);
    names(j+1) = {strcat('x',num2str(i))};
    names(j+2) = {strcat('vx',num2str(i))};
    names(j+3) = {strcat('fx',num2str(i))};
    
    names(j+4) = {strcat('y',num2str(i))};
    names(j+5) = {strcat('vy',num2str(i))};
    names(j+6) = {strcat('fy',num2str(i))};
    
    names(j+7) = {strcat('z',num2str(i))};
    names(j+8) = {strcat('vz',num2str(i))};
    names(j+9) = {strcat('fz',num2str(i))};
end

% Add Agent's Initial States
names=['x0','vx0','y0','vy0','z0','vz0',names];
model.varnames = names;

% Target States
for k=1:NumTar
    for i=1:N
        [a j]=size(names);
        names(j+1)  = {strcat(num2str(k),'xt',num2str(i), '_',num2str(i-1))};     
        names(j+2)  = {strcat(num2str(k),'yt',num2str(i), '_',num2str(i-1))};
        names(j+3)  = {strcat(num2str(k),'pt',num2str(i), '_',num2str(i-1))};
        
        names(j+4) = {strcat(num2str(k),'Rt',num2str(i))};
        names(j+5) = {strcat(num2str(k),'wt',num2str(i))};
        names(j+6) = {strcat(num2str(k),'inv_wt',num2str(i))};
        names(j+7) = {strcat(num2str(k),'kt',num2str(i))};

        names(j+8) = {strcat(num2str(k),'meastx',num2str(i))};
        names(j+9) = {strcat(num2str(k),'measty',num2str(i))};
        names(j+10) = {strcat(num2str(k),'innovtx',num2str(i))};
        names(j+11) = {strcat(num2str(k),'innovty',num2str(i))};
        names(j+12) = {strcat(num2str(k),'covK',num2str(i))};
        
        names(j+13)  = {strcat(num2str(k),'xt',num2str(i), '_',num2str(i))};
        names(j+14)  = {strcat(num2str(k),'yt',num2str(i), '_',num2str(i))};
        names(j+15) = {strcat(num2str(k),'pt',num2str(i), '_',num2str(i))};
        
    end
end

% Add Targets' Initial States
for k=1:NumTar
    index1 = find(strcmp(names, strcat(num2str(k),'xt1_0')));
    names=[names(1,1:index1-1),strcat(num2str(k),'xt0_0'),names(1,index1:end)];
    names=[names(1,1:index1),strcat(num2str(k),'vxt0_0'),names(1,index1+1:end)];
    names=[names(1,1:index1+1),strcat(num2str(k),'yt0_0'),names(1,index1+2:end)];
    names=[names(1,1:index1+2),strcat(num2str(k),'vyt0_0'),names(1,index1+3:end)];
    names=[names(1,1:index1+3),strcat(num2str(k),'pt0_0'),names(1,index1+4:end)];

    model.varnames = names;
end
model.varnames = names;

% Add Helper Binary variables
for i=1:N
    [a j]=size(names);
    names(j+1)  = {strcat('fov_xl',num2str(i))};
    names(j+2)  = {strcat('fov_xr',num2str(i))};
    names(j+3)  = {strcat('fov_yt',num2str(i))};
    names(j+4)  = {strcat('fov_yb',num2str(i))};
end

for k=1:NumTar
    for i=1:N
        [a j]=size(names);
        
        names(j+1)  = {strcat(num2str(k),'bl',num2str(i))};
        names(j+2)  = {strcat(num2str(k),'br',num2str(i))};
        names(j+3)  = {strcat(num2str(k),'bt',num2str(i))};
        names(j+4)  = {strcat(num2str(k),'bb',num2str(i))};
        names(j+5)  = {strcat(num2str(k),'sb',num2str(i))};
        names(j+6)  = {strcat(num2str(k),'b',num2str(i))};
    end
end

% Add Forces Smoothing variables
for i=1:N
    [a j]=size(names);
    names(j+1)  = {strcat('fxdiff',num2str(i))};
    names(j+2)  = {strcat('fydiff',num2str(i))};
    names(j+3)  = {strcat('fzdiff',num2str(i))};
end

% fprintf('Time past for constructing names: %.2f\n', toc)

%% Construct the matrix for the optimization
%The matrices are fomrulated as an optimization problem that needs to
%optimize N variables to min/max an Objective function
len=length(names);

%% Fill the matrices appropietly in regard to the agent's dynamic model
% tic
%Left hand side of the equaiton
a1=zeros(1,len);
j=1;
for i=1:6:6*(N)
    index1 = find(strcmp(names, strcat('x',num2str(j-1))));
    index2 = find(strcmp(names, strcat('x',num2str(j))));
    index3 = find(strcmp(names, strcat('fx',num2str(j))));
    
    % x(k+1) = A*x(k) + B*u(k)
    
    % A = [1, 0, 0,   dt,  0,       0;        B = [0, 0, 0,  dt^2/2,       0,        0;
    %      0, 1, 0,    0,  dt,      0;             0, 0, 0,       0,  dt^2/2,        0;
    %      0, 0, 1,    0,   0,     dt;             0, 0, 0,       0,       0,   dt^2/2;
    %      0, 0, 0,  phi,   0,      0;             0, 0, 0,      dt,       0,        0;
    %      0, 0, 0,    0, phi,      0;             0, 0, 0,       0,      dt,        0;
    %      0, 0, 0,    0,   0,  gamma;]            0, 0, 0,       0,       0,       dt;]
    
    % A*x
    a1(i,index1)=-1;
    a1(i,index2)=1;
    a1(i,index1+1)=-DT;
    a1(i,index3)=-0.5*(DT^2);
    
    % B*u
    a1(i+1,index1+1)=-0.9;
    a1(i+1,index2+1)=1;
    a1(i+1,index3)=-DT;
    
    index1 = find(strcmp(names, strcat('y',num2str(j-1))));
    index2 = find(strcmp(names, strcat('y',num2str(j))));
    index3 = find(strcmp(names, strcat('fy',num2str(j))));
    
    a1(i+2,index1)=-1;
    a1(i+2,index2)=1;
    a1(i+2,index1+1)=-DT;
    a1(i+2,index3)=-0.5*(DT^2);
    
    a1(i+3,index1+1)=-0.9;
    a1(i+3,index2+1)=1;
    a1(i+3,index3)=-DT;
    
    index1 = find(strcmp(names, strcat('z',num2str(j-1))));
    index2 = find(strcmp(names, strcat('z',num2str(j))));
    index3 = find(strcmp(names, strcat('fz',num2str(j))));
    
    a1(i+4,index1)=-1;
    a1(i+4,index2)=1;
    a1(i+4,index1+1)=-DT;
    a1(i+4,index3)=-0.5*(DT^2);
    
    a1(i+5,index1+1)=-0.9;
    a1(i+5,index2+1)=1;
    a1(i+5,index3)=-DT;
    
    j=j+1;
end
[n,m]=size(a1);
A1=zeros(1,len);
for i=1:6
    A1(i,i)=1;
end
A1=[A1;a1];

%Right hand side of the equation
[n m]=size(A1);
B1=zeros(1,n);
% B1(13:8:end)=vmax;
% B1(14:8:end)=vmin;
%agent initial state
B1(1,1:6)=agentX0(1:6);
B1_sense(1:n)= '=';
% B1_sense(13:8:n)= '<';
% B1_sense(14:8:n)= '>';

A11=zeros(1,len);
for i=1:3:3*(N-1)
    
    index1 = find(strcmp(names, strcat('fx',num2str(i))));
    index2 = find(strcmp(names, strcat('fx',num2str(i+1))));
    index3 = find(strcmp(names, strcat('fxdiff',num2str(i))));

    A11(i,index1)=1;
    A11(i,index2)=-1;
    A11(i,index3)=1;
    
    
    index1 = find(strcmp(names, strcat('fy',num2str(i))));
    index2 = find(strcmp(names, strcat('fy',num2str(i+1))));
    index3 = find(strcmp(names, strcat('fydiff',num2str(i))));

    A11(i+1,index1)=1;
    A11(i+1,index2)=-1;
    A11(i+1,index3)=1;
    
    
    index1 = find(strcmp(names, strcat('fz',num2str(i))));
    index2 = find(strcmp(names, strcat('fz',num2str(i+1))));
    index3 = find(strcmp(names, strcat('fzdiff',num2str(i))));

    A11(i+2,index1)=1;
    A11(i+2,index2)=-1;
    A11(i+2,index3)=1;
end
[n m]=size(A11);
B11=zeros(1,n);
B11_sense(1:n)= '=';

% fprintf('Time past for constructing agent dynamics contraints: %.2f\n', toc)

%% Fill the matrices appropietly in regard to the target's dynamic model
% tic
%Left hand side of the equaiton
a2 = zeros(1,len);
aQc2 = zeros(NumTar*N*8,len,len);
aqc2 = zeros(NumTar*N*8,len);
%Right hand side of the equation
b2(1) = 0;
b2_sense(1) = '=';
bc2 = zeros(NumTar*N*8,1);
bc2_sense(NumTar*N*8,1) = '=';

for k=1:NumTar
    for i=1:N
        [a,~]=size(a2);
        
        index1 = find(strcmp(names, strcat(num2str(k),'xt',num2str(i), '_',num2str(i-1))));
        index2 = find(strcmp(names, strcat(num2str(k),'xt',num2str(i-1), '_',num2str(i-1))));
        index3 = find(strcmp(names, strcat(num2str(k),'vxt0_0')));
        
        a2(a+1,index1)=1;
        a2(a+1,index2)=-1;
        a2(a+1,index3)=-DT;
        b2(a+1) = 0;
        b2_sense(a+1) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'yt',num2str(i), '_',num2str(i-1))));
        index2 = find(strcmp(names, strcat(num2str(k),'yt',num2str(i-1), '_',num2str(i-1))));
        index3 = find(strcmp(names, strcat(num2str(k),'vyt0_0')));
        
        a2(a+2,index1)=1;
        a2(a+2,index2)=-1;
        a2(a+2,index3)=-DT;
        b2(a+2) = 0;
        b2_sense(a+2) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'pt',num2str(i), '_',num2str(i-1))));
        index2 = find(strcmp(names, strcat(num2str(k),'pt',num2str(i-1), '_',num2str(i-1))));
        
        a2(a+3,index1)=1;
        a2(a+3,index2)=-1;
        b2(a+3) = qNoise;
        b2_sense(a+3) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'Rt',num2str(i))));
        index2 = find(strcmp(names, strcat('z',num2str(i))));
        
        a2(a+4,index1)=1;
        a2(a+4,index2)=-sigma;
        b2(a+4) = 0;
        b2_sense(a+4) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'wt',num2str(i))));
        index2 = find(strcmp(names, strcat(num2str(k),'Rt',num2str(i))));
        index3 = find(strcmp(names, strcat(num2str(k),'pt',num2str(i), '_',num2str(i-1))));
        
        a2(a+5,index1)=1;
        a2(a+5,index2)=-1;
        a2(a+5,index3)=-1;
        b2(a+5) = 0;
        b2_sense(a+5) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'wt',num2str(i))));
        index2 = find(strcmp(names, strcat(num2str(k),'inv_wt',num2str(i))));

        aQc2((k-1)*N+i,index1, index2) = 1;
        bc2((k-1)*N+i,1) = 1;
        bc2_sense((k-1)*N+i,1) = '=';
       
        index1 = find(strcmp(names, strcat(num2str(k),'kt',num2str(i))));
        index2 = find(strcmp(names, strcat(num2str(k),'inv_wt',num2str(i))));
        index3 = find(strcmp(names, strcat(num2str(k),'pt',num2str(i), '_',num2str(i-1))));
        
        aQc2((k-1)*N+i+N*NumTar,index2, index3) = -0.5;
        aQc2((k-1)*N+i+N*NumTar,index3, index2) = -0.5;
        aqc2((k-1)*N+i+N*NumTar,index1) = 1;
        bc2((k-1)*N+i+N*NumTar,1) = 0;
        bc2_sense((k-1)*N+i+N*NumTar,1) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'meastx',num2str(i))));
        index2 = find(strcmp(names, strcat(num2str(k),'Rt',num2str(i))));
        
        a2(a+6,index1)=1;
        a2(a+6,index2)=-randn();
        b2(a+6) = 0;
        b2_sense(a+6) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'measty',num2str(i))));
        index2 = find(strcmp(names, strcat(num2str(k),'Rt',num2str(i))));
        
        a2(a+7,index1)=1;
        a2(a+7,index2)=-randn();
        b2(a+7) = 0;
        b2_sense(a+7) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'kt',num2str(i))));
        index2 = find(strcmp(names, strcat(num2str(k),'meastx',num2str(i))));
        index3 = find(strcmp(names, strcat(num2str(k),'innovtx',num2str(i))));

        aQc2((k-1)*N+i+N*NumTar*2, index1, index2) = -0.5;
        aQc2((k-1)*N+i+N*NumTar*2, index2, index1) = -0.5;
        aqc2((k-1)*N+i+N*NumTar*2, index3) = 1;
        bc2((k-1)*N+i+N*NumTar*2,1)=0;
        bc2_sense((k-1)*N+i+N*NumTar*2,1) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'kt',num2str(i))));
        index2 = find(strcmp(names, strcat(num2str(k),'measty',num2str(i))));
        index3 = find(strcmp(names, strcat(num2str(k),'innovty',num2str(i))));

        aQc2((k-1)*N+i+N*NumTar*3, index1, index2) = -0.5;
        aQc2((k-1)*N+i+N*NumTar*3, index2, index1) = -0.5;
        aqc2((k-1)*N+i+N*NumTar*3, index3) = 1;
        bc2((k-1)*N+i+N*NumTar*3,1)=0;
        bc2_sense((k-1)*N+i+N*NumTar*3,1) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'xt',num2str(i), '_',num2str(i-1))));
        index2 = find(strcmp(names, strcat(num2str(k),'xt',num2str(i), '_',num2str(i))));
        index3 = find(strcmp(names, strcat(num2str(k),'b',num2str(i))));
        index4 = find(strcmp(names, strcat(num2str(k),'innovtx',num2str(i))));

        aQc2((k-1)*N+i+N*NumTar*4, index4, index3) = -0.5;
        aQc2((k-1)*N+i+N*NumTar*4, index3, index4) = -0.5;
        aqc2((k-1)*N+i+N*NumTar*4, index1) = -1;
        aqc2((k-1)*N+i+N*NumTar*4, index2) = 1;
        bc2((k-1)*N+i+N*NumTar*4,1)=0;
        bc2_sense((k-1)*N+i+N*NumTar*4,1) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'yt',num2str(i), '_',num2str(i-1))));
        index2 = find(strcmp(names, strcat(num2str(k),'yt',num2str(i), '_',num2str(i))));
        index3 = find(strcmp(names, strcat(num2str(k),'b',num2str(i))));
        index4 = find(strcmp(names, strcat(num2str(k),'innovty',num2str(i))));

        aQc2((k-1)*N+i+N*NumTar*5, index4, index3) = -0.5;
        aQc2((k-1)*N+i+N*NumTar*5, index3, index4) = -0.5;
        aqc2((k-1)*N+i+N*NumTar*5, index1) = -1;
        aqc2((k-1)*N+i+N*NumTar*5, index2) = 1;
        bc2((k-1)*N+i+N*NumTar*5,1)=0;
        bc2_sense((k-1)*N+i+N*NumTar*5,1) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'pt',num2str(i), '_',num2str(i-1))));
        index3 = find(strcmp(names, strcat(num2str(k),'kt',num2str(i))));
        index4 = find(strcmp(names, strcat(num2str(k),'covK',num2str(i))));

        aQc2((k-1)*N+i+N*NumTar*6, index1, index3) = -0.5;
        aQc2((k-1)*N+i+N*NumTar*6, index3, index1) = -0.5;
        aqc2((k-1)*N+i+N*NumTar*6, index4) = 1;
        bc2((k-1)*N+i+N*NumTar*6,1)=0;
        bc2_sense((k-1)*N+i+N*NumTar*6,1) = '=';
        
        index1 = find(strcmp(names, strcat(num2str(k),'pt',num2str(i), '_',num2str(i-1))));
        index2 = find(strcmp(names, strcat(num2str(k),'pt',num2str(i), '_',num2str(i))));
        index3 = find(strcmp(names, strcat(num2str(k),'b',num2str(i))));
        index4 = find(strcmp(names, strcat(num2str(k),'covK',num2str(i))));

        aQc2((k-1)*N+i+N*NumTar*7, index4, index3) = 0.5;
        aQc2((k-1)*N+i+N*NumTar*7, index3, index4) = 0.5;
        aqc2((k-1)*N+i+N*NumTar*7, index1) = -1;
        aqc2((k-1)*N+i+N*NumTar*7, index2) = 1;
        bc2((k-1)*N+i+N*NumTar*7,1)=0;
        bc2_sense((k-1)*N+i+N*NumTar*7,1) = '=';
    end
end
a2(1,:)=[];
b2(1)=[];
b2_sense(1)=[];

[~,m]=size(a2);
A2=zeros(1,m);
B2(1) = 0;
B2_sense(1) = '=';

for k=1:NumTar
    index1 = find(strcmp(names, strcat(num2str(k),'xt0_0')));
    A2((k-1)*5+1,index1)=1;
    B2((k-1)*5+1) = targetX0(k,1);
    B2_sense((k-1)*5+1) = '=';

    A2((k-1)*5+2,index1+1)=1;
    B2((k-1)*5+2) = targetX0(k,2);
    B2_sense((k-1)*5+2) = '=';

    index1 = find(strcmp(names, strcat(num2str(k),'yt0_0')));
    A2((k-1)*5+3,index1)=1;
    B2((k-1)*5+3) = targetX0(k,3);
    B2_sense((k-1)*5+3) = '=';

    A2((k-1)*5+4,index1+1)=1;
    B2((k-1)*5+4) = targetX0(k,4);
    B2_sense((k-1)*5+4) = '=';
    
    index1 = find(strcmp(names, strcat(num2str(k),'pt0_0')));
    A2((k-1)*5+5,index1)=1;
    B2((k-1)*5+5) = P(k);
    B2_sense((k-1)*5+5) = '=';
end

A2 = [A2;a2];
B2 = [B2,b2];
B2_sense = [B2_sense,b2_sense];

% fprintf('Time past for constructing targets dynamics contraints: %.2f\n', toc)

%% Fill the matrices approprietly in regard to the binaries
% tic
%Left hand side of the equaiton - equations about constracting the fov
a3=zeros(1,len);
j=1;
for i=1:4:4*(N)
    index1 = find(strcmp(names, strcat('x',num2str(j))));%agent's x
    index2 = find(strcmp(names, strcat('z',num2str(j))));%agent's z
    index3 = find(strcmp(names, strcat('fov_xl',num2str(j))));%xl
    index4 = find(strcmp(names, strcat('fov_xr',num2str(j))));%xr
    
    a3(i,index1)=-1;
    a3(i,index2)=tan_fov_x-0.15;
    a3(i,index3)=1;
    
    a3(i+1,index1)=-1;
    a3(i+1,index2)=-tan_fov_x+0.15;
    a3(i+1,index4)=1;
    
    index1 = find(strcmp(names, strcat('y',num2str(j))));%agent's y
    index3 = find(strcmp(names, strcat('fov_yt',num2str(j))));%yt
    index4 = find(strcmp(names, strcat('fov_yb',num2str(j))));%yb
    
    a3(i+2,index1)=-1;
    a3(i+2,index2)=-tan_fov_y+0.15;
    a3(i+2,index3)=1;
    
    a3(i+3,index1)=-1;
    a3(i+3,index2)=tan_fov_y-0.15;
    a3(i+3,index4)=1;
    
    j=j+1;
end
[n,m]=size(a3);
A3=zeros(1,len);
A3=[a3];
%Right hand side of the equation
[n m]=size(A3);
B3=zeros(1,n);
%agent initial state
B3_sense(1:n)= '=';

%Left hand side of the equaiton - equations about the target being in the fov
a4=zeros(1,len);
for k=1:NumTar
    for i=1:N
        [a ~]=size(a4);

        index1 = find(strcmp(names, strcat(num2str(k),'xt',num2str(i),'_',num2str(i-1))));%target's x
        index2 = find(strcmp(names, strcat('fov_xl',num2str(i))));%xl
        index3 = find(strcmp(names, strcat(num2str(k),'bl',num2str(i))));%bl

        a4(a+1,index1)=1;
        a4(a+1,index2)=-1;
        a4(a+1,index3)=-M;

        a4(a+2,index1)=1;
        a4(a+2,index2)=-1;
        a4(a+2,index3)=-M;

        index1 = find(strcmp(names, strcat(num2str(k),'xt',num2str(i),'_',num2str(i-1))));%target's x
        index2 = find(strcmp(names, strcat('fov_xr',num2str(i))));%xr
        index3 = find(strcmp(names, strcat(num2str(k),'br',num2str(i))));%br

        a4(a+3,index1)=-1;
        a4(a+3,index2)=1;
        a4(a+3,index3)=-M;

        a4(a+4,index1)=-1;
        a4(a+4,index2)=1;
        a4(a+4,index3)=-M;

        index1 = find(strcmp(names, strcat(num2str(k),'yt',num2str(i),'_',num2str(i-1))));%target's y
        index2 = find(strcmp(names, strcat('fov_yt',num2str(i))));%yt
        index3 = find(strcmp(names, strcat(num2str(k),'bt',num2str(i))));%bt

        a4(a+5,index1)=-1;
        a4(a+5,index2)=1;
        a4(a+5,index3)=-M;

        a4(a+6,index1)=-1;
        a4(a+6,index2)=1;
        a4(a+6,index3)=-M;

        index1 = find(strcmp(names, strcat(num2str(k),'yt',num2str(i),'_',num2str(i-1))));%target's y
        index2 = find(strcmp(names, strcat('fov_yb',num2str(i))));%yb
        index3 = find(strcmp(names, strcat(num2str(k),'bb',num2str(i))));%bb

        a4(a+7,index1)=1;
        a4(a+7,index2)=-1;
        a4(a+7,index3)=-M;

        a4(a+8,index1)=1;
        a4(a+8,index2)=-1;
        a4(a+8,index3)=-M;

    end
end
a4(1,:)=[];
[n,m]=size(a4);
A4=zeros(1,len);
A4=[a4];
%Right hand side of the equation
[n m]=size(A4);
B4=ones(1,n).*-M;
B4(2:2:n)=0;
%agent initial state
B4_sense(1:n)= '>';
B4_sense(2:2:n)= '<';


%Left hand side of the equaiton
a5=zeros(1,len);
for k=1:NumTar
    for i=1:N
        [a ~]=size(a5);
        
        index1 = find(strcmp(names, strcat(num2str(k),'bl',num2str(i))));%bl
        index2 = find(strcmp(names, strcat(num2str(k),'br',num2str(i))));%br
        index3 = find(strcmp(names, strcat(num2str(k),'bt',num2str(i))));%bt
        index4 = find(strcmp(names, strcat(num2str(k),'bb',num2str(i))));%bb
        index5 = find(strcmp(names, strcat(num2str(k),'sb',num2str(i))));%sb

        a5(a+1,index1)=-1; 
        a5(a+1,index2)=-1;
        a5(a+1,index3)=-1;
        a5(a+1,index4)=-1;
        a5(a+1,index5)=1;

        index1 = find(strcmp(names, strcat(num2str(k),'sb',num2str(i))));%sb
        index2 = find(strcmp(names, strcat(num2str(k),'b',num2str(i))));%b
        
        a5(a+2,index1)=1;
        a5(a+2,index2)=-M;

        a5(a+3,index1)=1;
        a5(a+3,index2)=-M;
    end
end
a5(1,:)=[];
[n,m]=size(a5);
A5=zeros(1,len);
A5=[a5];

%Right hand side of the equation
[n m]=size(A5);
B5=zeros(1,n);
B5(2:3:n)=-M+4;
B5(3:3:n)=4;

%agent initial state
B5_sense(1:n)= '=';
B5_sense(2:3:n)= '>';
B5_sense(3:3:n)= '<';

% fprintf('Time past for constructing binary contraints: %.2f\n', toc)

%% Upper and Lower Bounds
% tic
%Constaint: Maximum and minimum velocity,Boundaries of the map

model.vtype(1:len) = 'C';

% Upper and lower bounds for agent's initial state
index1 = find(strcmp(names, 'x0'));
index2 = find(strcmp(names, 'z0'));
model.lb(index1:2:index2) = worldMin;
model.ub(index1:2:index2) = worldMax;
model.vtype(index1:2:index2) = 'C';

index3 = find(strcmp(names, 'vx0'));
index4 = find(strcmp(names, 'vz0'));
model.lb(index3:2:index4) = vmin*DT;
model.ub(index3:2:index4) = vmax*DT;
model.vtype(index1:2:index2) = 'C';

% Upper and lower bounds for agent's x,y states
index1 = find(strcmp(names, 'x1'));
index2 = find(strcmp(names, strcat('y',num2str(N))));
model.lb(index1:3:index2) = worldMin;
model.ub(index1:3:index2) = worldMax;
model.vtype(index1:3:index2) = 'C';

index3 = find(strcmp(names, 'vx1'));
index4 = find(strcmp(names, strcat('vy',num2str(N))));
model.lb(index3:3:index4) = vmin*DT;
model.ub(index3:3:index4) = vmax*DT;
model.vtype(index1:3:index2) = 'C';

index5 = find(strcmp(names, 'fx1'));
index6 = find(strcmp(names, strcat('fy',num2str(N))));
model.lb(index5:3:index6) = fmin*DT;
model.ub(index5:3:index6) = fmax*DT;
model.vtype(index1:3:index2) = 'C';

% Upper and lower bounds for agent's z states
index1 = find(strcmp(names, 'z1'));
index2 = find(strcmp(names, strcat('z',num2str(N))));
model.lb(index1:9:index2) = worldMinZ;
model.ub(index1:9:index2) = worldMaxZ;
model.vtype(index1:9:index2) = 'C';

index3 = find(strcmp(names, 'vz1'));
index4 = find(strcmp(names, strcat('vz',num2str(N))));
model.lb(index3:9:index4) = vminZ*DT;
model.ub(index3:9:index4) = vmaxZ*DT;
model.vtype(index1:9:index2) = 'C';

index5 = find(strcmp(names, 'fz1'));
index6 = find(strcmp(names, strcat('fz',num2str(N))));
model.lb(index5:9:index6) = fmin*DT;
model.ub(index5:9:index6) = fmax*DT;
model.vtype(index1:9:index2) = 'C';

% Upper and lower bounds for targets' xt, yt and zt
for k=1:NumTar
    index1 = find(strcmp(names, strcat(num2str(k),'xt0_0')));
    model.lb(index1) = worldMin*1000;
    model.ub(index1) = worldMax*1000;
    model.vtype(index1) = 'C';
    model.lb(index1+1) = vmin*1000;
    model.ub(index1+1) = vmax*1000;
    model.vtype(index1+1) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'xt1_0')));
    index2 = find(strcmp(names, strcat(num2str(k),'xt',num2str(N),'_',num2str(N-1))));
    model.lb(index1:15:index2) = worldMin*1000;
    model.ub(index1:15:index2) = worldMax*1000;
    model.vtype(index1:15:index2) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'xt1_1')));
    index2 = find(strcmp(names, strcat(num2str(k),'xt',num2str(N),'_',num2str(N))));
    model.lb(index1:15:index2) = worldMin*1000;
    model.ub(index1:15:index2) = worldMax*1000;
    model.vtype(index1:15:index2) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'yt0_0')));
    model.lb(index1) = worldMin*1000;
    model.ub(index1) = worldMax*1000;
    model.vtype(index1) = 'C';
    model.lb(index1+1) = vmin*1000;
    model.ub(index1+1) = vmax*1000;
    model.vtype(index1+1) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'yt1_0')));
    index2 = find(strcmp(names, strcat(num2str(k),'yt',num2str(N),'_',num2str(N-1))));
    model.lb(index1:15:index2) = worldMin*1000;
    model.ub(index1:15:index2) = worldMax*1000;
    model.vtype(index1:15:index2) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'yt1_1')));
    index2 = find(strcmp(names, strcat(num2str(k),'yt',num2str(N),'_',num2str(N))));
    model.lb(index1:15:index2) = worldMin*1000;
    model.ub(index1:15:index2) = worldMax*1000;
    model.vtype(index1:15:index2) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'pt0_0')));
    index2 = find(strcmp(names, strcat(num2str(k),'pt',num2str(N),'_',num2str(N))));
    model.lb(index1:15:index2) = 0;
    model.ub(index1:15:index2) = inf;
    model.vtype(index1:15:index2) = 'C';

    index1 = find(strcmp(names, strcat(num2str(k),'pt1_0')));
    index2 = find(strcmp(names, strcat(num2str(k),'pt',num2str(N),'_',num2str(N-1))));
    model.lb(index1:15:index2) = 0;
    model.ub(index1:15:index2) = inf;
    model.vtype(index1:15:index2) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'Rt1')));
    index2 = find(strcmp(names, strcat(num2str(k),'Rt',num2str(N))));
    model.lb(index1:15:index2) = 0;
    model.ub(index1:15:index2) = inf;
    model.vtype(index1:15:index2) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'wt1')));
    index2 = find(strcmp(names, strcat(num2str(k),'wt',num2str(N))));
    model.lb(index1:15:index2) = 0;
    model.ub(index1:15:index2) = inf;
    model.vtype(index1:15:index2) = 'C';
    model.lb(index1+1:15:index2+1) = 0;
    model.ub(index1+1:15:index2+1) = inf;
    model.vtype(index1+1:15:index2+1) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'kt1')));
    index2 = find(strcmp(names, strcat(num2str(k),'kt',num2str(N))));
    model.lb(index1:15:index2) = 0;
    model.ub(index1:15:index2) = inf;
    model.vtype(index1:15:index2) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'meastx1')));
    index2 = find(strcmp(names, strcat(num2str(k),'meastx',num2str(N))));
    model.lb(index1:15:index2) = worldMin;
    model.ub(index1:15:index2) = worldMax;
    model.vtype(index1:15:index2) = 'C';
    model.lb(index1+1:15:index2+1) = worldMin;
    model.ub(index1+1:15:index2+1) = worldMax;
    model.vtype(index1+1:15:index2+1) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'innovtx1')));
    index2 = find(strcmp(names, strcat(num2str(k),'innovtx',num2str(N))));
    model.lb(index1:15:index2) = worldMin;
    model.ub(index1:15:index2) = worldMax;
    model.vtype(index1:15:index2) = 'C';
    model.lb(index1+1:15:index2+1) = worldMin;
    model.ub(index1+1:15:index2+1) = worldMax;
    model.vtype(index1+1:15:index2+1) = 'C';
    
    index1 = find(strcmp(names, strcat(num2str(k),'covK1')));
    index2 = find(strcmp(names, strcat(num2str(k),'covK',num2str(N))));
    model.lb(index1:15:index2) = 0;
    model.ub(index1:15:index2) = inf;
    model.vtype(index1:15:index2) = 'C';
end

% Upper and lower bounds for binaries
index1 = find(strcmp(names, '1bl1'));
index2 = find(strcmp(names, strcat(num2str(NumTar),'b',num2str(N))));
model.lb(index1:index2) = 0;
model.ub(index1:index2) = 1;
model.vtype(index1:index2) = 'B';


% Upper and lower bounds for helpers
index1 = find(strcmp(names, '1sb1'));
index2 = find(strcmp(names, strcat(num2str(NumTar),'sb',num2str(N))));
model.lb(index1:6:index2) = 0;
model.ub(index1:6:index2) = 4;
model.vtype(index1:6:index2) = 'I';


for i=1:N
    index1 = find(strcmp(names, strcat('fov_xl',num2str(i))));
    index2 = find(strcmp(names, strcat('fov_yb',num2str(i))));
    model.lb(index1:index2) = worldMin;
    model.ub(index1:index2) = worldMax;
    model.vtype(index1:index2) = 'C';

end

% Upper and lower bounds for smoothing the forces
index1 = find(strcmp(names, 'fxdiff1'));
index2 = find(strcmp(names, strcat('fzdiff',num2str(N))));
model.lb(index1:index2) = -smoothingFactor;
model.ub(index1:index2) = smoothingFactor;
model.vtype(index1:index2) = 'C';

% fprintf('Time past for constructing boundaries: %.2f\n', toc)

%% Contraints table contruction
A=[A1;A2;A3;A4;A5;A11];%
B=[B1,B2,B3,B4,B5,B11];%
B_sense=[B1_sense, B2_sense, B3_sense, B4_sense, B5_sense, B11_sense];%

clearvars a1 a2 a3 a4 a5 A1 A2 A3 A4 A5 A11 B1 b2 B2 B3 B4 B5 B11 B1_sense b2_sense B2_sense B3_sense B4_sense B5_sense B11_sense

%% Objective Contruction
[Obj, Q] = setupQuadObjective(len, N, NumTar, names, 'cov');

model.Q = sparse(Q);
model.obj = (Obj);

model.A  = sparse(A);
model.rhs  = (B);
model.sense = B_sense';

[quadConNum,~,~] = size(aQc2);
for i=1:quadConNum
    model.quadcon(i).Qc = sparse(squeeze(aQc2(i,:,:)));
    model.quadcon(i).q = aqc2(i,:);
    model.quadcon(i).rhs = bc2(i,:);
    model.quadcon(i).sense = bc2_sense(i,:);
end

model.varnames = names;
% gurobi_write(model, 'mip1.lp');

%% Solve the problem
paramsGurobi.outputflag = 0;
paramsGurobi.Threads = 8;
paramsGurobi.NonConvex = 2;
% paramsGurobi.method=5;
tic
try
    result = gurobi(model, paramsGurobi);
    fprintf('Time past for calculating the solution: %.2f\n', toc)
    x=result.x;
catch E
    fprintf('Time past for attempting solution: %.2f\n', toc)
    disp(E)
    error = gurobi_iis(model);
    disp('gurobi error ')
end
objValue = result.objval;

% checkResults = [model.varnames',num2cell(x)];
% 
% checkBinaryResults = [model.varnames(find(strcmp(names, '1b1')):6:find(strcmp(names, strcat(num2str(NumTar),'b',num2str(N)))))', num2cell(x(find(strcmp(names, '1b1')):6:find(strcmp(names, strcat(num2str(NumTar),'b',num2str(N))))))];
% 
% checkHeightResults = [model.varnames(find(strcmp(names, 'z1')):9:find(strcmp(names, 'z10')))', num2cell(x(find(strcmp(names, 'z1')):9:find(strcmp(names, 'z10'))));...
%                       model.varnames(find(strcmp(names, 'alt_l1')):9:find(strcmp(names, 'alt_l10')))', num2cell(x(find(strcmp(names, 'alt_l1')):9:find(strcmp(names, 'alt_l10'))));...
%                       model.varnames(find(strcmp(names, 'alt_h1')):9:find(strcmp(names, 'alt_h10')))', num2cell(x(find(strcmp(names, 'alt_h1')):9:find(strcmp(names, 'alt_h10'))));...
%                       model.varnames(find(strcmp(names, 'alt_ha1')):9:find(strcmp(names, 'alt_ha10')))', num2cell(x(find(strcmp(names, 'alt_ha1')):9:find(strcmp(names, 'alt_ha10'))));...
%                       model.varnames(find(strcmp(names, 'alt_d1')):9:find(strcmp(names, 'alt_d10')))', num2cell(x(find(strcmp(names, 'alt_d1')):9:find(strcmp(names, 'alt_d10'))));...
%                       ];
%  
% checkProbabilityResults = [model.varnames(find(strcmp(names, 'Pd_l1')):find(strcmp(names, 'Pd_d10')))', num2cell(x(find(strcmp(names, 'Pd_l1')):find(strcmp(names, 'Pd_d10'))))];

index11=find(strcmp(names, 'x1'));
index12=find(strcmp(names, strcat('x',num2str(N))));
index21=find(strcmp(names, 'y1'));
index22=find(strcmp(names, strcat('y',num2str(N))));
index31=find(strcmp(names, 'z1'));
index32=find(strcmp(names, strcat('z',num2str(N))));
xx=x(index11:9:index12);
yy=x(index21:9:index22);
zz=x(index31:9:index32);

index11=find(strcmp(names, 'vx1'));
index12=find(strcmp(names, strcat('vx',num2str(N))));
index21=find(strcmp(names, 'vy1'));
index22=find(strcmp(names, strcat('vy',num2str(N))));
index31=find(strcmp(names, 'vz1'));
index32=find(strcmp(names, strcat('vz',num2str(N))));
ux=x(index11:9:index12);
uy=x(index21:9:index22);
uz=x(index31:9:index32);
ux1=ux(1);
uy1=uy(1);
uz1=uz(1);

index11=find(strcmp(names, 'fx1'));
index12=find(strcmp(names, strcat('fx',num2str(N))));
index21=find(strcmp(names, 'fy1'));
index22=find(strcmp(names, strcat('fy',num2str(N))));
index31=find(strcmp(names, 'fz1'));
index32=find(strcmp(names, strcat('fz',num2str(N))));
ax=x(index11:9:index12);
ay=x(index21:9:index22);
az=x(index31:9:index32);
ax1=ax(1);
ay1=ay(1);
az1=az(1);

xxt = zeros(NumTar,N+1);
yyt = zeros(NumTar,N+1);

for targetIndex=1:NumTar
    index11=find(strcmp(names, strcat(num2str(targetIndex),'xt0_0')));
    index21=find(strcmp(names, strcat(num2str(targetIndex),'yt0_0')));
    xxt(targetIndex,1)=x(index11);
    yyt(targetIndex,1)=x(index21);
    
    index11=find(strcmp(names, strcat(num2str(targetIndex),'xt1_1')));
    index21=find(strcmp(names, strcat(num2str(targetIndex),'yt1_1')));
    index12=find(strcmp(names, strcat(num2str(targetIndex),'xt',num2str(N),'_',num2str(N))));
    index22=find(strcmp(names, strcat(num2str(targetIndex),'yt',num2str(N),'_',num2str(N))));
    xxt(targetIndex,2:N+1)=x(index11:15:index12);
    yyt(targetIndex,2:N+1)=x(index21:15:index22);
end
% x;

fprintf('Time past for MPC: %.2f\n', toc(start))
end