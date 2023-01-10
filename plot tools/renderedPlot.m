%% test
close all;clc;clear all;
load('Multi-Castaway Tracking using MPC(time 3599)_4targets_3sources_24_10_22_17_05.mat');
load('CastawayDrift_4sources_1hr_06_10_22.mat')
N=n;
% Ns=3;
Nb=4;
tx = squeeze(ballsPos(1:Nb,1,:)); ty = squeeze(ballsPos(1:Nb,2,:)); tz = squeeze(ballsPos(1:Nb,3,:));
drone_Animation(false,false,w(1,:),w(2,:),w(3,:),FoVh,FoVv,tx,ty,tz,Ns,xSource,ySource,waveLength,decayrate,amplitude)

%% Results Figure
sub_fig_plot = figure('units','normalized','outerposition',[0.13,0.559259259259259,0.775,0.365740740740741], ...
                        'DefaultTextFontName', "Verdana", 'DefaultAxesFontName', "Verdana",'defaultAxesTickLabelInterpreter','latex',  'defaultLegendInterpreter','latex');
% % plotTilte = suptitle(title);
% % set(plotTilte, 'fontSize', 22);
% subplot(3,1,1)
% % inFoVhistory = sum(detectHistory(:,:,1));
% % detectedHistory = sum(detectHistory(:,:,2));
% % bar([inFoVhistory'./249,detectedHistory'./249]);
% plot(w(2,:), w(1,:))
% % xlim([-50 50])
% grid on;
% hold on;
% % legend({'$\textit{Target in FoV}$', '$\textit{Target Detected}$'},'Interpreter','latex', 'fontSize', 15, 'location', 'Southeast');%,'$x_{agent}-x_{target}$','$y_{agent}-y_{target}$'
% ylabel('Agent X', 'fontSize'.25);
% xlabel('Agent Y', 'fontSize'.25);
% 
% subplot(3,1,1)
% yyaxis right
% h = plot(smoothdata(w(3,1:end), 'movmedian',40));
h = plot(w(3,1:35));
set(gca,'FontSize',30)
grid on;
hold on;
h.LineWidth = 2;
% legend({'$z^a_{k}$'},'Interpreter','latex', 'fontSize', 15, 'location', 'Southeast')
ylabel('$Altitude [m]$', 'fontsize', 30, 'Interpreter','latex')
xlabel('$k$', 'fontsize', 30, 'Interpreter','latex')
xlim([0 35])
ylim([25 50])
xlim([0 n])

for targIndx=1:length(w)
   X=([-w(3,targIndx)*tand(FoVh/2), w(3,targIndx)*tand(FoVh/2), w(3,targIndx)*tand(FoVh/2), -w(3,targIndx)*tand(FoVh/2), -w(3,targIndx)*tand(FoVh/2)]);
   Y=([-w(3,targIndx)*tand(FoVv/2), -w(3,targIndx)*tand(FoVv/2), w(3,targIndx)*tand(FoVv/2), w(3,targIndx)*tand(FoVv/2), -w(3,targIndx)*tand(FoVv/2)]); 
   [~, volume(targIndx)] = boundary(X(:), Y(:), 1);
end
yyaxis left
targetsInView = detectHistory(:,1,1) + detectHistory(:,2,1);
[~, NumTar] = size(detectHistory(:,:,1));
g = plot(smoothdata(volume, 'movmedian',40));
grid on;
hold on;
g.LineWidth = 2;
% legend({'$Num of Targets$'},'Interpreter','latex', 'fontSize', 15, 'location', 'Southeast')
ylabel('FoV Volume (m^2) ', 'fontSize',25);
xlabel('Timestep', 'fontSize',25)
xlim([0 n])



subplot(3,1,2)
yyaxis right
[~, NumTar] = size(detectHistory(:,:,1));
targetsInView = zeros(size(detectHistory(:,1,1)));
for targIndx=1:NumTar
    targetsInView = targetsInView + detectHistory(:,targIndx,1);
end
g = plot(targetsInView);
yticks(1:1:NumTar)
grid on;
hold on;
g.LineWidth = 4;
% legend({'$Num of Targets$'},'Interpreter','latex', 'fontSize', 15, 'location', 'Southeast')
ylabel('Targets in FoV', 'fontSize',25);
xlabel('Timestep', 'fontSize',25)
xlim([0 n])

% subplot(3,1,3)
yyaxis left
g = plot(smoothdata(Fmax(1:end), 'movmedian',100));
grid on;
hold on;
g.LineWidth = 2;
% legend({'$Pd_{k}$','$z^a_{k}$'},'Interpreter','latex', 'fontSize', 15, 'location', 'Southeast')
ylabel('Probability (%)', 'fontSize',25);
xlabel('Timestep', 'fontSize',25)
xlim([0 n])

subplot(3,1,3)
op = plot(smoothdata(objValue, 'movmedian',100));
grid on;
hold on
op.LineWidth=2;
ylabel('Objective Value', 'fontsize', 25)
xlabel('Timestep', 'fontSize',25)
xlim([0 n])

%%
figure('DefaultTextFontName', "Verdana", 'DefaultAxesFontName', "Verdana",'defaultAxesTickLabelInterpreter','latex',...
            'defaultLegendInterpreter','latex','units','normalized','outerposition', [0.13,0.559259259259259,0.775,0.365740740740741]);
plot(1:35,uz(1:35), 'b', 'linewidth', 2);
ylabel('$u^a_z$', 'fontsize', 30, 'Interpreter','latex')
xlabel('$k$', 'fontSize',30, 'Interpreter','latex')

set(gca,'FontSize',30, 'yMinorGrid', 'on')
grid on;
xlim([0 35]);
hold on;
plot(1:N,uy(1:N), 'linewidth', 2.5);
plot(1:N,uz(1:N), 'linewidth', 2);
xlim([0 N])
grid on;

%% trace covariance
figure('DefaultTextFontName', "Verdana", 'DefaultAxesFontName', "Verdana",'defaultAxesTickLabelInterpreter','latex',  'defaultLegendInterpreter','latex')
% colors = distinguishable_colors(numOfTargets+1);
colors = [0,0,255;198,77,24;233,174,31;126,47,142]./255;
styles = ["-.", "-", "--", ":"];
plotLimStart=2;
plotLim=35;
width = [2,2,2,2];
for targIndx=1:numOfTargets
    plot(1, trace(squeeze(P(1,targIndx,:,:))), styles(targIndx), 'lineWidth',width(targIndx), 'color', colors(targIndx,:));
    for q=plotLimStart:plotLim
        plot([q, q-1], [trace(squeeze(P(q,targIndx,:,:))), trace(squeeze(P(q-1,targIndx,:,:)))], styles(targIndx), 'lineWidth',width(targIndx), 'color', colors(targIndx,:));
        hold on;
    end
end
ylabel('tr($P^{c_i}_{k \mid k}$)', 'fontsize', 30, 'Interpreter','latex')
xlabel('$k$', 'fontSize',30, 'Interpreter','latex')
set(gca, 'YScale', 'log', 'yMinorGrid', 'on', 'fontsize', 25)
ylim([0 10000])
xlim([1 plotLim])
xticks(0:5:plotLim)
grid on;

%% target and agent plot
figure('DefaultTextFontName', "Verdana", 'DefaultAxesFontName', "Verdana",'defaultAxesTickLabelInterpreter','latex',  'defaultLegendInterpreter','latex')
colors = distinguishable_colors(numOfTargets+1);
styles = ["-.", "-", "--", ":"];

for tarIndx = 1:Nb
    plot(tx(tarIndx,:), ty(tarIndx,:), styles(tarIndx), 'lineWidth', 2, 'color', colors(tarIndx,:));
    hold on;
end
plot(w(1,:),w(2,:), '--', 'lineWidth', 1, 'color', colors(end,:));
hold on;
ylabel('y (m)', 'fontsize', 30, 'Interpreter','latex')
xlabel('x (m)', 'fontSize',30, 'Interpreter','latex')
set(gca, 'yMinorGrid', 'on', 'xMinorGrid', 'on', 'fontsize', 25)
grid on;

