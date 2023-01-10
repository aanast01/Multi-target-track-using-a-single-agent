function [realPlot] = realTimePlot(realPlot, w, x, n, x_coor, y_coor, keepTargetPath, pauseTime, action)

colors = distinguishable_colors(length(x)+1);

if strcmp(action,'init')
    
    realPlot.figure = figure('units','normalized','windowState','maximized', 'OuterPosition', [-0.879166666666667,0.037037037037037,0.883333333333333,0.95]);
    axis([00 500 0 250 0 200])
%     axis([-200 0 -200 0 0 200])

    view([0 90])
    hold on;
    grid on;
    realPlot.agent_position_plot=0;
    realPlot.fov_plot=0;
    realPlot.target_position_plot=0;
elseif strcmp(action,'plot')
    [numOfTargets,~] = size(x);
    
    try
        delete(realPlot.agent_position_plot_head)
        delete(realPlot.fov_plot)
        delete(realPlot.target_position_plot_head)
        if (~keepTargetPath)
            delete(realPlot.target_position_plot);
        end
    catch
    end
    
    realPlot.agent_position_plot_head = plot3(w(1,n),w(2,n),w(3,n), '*', 'color', colors(1,:), 'MarkerSize', 12, 'LineWidth', 2);
%     realPlot.agent_position_plot = plot3(w(1,n),w(2,n),w(3,n), '.', 'color', [0 0 1], 'MarkerSize', 8);
    try
        realPlot.fov_plot = plot(x_coor, y_coor, '-.', 'LineWidth', 2, 'Color', 'blue');
        plot3([w(1,n),w(1,n-1)],[w(2,n),w(2,n-1)],[w(3,n),w(3,n-1)],'.-', 'color', abs(colors(1,:)-[0.3 0.3 0.3]), 'LineWidth', 2);
    catch E
        disp(E)
    end
    for plIndex=1:numOfTargets
        if(keepTargetPath)
            realPlot.target_position_plot(plIndex) = plot3(x(plIndex, 1),x(plIndex,2),0, '.', 'color', colors(plIndex+1,:), 'MarkerSize', 8);
        end
        realPlot.target_position_plot_head(plIndex) = plot3(x(plIndex, 1),x(plIndex,2),0, '.', 'color', abs(colors(plIndex+1,:)-[0.3 0.3 0.3]), 'MarkerSize', 15);
    end
    pause(pauseTime);
end

end