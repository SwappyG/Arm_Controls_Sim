function plot_arm(P)

    plot_data = reshape(cell2mat(P), [3, length(P)]);
    plot3(plot_data(1,:),plot_data(2,:),plot_data(3,:), '-o', 'LineWidth',5, 'MarkerSize',5)
    xlim([-15 15])
    ylim([-15 15])
    zlim([-15 15])
    
end
