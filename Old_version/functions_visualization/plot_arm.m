function plot_arm(P)

    plot_data = reshape(cell2mat(P), [3, length(P)]);
    plot3(plot_data(1,:),plot_data(2,:),plot_data(3,:), '-o', 'LineWidth',5, 'MarkerSize',5)
    xlim([-25 25])
    ylim([-25 25])
    zlim([-15 30])
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end
