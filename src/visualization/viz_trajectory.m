figure

% True states
args_true_trajectory.line_color = color_list.true;
args_true_trajectory.line_style = '-';
args_true_trajectory.line_width = 0.3;
args_true_target.marker_symbol = 'o';
args_true_target.marker_size = 30;
args_true_target.marker_edge_color = [0, 0, 0];
args_true_target.marker_face_color = color_list.true;
for iAgents = 1:num_agents
    args_true_target.txt = num2str(iAgents);
    v_agents_true_(iAgents).visualizeAgentTrajectoryCustomized(args_true_trajectory);
    plot_true = v_agents_true_(iAgents).visualizeAgentPositionCustomized(num_steps, args_true_target);
end

% Reference agent
args_ref_trajectory.line_color = color_list.ref;
args_ref_trajectory.line_style = '-';
args_ref_trajectory.line_width = 0.3;
args_ref_target.marker_symbol = '^';
args_ref_target.marker_size = 30;
args_ref_target.marker_edge_color = [0, 0, 0];
args_ref_target.marker_face_color = color_list.ref;
args_ref_target.txt = 'ref(0)';
v_agent_ref_.visualizeAgentTrajectoryCustomized(args_ref_trajectory);
plot_ref =v_agent_ref_.visualizeAgentPositionCustomized(num_steps, args_ref_target);

% Centralized Extended Information Filter
if (b_use_ceif)
    args_ceif_trajectory.line_color = color_list.ceif;
    args_ceif_trajectory.line_style = '-';
    args_ceif_trajectory.line_width = 0.3;
    args_ceif_target.marker_symbol = 'o';
    args_ceif_target.marker_size = 30;
    args_ceif_target.marker_edge_color = [0, 0, 0];
    args_ceif_target.marker_face_color = color_list.ceif;
    for iAgents = 1:num_agents
        args_ceif_target.txt = num2str(iAgents);
        v_ceif_(iAgents).visualizeAgentTrajectoryCustomized(args_ceif_trajectory);
        plot_ceif = v_ceif_(iAgents).visualizeAgentPositionCustomized(num_steps, args_ceif_target);
    end
end

% Decentralized Extended Information Filter
if (b_use_deif)
    args_deif_trajectory.line_color = color_list.deif;
    args_deif_trajectory.line_style = '-';
    args_deif_trajectory.line_width = 0.3;
    args_deif_target.marker_symbol = 'o';
    args_deif_target.marker_size = 15;
    args_deif_target.marker_edge_color = [0, 0, 0];
    args_deif_target.marker_face_color = color_list.deif;
    for iAgents = 1:num_agents
        for jAgents = 1:num_agents
            if iAgents == 1
                args_deif_target.txt = num2str(jAgents);
            else
                args_deif_target.txt = '';
            end
            v_deif_(iAgents).visualizeAgentTrajectoryCustomized(jAgents, args_deif_trajectory);
            plot_deif = v_deif_(iAgents).visualizeAgentPositionCustomized(jAgents, num_steps, args_deif_target);
        end
    end
end

axis equal
grid on
ax = gca;

plots = [plot_true, plot_ref];
plot_names = {'True Position', 'Reference Agent'};
if (b_use_ceif)
    plots = horzcat(plots, [plot_ceif]);
    plot_names = horzcat(plot_names, {'CEIF'});
end
if (b_use_deif)
    plots = horzcat(plots, [plot_deif]);
    plot_names = horzcat(plot_names, {'DEIF'});
end
legend(plots, plot_names, 'Location', 'northwest', 'FontSize', 8);

xlabel('X [m]','FontSize',12)
ylabel('Y [m]','FontSize',12)
zlabel('Z [m]','FontSize',12)
ax.FontSize = 10;
hold off