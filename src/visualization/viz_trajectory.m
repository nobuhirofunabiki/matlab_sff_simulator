figure
% True states
args_true_trajectory.line_color = color_list.true;
args_true_trajectory.line_style = '-';
args_true_trajectory.line_width = 0.3;
args_target_true.marker_symbol = 'o';
args_target_true.marker_size = 30;
args_target_true.marker_edge_color = [0, 0, 0];
args_target_true.marker_face_color = color_list.true;
for iAgents = 1:num_agents
    args_target_true.txt = num2str(iAgents);
    v_agents_true_(iAgents).visualizeAgentTrajectoryCustomized(args_true_trajectory);
    plot_true = v_agents_true_(iAgents).visualizeAgentPositionCustomized(num_steps, args_target_true);
end
% Reference agent
args_ref_trajectory.line_color = color_list.ref;
args_ref_trajectory.line_style = '-';
args_ref_trajectory.line_width = 0.3;
args_target_ref.marker_symbol = '^';
args_target_ref.marker_size = 30;
args_target_ref.marker_edge_color = [0, 0, 0];
args_target_ref.marker_face_color = color_list.ref;
args_target_ref.txt = 'ref(0)';
v_agent_ref_.visualizeAgentTrajectoryCustomized(args_ref_trajectory);
plot_ref =v_agent_ref_.visualizeAgentPositionCustomized(num_steps, args_target_ref);
% Network
args_v_network.line_color = color_generator_.getNormalizedRGB('nezumi');
args_v_network.line_style = '-';
args_v_network.line_width = 0.05;
network_.visualizeConnectedNetwork3DCustomized(args_v_network);
axis equal
grid on
ax = gca;
xlabel('X [m]','FontSize',12)
ylabel('Y [m]','FontSize',12)
ylabel('Z [m]','FontSize',12)
ax.FontSize = 10;
hold off