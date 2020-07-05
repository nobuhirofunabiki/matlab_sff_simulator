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

% Network
% args_v_network.line_color = color_generator_.getNormalizedRGB('nezumi');
args_v_network.line_color = 'k';
args_v_network.line_style = '-';
args_v_network.line_width = 0.5;
network_comm_.visualizeConnectedNetwork3DCustomized(args_v_network);

axis equal
grid on
ax = gca;
xlabel('X [m]','FontSize',12)
ylabel('Y [m]','FontSize',12)
zlabel('Z [m]','FontSize',12)
ax.FontSize = 10;
hold off


figure
line_width = 2.0;
network_comm_.visualizeConnectionRate(line_width);
xlabel('Time [sec]','FontSize',12)
ylabel('Connection Rate [%]','FontSize',12)
grid on
hold off

figure
line_width = 2.0;
network_comm_.visualizeReachableNodes(line_width);
xlabel('Time [sec]','FontSize',12)
ylabel('Number of Reachable Nodes','FontSize',12)
grid on
hold off