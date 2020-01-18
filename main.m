%===================================================================
% Title: Spacecraft Formation Flight Simulator
% Feature: Integration with mesh_network_simulator
% Author1: Nobuhiro Funabiki (funabiki@space.t.u-tokyo.ac.jp)
%===================================================================

clc
clear
close all;

restoredefaultpath
addpath(genpath('../matlab_agent_handler'));
addpath(genpath('../dynamics_model'));
addpath(genpath('../matlab_visualizer'));
addpath(genpath('../YAMLMatlab_0.4.3'));

run('src/load_parameters.m');

%% Instanciation

% General parameters
time_stamp      = 0.0;
time_list       = zeros(1,num_steps);
display_timer   = 0.0;

% True states
for iAgents = 1:num_agents
    args_agent_true.id = iAgents;
    args_agent_true.position = initial_states(1:num_dims, iAgents);
    args_agent_true.velocity = initial_states(1+num_dims:2*num_dims, iAgents);
    agents_true_(iAgents) = AgentHandler(args_agent_true);
end

% Reference agent
args_agent_ref.id = 0;
for iDims = 1:num_dims
    args_agent_ref.position(iDims, 1) = init_state_ref.position(iDims);
    args_agent_ref.velocity(iDims, 1) = init_state_ref.velocity(iDims);
end
agent_ref_ = AgentHandler(args_agent_ref);

% Dynamics model
args_dynamics.discrete_time = delta_time_rk;
args_dynamics.disturbance   = disturbance_sigma;
args_dynamics.delta_time_rk = delta_time_rk;
args_dynamics.angular_rate  = angular_rate;
dynamics_ = HillDynamics3D(args_dynamics);

% Visualizers
args_visualizer.memory_size = num_steps;
% Visualization: True states
for iAgents = 1:num_agents
    v_agents_true_(iAgents) = AgentVisualizer3D(args_visualizer);
end
% Visualization: Reference agent
v_agent_ref_ = AgentVisualizer3D(args_visualizer);

% Control input
control_input = zeros(num_dims, 1);

%% Simulation

for iSteps = 1:num_steps

    % Visualization
    % Visualization: True states
    for iAgents = 1:num_agents
        position_true = agents_true_(iAgents).getPosition();
        v_agents_true_(iAgents).setPosition(position_true, iSteps);
    end

    time_stamp = time_stamp + delta_time_rk;
    time_list(1,iSteps) = time_stamp;
    display_timer = display_timer + delta_time_rk;
    if (display_timer >= display_period)
        disp(time_stamp);
        display_timer = 0;
    end

    % Propagation
    % Propagate the true state of agent by Runge-Kutta
    for iAgents = 1:num_agents
        output_true = dynamics_.propagatePositionRungeKutta(...
            agents_true_(iAgents).getPosition(), agents_true_(iAgents).getVelocity(), control_input);
        agents_true_(iAgents).setPositionVelocity(output_true);       
    end

end