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
addpath(genpath('../sensor_model'));
addpath(genpath('../state_estimation'));
addpath(genpath('../matlab_network_manager'));
addpath(genpath('../matlab_visualizer'));
addpath(genpath('../common_utilities'));
addpath(genpath('../matlab_multiagent_utility'));
addpath(genpath('../YAMLMatlab_0.4.3'));

run('src/load_parameters.m');


%% Instanciation

% General parameters
time_stamp      = 0.0;
time_list       = zeros(1,num_steps);
display_timer   = 0.0;
num_vars        = 2*num_dims*num_agents;
num_edges       = nchoosek(num_agents, 2) + num_agents;

% True states
for iAgents = 1:num_agents
    args_agent_true.id = iAgents;
    args_agent_true.position = initial_states(1:num_dims, iAgents);
    args_agent_true.velocity = initial_states(1+num_dims:2*num_dims, iAgents);
    agents_true_(iAgents) = AgentHandler(args_agent_true);
end

% Reference agent
args_agent_ref.id = 0;
args_agent_ref.position = init_state_ref.position;
args_agent_ref.velocity = init_state_ref.velocity;
agent_ref_ = AgentHandler(args_agent_ref);

% Estimated states
initial_states_estimate = makeInitialEstimate(...
    num_dims, initial_states, initial_estimate_sigma.position, initial_estimate_sigma.velocity, 0.0, 0.0);
% Estimated states (Centralized Extended Information Filter)
for iAgents = 1:num_agents
    args_est_ceif.id = iAgents;
    args_est_ceif.position = initial_states_estimate(1:num_dims, iAgents);
    args_est_ceif.velocity = initial_states_estimate(1+num_dims:2*num_dims, iAgents);
    agents_ceif_(iAgents) = AgentHandler(args_est_ceif);
end

% Dynamics model
args_dynamics.discrete_time = delta_time_rk;
args_dynamics.disturbance   = disturbance_sigma;
args_dynamics.delta_time_rk = delta_time_rk;
args_dynamics.angular_rate  = angular_rate;
dynamics_ = HillDynamics3D(args_dynamics);

% Network manager
node_positions = zeros(num_dims, num_agents);
for iAgents = 1:num_agents
    node_positions(:,iAgents) = agents_true_(iAgents).getPosition();
end
args_network.range_threshold        = range_threshold;
args_network.num_nodes_nonref       = num_agents;
args_network.node_positions_nonref  = node_positions;
args_network.node_position_ref      = agent_ref_.getPosition();
network_ = NetworkManagerWithReferenceNode(args_network);
network_.updateAdjacentMatrixByRange();

% Communication time
iTimeTableRows = 1;
estimation_period = sum(comm_time_table(iTimeTableRows,2:num_agents));

% Sensor models
args_range_sensor.noise_sigma       = sensor_params.range.noise_sigma;
args_range_sensor.num_measures      = num_edges;
args_range_sensor.num_variables     = num_vars;
args_range_sensor.num_agents        = num_agents;
args_range_sensor.num_dimensions    = num_dims;
range_sensor_ = RangeMeasurementMultiAgentWithReference(args_range_sensor);

% Estimators
estimate_timer = 0.0;
discrete_system_matrix = dynamics_.getDiscreteSystemMatrixSpecificTimestep(0);
init_state_vector = zeros(num_vars, 1);
for iAgents = 1:num_agents
    init_state_vector(2*num_dims*(iAgents-1)+1:2*num_dims*iAgents) ...
        = initial_states_estimate(1:2*num_dims, iAgents);
end
% Estimators: Centralized Extended Information Filter
args_ceif.num_variables             = num_vars;
args_ceif.num_dimensions            = num_dims;
args_ceif.num_agents                = num_agents;
args_ceif.state_vector              = init_state_vector;
args_ceif.process_noise_covmat      = zeros(num_vars, num_vars);
args_ceif.sigma_position            = initial_covariance.sigma_position;
args_ceif.sigma_velocity            = initial_covariance.sigma_velocity;
args_ceif.discrete_system_matrix    = discrete_system_matrix;
args_ceif.range_sensor              = args_range_sensor;
estimator_ceif_ = EIF_3D_FormationEstimationByRangeAngleWithReference(args_ceif);

% Visualizers
args_visualizer.memory_size = num_steps;
% Visualization: True states
for iAgents = 1:num_agents
    v_agents_true_(iAgents) = AgentVisualizer3D(args_visualizer);
end
% Visualization: Reference agent
v_agent_ref_ = AgentVisualizer3D(args_visualizer);
% Visualization: Centralized Extended Information Filter
for iAgents = 1:num_agents
    v_ceif_(iAgents) =  AgentVisualizer3D(args_visualizer);
end

% Control input
control_input = zeros(num_dims, 1);


%% Simulation
for iSteps = 1:num_steps

    % Visualization
    % Visualization: True states
    for iAgents = 1:num_agents
        position = agents_true_(iAgents).getPosition();
        v_agents_true_(iAgents).setPosition(position, iSteps);
    end
    % Visualization: Reference agent
    position = agent_ref_.getPosition();
    v_agent_ref_.setPosition(position, iSteps);
    % Visualization: Centralized Extended Information Filter
    for iAgents = 1:num_agents
        position = agents_ceif_(iAgents).getPosition();
        v_ceif_(iAgents).setPosition(position, iSteps);
    end

    % Update Network
    node_positions = zeros(num_dims, num_agents);
    for iAgents = 1:num_agents
        node_positions(:,iAgents) = agents_true_(iAgents).getPosition();
    end
    position_ref = agent_ref_.getPosition();
    network_.setNodePositions(node_positions, position_ref);
    network_.updateAdjacentMatrixByRange();

    % Communication time table
    % Update the estimation period based on the result of mesh_network_simulator
    if time_stamp > comm_time_table(iTimeTableRows,1)
        iTimeTableRows = iTimeTableRows + 1;
        estimation_period = sum(comm_time_table(iTimeTableRows,2:num_agents));
    end

    % Estimation Sequence
    if (estimate_timer >= estimation_period || iSteps == 1)
        
        % Sensor measurments
        true_positions = zeros(num_dims*num_agents, 1);
        for iAgents = 1:num_agents
            position = agents_true_(iAgents).getPosition();
            true_positions(num_dims*(iAgents-1)+1:num_dims*iAgents) = position;
        end
        position_ref = agent_ref_.getPosition();
        % Range measurements
        range_sensor_.computeMeasurementVector(true_positions, position_ref, true);
        measurements.ranges = range_sensor_.getMeasurements();

        % Network
        arg_adjacent_matrix.range = network_.getAdjacentMatrix();

        % Sequential Estimation Phase
        discrete_system_matrix = dynamics_.getDiscreteSystemMatrixSpecificTimestep(estimate_timer);
        % Centralized Extended Information Filter
        estimator_ceif_.executeInformationFilter(measurements, discrete_system_matrix, arg_adjacent_matrix , position_ref);
        state_vector_ceif = estimator_ceif_.getStateVector();
        for iAgents = 1:num_agents
            posvel = state_vector_ceif((2*num_dims)*(iAgents-1)+1:(2*num_dims)*iAgents, 1);
            agents_ceif_(iAgents).setPositionVelocity(posvel);
        end

        estimate_timer = 0.0;
    end
    estimate_timer = estimate_timer + delta_time_rk;

    % Timer
    time_list(1,iSteps) = time_stamp;
    time_stamp = time_stamp + delta_time_rk;
    display_timer = display_timer + delta_time_rk;
    if (display_timer >= display_period)
        disp(time_stamp);
        display_timer = 0;
    end

    % Propagation
    % Propagate the true state of agent by Runge-Kutta
    for iAgents = 1:num_agents
        output = dynamics_.propagatePositionRungeKutta(...
            agents_true_(iAgents).getPosition(), agents_true_(iAgents).getVelocity(), control_input);
        agents_true_(iAgents).setPositionVelocity(output);       
    end
    % Propagation: Reference agent
    output = dynamics_.propagatePositionRungeKutta(...
        agent_ref_.getPosition(), agent_ref_.getVelocity(), control_input);
    agent_ref_.setPositionVelocity(output);
    % Propagation: Centralized Extended Information Filter
    for iAgents = 1:num_agents
        output = dynamics_.propagatePositionRungeKutta(...
            agents_ceif_(iAgents).getPosition(), agents_ceif_(iAgents).getVelocity(), control_input);
        agents_ceif_(iAgents).setPositionVelocity(output);
    end

end


%% Visualization
run('src/visualization/viz_main.m');