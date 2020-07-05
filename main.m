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
% Estimated states: Centralized Extended Information Filter
if (b_use_ceif)
    for iAgents = 1:num_agents
        args_est_ceif.id = iAgents;
        args_est_ceif.position = initial_states_estimate(1:num_dims, iAgents);
        args_est_ceif.velocity = initial_states_estimate(1+num_dims:2*num_dims, iAgents);
        agents_ceif_(iAgents) = AgentHandler(args_est_ceif);
    end
end
% Estimated states: Decentralized Extended Information Filter
if (b_use_deif)
    args_est_deif.num_agents = num_agents;
    args_est_deif.num_dimensions = num_dims;
    for iAgents = 1:num_agents
        agents_deif_(iAgents) = MultiAgentHandler(args_est_deif, initial_states_estimate);
    end
end


% Dynamics model
args_dynamics.discrete_time = delta_time_rk;
args_dynamics.disturbance   = disturbance_sigma;
args_dynamics.delta_time_rk = delta_time_rk;
args_dynamics.angular_rate  = angular_rate;
dynamics_ = HillDynamics3D(args_dynamics);
args_dynamics.angular_rate  = angular_rate*0.999;
% args_dynamics.angular_rate  = angular_rate;
dynamics_2_ = HillDynamics3D(args_dynamics);

% Network manager
node_positions = zeros(num_dims, num_agents);
for iAgents = 1:num_agents
    node_positions(:,iAgents) = agents_true_(iAgents).getPosition();
end
args_network.num_steps              = num_steps;
args_network.num_nodes_nonref       = num_agents;
args_network.node_positions_nonref  = node_positions;
args_network.node_position_ref      = agent_ref_.getPosition();
% Communication network
args_network.range_threshold = range_threshold.comm;
network_comm_ = NetworkManagerWithReferenceNode(args_network);
% Range measurement network
args_network.range_threshold = range_threshold.range;
network_range_ = NetworkManagerWithReferenceNode(args_network);
% Angle measurement network
args_network.range_threshold = range_threshold.angle;
network_angle_ = NetworkManagerWithReferenceNode(args_network);

% Communication time
iTimeTableRows = 1;
if (b_use_mesh_network_simulator_output)
    estimation_period = sum(comm_time_table(iTimeTableRows,2:num_agents));
end

% Sensor models
% Range measurements
args_range_sensor.noise_sigma       = sensor_params.range.noise_sigma;
args_range_sensor.num_measures      = num_edges;
args_range_sensor.num_variables     = num_vars;
args_range_sensor.num_agents        = num_agents;
args_range_sensor.num_dimensions    = num_dims;
range_sensor_ = RangeMeasurementMultiAgentWithReference(args_range_sensor);
% Angle measurements
args_angle_sensor.num_dimensions    = num_dims;
args_angle_sensor.noise_sigma       = transpose(cell2mat(sensor_params.angle.noise_sigma));
args_angle_sensor.num_agents        = num_agents;
args_angle_sensor.num_measurements  = 2*(num_agents*(num_agents-1) + num_agents);
args_angle_sensor.num_variables     = num_vars;
angle_sensor_ = AngleMeasurementMultiAgentWithReference3D(args_angle_sensor);

% Estimators
estimate_timer = 0.0;
discrete_system_matrix = dynamics_.getDiscreteSystemMatrixSpecificTimestep(0);
init_state_vector = zeros(num_vars, 1);
for iAgents = 1:num_agents
    init_state_vector(2*num_dims*(iAgents-1)+1:2*num_dims*iAgents) ...
        = initial_states_estimate(1:2*num_dims, iAgents);
end
% Estimators: Centralized Extended Information Filter
if (b_use_ceif)
    args_ceif.num_variables             = num_vars;
    args_ceif.num_dimensions            = num_dims;
    args_ceif.num_agents                = num_agents;
    args_ceif.state_vector              = init_state_vector;
    args_ceif.process_noise_covmat      = process_noise_covmat;
    args_ceif.sigma_position            = initial_covariance.sigma_position;
    args_ceif.sigma_velocity            = initial_covariance.sigma_velocity;
    args_ceif.discrete_system_matrix    = discrete_system_matrix;
    args_ceif.range_sensor              = args_range_sensor;
    args_ceif.angle_sensor              = args_angle_sensor;
    estimator_ceif_ = EIF_3D_FormationEstimationByRangeAngleWithReference(args_ceif);
end
% Estimators: Decentralized Extended Information Filter
if (b_use_deif)
    args_deif.num_agents                = num_agents;
    args_deif.number_variables          = num_vars;
    args_deif.num_dimensions            = num_dims;
    args_deif.process_noise_covmat      = process_noise_covmat;
    args_deif.discrete_system_matrix    = discrete_system_matrix;
    args_deif.sigma_position            = initial_covariance.sigma_position;
    args_deif.sigma_velocity            = initial_covariance.sigma_velocity;
    args_deif.state_vector              = init_state_vector;
    args_range_sensor.noise_sigma       = args_range_sensor.noise_sigma;
    args_angle_sensor.noise_sigma       = args_angle_sensor.noise_sigma;
    args_deif.range_sensor              = args_range_sensor;
    args_deif.angle_sensor              = args_angle_sensor;
    args_deif.wait_steps                = deif_wait_steps;
    for iAgents = 1:num_agents
        args_deif.agent_id = iAgents;
        estimator_deif_(iAgents) = DEIF_3D_FormationEstimationByRangeAngleWithReference(args_deif);
    end
end

% Visualizers
args_visualizer.memory_size = num_steps;
% Visualization: True states
for iAgents = 1:num_agents
    v_agents_true_(iAgents) = AgentVisualizer3D(args_visualizer);
end
% Visualization: Reference agent
v_agent_ref_ = AgentVisualizer3D(args_visualizer);
% Visualization: Centralized Extended Information Filter
if (b_use_ceif)
    for iAgents = 1:num_agents
        v_ceif_(iAgents) =  AgentVisualizer3D(args_visualizer);
    end
end
% Visualization: Decentralized Extended Information Filter
if (b_use_ceif)
    args_multi_visualizer.num_agents = num_agents;
    args_multi_visualizer.memory_size = num_steps;
    for iAgents = 1:num_agents
        v_deif_(iAgents) = MultiAgentVisualizer3D(args_multi_visualizer);
    end
end

% Estimation Performance Analysis
args_analysis.num_agents       = num_agents;
args_analysis.memory_size      = num_steps;
args_analysis.num_dimensions   = num_dims;
args_analysis.chi2             = chi2inv(0.99, (2*num_dims)*num_agents);
% Analysis: Centralized Extended Information Filter
if (b_use_ceif)
    analysis_ceif_ = MultiEstimationAnalysisVisualizer(args_analysis);
end
% Analysis: Decentralized Extended Information Filter
if (b_use_deif)
    for iAgents = 1:num_agents
        analysis_deif_(iAgents) = MultiEstimationAnalysisVisualizer(args_analysis);
    end
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
    if (b_use_ceif)
        for iAgents = 1:num_agents
            position = agents_ceif_(iAgents).getPosition();
            v_ceif_(iAgents).setPosition(position, iSteps);
        end
    end
    % Visualization: Decentralized Extended Information Filter
    if (b_use_deif)
        for iAgents = 1:num_agents
            for jAgents = 1:num_agents
                position = agents_deif_(iAgents).getAgentPosition(jAgents);
                v_deif_(iAgents).setPosition(jAgents, position, iSteps);
            end
        end
    end

    % Estimation Performance Analysis
    % Analysis: Centralized Extended Information Filter
    if (b_use_ceif)
        for iAgents = 1:num_agents
            analysis_ceif_.setEstimateErrorPositionScalar(iAgents, ...
                agents_true_(iAgents).getPosition(), ...
                agents_ceif_(iAgents).getPosition(), iSteps);
            analysis_ceif_.setEstimateErrorVelocityScalar(iAgents, ...
                agents_true_(iAgents).getVelocity(), ...
                agents_ceif_(iAgents).getVelocity(), iSteps);
        end
    end
    % Analysis: Decentralized Extended Information Filter
    if (b_use_deif)
        for iAgents = 1:num_agents
            for jAgents = 1:num_agents
                analysis_deif_(iAgents).setEstimateErrorPositionScalar(jAgents, ...
                    agents_true_(jAgents).getPosition(), ...
                    agents_deif_(iAgents).getAgentPosition(jAgents), iSteps);
                analysis_deif_(iAgents).setEstimateErrorVelocityScalar(jAgents, ...
                    agents_true_(jAgents).getVelocity(), ...
                    agents_deif_(iAgents).getAgentVelocity(jAgents), iSteps);
            end
        end
    end

    % Update Network
    node_positions = zeros(num_dims, num_agents);
    for iAgents = 1:num_agents
        node_positions(:,iAgents) = agents_true_(iAgents).getPosition();
    end
    position_ref = agent_ref_.getPosition();
    network_comm_.setNodePositions(node_positions, position_ref);
    network_range_.setNodePositions(node_positions, position_ref);
    network_angle_.setNodePositions(node_positions, position_ref);
    network_comm_.updateNetwork(iSteps, time_stamp);
    network_range_.updateNetwork(iSteps, time_stamp);
    network_angle_.updateNetwork(iSteps, time_stamp);

    % Communication time table
    % Update the estimation period based on the result of mesh_network_simulator
    if (b_use_mesh_network_simulator_output)
        if time_stamp > comm_time_table(iTimeTableRows,1)
            iTimeTableRows = iTimeTableRows + 1;
            estimation_period = sum(comm_time_table(iTimeTableRows,2:num_agents));
        end
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
        % Angle measurements
        angle_sensor_.computeMeasurementVector(true_positions, position_ref, true);
        measurements.angles = angle_sensor_.getMeasurements();

        % Network
        args_adjacent_matrix.range = network_range_.getAdjacentMatrix();
        args_adjacent_matrix.angle = network_angle_.getAdjacentMatrix();

        % Sequential Estimation Phase
        % TODO: Precision assessment of non-constant discrete system matrix is required
        discrete_system_matrix = dynamics_.getDiscreteSystemMatrixSpecificTimestep(estimate_timer);

        % Centralized Extended Information Filter
        if (b_use_ceif)
            estimator_ceif_.executeInformationFilter(measurements, discrete_system_matrix, args_adjacent_matrix , position_ref);
            state_vector_ceif = estimator_ceif_.getStateVector();
            for iAgents = 1:num_agents
                posvel = state_vector_ceif((2*num_dims)*(iAgents-1)+1:(2*num_dims)*iAgents, 1);
                agents_ceif_(iAgents).setPositionVelocity(posvel);
            end
        end

        % Decentralized Extended Information Filter
        if (b_use_deif)
            Aij = network_comm_.getStochasticAdjacencyMatrix();
            for iAgents = 1:num_agents
                outsource_info_vector = zeros(size(estimator_deif_(iAgents).getPreviousJointInformationVector()));
                outsource_info_matrix = zeros(size(estimator_deif_(iAgents).getPreviousJointInformationMatrix()));
                for jAgents = 1:num_agents
                    outsource_info_vector = outsource_info_vector ...
                        + Aij(iAgents, jAgents)*estimator_deif_(jAgents).getPreviousJointInformationVector();
                    outsource_info_matrix = outsource_info_matrix ...
                        + Aij(iAgents, jAgents)*estimator_deif_(jAgents).getPreviousJointInformationMatrix();
                end
                % local_adjacent_matrix = network_.getLocalAdjacentMatrix(iAgents);
                args_adjacent_matrix.range = network_range_.getLocalAdjacentMatrix(iAgents);
                args_adjacent_matrix.angle = network_angle_.getLocalAdjacentMatrix(iAgents);
                estimator_deif_(iAgents).executeFiltering(measurements, discrete_system_matrix, args_adjacent_matrix, ...
                    outsource_info_vector, outsource_info_matrix, position_ref);
                agents_deif_(iAgents).setStateVector(estimator_deif_(iAgents).getStateVector());
            end
            for iAgents = 1:num_agents
                estimator_deif_(iAgents).updateEstimatorStatus();
            end
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
        output = dynamics_2_.propagatePositionRungeKutta(...
            agents_true_(iAgents).getPosition(), agents_true_(iAgents).getVelocity(), control_input);
        agents_true_(iAgents).setPositionVelocity(output);
        % agents_true_(iAgents).setPosition(true_states(iAgents).position(:,iSteps+1));
        % agents_true_(iAgents).setVelocity(true_states(iAgents).velocity(:,iSteps+1));
    end
    % Propagation: Reference agent
    output = dynamics_2_.propagatePositionRungeKutta(...
        agent_ref_.getPosition(), agent_ref_.getVelocity(), control_input);
    agent_ref_.setPositionVelocity(output);
    % Propagation: Centralized Extended Information Filter
    if (b_use_ceif)
        for iAgents = 1:num_agents
            output = dynamics_.propagatePositionRungeKutta(...
                agents_ceif_(iAgents).getPosition(), agents_ceif_(iAgents).getVelocity(), control_input);
            agents_ceif_(iAgents).setPositionVelocity(output);
        end
    end
    % Propagation: Decentralized Extended Information Filter
    if (b_use_deif)
        for iAgents = 1:num_agents
            for jAgents = 1:num_agents
                output = dynamics_.propagatePositionRungeKutta(...
                    agents_deif_(iAgents).getAgentPosition(jAgents), ...
                    agents_deif_(iAgents).getAgentVelocity(jAgents), ...
                    control_input);
                agents_deif_(iAgents).setAgentPositionVelocity(jAgents, output);
            end
        end
    end

end


%% Visualization
run('src/visualization/viz_main.m');