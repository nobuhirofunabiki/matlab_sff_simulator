%% Load parameters from the configuration file

params = ReadYaml('../config/parameters.yaml');

% Load files
initial_states = readmatrix(['../config/initial_states/', params.file_name.initial_states]);
comm_time_table = readmatrix(['../config/mesh_network/', params.file_name.mesh_network]);
data_true_states = readtable(['../config/true_states/', params.file_name.true_states]);

% General parameters
num_agents = size(initial_states,2);

% Time parameters
total_simulation_time   = params.time.total_simulation;
delta_time_rk           = params.time.delta_runge_kutta;
display_period          = params.time.display_period;
num_steps               = round(total_simulation_time/delta_time_rk);
b_use_mesh_network_simulator_output = params.time.estimation.b_use_mesh_network_simulator_output;
estimation_period.ceif = params.time.estimation.period.ceif;
estimation_period.deif = params.time.estimation.period.deif;

% Dynamics parameters
num_dims = params.dynamics.dimension;
disturbance_sigma = transpose(cell2mat(params.dynamics.disturbance_sigma));
angular_rate = params.dynamics.hill.angular_rate;

% Communication network
range_threshold.range = params.network.range_threshold.range_measurement;
range_threshold.angle = params.network.range_threshold.angle_measurement;
range_threshold.comm = params.network.range_threshold.communication;

% Sensors
sensor_params = params.sensor;

% True states
b_read_true_state_from_csv = params.true_states.b_read_from_csv;

% Rerence agent
init_state_ref.position = transpose(cell2mat(params.reference_agent.initial_state.position));
init_state_ref.velocity = transpose(cell2mat(params.reference_agent.initial_state.velocity));

% Initial estimates
initial_estimate_sigma.position = params.initial_estimate.sigma_position;
initial_estimate_sigma.velocity = params.initial_estimate.sigma_velocity;

% Estimator parameters
initial_covariance = params.estimators.initial_covariance;
b_use_ceif = params.estimators.b_use.centralized_extended_information_filter;
b_use_deif = params.estimators.b_use.decentralized_extended_information_filter;
ratio_noise_model.ceif.range = params.estimators.ceif.ratio_noise_model.range;
ratio_noise_model.ceif.angle = params.estimators.ceif.ratio_noise_model.angle;
ratio_noise_model.deif.range = params.estimators.deif.ratio_noise_model.range;
ratio_noise_model.deif.angle = params.estimators.deif.ratio_noise_model.angle;
deif_wait_steps = params.estimators.deif.wait_steps;
process_noise_covmat.ceif = zeros(2*num_dims*num_agents, 2*num_dims*num_agents);
process_noise_covmat.deif = zeros(2*num_dims*num_agents, 2*num_dims*num_agents);
for iAgents = 1:num_agents
    for iDims = 1:num_dims
        % For centralized extended information filter
        process_noise_covmat.ceif(2*num_dims*(iAgents-1)+iDims, 2*num_dims*(iAgents-1)+iDims) ...
            = params.estimators.process_noise.sigma_position*(estimation_period.ceif);
        process_noise_covmat.ceif(2*num_dims*(iAgents-1)+num_dims+iDims, 2*num_dims*(iAgents-1)+num_dims+iDims) ...
            = params.estimators.process_noise.sigma_velocity*(estimation_period.ceif);
        % For decentralized extended information filter
        process_noise_covmat.deif(2*num_dims*(iAgents-1)+iDims, 2*num_dims*(iAgents-1)+iDims) ...
            = params.estimators.process_noise.sigma_position*(estimation_period.deif);
        process_noise_covmat.deif(2*num_dims*(iAgents-1)+num_dims+iDims, 2*num_dims*(iAgents-1)+num_dims+iDims) ...
            = params.estimators.process_noise.sigma_velocity*(estimation_period.deif);
    end
end