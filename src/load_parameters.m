%% Load parameters from the configuration file

params = ReadYaml('../config/parameters.yaml');

% Load files
initial_states = readmatrix(['../config/initial_states/', params.file_name.initial_states]);
comm_time_table = readmatrix(['../config/mesh_network/', params.file_name.mesh_network]);

% General parameters
num_agents = size(initial_states,2);

% Time parameters
total_simulation_time   = params.time.total_simulation;
delta_time_rk           = params.time.delta_runge_kutta;
display_period          = params.time.display_period;
num_steps               = round(total_simulation_time/delta_time_rk);

% Dynamics parameters
num_dims = params.dynamics.dimension;
disturbance_sigma = transpose(cell2mat(params.dynamics.disturbance_sigma));
angular_rate = params.dynamics.hill.angular_rate;

% Communication network
range_threshold = params.network.range_threshold;

% Sensors
sensor_params = params.sensor;

% Rerence agent
init_state_ref.position = transpose(cell2mat(params.reference_agent.initial_state.position));
init_state_ref.velocity = transpose(cell2mat(params.reference_agent.initial_state.velocity));

% Initial estimates
initial_estimate_sigma.position = params.initial_estimate.sigma_position;
initial_estimate_sigma.velocity = params.initial_estimate.sigma_velocity;

% Estimator parameters
initial_covariance = params.estimators.initial_covariance;