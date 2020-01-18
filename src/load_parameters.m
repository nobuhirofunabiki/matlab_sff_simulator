%% Load parameters from the configuration file

params = ReadYaml('../config/parameters.yaml');

% Load files
initial_states = readmatrix(['../config/initial_states/', params.file_name.initial_states]);
communication_time_table = readmatrix(['../config/mesh_network/', params.file_name.mesh_network]);

% General parameters
num_agents = size(initial_states,2);

% Time parameters
total_simulation_time   = params.time.total_simulation;
delta_time_rk           = params.time.delta_runge_kutta;
display_period          = params.time.display_period;
num_steps               = round(total_simulation_time/delta_time_rk);

% Dynamics parameters
num_dims = params.dynamics.dimension;

% Rerence agent
init_state_ref = params.reference_agent.initial_state;