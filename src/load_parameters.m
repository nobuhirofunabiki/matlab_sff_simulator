%% Load parameters from the configuration file

params = ReadYaml('../config/parameters.yaml');

% Time parameters
total_simulation_time   = params.time.total_simulation;
delta_time_rk           = params.time.delta_runge_kutta;
display_period          = params.time.display_period;
num_steps               = round(total_simulation_time/delta_time_rk);;