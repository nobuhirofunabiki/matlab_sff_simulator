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
addpath(genpath('../YAMLMatlab_0.4.3'));

run('src/load_parameters.m');

time_stamp      = 0.0;
time_list       = zeros(1,num_steps);
display_timer   = 0.0;