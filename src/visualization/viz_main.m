
color_generator_    = ColorGenerator();
color_list.true     = color_generator_.getNormalizedRGB('benihi');
color_list.ref      = color_generator_.getNormalizedRGB('akabeni');
color_list.ceif     = color_generator_.getNormalizedRGB('ruri');
color_list.deif     = color_generator_.getNormalizedRGB('purple');

language = 'Japanese';   % English or Japanese

run('viz_trajectory.m');
run('viz_estimation_performance.m');
run('viz_network.m');