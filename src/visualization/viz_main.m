
color_generator_    = ColorGenerator();
color_list.true     = color_generator_.getNormalizedRGB('benihi');
color_list.ref      = color_generator_.getNormalizedRGB('akabeni');

run('viz_trajectory.m');