
color_generator_    = ColorGenerator();
color_list.true     = color_generator_.getNormalizedRGB('benihi');
color_list.ref      = color_generator_.getNormalizedRGB('akabeni');
color_list.ceif     = color_generator_.getNormalizedRGB('ruri');

run('viz_trajectory.m');