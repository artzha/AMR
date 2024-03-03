function Vector2(x, y)
    return {x = x, y = y}
  end

map = "GDC1";
init_x = 14.7;
init_y = 14.24;
init_r = 0; -- in degrees

k_x = 0.5;
k_y = 1.0;
k_theta = 3.0;
num_particles = 20;
ds_factor = 3;

k_laser_loc=Vector2(0.2, 0.0);

d_short = 0.5;
d_long = 0.5;
likelihood_sigma = 1.0;
likelihood_gamma = 0.8;

dist_threshold = 5.0;
angle_threshold = 0.1;