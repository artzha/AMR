function Vector2(x, y)
    return {x = x, y = y}
  end

map = "GDC1";
init_x = 14.7;
init_y = 14.24;
init_r = 0; -- in degrees

k_x = 1.0;
k_y = 1.0;
k_theta = 1.0;
num_particles = 50;

k_laser_loc=Vector2(0.2, 0.0);

d_short = 1;
d_long = 1;
likelihood_sigma = 0.1;
likelihood_gamma = 0.8;