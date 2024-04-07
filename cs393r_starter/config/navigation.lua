NavigationParameters = {
    dt = 0.05;
    -- 0.05 is the default value for system latency on real robot
    system_latency = 0.2; 

    max_linear_accel = 5.0;
    max_linear_deccel = 5.0;
    max_linear_speed = 1.0;

    max_angular_accel = 3.0;
    max_angular_deccel = 3.0;
    max_angular_speed = 1.0;

    max_curvature = 1.5;
    max_path_length = 10.0;
    max_clearance = 1.0;

    clearance_weight = 10.0;
    arc_length_weight = 3.0;
    distance_weight = 6.0;

    goal_tolerance = 0.1;

    robot_length = 0.535;
    robot_width = 0.281;
    robot_wheelbase = 0.324;
    obstacle_margin = 0.05;
    lidar_offset = 0.21;

    resolution = 0.15; -- 0.5 meter of accuracy -> meters / cell
    robotRadius = 0.4; -- max radius of robot when turning
    -- robot_radius    = 0.45; -- radius of robot length and width + obstacle margin

    waypoints_coeff = 5; -- distance between waypoints = coeff * 1 / max_curvature
    carrot_distance = 1.5; -- distance threshold to carrot before moving it
    goal_distance   = 0.1; -- distance threshold to goal before stopping
}
