% Moments of inertia of a solid rectangular cuboid and a solid cylinder
% https://en.wikipedia.org/wiki/List_of_moments_of_inertia

half_front_track_width = 0.819;
half_rear_track_width = 0.8;
half_wheelbase = 1.326;

body_mass = 1620.0;
body_width = 2 * half_rear_track_width;
body_depth = 2 * half_wheelbase + 0.8;
body_length = 1.25;

body_ixx = body_mass / 12 * (body_width^2 + body_length^2)
body_iyy = body_mass / 12 * (body_length^2 + body_depth^2)
body_izz = body_mass / 12 * (body_width^2 + body_depth^2)


wheel_mass = 40.0;
wheel_radius = 0.36;
wheel_thickness = 0.25;

wheel_ixx = wheel_mass / 12 * (3 * wheel_radius^2 + wheel_thickness^2)
wheel_iyy = wheel_mass / 12 * (3 * wheel_radius^2 + wheel_thickness^2)
wheel_izz = wheel_mass / 2 * wheel_radius^2
