function mycar = init_mycar_rand(mycar_pos_rand, mycar_vel_rand)
% INITIALIZE MY CAR
mycar.pos = [mycar_pos_rand, 1700, 0];
mycar.vel = [mycar_vel_rand, 0];

[W, H] = get_carsize();
mycar.W = W; mycar.H = H;
mycar.bd = get_carshape(mycar.pos, mycar.W, mycar.H);

% RANGEFINDER SENSOR
mycar.r         = 0;
mycar.rfs_dist  = 20000;  % 20000
mycar.rfs_deg   = 270;     % 270
mycar.nr_rfs    = 19;      % 19
mycar.rfs_degs  = linspace(-mycar.rfs_deg/2, mycar.rfs_deg/2, mycar.nr_rfs);
mycar.rfs_dists = mycar.rfs_dist*ones(1, mycar.nr_rfs);


