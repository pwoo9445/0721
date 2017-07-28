function [mycar, dec_me] = update_mycar_intelligent(mycar, sim, othercars, track, front_num)

% PARAMETER OF INTELLIGENT DRIVING MODEL---------------------
v0 = 10000; % desired velocity
T = 1.0; % Safe time headway
a = 4000; % maximum acceleration
b = 6000; %desired deceleration
delta = 4; %acceleration exponent
s0 = 5000; % minimum distance
l = 2500; % vehicle length
%============================================================


A1 = othercars.car{front_num}.vel(1)/v0;

if othercars.car{front_num}.pos(1) - mycar.pos(1) < 0 % if there is no other car front of this car
    A3 = track.xmax - mycar.pos(1) + othercars.car{front_num}.pos(1) - l;
else
    A3 = othercars.car{front_num}.pos(1) - mycar.pos(1) - l;
end
A2 = (s0 + mycar.vel(1)*T + mycar.vel(1) * (mycar.vel(1) - othercars.car{front_num}.vel(1))/2/sqrt(a*b))/A3;

mycar.vel(1) = mycar.vel(1) + a*(1 - A1^delta - A2^2)*sim.T;

dec_me = - a*(1 - A1^delta - A2^2);

if mycar.vel(1) < 0
    mycar.vel(1) = 0;
end

% UPDATE MY CAR INFORMATION
mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
mycar.bd  = get_carshape(mycar.pos, mycar.W, mycar.H);
mycar = update_rfs(mycar, othercars);



function mycar = update_rfs(mycar, othercars)
mycar.r = 0;
% EMULATE RFS
D2R    = pi/180;
nr_rfs = mycar.nr_rfs;
result = mycar.rfs_dist*ones(1, nr_rfs);
for i = 1:mycar.nr_rfs
    % FOR ALL RANGEFINDER BEAMS
    rfs_deg = mycar.pos(3) + mycar.rfs_degs(i);
    rfs_rad = rfs_deg * D2R;
    rfs_fr  = mycar.pos(1:2);
    rfs_to  = mycar.pos(1:2) ...
        + (mycar.r+mycar.rfs_dist)*[cos(rfs_rad) sin(rfs_rad)];
    min_dist2obs = mycar.rfs_dist;
    obs_boundary = zeros(2, 1E3);
    nr_obs_b = 0;
    for j = 1:othercars.n
        % FOR ALL OBSTACLES
        curr_obs_boundary = othercars.car{j}.bd;
        curr_obs_boundary = [curr_obs_boundary' [NaN ; NaN]];
        nr_curr_obs_b = size(curr_obs_boundary, 2);
        obs_boundary(:, nr_obs_b+1:nr_obs_b + nr_curr_obs_b) = curr_obs_boundary;
        nr_obs_b = nr_obs_b + nr_curr_obs_b;
    end
    obs_boundary = obs_boundary(:, 1:nr_obs_b);
    
    % Use InterX
    int_xy = InterX([rfs_fr(1) rfs_to(1) ; rfs_fr(2), rfs_to(2)], obs_boundary);
    if ~isempty(int_xy)
        xi = int_xy(1, :);
        yi = int_xy(2, :);
        for k = 1:length(xi)
            dist = norm([ mycar.pos(1)-xi(k), mycar.pos(2)-yi(k) ]);
            dist = dist - mycar.r;
            if dist < min_dist2obs
                min_dist2obs = dist;
            end
        end
    end
    result(i) = min_dist2obs;
end
mycar.rfs_dists = result;
