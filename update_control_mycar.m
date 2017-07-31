function mycar = update_control_mycar(mycar, sim, othercars,pathTranslated,ratioSpeed)
% UPDATE MY CAR INFORMATION
persistent first_flag
if isempty(first_flag)
    first_flag = true;
end
%----Initial acceleration for lane changing----
if first_flag
    first_flag = false;
    mycar.vel(1) = mycar.vel(1)*ratioSpeed;
end
%----------------------------------------------

%---- Control angular velocity: vel(2) --------
pos = predict_pos(mycar.pos, mycar.vel, sim.T);
targetDegree = get_tatgetTheta(pos,pathTranslated);
if targetDegree - pos(3) > 1
    mycar.vel(2) = mycar.vel(2) + 1/sim.T;
elseif targetDegree - pos(3) < -1
    mycar.vel(2) = mycar.vel(2) - 1/sim.T;
else
    mycar.vel(2) = mycar.vel(2) + (targetDegree - pos(3))/sim.T;
end
%----------------------------------------------

% fprintf(1, 'mycar.pos(3) = [%4d] targetDegree = [%4d] mycar.vel(2) = [%4d] (targetDegree - pos(3))/sim.T = [%4d]\n', mycar.pos(3), targetDegree, mycar.vel(2), (targetDegree - pos(3))/sim.T);
mycar.pos = update_pos(mycar.pos, mycar.vel, sim.T);
mycar.bd  = get_carshape(mycar.pos, mycar.W, mycar.H);
mycar = update_rfs(mycar, othercars);

end

function targetDegree = get_tatgetTheta(pos,path)

nData = size(path,1);

dist=bsxfun(@hypot,path(:,1)-pos(1),path(:,2)-pos(2));
[~,idx]=min(dist);

if idx~=nData
   vx= path(idx+1,1)-path(idx,1); 
   vy= path(idx+1,2)-path(idx,2); 
else
   vx= path(idx,1)-path(idx-1,1); 
   vy= path(idx,2)-path(idx-1,2);   
end

targetDegree =atan(vy/vx)*180/pi;

end

function pos = predict_pos(pos, vel, T)

c = cos(pos(3)*pi/180);
s = sin(pos(3)*pi/180);
pos(1:2) = pos(1:2) + vel(1)*T*[c s];
pos(3) = pos(3) + vel(2)*T;

% DETERMINE DEGREE TO LIE BETWEEN -180~180
while pos(3) > 180
    pos(3) = pos(3) - 360;
end
while pos(3) < -180
    pos(3) = pos(3) + 360;
end

end


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

end
