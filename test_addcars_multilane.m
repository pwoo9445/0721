function othercars = test_addcars_multilane(othercars, track, nr_cars, NR_LANE, vel)
% ADD CARS IN RANDOM POSTIONS
VERBOSE = 0;

% fprintf('nr_seg: %2d nr_lane: %2d \n', track.nr_seg, track.nr_lane);
%{
randtemp = randperm((track.nr_seg-1)*track.nr_lane);
carposlist = randtemp(1:nr_cars);
for posidx = carposlist
    segidx = ceil(posidx / track.nr_lane)+1;
    laneidx = mod(posidx, track.nr_lane)+1;
    
    if VERBOSE
        fprintf('posidx: %2d segidx: %2d laneidx: %2d \n' ...
            , posidx, segidx, laneidx);
    end
    
    % RANDOM OFFSET
    seglen = track.seg{segidx}.d;
    randoffset = seglen*rand;
    
    % SET POSITION
    carpos = get_posintrack(track, segidx, randoffset, laneidx, 0);
    othercars = add_othercars(othercars, carpos, [0 0], 'normal');
end
%}

for laneidx = 2:NR_LANE
    for carid = 1:nr_cars
        %     if carid <= track.nr_seg
        %        laneidx= 2;
        %     end
        %     if carid > track.nr_seg
        %        laneidx= 3;
        %     end
        segidx = mod(carid-1,track.nr_seg)+1;
        seglen = track.seg{segidx}.d;
        randoffset = seglen*rand*0.5;
        %randoffset = seglen*0.5;
        carpos = get_posintrack(track, segidx, randoffset, laneidx, 0);
        %    othercars = add_othercars(othercars, carpos, [15000 0], 'normal');
        othercars = add_othercars(othercars, carpos, [vel 0], 'normal'); % 10000 mm/s = 36 km/h
        %othercars = add_othercars(othercars, carpos, [0 0], 'stop');
    end
end

% 残り1台の処理
% laneidx= 1;
% segidx = track.nr_seg;
% seglen = track.seg{segidx}.d;
% randoffset = seglen*rand*0.5;
% %randoffset = seglen*0.5;
% carpos = get_posintrack(track, segidx, randoffset, laneidx, 0);
% othercars = add_othercars(othercars, carpos, [0 0], 'stop');



function othercars = add_othercars(othercars, pos, vel, ctrlmode)
if nargin == 3
    ctrlmode = 'normal';
end
othercars.n = othercars.n + 1;
othercars.car{othercars.n}.pos = pos;
othercars.car{othercars.n}.vel = vel;
othercars.car{othercars.n}.ctrlmode = ctrlmode;
othercars.car{othercars.n}.paths = [];

