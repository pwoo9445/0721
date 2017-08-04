function ems = plot_turnSignal(mycar,sim)
% PLOT TURN SIGNAL
persistent first_flag signal_flag signal_time h
if isempty(first_flag)
    first_flag = true;
end
iclk = clock;

%--- initialize --------
if isfield(mycar,'turnSignal')==0
    mycar.turnSignal = '';
end
%-----------------------


%-- blinker position -----------
L =500; % blinker size
%-------
pt(1:2)  = mycar.bd(1,1:2);
bd1 = [pt(1), pt(1)+L, pt(1)+L, pt(1), pt(1); pt(2), pt(2), pt(2)+L,  pt(2)+L, pt(2)];
bd1 = rotationBD(bd1, pt, mycar);
%-------
pt(1:2)  = mycar.bd(2,1:2);
bd2 = [pt(1), pt(1), pt(1)-L, pt(1)-L, pt(1); pt(2), pt(2)+L, pt(2)+L,  pt(2), pt(2)];
bd2 = rotationBD(bd2, pt, mycar);
%-------
pt(1:2)  = mycar.bd(5,1:2);
bd3 = [pt(1), pt(1)-L, pt(1)-L, pt(1), pt(1); pt(2), pt(2), pt(2)-L,  pt(2)-L, pt(2)];
bd3 = rotationBD(bd3, pt, mycar);
%-------
pt(1:2)  = mycar.bd(7,1:2);
bd4 = [pt(1), pt(1), pt(1)+L, pt(1)+L, pt(1); pt(2), pt(2)-L, pt(2)-L,  pt(2), pt(2)];
bd4 = rotationBD(bd4, pt, mycar);
%-------------------------------

if first_flag
    first_flag = false;    
    % CURRENT POSITION
    h.carfill{1} = fill(bd1(1, :), bd1(2, :), 'k');
    h.carfill{2} = fill(bd2(1, :), bd2(2, :), 'k');
    h.carfill{3} = fill(bd1(1, :), bd1(2, :), 'k');
    h.carfill{4} = fill(bd2(1, :), bd2(2, :), 'k');
    signal_flag = 0;
else
    % UPDATE POSITION
    h.carfill{1}.Vertices = bd1';
    h.carfill{2}.Vertices = bd2';
    h.carfill{3}.Vertices = bd3';
    h.carfill{4}.Vertices = bd4';

    % UPDATE SIGNAL COLOR    
    switch lower(mycar.turnSignal)
    case ''
       h.carfill{1}.FaceColor = 'k';
       h.carfill{2}.FaceColor = 'k';
       h.carfill{3}.FaceColor = 'k';
       h.carfill{4}.FaceColor = 'k';
       signal_flag = 0;
       signal_time = sim.sec;
    case 'right'
        if (signal_flag == 1)&& ((sim.sec - signal_time)> 0.2)
          h.carfill{1}.FaceColor = 'k';
          h.carfill{2}.FaceColor = 'k';
          h.carfill{3}.FaceColor = 'k';
          h.carfill{4}.FaceColor = 'k';
          signal_flag = 0;
          signal_time = sim.sec;
        elseif (signal_flag == 0)&& ((sim.sec - signal_time)> 0.2)
          h.carfill{1}.FaceColor = 'y';
          h.carfill{2}.FaceColor = 'y';          
          h.carfill{3}.FaceColor = 'k';
          h.carfill{4}.FaceColor = 'k';
          signal_flag = 1;
          signal_time = sim.sec;
        end
    case 'left'
        if (signal_flag == 1)&& ((sim.sec - signal_time)> 0.2)
          h.carfill{1}.FaceColor = 'k';
          h.carfill{2}.FaceColor = 'k';
          h.carfill{3}.FaceColor = 'k';
          h.carfill{4}.FaceColor = 'k';
          signal_flag = 0;
          signal_time = sim.sec;
        elseif (signal_flag == 0)&& ((sim.sec - signal_time)> 0.2)
          h.carfill{1}.FaceColor = 'k';
          h.carfill{2}.FaceColor = 'k';
          h.carfill{3}.FaceColor = 'y';
          h.carfill{4}.FaceColor = 'y';          
          signal_flag = 1;
          signal_time = sim.sec;
        end
    end 
end
ems = etime(clock, iclk)*1000;

end

function box_rot = rotationBD(box_ori, centerPt, mycar)

pos  = centerPt;
theta = mycar.pos(3);
one = ones(1,5);

c    = cos(theta*pi/180);
s    = sin(theta*pi/180);
rotation = [c -s; s c];


box_ori(1,:) = box_ori(1,:) - pos(1)*one;
box_ori(2,:) = box_ori(2,:) - pos(2)*one;

box_rot = rotation*box_ori;

box_rot(1,:) = box_rot(1,:)+ pos(1)*one;
box_rot(2,:) = box_rot(2,:)+ pos(2)*one;

end