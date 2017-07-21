function othercars = respawn_othercars(othercars,track)
% RESPAWN OTHERCARS outside the course 
% made by kumano

for i = 1:othercars.n
    if othercars.car{i}.pos(1) > track.xmax
        if i <= track.nr_seg
            othercars.car{i}.pos = get_posintrack(track, 1, 0, 2, 0); % respawn from 2nd lane (no offset)
        else
            othercars.car{i}.pos = get_posintrack(track, 1, 0, 3, 0); % respawn from 2nd lane (no offset)
        end
    end
end
