function mycar = respawn_mycar(mycar,track)
% RESPAWN OTHERCARS outside the course 
% made by kumano

if mycar.pos(1) > track.xmax
    mycar.pos = get_posintrack(track, 1, 0, 2, 0); % respawn from 2nd lane (no offset)
end
