function [guiding_vector]=convertCommand(agt,vel_c,omega_c,dt)
vel_mag = vel_c;
ang = agt.orientation + omega_c * dt;
ang = atan2(sin(ang),cos(ang));
guiding_vector = vel_mag * [cos(ang),sin(ang)];
end