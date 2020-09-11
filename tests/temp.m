omega=FK_omega2_c;

ang = fo2.orientation + omega*dt;
ang = atan2(sin(ang),cos(ang));
vec = FK_vel2_c*[cos(ang),sin(ang)];