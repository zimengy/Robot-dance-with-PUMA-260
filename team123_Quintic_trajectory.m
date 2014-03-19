function [thetas, velo] = team123_Quintic_trajectory(t,t0,t1,q0,q1,v0,v1,a0,a1)

quintic_matrix = [1 t0 t0^2 t0^3 t0^4 t0^5; 0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 0 0 2 6*t0 12*t0^2 20*t0^3;1 t1 t1^2 t1^3 t1^4 t1^5; 0 1 2*t1 3*t1^2 4*t1^3 5*t1^4; 0 0 2 6*t1 12*t1^2 20*t1^3];
y=[q0'; v0'; a0'; q1'; v1'; a1'];
a=quintic_matrix \ y;
a=a';
% Calculate joint angles
thetas = a*[1;t;t^2;t^3;t^4;t^5];
aa = [a(:,2) 2*a(:,3) 3*a(:,4) 4*a(:,5) 5*a(:,6)]; 
% Calculate joint velocities
velo = aa*[1;t;t^2;t^3;t^4]; 
