function [thetas, velo] = team123_cubic_trajectory(t,tvia0,tvia1,thetavia0,thetavia1,thetadotvia0,thetadotvia1)

cubic_matrix = [1 tvia0 tvia0^2 tvia0^3; 0 1 2*tvia0 3*tvia0^2; 1 tvia1 tvia1^2 tvia1^3; 0 1 2*tvia1 3*tvia1^2];
y=[thetavia0'; thetadotvia0'; thetavia1'; thetadotvia1'];

a=cubic_matrix \ y;
a=a';
% Calculate joint angles
thetas = a*[1;t;t^2;t^3];
aa = [a(:,2) 2*a(:,3) 3*a(:,4)]; 
% Calculate joint velocities
velo = aa*[1;t;t^2]; 
