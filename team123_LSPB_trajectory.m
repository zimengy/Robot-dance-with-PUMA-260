function [thetas,velo] = team123_LSPB_trajectory(t,t0,t1,q0,q1,v0,v1)
% Make tb to be 15% of the total time
tb=(t1-t0)*0.15;
LSPB_matrix = [
1 t0 t0^2 0 0 0 0 0;
0 1 2*t0 0 0 0 0 0;
1 t0+tb (t0+tb)^2 -1 -(t0+tb) 0 0 0;
0 1 2*(t0+tb) 0 -1 0 0 0;
0 0 0 -1 -(t1-tb) 1 t1-tb (t1-tb)^2;
0 0 0 0 -1 0 1 2*(t1-tb);
0 0 0 0 0 1 t1 t1^2;
0 0 0 0 0 0 1 2*t1 ;
];
Q = [ q0'; v0'; zeros(1,6); zeros(1,6); zeros(1,6); zeros(1,6); q1'; v1'];

abc = LSPB_matrix \ Q;
b0=abc(1,:);b1=abc(2,:);b2=abc(3,:);a0=abc(4,:);a1=abc(5,:);c0=abc(6,:);c1=abc(7,:);c2=abc(8,:);
thetas = zeros(6,1);
velo = zeros(6,1);
% Calculate joint angles and velocities 
for i=1:6
   if t>=t0 && t<t0+tb
       thetas(i)=b0(i)+b1(i)*t+b2(i)*t^2;
       velo(i) = b1(i)+2*b2(i)*t;
   elseif t>=t0+tb && t<= t1-tb
       thetas(i) = a0(i)+a1(i)*t;
       velo(i) = a1(i);
   else
       thetas(i)=c0(i)+c1(i)*t+c2(i)*t^2;
       velo(i) = c1(i)+2*c2(i)*t;
   end
end



