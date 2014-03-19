function [thetas, velo] = team123_lin_trajectory(t,tvia0,tvia1,thetavia0,thetavia1)
% tvia: via point times
% thetavia: joint angles

thetas = (thetavia1-thetavia0)/(tvia1-tvia0)*(t-tvia0)+thetavia0;
velo = (thetavia1-thetavia0)/(tvia1-tvia0);
