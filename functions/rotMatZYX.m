function [R] = rotMatZYX(ang)
%ROSMAT Summary of this function goes here
%   The first rotation is about the Z axis
%   The second rotation is about the new Y axis.
%   The third rotation is about the new X axis.

    phi=ang(1);
    theta=ang(2);
    psi=ang(3);
    
    R=rotMat_Z(psi)*rotMat_Y(theta)*rotMat_X(phi);
end

function R = rotMat_Z(ang)
    R=[cos(ang), -sin(ang),0;sin(ang),cos(ang),0;0,0,1];
end

function R = rotMat_Y(ang)
    R=[cos(ang), 0, sin(ang); 0, 1, 0;-sin(ang), 0 cos(ang)];
end

function R = rotMat_X(ang)
    R=[1,0,0;0 cos(ang), -sin(ang);0, sin(ang), cos(ang)];
end



