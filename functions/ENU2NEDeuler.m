function euler_NED = ENU2NEDeuler(euler_ENU)
%ENU2NEDEULER Summary of this function goes here
%   Detailed explanation goes here
    R_en=rotMatZYX([pi,0,pi/2]);
    euler_NED=R_en*euler_ENU;
end

