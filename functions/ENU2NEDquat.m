function q_NED = ENU2NEDquat(q_ENU)
%Converts a quaternion given in ENU to NED
%   q is given on the form [w,i,j,k]
    
    q_NED=[q_ENU(:,1),q_ENU(:,3),q_ENU(:,2),-q_ENU(:,4)];
end

