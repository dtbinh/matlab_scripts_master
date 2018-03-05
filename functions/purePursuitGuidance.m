function [v_d] = purePursuitGuidance(p_c,p_t,dp)
%PURE_PURSUIT_GUIDANCE Summary of this function goes here
%   p_c -> position vector NED to craft
%   p_t -> position vector NED to target
%   dp -> Tuning constant
    p_d=p_c-p_t;    %Diff
    
    v_d=-dp*p_d/(norm(p_d)+0.01);
end

