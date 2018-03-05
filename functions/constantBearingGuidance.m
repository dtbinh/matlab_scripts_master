function v_d = constantBearingGuidance(p_c,p_t,v_t,v_c_max,dp)
%Constant Bearing Guidance controller
%   p_c -> position vector NED to craft
%   p_t -> position vector NED to target
%   v_t -> velocity vector target/NED
%   v_c_max -> maxs vraft velocity
%   dp -> 
    p_d=p_c-p_t;    %Diff
    
    % Debug:    
    K=v_c_max*norm(p_d)/sqrt(p_d'*p_d+dp^2);
    %Limit vel
    K=min(K,v_c_max-norm(v_t));
    
    v_a=-K*p_d/(norm(p_d)+0.01);
    v_d=v_t+v_a;
end