function v_d = constantBearingGuidance(p_c,p_t,v_t,v_c_max,dp,v_max)
%Constant Bearing Guidance controller
%   p_c -> position vector NED to craft
%   p_t -> position vector NED to target
%   v_t -> velocity vector target/NED
%   v_c_max -> maxs vraft velocity
%   dp -> 
    p_d=p_c-p_t;    %Diff
    
    % Debug:    
    K=v_c_max*norm(p_d)/sqrt(p_d'*p_d+dp^2);
    v_a=-K*p_d/(norm(p_d)+0.01);
    %Limit vel
    if 1==0
        if norm(v_t+v_a) > v_max
            v_a=findNumva(v_t,v_a,v_max);
        end
    end

    v_d=v_t+v_a;

    function v_a_ret = findNumva(v_t,v_a,v_max)
        if norm(v_t)>v_max
            v_a=v_a*0;
        else
            z=0;
            for zeta=0:0.01:1
                if norm(v_t+zeta*v_a)>v_max
                    break;
                else
                    z=zeta;
                end
            end
            v_a=z*v_a;
        end
        v_a_ret=v_a;
    end
end

