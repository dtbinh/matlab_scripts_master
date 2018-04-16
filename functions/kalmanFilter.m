classdef kalmanFilter < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        H = zeros(9,6);
        z = zeros(9,1);
        x_hat
        x_hat_pri
        P_pri
        P
        Phi=zeros(6,6);
        Delta=zeros(6,3);
        u=zeros(3,1);
        Q=zeros(6,6);
        R=zeros(9,9);
    end
    
    methods
        function obj =  kalmanFilter(Q,R,x0,P0)
            obj.Q=Q;
            obj.R=R;
            obj.x_hat=x0;
            obj.P=P0;
        end
        
        function projectAhead(obj,dt)
            %Update phi and delta:
            [obj.Phi,obj.Delta]=obj.updatePhiDelta(dt);
            
            % Project ahead:
            obj.x_hat_pri=obj.Phi*obj.x_hat+obj.Delta*obj.u;
            obj.P_pri=obj.Phi*obj.P*transpose(obj.Phi)+obj.Q;
        end
        
        function updateMeasurement(obj,type,measurement)
            % Update the H  z matrix
            obj.H=obj.getH(type);
            obj.z=obj.getZ(type,measurement);
            
            obj.updateFilter();
        end
    %end
    
    %methods(Access=private)
        function updateFilter(obj)
            
            % Compute Kalman gain:
            K=obj.P_pri*transpose(obj.H)*inv(obj.H*obj.P_pri*transpose(obj.H)+obj.R);

            % Update estimate with measurement z:
            I=eye(6,6);
            obj.x_hat=obj.x_hat_pri+K*(obj.z-obj.H*obj.x_hat_pri);
            obj.P=(I-K*obj.H)*obj.P_pri*transpose(I-K*obj.H)+K*obj.R*transpose(K);
            
            % Set predictionas a priori if there is several measurement
            % updates
            obj.x_hat_pri=obj.x_hat;
            obj.P_pri=obj.P;
        end
        
        function [Phi,Delta] = updatePhiDelta(obj,dt)
            Phi=eye(6);
            Phi(1:3,4:6)=eye(3)*dt;

            Delta=[-eye(3)*dt;zeros(3)];
        end

        function z = getZ(obj,measure,data)
            z=zeros(9,1);
            if measure=="Aruco"
                z(4:6)=data;
            elseif measure=="LP_pos"
                z(1:3)=data;
            elseif measure=="LP_vel"
                z(7:9)=data;
            end
        end

        function H = getH(obj,measure)
            if measure=="Aruco"
                H=[zeros(3,6);eye(3,3),zeros(3,3);zeros(3,6)];
            elseif measure=="LP_pos"
                H=[eye(3,3),zeros(3,3);zeros(3,6);zeros(3,6)];
            elseif measure=="LP_vel"
                H=[zeros(3,6);zeros(3,6);zeros(3,3),eye(3,3)];
            else
                H=zeros(9,6);
            end
        end
    end
end

